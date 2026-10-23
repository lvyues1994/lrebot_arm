#!/usr/bin/env python3
"""Fits the links' masses and centers of mass of the reBot B601-RS arm to a gravity calibration.

Reads the `calibration` lines that `larm_driver_probe calibrate` writes. Each pose was approached
from below and from above every joint; the mean of the two holding torques cancels Coulomb
friction, half their difference estimates it. Gravity is linear in each link's mass m and first
moment m*c, tau = Y(q) phi. The fit is the most probable phi given the data and the CAD values as a
prior (Gaussian, 20 % on masses, 1 cm on centers of mass), so the directions the poses do not
excite stay at CAD. Prints residuals before and after, per joint, and the profile section
`dynamics.links` with the links whose parameters changed.

Usage:
    fit_gravity.py DATA.jsonl [--validation DATA.jsonl] [--output FILE]
    fit_gravity.py --synthetic [--seed S]    # checks the fit on data from a perturbed model

Needs the ROS 2 Pinocchio (source /opt/ros/<distro>/setup.bash).
"""

import argparse
import collections
import json
import pathlib
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
URDF = ROOT / "generated/urdf/rebot_b601_rs.urdf"
POSES = ROOT / "config/gravity_calibration_poses.yaml"
LOCKED = ["joint_left", "joint_right"]
LINKS = ["link1", "link2", "link3", "link4", "link5", "link6"]
# Prior spread of the CAD values, and of a holding torque once friction has been averaged out.
MASS_SPREAD = 0.2
COM_SPREAD = 0.01
TORQUE_SPREAD = 0.1
# Changes below these are not written to the profile.
MASS_CHANGE = 0.005
COM_CHANGE = 0.0005


def arm_model():
    full = pin.buildModelFromXML(URDF.read_text())
    locked = [full.getJointId(name) for name in LOCKED]
    return pin.buildReducedModel(full, locked, pin.neutral(full))


def described_links():
    """Each link's own mass and first moment in its frame, as the URDF has them."""
    links = {}
    for link in ET.parse(URDF).getroot().iter("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = float(inertial.find("mass").get("value"))
        origin = inertial.find("origin")
        com = np.array([float(v) for v in origin.get("xyz").split()]) if origin is not None else np.zeros(3)
        links[link.get("name")] = np.concatenate([[mass], mass * com])
    return links


class Gravity:
    """Static joint torques of the arm as a linear function of its bodies' masses and first moments."""

    def __init__(self, model):
        self.model = model
        self.data = model.createData()
        self.bodies = model.njoints - 1
        self.prior = np.concatenate([model.inertias[k].toDynamicParameters()[:4] for k in range(1, model.njoints)])

    def regressor(self, q):
        zero = np.zeros(self.model.nv)
        full = pin.computeJointTorqueRegressor(self.model, self.data, np.asarray(q), zero, zero)
        return full[:, [10 * body + k for body in range(self.bodies) for k in range(4)]]

    def torque(self, q, phi):
        return self.regressor(q) @ phi


def prior_spread(phi):
    spread = np.empty_like(phi)
    for body in range(len(phi) // 4):
        mass = phi[4 * body]
        spread[4 * body] = max(MASS_SPREAD * mass, 0.01)
        moment = phi[4 * body + 1:4 * body + 4]
        spread[4 * body + 1:4 * body + 4] = np.sqrt((MASS_SPREAD * moment) ** 2 + (mass * COM_SPREAD) ** 2) + 1e-4
    return spread


def fit(gravity, poses):
    """Most probable parameters given the averaged torques of each pose and the CAD prior."""
    rows = np.vstack([y for y, _ in poses]) / TORQUE_SPREAD
    targets = np.concatenate([tau for _, tau in poses]) / TORQUE_SPREAD
    spread = prior_spread(gravity.prior)
    a = np.vstack([rows, np.diag(1.0 / spread)])
    b = np.concatenate([targets, gravity.prior / spread])
    return np.linalg.lstsq(a, b, rcond=None)[0]


def residual(gravity, poses, phi):
    errors = np.array([y @ phi - tau for y, tau in poses])
    return np.sqrt((errors ** 2).mean(axis=0))


def load(files, gravity):
    """Averaged regressor and torque of each pose visited from both sides, and the friction estimates."""
    sides = collections.defaultdict(dict)
    for file in files:
        for line in pathlib.Path(file).read_text().splitlines():
            event = json.loads(line)
            if event.get("event") == "calibration":
                sides[(file, event["set"], event["index"])][event["side"]] = event
    poses, friction, still = [], [], []
    for key, pair in sorted(sides.items()):
        if set(pair) != {-1, 1}:
            continue
        low, high = pair[-1], pair[1]
        y = 0.5 * (gravity.regressor(low["position"]) + gravity.regressor(high["position"]))
        poses.append((y, 0.5 * (np.array(low["torque"]) + np.array(high["torque"]))))
        friction.append(0.5 * (np.array(low["torque"]) - np.array(high["torque"])))
        still.append(max(low["max_speed"], high["max_speed"]))
    return poses, np.array(friction), max(still, default=0.0)


def link_parameters(gravity, phi):
    """The fitted bodies as the URDF links' own mass and center of mass; fixed extras stay at CAD."""
    described = described_links()
    result = {}
    for body, link in enumerate(LINKS):
        extra = gravity.prior[4 * body:4 * body + 4] - described[link]
        own = phi[4 * body:4 * body + 4] - extra
        result[link] = (own[0], own[1:] / own[0], described[link][0], described[link][1:] / described[link][0])
    return result


def report(gravity, poses, validation, friction, phi):
    names = [f"joint{i}" for i in range(1, gravity.model.nq + 1)]
    print("RMS of (model - measured) holding torque, N m:")
    print("  " + " ".join(f"{name:>8}" for name in names))
    for label, data in (("calibration", poses), ("validation", validation)):
        if not data:
            continue
        for phase, parameters in (("CAD", gravity.prior), ("fitted", phi)):
            print(f"  {' '.join(f'{v:8.3f}' for v in residual(gravity, data, parameters))}   {label}, {phase}")
    if len(friction):
        print("Friction (half the difference between the two approaches), mean |.| N m:")
        print("  " + " ".join(f"{v:8.3f}" for v in np.abs(friction).mean(axis=0)))


def profile_section(links):
    lines = ["dynamics:", "  links:"]
    for link, (mass, com, cad_mass, cad_com) in links.items():
        if abs(mass - cad_mass) < MASS_CHANGE and np.abs(com - cad_com).max() < COM_CHANGE:
            continue
        lines.append(f"    {link}: {{mass: {mass:.4f}, com: [{com[0]:.5f}, {com[1]:.5f}, {com[2]:.5f}]}}"
                     f"  # CAD {cad_mass:.4f} kg, [{cad_com[0]:.5f}, {cad_com[1]:.5f}, {cad_com[2]:.5f}]")
    return "\n".join(lines) + "\n"


def synthetic(gravity, seed):
    """Data from a perturbed arm with friction and noise, fitted and checked against the truth."""
    random = np.random.default_rng(seed)
    truth = gravity.prior.copy()
    for body in range(gravity.bodies):
        mass = truth[4 * body] * (1.0 + random.uniform(-0.15, 0.15))
        com = gravity.prior[4 * body + 1:4 * body + 4] / gravity.prior[4 * body] + random.uniform(-0.01, 0.01, 3)
        truth[4 * body:4 * body + 4] = np.concatenate([[mass], mass * com])
    plan = yaml.safe_load(POSES.read_text())

    def measured(q_list):
        poses = []
        for q in q_list:
            q = np.array(q)
            coulomb = random.uniform(0.2, 0.5, len(q))
            low = gravity.torque(q, truth) + coulomb + random.normal(0, 0.03, len(q))
            high = gravity.torque(q, truth) - coulomb + random.normal(0, 0.03, len(q))
            poses.append((gravity.regressor(q), 0.5 * (low + high)))
        return poses

    poses, validation = measured(plan["poses"]), measured(plan["validation"])
    phi = fit(gravity, poses)
    report(gravity, poses, validation, [], phi)
    checks = [np.array(gravity.model.lowerPositionLimit) + (np.array(gravity.model.upperPositionLimit) -
              np.array(gravity.model.lowerPositionLimit)) * random.random(gravity.model.nq) for _ in range(200)]
    for label, parameters in (("CAD", gravity.prior), ("fitted", phi)):
        errors = np.array([gravity.torque(q, parameters) - gravity.torque(q, truth) for q in checks])
        print(f"over the whole joint range, RMS error against the truth ({label}): "
              + " ".join(f"{v:.3f}" for v in np.sqrt((errors ** 2).mean(axis=0))))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("data", nargs="*", help="probe output with calibration lines")
    parser.add_argument("--validation", nargs="*", default=[], help="probe output of validation poses")
    parser.add_argument("--output", type=pathlib.Path, help="write the dynamics section here")
    parser.add_argument("--synthetic", action="store_true")
    parser.add_argument("--seed", type=int, default=1)
    arguments = parser.parse_args()

    gravity = Gravity(arm_model())
    check = np.array([0.0, 0.7, 1.1, 0.3, -0.4, 0.2])
    assert np.allclose(gravity.torque(check, gravity.prior),
                       pin.computeGeneralizedGravity(gravity.model, gravity.model.createData(), check))

    if arguments.synthetic:
        synthetic(gravity, arguments.seed)
        return
    if not arguments.data:
        parser.error("give the calibration data, or --synthetic")
    poses, friction, still = load(arguments.data, gravity)
    validation, _, _ = load(arguments.validation, gravity)
    print(f"{len(poses)} calibration poses, {len(validation)} validation poses; fastest joint while recording "
          f"{still:.4f} rad/s")
    phi = fit(gravity, poses)
    report(gravity, poses, validation, friction, phi)
    section = profile_section(link_parameters(gravity, phi))
    print(section, end="")
    if arguments.output:
        arguments.output.write_text(section)


if __name__ == "__main__":
    main()
