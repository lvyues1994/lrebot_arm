#!/usr/bin/env python3
"""Poses for the gravity calibration of the reBot B601-RS arm.

Samples arm configurations around the ready pose and keeps those that are free
of contact: every collision pair the SRDF checks stays 5 mm apart, and every
moving part stays 5 cm above the table (the plane of the base). The same holds
for the approach points on both sides of each pose and for the joint-space path
from one pose to the next. Calibration poses are chosen greedily for the
information they add to the gravity regressor (masses and first moments of the
links); validation poses are drawn from the rest.

Usage:
    calibration_poses.py [--count N] [--validation M] [--seed S] [--output FILE]

Needs the ROS 2 Pinocchio (source /opt/ros/<distro>/setup.bash) and the
generated description (generate_description.py).
"""

import argparse
import pathlib
import tempfile

import numpy as np
import pinocchio as pin

ROOT = pathlib.Path(__file__).resolve().parents[1]
URDF = ROOT / "generated/urdf/rebot_b601_rs.urdf"
SRDF = ROOT / "generated/srdf/rebot_b601_rs.srdf"
OUTPUT = ROOT / "config/gravity_calibration_poses.yaml"

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
LOCKED = ["joint_left", "joint_right"]
READY = np.array([0.0, 0.7, 1.1, 0.0, 0.0, 0.0])
# Sampled ranges; joint1 turns about the vertical and carries no gravity.
RANGES = np.array([[0.0, 0.0], [0.3, 1.4], [0.4, 1.9], [-1.2, 1.2], [-1.2, 1.2], [-1.5, 1.5]])
APPROACH = np.array([0.0, 0.05, 0.05, 0.05, 0.05, 0.05])
# The forearm and the wrist (link3, link5) come within 8 mm of each other at the ready pose.
SELF_CLEARANCE = 0.005
TABLE_CLEARANCE = 0.05
# Parts that do not move relative to the table.
FIXED_PARTS = ("base_link", "link1")
PATH_STEP = 0.02


class Arm:
    def __init__(self):
        # Pinocchio's Python parser does not resolve mesh paths relative to the URDF.
        text = URDF.read_text().replace('filename="../', f'filename="{URDF.parent.parent}/')
        self.urdf = tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False)
        self.urdf.write(text)
        self.urdf.close()
        full = pin.buildModelFromUrdf(self.urdf.name)
        geometry = pin.buildGeomFromUrdf(full, self.urdf.name, pin.GeometryType.COLLISION)
        geometry.addAllCollisionPairs()
        pin.removeCollisionPairs(full, geometry, str(SRDF))
        locked = [full.getJointId(name) for name in LOCKED]
        self.model, self.geometry = pin.buildReducedModel(full, geometry, locked, pin.neutral(full))
        self.data = self.model.createData()
        self.geometryData = pin.GeometryData(self.geometry)
        # Collision with a margin: exact distances between the meshes are a thousand times slower.
        for request in self.geometryData.collisionRequests:
            request.security_margin = SELF_CLEARANCE
        self.moving = [
            index for index, part in enumerate(self.geometry.geometryObjects)
            if not part.name.startswith(FIXED_PARTS)
        ]
        self.corners = []
        for part in self.geometry.geometryObjects:
            part.geometry.computeLocalAABB()
            low, high = np.asarray(part.geometry.aabb_local.min_), np.asarray(part.geometry.aabb_local.max_)
            self.corners.append(np.array([[x, y, z] for x in (low[0], high[0]) for y in (low[1], high[1])
                                          for z in (low[2], high[2])]))

    def clear(self, q):
        """True when q keeps every checked pair and every moving part clear."""
        pin.updateGeometryPlacements(self.model, self.data, self.geometry, self.geometryData, q)
        for index in self.moving:
            placement = self.geometryData.oMg[index]
            heights = self.corners[index] @ placement.rotation.T[:, 2] + placement.translation[2]
            if heights.min() < TABLE_CLEARANCE:
                return False
        return not pin.computeCollisions(self.model, self.data, self.geometry, self.geometryData, q, True)

    def clear_with_approaches(self, q):
        return all(self.clear(point) for point in (q, q - APPROACH, q + APPROACH))

    def clear_path(self, a, b):
        steps = max(1, int(np.ceil(np.abs(b - a).max() / PATH_STEP)))
        return all(self.clear(a + (b - a) * s) for s in np.linspace(0.0, 1.0, steps + 1))

    def regressor(self, q):
        """Static joint torques per link parameter: mass and first moments of each moving body."""
        zero = np.zeros(self.model.nv)
        full = pin.computeJointTorqueRegressor(self.model, self.data, q, zero, zero)
        columns = [10 * body + k for body in range(self.model.njoints - 1) for k in range(4)]
        return full[:, columns]


def greedy_design(arm, candidates, count):
    """Adds, one at a time, the pose that most increases log det of the information matrix."""
    chosen = [0]
    information = arm.regressor(candidates[0]).T @ arm.regressor(candidates[0])
    regressors = [arm.regressor(q) for q in candidates]
    ridge = 1e-6 * np.eye(information.shape[0])
    while len(chosen) < count:
        best, best_gain = None, -np.inf
        for index, y in enumerate(regressors):
            if index in chosen:
                continue
            gain = np.linalg.slogdet(information + y.T @ y + ridge)[1]
            if gain > best_gain:
                best, best_gain = index, gain
        chosen.append(best)
        information += regressors[best].T @ regressors[best]
    return chosen


def tour(arm, poses):
    """Nearest-neighbour order from ready, with ready inserted where a direct path is not clear."""
    remaining = list(range(len(poses)))
    order, current = [], READY
    while remaining:
        nearest = min(remaining, key=lambda index: np.abs(poses[index] - current).max())
        remaining.remove(nearest)
        if not arm.clear_path(current, poses[nearest]):
            if not (arm.clear_path(current, READY) and arm.clear_path(READY, poses[nearest])):
                raise RuntimeError(f"no clear path to pose {np.round(poses[nearest], 3)}")
            order.append(READY)
        order.append(poses[nearest])
        current = poses[nearest]
    return order


def write(path, calibration, validation, seed):
    def rows(poses):
        return "\n".join("  - [" + ", ".join(f"{value:.3f}" for value in q) + "]" for q in poses)

    path.write_text(
        "# Generated by scripts/calibration_poses.py --seed " + str(seed) + "; do not edit by hand.\n"
        "# Arm joints joint1..joint6 in radians, in the order they are visited from the ready pose.\n"
        f"# Contact-free: checked pairs {SELF_CLEARANCE * 1000:.0f} mm apart, moving parts "
        f"{TABLE_CLEARANCE * 100:.0f} cm above the table,\n"
        "# also at both approach points and along the joint-space path between neighbours.\n"
        "approach: [" + ", ".join(f"{value:g}" for value in APPROACH) + "]\n"
        "poses:\n" + rows(calibration) + "\n"
        "validation:\n" + rows(validation) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--count", type=int, default=25)
    parser.add_argument("--validation", type=int, default=8)
    parser.add_argument("--candidates", type=int, default=600)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--output", type=pathlib.Path, default=OUTPUT)
    arguments = parser.parse_args()

    arm = Arm()
    random = np.random.default_rng(arguments.seed)
    candidates = [READY]
    attempts = 0
    while len(candidates) < arguments.candidates:
        attempts += 1
        q = np.round(RANGES[:, 0] + (RANGES[:, 1] - RANGES[:, 0]) * random.random(len(ARM)), 3)
        # A clear path from ready lets the tour pass through ready where a direct path is blocked.
        if arm.clear_with_approaches(q) and arm.clear_path(READY, q):
            candidates.append(q)
    print(f"{len(candidates)} clear candidates out of {attempts} samples")

    chosen = greedy_design(arm, candidates, arguments.count)
    rest = [index for index in range(len(candidates)) if index not in chosen]
    validation = [candidates[index] for index in random.choice(rest, arguments.validation, replace=False)]
    calibration = tour(arm, [candidates[index] for index in chosen])
    validation = tour(arm, validation)

    y = np.vstack([arm.regressor(q) for q in calibration])
    singular = np.linalg.svd(y, compute_uv=False)
    rank = int((singular > 1e-6 * singular[0]).sum())
    print(f"regressor rank {rank} of {y.shape[1]}; condition of the identifiable part "
          f"{singular[0] / singular[rank - 1]:.1f}")
    tool = arm.model.getFrameId("gripper_end")
    reach = []
    for q in calibration + validation:
        pin.framesForwardKinematics(arm.model, arm.data, q)
        reach.append(arm.data.oMf[tool].translation.copy())
    reach = np.array(reach)
    print("gripper_end spans x {:.2f}..{:.2f}, y {:.2f}..{:.2f}, z {:.2f}..{:.2f} m".format(
        *np.ravel(np.column_stack([reach.min(axis=0), reach.max(axis=0)]))))
    write(arguments.output, calibration, validation, arguments.seed)
    print(f"wrote {arguments.output}")


if __name__ == "__main__":
    main()
