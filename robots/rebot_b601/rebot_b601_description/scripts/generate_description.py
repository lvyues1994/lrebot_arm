#!/usr/bin/env python3
"""Generate the reBot B601-RS description used by larm.

The upstream URDF and meshes are fetched at a pinned commit (they are not
vendored), then compiled to MJCF with MuJoCo's `compile` tool and extended with
the simulation details larm relies on: torque actuators named after the control
joints, finger coupling, joint friction/armature, and collision groups.

Usage:
    generate_description.py [--upstream DIR] [--mujoco-compile PATH] [--output DIR]

Without --upstream the pinned commit is cloned into <repo>/.deps.
Output layout (ignored by git):
    generated/urdf/rebot_b601_rs.urdf
    generated/meshes/*.STL
    generated/mjcf/rebot_b601_rs.xml     robot only
    generated/mjcf/scene.xml             robot on a floor
"""

import argparse
import hashlib
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

UPSTREAM_URL = "https://github.com/Seeed-Projects/reBotArm_control_py.git"
UPSTREAM_COMMIT = "5ba28acef46237eb6a7560658bbc43b06cf8a259"
UPSTREAM_URDF = "urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf"
UPSTREAM_URDF_SHA256 = "2012b5aa3b58878109cb9e3c5deef919a87bd09a67d561662b2904a30dd4397e"

ROBOT_NAME = "rebot_b601_rs"

# Armature and damping are estimates. Friction loss for joint2-joint5 comes from
# the RS gravity calibration (2026-07-17); joint1, joint6 and the fingers are estimates.
JOINT_PARAMS = {
    "joint1": {"armature": "0.01", "damping": "0.05", "frictionloss": "0.3"},
    "joint2": {"armature": "0.01", "damping": "0.05", "frictionloss": "0.53"},
    "joint3": {"armature": "0.01", "damping": "0.05", "frictionloss": "0.49"},
    "joint4": {"armature": "0.005", "damping": "0.02", "frictionloss": "0.3"},
    "joint5": {"armature": "0.005", "damping": "0.02", "frictionloss": "0.21"},
    "joint6": {"armature": "0.005", "damping": "0.02", "frictionloss": "0.2"},
    "joint_left": {"armature": "0.05", "damping": "2", "frictionloss": "1"},
    "joint_right": {"armature": "0.05", "damping": "2", "frictionloss": "1"},
}

# Control joint -> (MJCF joint, torque or force limit). Actuator names equal control joint names.
ACTUATORS = [
    ("joint1", "joint1", 36.0),
    ("joint2", "joint2", 36.0),
    ("joint3", "joint3", 36.0),
    ("joint4", "joint4", 14.0),
    ("joint5", "joint5", 14.0),
    ("joint6", "joint6", 14.0),
    ("gripper", "joint_left", 20.0),
]

VISUAL_COLORS = [
    ("green", "0.18 0.55 0.32 1"),
    ("black", "0.08 0.08 0.08 1"),
    ("motor", "0.15 0.15 0.17 1"),
    ("cnc", "0.72 0.72 0.75 1"),
]
DEFAULT_VISUAL_COLOR = "0.82 0.82 0.84 1"

# Robot geoms touch the world but not each other; self-collision is checked at planning time.
ROBOT_COLLISION = {"contype": "2", "conaffinity": "1"}
WORLD_COLLISION = {"contype": "1", "conaffinity": "2"}


def fail(message: str) -> None:
    print(f"error: {message}", file=sys.stderr)
    sys.exit(1)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clone_upstream(destination: Path) -> Path:
    if (destination / UPSTREAM_URDF).exists():
        return destination
    destination.mkdir(parents=True, exist_ok=True)
    git = ["git", "-C", str(destination)]
    subprocess.run(git + ["init", "-q"], check=True)
    subprocess.run(git + ["fetch", "-q", "--depth", "1", UPSTREAM_URL, UPSTREAM_COMMIT], check=True)
    subprocess.run(git + ["checkout", "-q", "FETCH_HEAD"], check=True)
    return destination


def write_urdf(upstream_urdf: Path, output: Path) -> list[str]:
    tree = ET.parse(upstream_urdf)
    robot = tree.getroot()
    robot.set("name", ROBOT_NAME)
    meshes = []
    for mesh in robot.iter("mesh"):
        name = Path(mesh.get("filename", "")).name
        mesh.set("filename", f"../meshes/{name}")
        meshes.append(name)
    (output / "urdf").mkdir(parents=True, exist_ok=True)
    tree.write(output / "urdf" / f"{ROBOT_NAME}.urdf", encoding="utf-8", xml_declaration=True)
    return sorted(set(meshes))


def copy_meshes(upstream_urdf: Path, names: list[str], output: Path) -> None:
    source = upstream_urdf.parent.parent / "meshes"
    target = output / "meshes"
    target.mkdir(parents=True, exist_ok=True)
    for name in names:
        shutil.copy2(source / name, target / name)


def compile_raw_mjcf(output: Path, mujoco_compile: Path) -> ET.ElementTree:
    tree = ET.parse(output / "urdf" / f"{ROBOT_NAME}.urdf")
    mujoco = ET.SubElement(tree.getroot(), "mujoco")
    ET.SubElement(mujoco, "compiler", {
        "meshdir": "../meshes/",
        "balanceinertia": "true",
        "discardvisual": "false",
        "fusestatic": "false",
        "strippath": "true",
    })
    staging = output / "urdf" / ".mujoco_input.urdf"
    raw = output / "mjcf" / ".compiled.xml"
    raw.parent.mkdir(parents=True, exist_ok=True)
    tree.write(staging, encoding="utf-8", xml_declaration=True)
    subprocess.run([str(mujoco_compile), str(staging), str(raw)], check=True, stdout=subprocess.DEVNULL)
    staging.unlink()
    result = ET.parse(raw)
    raw.unlink()
    return result


def visual_color(mesh: str) -> str:
    for key, rgba in VISUAL_COLORS:
        if key in mesh:
            return rgba
    return DEFAULT_VISUAL_COLOR


def extend_mjcf(tree: ET.ElementTree) -> None:
    root = tree.getroot()
    root.set("model", ROBOT_NAME)
    root.find("compiler").set("autolimits", "true")

    option = ET.Element("option", {"integrator": "implicitfast"})
    root.insert(1, option)

    default = ET.Element("default")
    visual = ET.SubElement(default, "default", {"class": "visual"})
    ET.SubElement(visual, "geom", {"contype": "0", "conaffinity": "0", "group": "2", "density": "0"})
    collision = ET.SubElement(default, "default", {"class": "collision"})
    ET.SubElement(collision, "geom", {"group": "3", "rgba": "0.9 0.4 0.2 0.4", **ROBOT_COLLISION})
    root.insert(2, default)

    for geom in root.find("worldbody").iter("geom"):
        is_visual = geom.get("contype") == "0"
        for attribute in ("contype", "conaffinity", "group", "density", "rgba"):
            geom.attrib.pop(attribute, None)
        geom.set("class", "visual" if is_visual else "collision")
        if is_visual:
            geom.set("rgba", visual_color(geom.get("mesh", "")))

    joints = {joint.get("name"): joint for joint in root.iter("joint")}
    for name, params in JOINT_PARAMS.items():
        if name not in joints:
            fail(f"joint {name} missing from the compiled MJCF")
        joints[name].attrib.update(params)

    equality = ET.SubElement(root, "equality")
    ET.SubElement(equality, "joint", {
        "name": "finger_coupling", "joint1": "joint_right", "joint2": "joint_left", "polycoef": "0 1 0 0 0",
    })

    actuator = ET.SubElement(root, "actuator")
    for control_joint, mjcf_joint, limit in ACTUATORS:
        ET.SubElement(actuator, "motor", {
            "name": control_joint, "joint": mjcf_joint, "ctrlrange": f"{-limit:g} {limit:g}",
        })


SCENE = f"""<mujoco model="{ROBOT_NAME}_scene">
  <include file="{ROBOT_NAME}.xml"/>
  <statistic center="0.25 0 0.2" extent="0.8"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <global azimuth="135" elevation="-25"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
             width="512" height="512" mark="edge" markrgb="0.8 0.8 0.8"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.1"/>
  </asset>
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="2 2 0.05" material="grid"
          contype="{WORLD_COLLISION['contype']}" conaffinity="{WORLD_COLLISION['conaffinity']}"/>
  </worldbody>
</mujoco>
"""


def main() -> None:
    package = Path(__file__).resolve().parent.parent
    repository = package.parents[2]
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--upstream", type=Path, help="existing reBotArm_control_py checkout")
    parser.add_argument("--mujoco-compile", type=Path,
                        default=repository / ".deps" / "mujoco-3.8.0" / "bin" / "compile")
    parser.add_argument("--output", type=Path, default=package / "generated")
    args = parser.parse_args()

    upstream = args.upstream or clone_upstream(repository / ".deps" / "reBotArm_control_py")
    upstream_urdf = upstream / UPSTREAM_URDF
    if not upstream_urdf.exists():
        fail(f"{upstream_urdf} not found")
    if sha256(upstream_urdf) != UPSTREAM_URDF_SHA256:
        fail(f"{upstream_urdf} does not match the pinned upstream commit {UPSTREAM_COMMIT}")
    if not args.mujoco_compile.exists():
        fail(f"MuJoCo compile tool not found at {args.mujoco_compile}; run tools/fetch_mujoco.sh")

    output = args.output.resolve()
    shutil.rmtree(output, ignore_errors=True)
    meshes = write_urdf(upstream_urdf, output)
    copy_meshes(upstream_urdf, meshes, output)
    mjcf = compile_raw_mjcf(output, args.mujoco_compile)
    extend_mjcf(mjcf)
    ET.indent(mjcf)
    mjcf.write(output / "mjcf" / f"{ROBOT_NAME}.xml", encoding="utf-8")
    (output / "mjcf" / "scene.xml").write_text(SCENE, encoding="utf-8")

    check = output / "mjcf" / ".check.xml"
    subprocess.run([str(args.mujoco_compile), str(output / "mjcf" / "scene.xml"), str(check)],
                   check=True, stdout=subprocess.DEVNULL)
    check.unlink()
    (output / "SOURCE").write_text(f"{UPSTREAM_URL} {UPSTREAM_COMMIT}\n", encoding="utf-8")
    print(f"generated {output}")


if __name__ == "__main__":
    main()
