#!/usr/bin/env python3
"""Generate the reBot B601-RS description used by larm.

The upstream URDF and meshes are fetched at a pinned commit (they are not
vendored), then compiled to MJCF with MuJoCo's `compile` tool and extended with
the simulation details larm relies on: torque actuators named after the control
joints, finger coupling, joint friction/armature, and self-collision.

Each collision mesh is a CAD assembly of separate solids (plates, motor cases,
screws). It is replaced by convex parts, one hull per solid, with small solids
merged into a neighbouring part when that barely grows it. MuJoCo collides with
convex hulls anyway, so simulation and the planner's self-collision checks use
the same shapes, and they fit the links far closer than one hull per link.
Link pairs that are never checked (adjacent links, the fingers) are listed in
the SRDF and excluded from contacts in the MJCF.

Usage:
    generate_description.py [--upstream DIR] [--mujoco-compile PATH] [--output DIR]

Needs numpy and scipy (python3-scipy). Without --upstream the pinned commit is
cloned into <repo>/.deps.
Output layout (ignored by git):
    generated/urdf/rebot_b601_rs.urdf    joint_right mimics joint_left
    generated/srdf/rebot_b601_rs.srdf    link pairs without collision checks
    generated/meshes/*.STL               visual meshes
    generated/meshes/*_part<N>.stl       convex collision parts
    generated/mjcf/rebot_b601_rs.xml     robot only
    generated/mjcf/scene.xml             robot on a floor
"""

import argparse
import copy
import hashlib
import shutil
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import ConvexHull, QhullError

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

# Besides adjacent links: the fingers close onto each other, and their hulls overlap when closed.
DISABLED_COLLISIONS = [("gripper_left", "gripper_right", "User")]

# A solid joins a collision part when the part's hull grows by less than this fraction of its volume.
MERGE_GROWTH = 0.05

# Must match larm_model's convexity check (PinocchioCollision.cpp).
CONVEXITY_TOLERANCE = 1e-5
MIN_NORMAL_LENGTH = 1e-12

STL_TRIANGLE = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])


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


def mesh_names(robot: ET.Element, kind: str) -> list[str]:
    return sorted({Path(mesh.get("filename", "")).name for element in robot.iter(kind)
                   for mesh in element.iter("mesh")})


def write_urdf(upstream_urdf: Path, parts: dict[str, list[str]], output: Path) -> None:
    """Writes the URDF with each collision mesh replaced by one collision element per convex part."""
    tree = ET.parse(upstream_urdf)
    robot = tree.getroot()
    robot.set("name", ROBOT_NAME)
    for visual in robot.iter("visual"):
        for mesh in visual.iter("mesh"):
            mesh.set("filename", f"../meshes/{Path(mesh.get('filename', '')).name}")
    for link in robot.iter("link"):
        for collision in link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            if mesh is None:
                continue
            position = list(link).index(collision)
            link.remove(collision)
            for offset, part in enumerate(parts[Path(mesh.get("filename", "")).name]):
                replacement = copy.deepcopy(collision)
                replacement.find("geometry/mesh").set("filename", f"../meshes/{part}")
                link.insert(position + offset, replacement)
    (output / "urdf").mkdir(parents=True, exist_ok=True)
    tree.write(output / "urdf" / f"{ROBOT_NAME}.urdf", encoding="utf-8", xml_declaration=True)


def read_stl(path: Path) -> np.ndarray:
    """Triangles of a binary STL, shape (n, 3, 3)."""
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else -1
    if len(data) != 84 + 50 * count:
        fail(f"{path} is not a binary STL")
    return np.frombuffer(data, dtype=STL_TRIANGLE, count=count, offset=84)["vertices"]


def solids(triangles: np.ndarray) -> list[np.ndarray]:
    """Vertices of each connected piece of a mesh."""
    vertices, corners = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    corners = corners.reshape(-1, 3)
    edges = coo_matrix((np.ones(2 * len(corners)), (corners[:, :2].ravel(), corners[:, 1:].ravel())),
                       shape=(len(vertices), len(vertices)))
    count, labels = connected_components(edges, directed=False)
    return [vertices[labels == piece].astype(np.float64) for piece in range(count)]


def hull_volume(points: np.ndarray) -> float:
    try:
        return ConvexHull(points).volume
    except QhullError:
        return 0.0


def convex_parts(triangles: np.ndarray) -> list[np.ndarray]:
    """Points of convex parts covering a mesh: one per solid, largest first, each smaller solid merged
    into the part whose hull it grows least when that is under MERGE_GROWTH."""
    parts: list[tuple[np.ndarray, float]] = []
    for piece in sorted(solids(triangles), key=hull_volume, reverse=True):
        best = None
        for index, (points, volume) in enumerate(parts):
            merged = np.vstack([points, piece])
            growth = hull_volume(merged) - volume
            if growth < MERGE_GROWTH * volume and (best is None or growth < best[1]):
                best = (index, growth, merged)
        if best is not None:
            index, _, merged = best
            hull = ConvexHull(merged)
            parts[index] = (merged[hull.vertices], hull.volume)
        elif hull_volume(piece) > 0.0:
            hull = ConvexHull(piece)
            parts.append((piece[hull.vertices], hull.volume))
    return [points for points, _ in parts]


def convex_hull(points: np.ndarray) -> tuple[np.ndarray, ConvexHull]:
    """The hull of the points, on vertices rounded to the single precision STL stores.

    Qhull merges nearly coplanar facets by default, and triangulating a merged facet can leave dents
    of a fraction of a millimeter; the vertices are hulled again without merging where Qhull can.
    """
    vertices = points[ConvexHull(points).vertices].astype(np.float32).astype(np.float64)
    try:
        return vertices, ConvexHull(vertices, qhull_options="Q0")
    except QhullError:
        return vertices, ConvexHull(vertices)


def write_hull(points: np.ndarray, target: Path) -> None:
    """Writes the convex hull of the points as a binary STL with outward-facing triangles."""
    vertices, hull = convex_hull(points)
    triangles = vertices[hull.simplices]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    inward = np.einsum("ij,ij->i", normals, hull.equations[:, :3]) < 0
    triangles[inward] = triangles[inward][:, [0, 2, 1]]
    # The check larm_model applies when it loads the mesh: no vertex above a triangle's plane.
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    planar = lengths > MIN_NORMAL_LENGTH
    heights = np.einsum("tj,vtj->vt", normals[planar] / lengths[planar, None],
                        vertices[:, None, :] - triangles[None, planar, 0])
    if heights.max() > CONVEXITY_TOLERANCE:
        fail(f"the hull written to {target.name} is not convex ({heights.max() * 1e3:.3f} mm)")
    records = np.zeros(len(triangles), dtype=STL_TRIANGLE)
    records["normal"] = hull.equations[:, :3]
    records["vertices"] = triangles
    target.write_bytes(b"\0" * 80 + struct.pack("<I", len(records)) + records.tobytes())


def write_meshes(upstream_urdf: Path, output: Path) -> dict[str, list[str]]:
    """Copies the visual meshes and writes the convex parts of the collision meshes, by mesh name."""
    robot = ET.parse(upstream_urdf).getroot()
    source = upstream_urdf.parent.parent / "meshes"
    target = output / "meshes"
    target.mkdir(parents=True, exist_ok=True)
    for name in mesh_names(robot, "visual"):
        shutil.copy2(source / name, target / name)
    parts = {}
    for name in mesh_names(robot, "collision"):
        parts[name] = []
        for index, points in enumerate(convex_parts(read_stl(source / name))):
            parts[name].append(f"{Path(name).stem}_part{index}.stl")
            write_hull(points, target / parts[name][-1])
    return parts


def disabled_collisions(output: Path) -> list[tuple[str, str, str]]:
    """Adjacent links, then DISABLED_COLLISIONS."""
    robot = ET.parse(output / "urdf" / f"{ROBOT_NAME}.urdf").getroot()
    links = {link.get("name") for link in robot.iter("link")}
    pairs = [(joint.find("parent").get("link"), joint.find("child").get("link"), "Adjacent")
             for joint in robot.iter("joint")]
    for first, second, _ in DISABLED_COLLISIONS:
        if first not in links or second not in links:
            fail(f"disabled collision pair {first}/{second} names a link missing from the URDF")
    return pairs + DISABLED_COLLISIONS


def write_srdf(pairs: list[tuple[str, str, str]], output: Path) -> None:
    robot = ET.Element("robot", {"name": ROBOT_NAME})
    for first, second, reason in pairs:
        ET.SubElement(robot, "disable_collisions", {"link1": first, "link2": second, "reason": reason})
    tree = ET.ElementTree(robot)
    ET.indent(tree)
    (output / "srdf").mkdir(parents=True, exist_ok=True)
    tree.write(output / "srdf" / f"{ROBOT_NAME}.srdf", encoding="utf-8", xml_declaration=True)


def add_finger_mimic(output: Path) -> None:
    """Declares the right finger as following the left one, for robot_state_publisher.

    Added after the MJCF is compiled: MuJoCo would otherwise add a second coupling constraint.
    """
    path = output / "urdf" / f"{ROBOT_NAME}.urdf"
    tree = ET.parse(path)
    for joint in tree.getroot().iter("joint"):
        if joint.get("name") == "joint_right":
            ET.SubElement(joint, "mimic", {"joint": "joint_left", "multiplier": "1", "offset": "0"})
            tree.write(path, encoding="utf-8", xml_declaration=True)
            return
    fail("joint_right missing from the URDF")


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


def extend_mjcf(tree: ET.ElementTree, disabled: list[tuple[str, str, str]]) -> None:
    root = tree.getroot()
    root.set("model", ROBOT_NAME)
    root.find("compiler").set("autolimits", "true")

    option = ET.Element("option", {"integrator": "implicitfast"})
    root.insert(1, option)

    default = ET.Element("default")
    visual = ET.SubElement(default, "default", {"class": "visual"})
    ET.SubElement(visual, "geom", {"contype": "0", "conaffinity": "0", "group": "2", "density": "0"})
    collision = ET.SubElement(default, "default", {"class": "collision"})
    ET.SubElement(collision, "geom", {"group": "3", "rgba": "0.9 0.4 0.2 0.4"})
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

    # MuJoCo skips parent-child contacts itself, except under a body welded to the world (base_link).
    contact = ET.SubElement(root, "contact")
    for first, second, _ in disabled:
        ET.SubElement(contact, "exclude", {"body1": first, "body2": second})

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
    <geom name="floor" type="plane" size="2 2 0.05" material="grid"/>
  </worldbody>
</mujoco>
"""


def main() -> None:
    package = Path(__file__).resolve().parent.parent
    repository = package.parents[1]
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
    write_urdf(upstream_urdf, write_meshes(upstream_urdf, output), output)
    disabled = disabled_collisions(output)
    write_srdf(disabled, output)
    mjcf = compile_raw_mjcf(output, args.mujoco_compile)
    add_finger_mimic(output)
    extend_mjcf(mjcf, disabled)
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
