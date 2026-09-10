"""Build Isaac-compatible right- and left-handed G1 ping-pong URDFs.

The controller keeps the robot policy and the robot asset separate.  This
module therefore only transforms the XML asset; it never edits the canonical
URDF or any of its referenced meshes.
"""

from __future__ import annotations

import copy
import math
import os
import struct
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable, Sequence


_RIGHT_HAND = "right_rubber_hand"
_LEFT_HAND = "left_rubber_hand"
_RACKET_JOINT = "racket_joint"
_PADDLE_TOKEN = "paddle"
_BINARY_STL_HEADER_BYTES = 80
_BINARY_STL_PREFIX_BYTES = 84
_BINARY_STL_TRIANGLE_BYTES = 50
_MIRRORED_PADDLE_SUFFIX = "_left_mirror_generated"
_MIRRORED_STL_HEADER = b"Mirrored across Y for left-hand paddle".ljust(_BINARY_STL_HEADER_BYTES, b" ")


def build_right_handed_urdf(
    source_urdf: str | os.PathLike[str],
    output_path: str | os.PathLike[str],
) -> Path:
    """Copy the canonical right-handed URDF and activate its racket visual.

    Physics, meshes, joints, and body names are preserved.  The only semantic
    addition is the tiny visual required for Isaac's ``visuals/racket`` prim.
    """

    tree, root, _, destination = _load_urdf(source_urdf, output_path)
    _ensure_racket_visual(root)
    return _write_urdf(tree, root, destination)


def build_left_handed_urdf(
    source_urdf: str | os.PathLike[str],
    output_path: str | os.PathLike[str],
) -> Path:
    """Build a deterministic left-handed G1 URDF.

    ``source_urdf`` is normally the controller's canonical ``g1/main.urdf``.
    ``output_path`` names the generated file (its parent is created when
    needed).  The returned path is the resolved output path.

    The right-hand paddle visual and collision are moved to the left hand and
    reference a generated, reflected binary STL beside the canonical paddle.
    A regular right-hand visual is restored from the left hand's regular mesh;
    the unused hand intentionally has no collision.  The fixed racket joint is
    then attached to the left hand and mirrored across the XZ plane, with an
    additional local Z rotation of pi.  All link and joint names remain
    untouched.
    """

    tree, root, source_path, destination = _load_urdf(source_urdf, output_path)

    right_hand = _find_unique(root, "link", _RIGHT_HAND)
    left_hand = _find_unique(root, "link", _LEFT_HAND)
    racket_joint = _find_unique(root, "joint", _RACKET_JOINT)

    right_inertial = _find_link_element(right_hand, "inertial", _RIGHT_HAND)
    left_inertial = _find_link_element(left_hand, "inertial", _LEFT_HAND)
    right_visual = _find_render_element(right_hand, "visual", _RIGHT_HAND)
    right_collision = _find_render_element(right_hand, "collision", _RIGHT_HAND)
    left_visual = _find_render_element(left_hand, "visual", _LEFT_HAND)

    paddle_visual = copy.deepcopy(right_visual)
    paddle_collision = copy.deepcopy(right_collision)
    paddle_visual_mesh = _require_paddle_mesh(paddle_visual, "visual")
    paddle_collision_mesh = _require_paddle_mesh(paddle_collision, "collision")
    visual_filename = paddle_visual_mesh.get("filename", "")
    collision_filename = paddle_collision_mesh.get("filename", "")
    paddle_source = _resolve_mesh_path(source_path, visual_filename)
    collision_source = _resolve_mesh_path(source_path, collision_filename)
    if collision_source != paddle_source:
        raise ValueError("paddle visual and collision must reference the same source mesh")
    mirrored_paddle = paddle_source.with_name(
        f"{paddle_source.stem}{_MIRRORED_PADDLE_SUFFIX}{paddle_source.suffix}"
    )
    if destination in {paddle_source, mirrored_paddle}:
        raise ValueError("output_path must differ from paddle source and generated mesh paths")
    mirrored_payload = _reflected_binary_stl(paddle_source)
    paddle_visual_mesh.set("filename", _replace_mesh_basename(visual_filename, mirrored_paddle.name))
    paddle_collision_mesh.set("filename", _replace_mesh_basename(collision_filename, mirrored_paddle.name))

    right_regular_visual = copy.deepcopy(left_visual)
    _replace_hand_mesh(right_regular_visual, "left", "right")

    _replace_link_element(right_hand, "inertial", _mirrored_inertial(left_inertial))
    _replace_link_element(left_hand, "inertial", _mirrored_inertial(right_inertial))
    _replace_render_elements(
        right_hand,
        visual=right_regular_visual,
    )
    _replace_render_elements(
        left_hand,
        visual=paddle_visual,
        collision=paddle_collision,
    )

    parent = racket_joint.find("parent")
    if parent is None:
        raise ValueError(f"joint {_RACKET_JOINT!r} has no parent element")
    parent.set("link", _LEFT_HAND)
    origin = racket_joint.find("origin")
    if origin is None:
        raise ValueError(f"joint {_RACKET_JOINT!r} has no origin element")
    xyz = _parse_vector(origin.get("xyz"), "racket_joint xyz", 3)
    rpy = _parse_vector(origin.get("rpy"), "racket_joint rpy", 3)
    origin.set("xyz", _format_vector((xyz[0], -xyz[1], xyz[2])))
    origin.set("rpy", _format_vector(_mirrored_rpy(rpy)))
    _ensure_racket_visual(root)

    _write_bytes_if_changed(mirrored_paddle, mirrored_payload)
    return _write_urdf(tree, root, destination)


def build_left_hand_urdf(
    source_urdf: str | os.PathLike[str],
    output_path: str | os.PathLike[str],
) -> Path:
    """Backward-compatible concise alias for :func:`build_left_handed_urdf`."""

    return build_left_handed_urdf(source_urdf, output_path)


def _load_urdf(
    source_urdf: str | os.PathLike[str],
    output_path: str | os.PathLike[str],
) -> tuple[ET.ElementTree, ET.Element, Path, Path]:
    source_path = Path(source_urdf).expanduser().resolve()
    destination = Path(output_path).expanduser().resolve()
    if source_path == destination:
        raise ValueError("output_path must differ from source_urdf")
    if not source_path.is_file():
        raise FileNotFoundError(source_path)
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(source_path, parser=parser)
    root = tree.getroot()
    if root.tag != "robot":
        raise ValueError(f"expected a URDF robot root, got {root.tag!r}")
    return tree, root, source_path, destination


def _write_urdf(tree: ET.ElementTree, root: ET.Element, destination: Path) -> Path:
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="    ")
    payload = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    destination.write_bytes(payload + b"\n")
    return destination


def _find_unique(root: ET.Element, tag: str, name: str) -> ET.Element:
    matches = [element for element in root if element.tag == tag and element.get("name") == name]
    if len(matches) != 1:
        raise ValueError(f"expected exactly one {tag} named {name!r}, found {len(matches)}")
    return matches[0]


def _find_render_element(link: ET.Element, tag: str, link_name: str) -> ET.Element:
    matches = [child for child in link if child.tag == tag]
    if len(matches) != 1:
        raise ValueError(f"link {link_name!r} must contain exactly one {tag}")
    return matches[0]


def _find_link_element(link: ET.Element, tag: str, link_name: str) -> ET.Element:
    matches = [child for child in link if child.tag == tag]
    if len(matches) != 1:
        raise ValueError(f"link {link_name!r} must contain exactly one {tag}")
    return matches[0]


def _mesh_elements(element: ET.Element) -> list[ET.Element]:
    return [mesh for mesh in element.iter("mesh")]


def _require_paddle_mesh(element: ET.Element, element_name: str) -> ET.Element:
    meshes = _mesh_elements(element)
    if len(meshes) != 1:
        raise ValueError(f"paddle {element_name} must contain exactly one mesh")
    filename = meshes[0].get("filename", "")
    if _PADDLE_TOKEN not in filename.lower():
        raise ValueError(f"expected paddle mesh in {element_name}, got {filename!r}")
    return meshes[0]


def _resolve_mesh_path(source_urdf: Path, filename: str) -> Path:
    if filename.startswith("package://"):
        package_reference = filename[len("package://") :]
        package_name, separator, relative_path = package_reference.partition("/")
        if not separator or not package_name or not relative_path:
            raise ValueError(f"invalid package mesh URI: {filename!r}")
        package_root = next((parent for parent in source_urdf.parents if parent.name == package_name), None)
        if package_root is None:
            raise ValueError(
                f"cannot resolve package {package_name!r} from source URDF {str(source_urdf)!r}"
            )
        mesh_path = package_root / relative_path
    elif "://" in filename:
        raise ValueError(f"unsupported mesh URI: {filename!r}")
    else:
        mesh_path = Path(filename).expanduser()
        if not mesh_path.is_absolute():
            mesh_path = source_urdf.parent / mesh_path
    resolved = mesh_path.resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"paddle mesh not found: {resolved}")
    return resolved


def _replace_mesh_basename(filename: str, basename: str) -> str:
    separator_index = max(filename.rfind("/"), filename.rfind("\\"))
    return f"{filename[: separator_index + 1]}{basename}"


def _reflected_binary_stl(source: Path) -> bytes:
    payload = source.read_bytes()
    if len(payload) < _BINARY_STL_PREFIX_BYTES:
        raise ValueError(f"invalid binary STL {source}: file is shorter than 84 bytes")
    triangle_count = struct.unpack_from("<I", payload, _BINARY_STL_HEADER_BYTES)[0]
    if triangle_count == 0:
        raise ValueError(f"invalid binary STL {source}: mesh contains no triangles")
    expected_size = _BINARY_STL_PREFIX_BYTES + triangle_count * _BINARY_STL_TRIANGLE_BYTES
    if len(payload) != expected_size:
        raise ValueError(
            f"invalid binary STL {source}: triangle count requires {expected_size} bytes, got {len(payload)}"
        )

    reflected = bytearray(payload)
    reflected[:_BINARY_STL_HEADER_BYTES] = _MIRRORED_STL_HEADER
    for triangle_index in range(triangle_count):
        offset = _BINARY_STL_PREFIX_BYTES + triangle_index * _BINARY_STL_TRIANGLE_BYTES
        values = struct.unpack_from("<12f", payload, offset)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"invalid binary STL {source}: triangle {triangle_index} contains non-finite values")
        struct.pack_into(
            "<12f",
            reflected,
            offset,
            values[0],
            -values[1],
            values[2],
            values[3],
            -values[4],
            values[5],
            values[9],
            -values[10],
            values[11],
            values[6],
            -values[7],
            values[8],
        )
    return bytes(reflected)


def _write_bytes_if_changed(destination: Path, payload: bytes) -> None:
    if destination.is_file() and destination.read_bytes() == payload:
        return
    destination.write_bytes(payload)


def _replace_hand_mesh(element: ET.Element, source_side: str, destination_side: str) -> None:
    meshes = _mesh_elements(element)
    if len(meshes) != 1:
        raise ValueError("regular hand render/collision must contain exactly one mesh")
    mesh = meshes[0]
    filename = mesh.get("filename")
    if not filename:
        raise ValueError("regular hand mesh has no filename")
    source_name = f"{source_side}_rubber_hand"
    destination_name = f"{destination_side}_rubber_hand"
    if source_name not in filename:
        raise ValueError(f"expected {source_name!r} mesh, got {filename!r}")
    mesh.set("filename", filename.replace(source_name, destination_name))


def _replace_render_elements(
    link: ET.Element,
    *,
    visual: ET.Element,
    collision: ET.Element | None = None,
) -> None:
    old_render = [child for child in link if child.tag in {"visual", "collision"}]
    insertion_index = min((list(link).index(child) for child in old_render), default=len(link))
    for child in old_render:
        link.remove(child)
    link.insert(insertion_index, visual)
    if collision is not None:
        link.insert(insertion_index + 1, collision)


def _replace_link_element(link: ET.Element, tag: str, replacement: ET.Element) -> None:
    current = [child for child in link if child.tag == tag]
    if len(current) != 1:
        raise ValueError(f"link {link.get('name')!r} must contain exactly one {tag}")
    index = list(link).index(current[0])
    link.remove(current[0])
    link.insert(index, replacement)


def _mirrored_inertial(source: ET.Element) -> ET.Element:
    inertial = copy.deepcopy(source)
    origin = inertial.find("origin")
    if origin is None:
        origin = ET.Element("origin")
        inertial.insert(0, origin)
    xyz = _parse_vector(origin.get("xyz", "0 0 0"), "inertial origin xyz", 3)
    rpy = _parse_vector(origin.get("rpy", "0 0 0"), "inertial origin rpy", 3)
    origin.set("xyz", _format_vector((xyz[0], -xyz[1], xyz[2])))
    origin.set("rpy", _format_vector(_reflected_rpy(rpy)))

    inertia = inertial.find("inertia")
    if inertia is None:
        raise ValueError("inertial element has no inertia tensor")
    for attribute in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
        value = _parse_scalar(inertia.get(attribute), f"inertia {attribute}")
        if attribute in {"ixy", "iyz"}:
            value = -value
        inertia.set(attribute, _format_number(value))
    return inertial


def _ensure_racket_visual(root: ET.Element) -> None:
    """Give Isaac's URDF importer a concrete prim for the racket link.

    The controller's canonical asset leaves this link visual-less (its tiny
    sphere is commented out).  Isaac's generated physics layer still creates
    a reference to ``visuals/racket``; an active, tiny visual keeps that
    reference resolvable without changing the racket's collision geometry.
    """

    racket = _find_unique(root, "link", "racket")
    if any(child.tag == "visual" for child in racket):
        return
    visual = ET.Element("visual")
    ET.SubElement(visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "sphere", {"radius": "0.01"})
    material = ET.SubElement(visual, "material", {"name": "racket_marker"})
    ET.SubElement(material, "color", {"rgba": "1 0 0 1"})
    inertial = next((index for index, child in enumerate(racket) if child.tag == "inertial"), None)
    racket.insert((inertial + 1) if inertial is not None else 0, visual)


def _parse_vector(value: str | None, label: str, length: int) -> tuple[float, ...]:
    if value is None:
        raise ValueError(f"{label} is missing")
    try:
        values = tuple(float(item) for item in value.split())
    except ValueError as exc:
        raise ValueError(f"{label} must contain numbers, got {value!r}") from exc
    if len(values) != length or not all(math.isfinite(item) for item in values):
        raise ValueError(f"{label} must contain {length} finite numbers, got {value!r}")
    return values


def _parse_scalar(value: str | None, label: str) -> float:
    if value is None:
        raise ValueError(f"{label} is missing")
    try:
        result = float(value)
    except ValueError as exc:
        raise ValueError(f"{label} must be a number, got {value!r}") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite, got {value!r}")
    return result


def _format_number(value: float) -> str:
    if abs(value) < 1.0e-15:
        value = 0.0
    return format(value, ".15g")


def _format_vector(values: Iterable[float]) -> str:
    return " ".join(_format_number(float(value)) for value in values)


def _matmul(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(sum(left[row][index] * right[index][column] for index in range(3)) for column in range(3))
        for row in range(3)
    )


def _rotation_matrix(rpy: Sequence[float]) -> tuple[tuple[float, ...], ...]:
    roll, pitch, yaw = rpy
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    rx = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
    ry = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    rz = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    return _matmul(_matmul(rz, ry), rx)


def _mirrored_rpy(rpy: Sequence[float]) -> tuple[float, float, float]:
    reflection = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0))
    local_rz_pi = ((-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0))
    reflected = _matmul(_matmul(_matmul(reflection, _rotation_matrix(rpy)), reflection), local_rz_pi)
    return _rpy_from_matrix(reflected)


def _reflected_rpy(rpy: Sequence[float]) -> tuple[float, float, float]:
    reflection = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0))
    reflected = _matmul(_matmul(reflection, _rotation_matrix(rpy)), reflection)
    return _rpy_from_matrix(reflected)


def _rpy_from_matrix(reflected: Sequence[Sequence[float]]) -> tuple[float, float, float]:
    pitch = math.atan2(-reflected[2][0], math.hypot(reflected[0][0], reflected[1][0]))
    cosine_pitch = math.hypot(reflected[0][0], reflected[1][0])
    if cosine_pitch > 1.0e-12:
        roll = math.atan2(reflected[2][1], reflected[2][2])
        yaw = math.atan2(reflected[1][0], reflected[0][0])
    elif reflected[2][0] < 0.0:
        roll = math.atan2(reflected[0][1], reflected[0][2])
        yaw = 0.0
    else:
        roll = math.atan2(-reflected[0][1], -reflected[0][2])
        yaw = 0.0
    return roll, pitch, yaw
