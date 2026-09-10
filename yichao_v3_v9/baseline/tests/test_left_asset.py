from __future__ import annotations

import hashlib
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from doubles_planner.left_asset import build_left_handed_urdf, build_right_handed_urdf


MINIMAL_URDF = """<?xml version="1.0"?>
<robot name="fixture">
  <link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
  <link name="left_rubber_hand">
    <inertial>
      <origin xyz="0.05 -0.02 0.03" rpy="0.1 -0.2 0.3"/>
      <mass value="0.1"/>
      <inertia ixx="0.11" ixy="0.012" ixz="-0.013" iyy="0.22" iyz="0.014" izz="0.33"/>
    </inertial>
    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="pkg/left_rubber_hand.STL"/></geometry><material name="white"/></visual>
  </link>
  <link name="right_rubber_hand">
    <inertial>
      <origin xyz="0.08 0.04 0.06" rpy="-0.15 0.25 -0.35"/>
      <mass value="0.4"/>
      <inertia ixx="0.41" ixy="-0.021" ixz="-0.031" iyy="0.52" iyz="-0.041" izz="0.63"/>
    </inertial>
    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="pkg/paddle.STL"/></geometry></visual>
    <collision><origin xyz="0.1 0.2 0.3" rpy="0 0 0"/><geometry><mesh filename="pkg/paddle.STL"/></geometry></collision>
  </link>
  <joint name="racket_joint" type="fixed">
    <origin xyz="0.12305 0.023806 0.13196" rpy="-0.058906 0.52058 -0.11806"/>
    <parent link="right_rubber_hand"/><child link="racket"/>
  </joint>
  <link name="racket"/>
  <joint name="other_joint" type="fixed"><parent link="base"/><child link="left_rubber_hand"/></joint>
</robot>
"""

STL_HEADER = b"pingpang-planner binary STL fixture".ljust(80, b"\0")
MIRRORED_STL_HEADER = b"Mirrored across Y for left-hand paddle".ljust(80, b" ")
STL_NORMAL = (0.25, -0.5, 0.75)
STL_VERTICES = (
    (1.0, 2.0, 3.0),
    (-4.0, 5.0, -6.0),
    (7.0, -8.0, 9.0),
)
STL_ATTRIBUTE = 0x1234


def _binary_stl(
    normal: tuple[float, float, float] = STL_NORMAL,
    vertices: tuple[tuple[float, float, float], ...] = STL_VERTICES,
    attribute: int = STL_ATTRIBUTE,
) -> bytes:
    values = (*normal, *vertices[0], *vertices[1], *vertices[2])
    return STL_HEADER + struct.pack("<I12fH", 1, *values, attribute)


def _write_left_fixture(source: Path, paddle_payload: bytes | None = None) -> Path:
    source.parent.mkdir(parents=True, exist_ok=True)
    source.write_text(MINIMAL_URDF, encoding="utf-8")
    paddle = source.parent / "pkg" / "paddle.STL"
    paddle.parent.mkdir(parents=True, exist_ok=True)
    paddle.write_bytes(_binary_stl() if paddle_payload is None else paddle_payload)
    return paddle


def _stl_triangle(payload: bytes):
    assert payload[:80] == MIRRORED_STL_HEADER
    assert struct.unpack_from("<I", payload, 80)[0] == 1
    values = struct.unpack_from("<12f", payload, 84)
    attribute = struct.unpack_from("<H", payload, 132)[0]
    return values[:3], (values[3:6], values[6:9], values[9:12]), attribute


def _named(root: ET.Element, tag: str) -> list[str]:
    return [element.get("name", "") for element in root.findall(tag)]


def _mesh_filename(element: ET.Element) -> str:
    mesh = element.find("./geometry/mesh")
    assert mesh is not None
    filename = mesh.get("filename")
    assert filename is not None
    return filename


def _matrix_from_rpy(rpy: tuple[float, float, float]) -> tuple[tuple[float, ...], ...]:
    roll, pitch, yaw = rpy
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _matmul(left: tuple[tuple[float, ...], ...], right: tuple[tuple[float, ...], ...]):
    return tuple(
        tuple(sum(left[row][index] * right[index][column] for index in range(3)) for column in range(3))
        for row in range(3)
    )


def _xml_signature(element: ET.Element):
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or "").strip(),
        tuple(_xml_signature(child) for child in element),
    )


def test_right_builder_only_adds_racket_visual_and_is_deterministic(tmp_path: Path) -> None:
    source = tmp_path / "main.urdf"
    first = tmp_path / "one" / "main_right.generated.urdf"
    second = tmp_path / "two" / "main_right.generated.urdf"
    source.write_text(MINIMAL_URDF, encoding="utf-8")
    source_before = source.read_bytes()

    result = build_right_handed_urdf(source, first)
    build_right_handed_urdf(source, second)

    assert result == first.resolve()
    assert source.read_bytes() == source_before
    assert first.read_bytes() == second.read_bytes()
    source_root = ET.parse(source).getroot()
    generated_root = ET.parse(first).getroot()
    racket = generated_root.find("./link[@name='racket']")
    assert racket is not None
    visuals = racket.findall("visual")
    assert len(visuals) == 1
    assert visuals[0].find("./geometry/sphere").get("radius") == "0.01"
    assert visuals[0].find("material/color").get("rgba") == "1 0 0 1"
    racket.remove(visuals[0])
    assert _xml_signature(generated_root) == _xml_signature(source_root)


def test_build_moves_paddle_restores_right_mesh_and_preserves_source(tmp_path: Path) -> None:
    source = tmp_path / "main.urdf"
    output = tmp_path / "generated" / "main_left.urdf"
    paddle = _write_left_fixture(source)
    source_before = source.read_bytes()
    paddle_before = paddle.read_bytes()

    result = build_left_handed_urdf(source, output)

    assert result == output.resolve()
    assert source.read_bytes() == source_before
    assert paddle.read_bytes() == paddle_before
    root = ET.parse(output).getroot()
    assert _named(root, "link") == ["base", "left_rubber_hand", "right_rubber_hand", "racket"]
    assert _named(root, "joint") == ["racket_joint", "other_joint"]

    left = root.find("./link[@name='left_rubber_hand']")
    right = root.find("./link[@name='right_rubber_hand']")
    assert left is not None and right is not None
    assert _mesh_filename(left.find("visual")) == "pkg/paddle_left_mirror_generated.STL"
    assert _mesh_filename(left.find("collision")) == "pkg/paddle_left_mirror_generated.STL"
    assert _mesh_filename(right.find("visual")) == "pkg/right_rubber_hand.STL"
    assert right.find("visual/material") is not None
    assert right.find("collision") is None

    mirrored_paddle = paddle.with_name("paddle_left_mirror_generated.STL")
    assert "." not in mirrored_paddle.stem
    normal, vertices, attribute = _stl_triangle(mirrored_paddle.read_bytes())
    assert normal == pytest.approx((0.25, 0.5, 0.75))
    expected_vertices = (
        (1.0, -2.0, 3.0),
        (7.0, 8.0, 9.0),
        (-4.0, -5.0, -6.0),
    )
    for vertex, expected in zip(vertices, expected_vertices):
        assert vertex == pytest.approx(expected)
    assert attribute == STL_ATTRIBUTE

    racket = root.find("./link[@name='racket']")
    assert racket is not None
    racket_visuals = racket.findall("visual")
    assert len(racket_visuals) == 1
    assert racket_visuals[0].find("./geometry/sphere").get("radius") == "0.01"
    assert racket_visuals[0].find("material/color").get("rgba") == "1 0 0 1"

    racket_joint = root.find("./joint[@name='racket_joint']")
    assert racket_joint is not None
    assert racket_joint.find("parent").get("link") == "left_rubber_hand"
    assert racket_joint.find("origin").get("xyz") == "0.12305 -0.023806 0.13196"


def test_racket_rotation_is_reflection_then_local_z_pi(tmp_path: Path) -> None:
    source = tmp_path / "main.urdf"
    output = tmp_path / "main_left.urdf"
    _write_left_fixture(source)
    build_left_handed_urdf(source, output)
    root = ET.parse(output).getroot()
    origin = root.find("./joint[@name='racket_joint']/origin")
    assert origin is not None
    actual_rpy = tuple(float(item) for item in origin.get("rpy").split())

    source_rpy = (-0.058906, 0.52058, -0.11806)
    source_rotation = _matrix_from_rpy(source_rpy)
    reflection = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0))
    local_rz_pi = ((-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0))
    expected = _matmul(_matmul(_matmul(reflection, source_rotation), reflection), local_rz_pi)
    actual = _matrix_from_rpy(actual_rpy)
    for expected_row, actual_row in zip(expected, actual):
        assert actual_row == pytest.approx(expected_row, abs=1.0e-12)


def test_hand_inertials_follow_geometry_and_are_reflected_across_xz(tmp_path: Path) -> None:
    source = tmp_path / "main.urdf"
    output = tmp_path / "main_left.urdf"
    _write_left_fixture(source)
    build_left_handed_urdf(source, output)
    root = ET.parse(output).getroot()

    left = root.find("./link[@name='left_rubber_hand']/inertial")
    right = root.find("./link[@name='right_rubber_hand']/inertial")
    assert left is not None and right is not None
    assert float(left.find("mass").get("value")) == pytest.approx(0.4)
    assert float(right.find("mass").get("value")) == pytest.approx(0.1)
    assert tuple(float(value) for value in left.find("origin").get("xyz").split()) == pytest.approx((0.08, -0.04, 0.06))
    assert tuple(float(value) for value in right.find("origin").get("xyz").split()) == pytest.approx((0.05, 0.02, 0.03))

    left_tensor = left.find("inertia")
    right_tensor = right.find("inertia")
    assert {name: float(left_tensor.get(name)) for name in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")} == pytest.approx(
        {"ixx": 0.41, "ixy": 0.021, "ixz": -0.031, "iyy": 0.52, "iyz": 0.041, "izz": 0.63}
    )
    assert {name: float(right_tensor.get(name)) for name in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")} == pytest.approx(
        {"ixx": 0.11, "ixy": -0.012, "ixz": -0.013, "iyy": 0.22, "iyz": -0.014, "izz": 0.33}
    )

    reflection = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0))
    for output_inertial, source_rpy in ((left, (-0.15, 0.25, -0.35)), (right, (0.1, -0.2, 0.3))):
        actual_rpy = tuple(float(value) for value in output_inertial.find("origin").get("rpy").split())
        expected = _matmul(_matmul(reflection, _matrix_from_rpy(source_rpy)), reflection)
        actual = _matrix_from_rpy(actual_rpy)
        for expected_row, actual_row in zip(expected, actual):
            assert actual_row == pytest.approx(expected_row, abs=1.0e-12)


def test_output_is_byte_for_byte_deterministic(tmp_path: Path) -> None:
    source = tmp_path / "main.urdf"
    first = tmp_path / "one" / "main_left.urdf"
    second = tmp_path / "two" / "main_left.urdf"
    paddle = _write_left_fixture(source)
    build_left_handed_urdf(source, first)
    mirrored = paddle.with_name("paddle_left_mirror_generated.STL")
    first_mesh = mirrored.read_bytes()
    build_left_handed_urdf(source, second)
    assert first.read_bytes() == second.read_bytes()
    assert hashlib.sha256(first.read_bytes()).digest() == hashlib.sha256(second.read_bytes()).digest()
    assert mirrored.read_bytes() == first_mesh
    assert hashlib.sha256(mirrored.read_bytes()).digest() == hashlib.sha256(first_mesh).digest()


def test_package_uri_generates_mesh_beside_canonical_paddle(tmp_path: Path) -> None:
    package = tmp_path / "unitree_description"
    source = package / "urdf" / "g1" / "main.urdf"
    output = source.with_name("main_left.generated.urdf")
    source.parent.mkdir(parents=True)
    source.write_text(
        MINIMAL_URDF.replace(
            "pkg/paddle.STL",
            "package://unitree_description/meshes/g1/paddle.STL",
        ),
        encoding="utf-8",
    )
    paddle = package / "meshes" / "g1" / "paddle.STL"
    paddle.parent.mkdir(parents=True)
    paddle.write_bytes(_binary_stl())

    build_left_handed_urdf(source, output)

    mirrored = paddle.with_name("paddle_left_mirror_generated.STL")
    assert mirrored.is_file()
    root = ET.parse(output).getroot()
    expected = "package://unitree_description/meshes/g1/paddle_left_mirror_generated.STL"
    assert _mesh_filename(root.find("./link[@name='left_rubber_hand']/visual")) == expected
    assert _mesh_filename(root.find("./link[@name='left_rubber_hand']/collision")) == expected


@pytest.mark.parametrize(
    "payload",
    [
        b"solid paddle\nendsolid paddle\n",
        STL_HEADER + struct.pack("<I", 2) + _binary_stl()[84:],
        _binary_stl(normal=(math.nan, 0.0, 1.0)),
    ],
    ids=("ascii", "count-mismatch", "non-finite"),
)
def test_invalid_binary_stl_does_not_write_or_clobber_outputs(tmp_path: Path, payload: bytes) -> None:
    source = tmp_path / "main.urdf"
    output = tmp_path / "main_left.generated.urdf"
    paddle = _write_left_fixture(source, payload)
    mirrored = paddle.with_name("paddle_left_mirror_generated.STL")
    previous_generated = b"existing generated mesh"
    mirrored.write_bytes(previous_generated)

    with pytest.raises(ValueError, match="binary STL"):
        build_left_handed_urdf(source, output)

    assert paddle.read_bytes() == payload
    assert mirrored.read_bytes() == previous_generated
    assert not output.exists()


def test_source_and_output_cannot_be_the_same_file(tmp_path: Path) -> None:
    source = tmp_path / "main.urdf"
    source.write_text(MINIMAL_URDF, encoding="utf-8")
    with pytest.raises(ValueError, match="differ"):
        build_left_handed_urdf(source, source)
