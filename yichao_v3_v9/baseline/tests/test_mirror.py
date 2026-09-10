from __future__ import annotations

import unittest

import numpy as np

from doubles_planner.mirror import (
    G1_ACTION_DIM,
    G1_JOINT_NAMES,
    STUDENT_OBSERVATION_DIM,
    LeftHandStudentWrapper,
    joint_mirror_spec,
    mirror_axial_vectors,
    mirror_joint_values,
    mirror_orientation_6d,
    mirror_polar_vectors,
    mirror_policy_action,
    mirror_quaternions_wxyz,
    mirror_student_observation,
)


def _reflected_name(name: str) -> str:
    if name.startswith("left_"):
        return "right_" + name[5:]
    if name.startswith("right_"):
        return "left_" + name[6:]
    return name


def _joint_sign(name: str) -> int:
    return -1 if name.endswith(("_roll_joint", "_yaw_joint")) else 1


def _expected_joint(values: np.ndarray, names: tuple[str, ...]) -> np.ndarray:
    result = np.empty_like(values)
    for output_index, output_name in enumerate(names):
        source_index = names.index(_reflected_name(output_name))
        result[..., output_index] = _joint_sign(output_name) * values[..., source_index]
    return result


class MirrorTransformTests(unittest.TestCase):
    def test_joint_spec_validates_exact_set_and_supports_arbitrary_order(self) -> None:
        names = tuple(reversed(G1_JOINT_NAMES))
        spec = joint_mirror_spec(names)
        self.assertEqual(len(spec.permutation), G1_ACTION_DIM)
        self.assertEqual(len(spec.signs), G1_ACTION_DIM)

        values = np.arange(2 * G1_ACTION_DIM, dtype=float).reshape(2, G1_ACTION_DIM)
        np.testing.assert_array_equal(mirror_joint_values(values, names), _expected_joint(values, names))

        with self.assertRaisesRegex(ValueError, "exact 29"):
            joint_mirror_spec(names[:-1])
        with self.assertRaisesRegex(ValueError, "missing"):
            joint_mirror_spec(names[:-1] + ("not_a_g1_joint",))
        with self.assertRaisesRegex(ValueError, "duplicates"):
            joint_mirror_spec(names[:-1] + (names[0],))

    def test_geometry_transforms_and_round_trips(self) -> None:
        polar = np.asarray([[1.0, 2.0, 3.0], [-4.0, 5.0, -6.0]])
        axial = np.asarray([[1.0, 2.0, 3.0], [-4.0, 5.0, -6.0]])
        quaternion = np.asarray([[1.0, 2.0, 3.0, 4.0], [-1.0, 2.0, -3.0, 4.0]])
        orientation = np.arange(12, dtype=float).reshape(2, 6)

        np.testing.assert_array_equal(mirror_polar_vectors(polar), [[1, -2, 3], [-4, -5, -6]])
        np.testing.assert_array_equal(mirror_axial_vectors(axial), [[-1, 2, -3], [4, 5, 6]])
        np.testing.assert_array_equal(
            mirror_quaternions_wxyz(quaternion), [[1, -2, 3, -4], [-1, -2, -3, -4]]
        )
        np.testing.assert_array_equal(
            mirror_orientation_6d(orientation), orientation * np.asarray([1, -1, -1, 1, 1, -1])
        )

        for transform, values in (
            (mirror_polar_vectors, polar),
            (mirror_axial_vectors, axial),
            (mirror_quaternions_wxyz, quaternion),
            (mirror_orientation_6d, orientation),
        ):
            np.testing.assert_array_equal(transform(transform(values)), values)

    def test_1666_observation_uses_term_major_history_and_exact_geometry(self) -> None:
        rng = np.random.default_rng(20260815)
        observation = rng.normal(size=(2, STUDENT_OBSERVATION_DIM))
        mirrored = mirror_student_observation(observation, G1_JOINT_NAMES)

        np.testing.assert_array_equal(mirrored[..., 0:10], observation[..., 0:10])
        np.testing.assert_array_equal(mirrored[..., 1660:1666], observation[..., 1660:1666])

        expected = observation.copy()
        expected[..., 10:40] = (observation[..., 10:40].reshape(2, 10, 3) * [1, -1, 1]).reshape(2, 30)
        expected[..., 40:70] = (observation[..., 40:70].reshape(2, 10, 3) * [1, -1, 1]).reshape(2, 30)
        expected[..., 70:90] = (observation[..., 70:90].reshape(2, 10, 2) * [1, -1]).reshape(2, 20)
        expected[..., 90:120] = (observation[..., 90:120].reshape(2, 10, 3) * [1, -1, 1]).reshape(2, 30)
        expected[..., 120:180] = (observation[..., 120:180].reshape(2, 10, 6) * [1, -1, -1, 1, 1, -1]).reshape(2, 60)
        expected[..., 180:210] = (observation[..., 180:210].reshape(2, 10, 3) * [-1, 1, -1]).reshape(2, 30)
        for start, end in ((210, 500), (500, 790), (790, 1080)):
            expected[..., start:end] = _expected_joint(
                observation[..., start:end].reshape(2, 10, G1_ACTION_DIM), G1_JOINT_NAMES
            ).reshape(2, end - start)
        generated = observation[..., 1080:1660].reshape(2, 10, 58)
        expected[..., 1080:1660] = np.concatenate(
            (
                _expected_joint(generated[..., :29], G1_JOINT_NAMES),
                _expected_joint(generated[..., 29:], G1_JOINT_NAMES),
            ),
            axis=-1,
        ).reshape(2, 580)

        np.testing.assert_allclose(mirrored, expected)
        np.testing.assert_allclose(
            mirror_student_observation(mirrored, G1_JOINT_NAMES), observation
        )

        with self.assertRaisesRegex(ValueError, "1666"):
            mirror_student_observation(np.zeros(1665), G1_JOINT_NAMES)

    def test_normalized_action_mirror_is_absolute_target_round_trip(self) -> None:
        rng = np.random.default_rng(73)
        right_action = rng.normal(size=(3, G1_ACTION_DIM))
        offset = np.linspace(-0.4, 0.6, G1_ACTION_DIM)
        scale = np.linspace(0.3, 1.2, G1_ACTION_DIM)
        left_action = mirror_policy_action(right_action, G1_JOINT_NAMES, offset, scale)

        expected_target = _expected_joint(right_action * scale + offset, G1_JOINT_NAMES)
        np.testing.assert_allclose(left_action, (expected_target - offset) / scale)
        np.testing.assert_allclose(
            mirror_policy_action(left_action, G1_JOINT_NAMES, offset, scale), right_action
        )
        with self.assertRaisesRegex(ValueError, "zero"):
            mirror_policy_action(right_action, G1_JOINT_NAMES, offset, np.zeros_like(scale))

    def test_action_mirror_supports_distinct_source_and_target_normalization(self) -> None:
        rng = np.random.default_rng(23000)
        right_action = rng.normal(size=(4, G1_ACTION_DIM))
        right_offset = np.linspace(-0.55, 0.35, G1_ACTION_DIM)
        right_scale = np.linspace(0.21, 0.83, G1_ACTION_DIM)
        left_offset = np.linspace(0.42, -0.31, G1_ACTION_DIM)
        left_scale = np.linspace(1.17, 0.37, G1_ACTION_DIM)

        left_action = mirror_policy_action(
            right_action,
            G1_JOINT_NAMES,
            right_offset,
            right_scale,
            left_offset,
            left_scale,
        )
        expected_left_target = _expected_joint(
            right_action * right_scale + right_offset,
            G1_JOINT_NAMES,
        )
        np.testing.assert_allclose(
            left_action,
            (expected_left_target - left_offset) / left_scale,
        )
        np.testing.assert_allclose(
            mirror_policy_action(
                left_action,
                G1_JOINT_NAMES,
                left_offset,
                left_scale,
                right_offset,
                right_scale,
            ),
            right_action,
        )

    def test_previous_action_history_converts_between_distinct_normalizations(self) -> None:
        rng = np.random.default_rng(1666)
        observation = np.zeros((2, STUDENT_OBSERVATION_DIM), dtype=float)
        left_previous_action = rng.normal(size=(2, 10, G1_ACTION_DIM))
        observation[..., 790:1080] = left_previous_action.reshape(2, 290)
        left_offset = np.linspace(-0.62, 0.28, G1_ACTION_DIM)
        left_scale = np.linspace(0.25, 0.91, G1_ACTION_DIM)
        right_offset = np.linspace(0.37, -0.48, G1_ACTION_DIM)
        right_scale = np.linspace(1.09, 0.33, G1_ACTION_DIM)

        right_observation = mirror_student_observation(
            observation,
            G1_JOINT_NAMES,
            left_offset,
            left_scale,
            right_offset,
            right_scale,
        )
        expected_right_target = _expected_joint(
            left_previous_action * left_scale + left_offset,
            G1_JOINT_NAMES,
        )
        expected_right_action = (expected_right_target - right_offset) / right_scale
        np.testing.assert_allclose(
            right_observation[..., 790:1080].reshape(2, 10, G1_ACTION_DIM),
            expected_right_action,
        )
        np.testing.assert_allclose(
            mirror_student_observation(
                right_observation,
                G1_JOINT_NAMES,
                right_offset,
                right_scale,
                left_offset,
                left_scale,
            ),
            observation,
        )

    def test_policy_wrapper_mirrors_observation_and_action(self) -> None:
        observation = np.arange(STUDENT_OBSERVATION_DIM, dtype=float)
        offset = np.linspace(-0.2, 0.2, G1_ACTION_DIM)
        scale = np.ones(G1_ACTION_DIM)
        seen: list[np.ndarray] = []

        def policy(right_observation: np.ndarray) -> np.ndarray:
            seen.append(right_observation.copy())
            return right_observation[..., 210:239]

        wrapper = LeftHandStudentWrapper(policy, G1_JOINT_NAMES, offset, scale)
        action = wrapper(observation)
        np.testing.assert_allclose(
            seen[0],
            mirror_student_observation(
                observation,
                G1_JOINT_NAMES,
                offset,
                scale,
                offset,
                scale,
            ),
        )
        expected = _expected_joint(seen[0][210:239] * scale + offset, G1_JOINT_NAMES)
        np.testing.assert_allclose(action, expected - offset)


if __name__ == "__main__":
    unittest.main()
