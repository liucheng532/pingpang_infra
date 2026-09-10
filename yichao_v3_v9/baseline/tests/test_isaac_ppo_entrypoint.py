from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import train_isaac_doubles_ppo as entrypoint
from doubles_planner.isaac_rl import (
    IsaacDoublesPPOConfig,
    IsaacPPORunnerConfig,
    _load_student_policy,
    _require_requested_controller_root,
)


class IsaacPPOEntrypointTests(unittest.TestCase):
    @staticmethod
    def _v9_metadata(**overrides: object) -> dict[str, object]:
        metadata: dict[str, object] = {
            "status": "loaded",
            "sha256": entrypoint.V9_CHECKPOINT_SHA256,
            "iteration": entrypoint.V9_CHECKPOINT_ITERATION,
            "format": entrypoint.V9_CHECKPOINT_FORMAT,
            "student_obs_dim": entrypoint.V9_STUDENT_OBS_DIM,
            "student_action_dim": entrypoint.V9_STUDENT_ACTION_DIM,
            "phase_order": list(entrypoint.V9_PHASE_ORDER),
            "hit_teacher_sha256": entrypoint.V9_HIT_TEACHER_SHA256,
            "move_teacher_sha256": entrypoint.V9_MOVE_TEACHER_SHA256,
            "move_teacher_contract": entrypoint.V9_MOVE_TEACHER_CONTRACT,
        }
        metadata.update(overrides)
        return metadata

    @staticmethod
    def _v9_manifest() -> dict[str, object]:
        return {
            "contract_version": "v9-timed-handoff-v1",
            "training_git_revision": entrypoint.V9_CONTROLLER_GIT_REVISION,
            "move_teacher_contract": entrypoint.V9_MOVE_TEACHER_CONTRACT,
            "handoff_mode": "external_timed",
            "phase_order": list(entrypoint.V9_PHASE_ORDER),
            "post_delay_s": 0.20,
            "outward_hold_s": 3.0,
            "reference_transition_s": 0.30,
            "home_y_range_m": [0.0, 0.4],
            "locomotion_preemption": True,
            "hold_reference": "stable_lower_nominal_arms_neutral_waist",
        }

    @staticmethod
    def _valid_git_metadata(root: Path) -> dict[str, object]:
        return {
            "requested_root": str(root.resolve()),
            "git_root": str(root.resolve()),
            "revision": "f72d4ae000000000000000000000000000000000",
            "head_revision": "f72d4ae000000000000000000000000000000000",
            "training_revision": entrypoint.V9_CONTROLLER_GIT_REVISION,
            "training_revision_is_ancestor": True,
            "clean": True,
            "status_porcelain": "",
        }

    def test_parser_exposes_isaac_training_controls_without_simulator_import(self) -> None:
        parser = entrypoint.build_parser(include_app_launcher=False)
        args = parser.parse_args(
            [
                "--num-envs",
                "7",
                "--max_iterations",
                "11",
                "--steps_per_env",
                "13",
                "--low-level-steps",
                "3",
                "--preflight-steps",
                "2",
                "--device",
                "cpu",
            ]
        )
        self.assertEqual(args.num_envs, 7)
        self.assertEqual(args.iterations, 11)
        self.assertEqual(args.steps_per_env, 13)
        self.assertEqual(args.low_level_steps, 3)
        self.assertEqual(args.preflight_steps, 2)
        self.assertEqual(args.device, "cpu")

    def test_schema_is_versioned_and_json_serializable(self) -> None:
        environment = IsaacDoublesPPOConfig(
            controller_root=Path("../controller"),
            checkpoint=Path("model_23000.pt"),
            num_envs=3,
        )
        runner = replace(IsaacPPORunnerConfig(), max_iterations=9)
        payload = entrypoint.schema_payload(environment, runner, run_name="unit")
        encoded = json.dumps(payload)
        self.assertIn("doubles-centralized-v2", encoded)
        self.assertEqual(payload["backend"], "isaacsim")
        self.assertEqual(len(payload["action_names"]), 12)
        self.assertEqual(len(payload["observation_names"]), len(set(payload["observation_names"])))
        self.assertEqual(payload["runner"]["max_iterations"], 9)

    def test_schema_preserves_validated_checkpoint_controller_evidence(self) -> None:
        environment = IsaacDoublesPPOConfig()
        runner = IsaacPPORunnerConfig()
        compatibility = self._v9_metadata(
            controller_git={"head_revision": "descendant", "clean": True},
            controller_contract={"handoff_mode": "external_timed"},
        )
        payload = entrypoint.schema_payload(
            environment,
            runner,
            run_name="v9",
            checkpoint_compatibility=compatibility,
        )
        self.assertEqual(
            payload["frozen_low_level_policy"]["controller_git"],
            compatibility["controller_git"],
        )
        self.assertEqual(
            payload["compatibility"]["controller_contract"],
            compatibility["controller_contract"],
        )
        self.assertTrue(payload["compatibility"]["checkpoint_controller_validated"])

    def test_runner_config_matches_rsl_rl_contract(self) -> None:
        config = IsaacPPORunnerConfig(num_steps_per_env=4, max_iterations=8)
        values = config.to_dict()
        self.assertEqual(values["num_steps_per_env"], 4)
        self.assertEqual(values["max_iterations"], 8)
        self.assertEqual(values["obs_groups"], {"actor": ["policy"], "critic": ["policy"]})
        self.assertEqual(values["algorithm"]["class_name"], "PPO")
        self.assertEqual(values["actor"]["class_name"], "MLPModel")

    def test_source_requires_isaac_and_does_not_reference_synthetic_task(self) -> None:
        source = Path(entrypoint.__file__).read_text(encoding="utf-8")
        self.assertIn("AppLauncher", source)
        self.assertIn("create_isaac_doubles_high_level_env", source)
        self.assertIn("OnPolicyRunner", source)
        self.assertNotIn("StageOptionVectorEnv", source)
        self.assertNotIn("train_doubles_ppo", source)

    def test_invalid_sizes_are_rejected_before_launch(self) -> None:
        parser = entrypoint.build_parser(include_app_launcher=False)
        args = parser.parse_args(["--num-envs", "0"])
        with self.assertRaises(ValueError):
            entrypoint._validate_args(args)

    def test_checkpoint_load_error_fails_closed(self) -> None:
        with patch.object(
            entrypoint,
            "_checkpoint_metadata",
            return_value={"status": "load_error", "load_error": "UnpicklingError"},
        ):
            with self.assertRaisesRegex(RuntimeError, "load status is 'load_error'"):
                entrypoint._validate_checkpoint_controller_contract(
                    Path("corrupt.pt"), Path("controller")
                )

    def test_checkpoint_metadata_never_falls_back_to_unsafe_load(self) -> None:
        load = Mock(side_effect=TypeError("weights_only is unsupported"))
        fake_torch = SimpleNamespace(load=load)
        with tempfile.TemporaryDirectory() as directory, patch.dict(
            sys.modules, {"torch": fake_torch}
        ):
            checkpoint = Path(directory) / "student.pt"
            checkpoint.write_bytes(b"not a checkpoint")
            metadata = entrypoint._checkpoint_metadata(checkpoint)
        self.assertEqual(metadata["status"], "load_error")
        load.assert_called_once_with(
            checkpoint.resolve(), map_location="cpu", weights_only=True
        )

    def test_checkpoint_metadata_derives_student_action_dimension(self) -> None:
        payload = {
            "student_state_dict": {
                "actor.6.weight": SimpleNamespace(shape=(29, 192)),
            },
        }
        fake_torch = SimpleNamespace(load=Mock(return_value=payload))
        with tempfile.TemporaryDirectory() as directory, patch.dict(
            sys.modules, {"torch": fake_torch}
        ):
            checkpoint = Path(directory) / "student.pt"
            checkpoint.write_bytes(b"checkpoint")
            metadata = entrypoint._checkpoint_metadata(checkpoint)
        self.assertEqual(metadata["student_action_dim"], 29)
        self.assertEqual(metadata["student_output_weight_shape"], [29, 192])

    def test_v9_checkpoint_identity_is_fully_locked(self) -> None:
        invalid_values: dict[str, object] = {
            "sha256": "0" * 64,
            "iteration": entrypoint.V9_CHECKPOINT_ITERATION - 1,
            "format": "wrong_format",
            "student_obs_dim": entrypoint.V9_STUDENT_OBS_DIM - 1,
            "student_action_dim": entrypoint.V9_STUDENT_ACTION_DIM - 1,
            "phase_order": list(reversed(entrypoint.V9_PHASE_ORDER)),
            "hit_teacher_sha256": "1" * 64,
            "move_teacher_sha256": "2" * 64,
            "move_teacher_contract": "v9_wrong_contract",
        }
        for field, value in invalid_values.items():
            with self.subTest(field=field), patch.object(
                entrypoint,
                "_checkpoint_metadata",
                return_value=self._v9_metadata(**{field: value}),
            ):
                with self.assertRaisesRegex(RuntimeError, field):
                    entrypoint._validate_checkpoint_controller_contract(
                        Path("v9.pt"), Path("controller")
                    )

    def test_v9_checkpoint_requires_timed_handoff_controller_contract(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch.object(
                entrypoint, "_checkpoint_metadata", return_value=self._v9_metadata()
            ), patch.object(
                entrypoint,
                "_controller_git_metadata",
                return_value=self._valid_git_metadata(root),
            ):
                with self.assertRaisesRegex(RuntimeError, "V9 controller contract"):
                    entrypoint._validate_checkpoint_controller_contract(
                        Path("v9.pt"), root
                    )

    def test_matching_v9_controller_contract_and_descendant_git_pass(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / entrypoint.V9_CONTROLLER_CONTRACT_FILE).write_text(
                json.dumps(self._v9_manifest()), encoding="utf-8"
            )
            with patch.object(
                entrypoint, "_checkpoint_metadata", return_value=self._v9_metadata()
            ), patch.object(
                entrypoint,
                "_controller_git_metadata",
                return_value=self._valid_git_metadata(root),
            ):
                result = entrypoint._validate_checkpoint_controller_contract(
                    Path("v9.pt"), root
                )
        self.assertEqual(result["controller_contract"]["handoff_mode"], "external_timed")
        self.assertTrue(result["controller_git"]["training_revision_is_ancestor"])

    def test_v9_sidecar_locks_training_git_revision(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = self._v9_manifest()
            manifest["training_git_revision"] = "0" * 40
            (root / entrypoint.V9_CONTROLLER_CONTRACT_FILE).write_text(
                json.dumps(manifest), encoding="utf-8"
            )
            with patch.object(
                entrypoint, "_checkpoint_metadata", return_value=self._v9_metadata()
            ), patch.object(
                entrypoint,
                "_controller_git_metadata",
                return_value=self._valid_git_metadata(root),
            ):
                with self.assertRaisesRegex(RuntimeError, "training_git_revision"):
                    entrypoint._validate_checkpoint_controller_contract(
                        Path("v9.pt"), root
                    )

    def test_matching_sidecar_cannot_override_wrong_git_ancestry(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / entrypoint.V9_CONTROLLER_CONTRACT_FILE).write_text(
                json.dumps(self._v9_manifest()), encoding="utf-8"
            )
            git_metadata = self._valid_git_metadata(root)
            git_metadata["training_revision_is_ancestor"] = False
            with patch.object(
                entrypoint, "_checkpoint_metadata", return_value=self._v9_metadata()
            ), patch.object(
                entrypoint, "_controller_git_metadata", return_value=git_metadata
            ):
                with self.assertRaisesRegex(RuntimeError, "not an ancestor"):
                    entrypoint._validate_checkpoint_controller_contract(
                        Path("v9.pt"), root
                    )

    def test_dirty_v9_controller_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            git_metadata = self._valid_git_metadata(root)
            git_metadata.update({"clean": False, "status_porcelain": " M source/file.py"})
            with patch.object(
                entrypoint, "_checkpoint_metadata", return_value=self._v9_metadata()
            ), patch.object(
                entrypoint, "_controller_git_metadata", return_value=git_metadata
            ):
                with self.assertRaisesRegex(RuntimeError, "worktree is dirty"):
                    entrypoint._validate_checkpoint_controller_contract(
                        Path("v9.pt"), root
                    )

    def test_controller_git_metadata_checks_training_revision_ancestry(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            head = "f72d4ae000000000000000000000000000000000"
            outputs = {
                ("rev-parse", "--show-toplevel"): str(root),
                ("rev-parse", "HEAD"): head,
                ("status", "--porcelain", "--untracked-files=no"): "",
                (
                    "rev-parse",
                    "--verify",
                    f"{entrypoint.V9_CONTROLLER_GIT_REVISION}^{{commit}}",
                ): entrypoint.V9_CONTROLLER_GIT_REVISION,
                (
                    "merge-base",
                    "--is-ancestor",
                    entrypoint.V9_CONTROLLER_GIT_REVISION,
                    "HEAD",
                ): "",
            }

            def run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
                self.assertEqual(command[:3], ["git", "-C", str(root)])
                self.assertNotIn("shell", kwargs)
                arguments = tuple(command[3:])
                return subprocess.CompletedProcess(command, 0, outputs[arguments], "")

            with patch.object(entrypoint.subprocess, "run", side_effect=run) as git_run:
                result = entrypoint._controller_git_metadata(
                    root, entrypoint.V9_CONTROLLER_GIT_REVISION
                )
        self.assertEqual(git_run.call_count, 5)
        self.assertEqual(result["head_revision"], head)
        self.assertTrue(result["training_revision_is_ancestor"])

    def test_controller_git_metadata_rejects_non_ancestor(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()

            def run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
                arguments = tuple(command[3:])
                if arguments[:2] == ("merge-base", "--is-ancestor"):
                    raise subprocess.CalledProcessError(1, command)
                outputs = {
                    ("rev-parse", "--show-toplevel"): str(root),
                    ("rev-parse", "HEAD"): "f72d4ae000000000000000000000000000000000",
                    ("status", "--porcelain", "--untracked-files=no"): "",
                    (
                        "rev-parse",
                        "--verify",
                        f"{entrypoint.V9_CONTROLLER_GIT_REVISION}^{{commit}}",
                    ): entrypoint.V9_CONTROLLER_GIT_REVISION,
                }
                return subprocess.CompletedProcess(command, 0, outputs[arguments], "")

            with patch.object(entrypoint.subprocess, "run", side_effect=run):
                with self.assertRaisesRegex(RuntimeError, "merge-base --is-ancestor"):
                    entrypoint._controller_git_metadata(
                        root, entrypoint.V9_CONTROLLER_GIT_REVISION
                    )

    def test_requested_controller_root_cannot_silently_fall_back(self) -> None:
        requested = Path("/tmp/requested-controller")
        fallback = Path("/tmp/fallback-controller")
        with self.assertRaisesRegex(RuntimeError, "refusing fallback controller"):
            _require_requested_controller_root(requested, fallback)
        self.assertEqual(
            _require_requested_controller_root(requested, requested), requested.resolve()
        )

    def test_runtime_student_loader_never_falls_back_to_unsafe_load(self) -> None:
        load = Mock(side_effect=TypeError("weights_only is unsupported"))
        fake_torch = SimpleNamespace(load=load)
        with tempfile.TemporaryDirectory() as directory, patch.dict(
            sys.modules, {"torch": fake_torch}
        ):
            checkpoint = Path(directory) / "student.pt"
            checkpoint.write_bytes(b"checkpoint")
            with self.assertRaisesRegex(RuntimeError, "safely load"):
                _load_student_policy(checkpoint, "cpu", Mock(), Mock())
        load.assert_called_once_with(
            checkpoint.resolve(), map_location="cpu", weights_only=True
        )


if __name__ == "__main__":
    unittest.main()
