from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from doubles_planner import video_debug
from doubles_planner.video_debug import (
    HudFrameRecord,
    PlayerHudState,
    ProximityHudState,
    base_marker_specs,
    hud_text_lines,
    overlay_hud,
    postprocess_video,
    transcode_webm,
)


def _record(frame_index: int = 0, *, simultaneous_hit: bool = False) -> HudFrameRecord:
    return HudFrameRecord(
        frame_index=frame_index,
        sim_time_s=frame_index * 0.02,
        strategy="cbf_mpc",
        shot_index=0,
        shot_count=6,
        hitter="left",
        left=PlayerHudState(
            phase="HIT",
            actual_base_y=-0.35,
            terminal_base_y=-0.18,
            safety_base_y=-0.26,
        ),
        right=PlayerHudState(
            phase="HIT" if simultaneous_hit else "HOME_HOLD",
            actual_base_y=0.35,
            terminal_base_y=0.52,
            safety_base_y=0.43,
        ),
        base=ProximityHudState(distance_m=0.70, ttc_s=None),
        hand=ProximityHudState(distance_m=0.42, ttc_s=0.60),
        racket=ProximityHudState(distance_m=0.31, ttc_s=0.40),
        safety_active=True,
        warnings=("test warning",) if simultaneous_hit else (),
    )


def _write_video(path: Path, values: list[int], size: tuple[int, int] = (160, 120)) -> None:
    cv2 = pytest.importorskip("cv2")
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"MJPG"),
        20.0,
        size,
    )
    assert writer.isOpened()
    try:
        for value in values:
            writer.write(np.full((size[1], size[0], 3), value, dtype=np.uint8))
    finally:
        writer.release()


def _read_video(path: Path) -> list[np.ndarray]:
    cv2 = pytest.importorskip("cv2")
    capture = cv2.VideoCapture(str(path))
    assert capture.isOpened()
    frames: list[np.ndarray] = []
    try:
        while True:
            available, frame = capture.read()
            if not available:
                break
            frames.append(frame)
    finally:
        capture.release()
    return frames


def test_record_detects_simultaneous_hit_and_validates_ranges() -> None:
    assert _record(simultaneous_hit=True).simultaneous_hit is True
    assert _record(simultaneous_hit=False).simultaneous_hit is False
    with pytest.raises(ValueError, match="distance_m must not be negative"):
        ProximityHudState(distance_m=-0.1, ttc_s=None)
    with pytest.raises(ValueError, match="shot_index"):
        HudFrameRecord(**{**vars(_record()), "shot_index": 6})


def test_base_marker_specs_encode_actual_terminal_and_safety() -> None:
    markers = base_marker_specs(_record(), {"left": 0.1, "right": 0.2})
    assert len(markers) == 6
    assert [marker.role for marker in markers[:3]] == ["actual", "terminal", "safety"]
    assert markers[0].position == pytest.approx((0.1, -0.35, 0.025))
    assert markers[1].position == pytest.approx((0.1, -0.18, 0.070))
    assert markers[2].position == pytest.approx((0.1, -0.26, 0.115))
    assert markers[0].color_rgb != markers[3].color_rgb
    assert {marker.shape for marker in markers[:3]} == {"cylinder", "sphere", "cuboid"}


def test_hud_text_explicitly_labels_phase_hit_and_command_roles() -> None:
    lines = hud_text_lines(_record(simultaneous_hit=True))
    assert any("LEFT phase=HIT HIT=ACTIVE role=hold" == line for line in lines)
    assert any("RIGHT phase=HIT HIT=ACTIVE role=hold" == line for line in lines)
    assert "planner terminal command=" in lines[2]
    assert "safety command=" in lines[2]
    assert "actual=" in lines[2]
    assert "selected_hitter=LEFT" in lines[0]
    assert "stage=idle" in lines[0]
    assert "commit=no" in lines[0]


def test_overlay_hud_returns_a_modified_copy() -> None:
    pytest.importorskip("cv2")
    source = np.full((360, 640, 3), 90, dtype=np.uint8)
    rendered = overlay_hud(source, _record(simultaneous_hit=True))
    assert rendered.shape == source.shape
    assert rendered.dtype == source.dtype
    assert np.any(rendered != source)
    assert np.all(source == 90)


def test_missing_cv2_has_clear_lazy_runtime_error(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(video_debug, "_cv2", None)
    monkeypatch.setattr(video_debug, "_CV2_IMPORT_ERROR", ImportError("missing"))
    with pytest.raises(RuntimeError, match="opencv-python"):
        overlay_hud(np.zeros((120, 160, 3), dtype=np.uint8), _record())


def test_postprocess_aligns_start_frame_temporally_crops_and_spatially_crops(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "source.avi"
    output = tmp_path / "hud.avi"
    _write_video(source, [10, 20, 40, 60, 80, 100, 120])
    monkeypatch.setattr(video_debug, "overlay_hud", lambda frame, record: frame.copy())

    result = postprocess_video(
        source,
        output,
        [_record(index) for index in range(3)],
        20.0,
        start_frame=2,
        crop=(10, 5, 100, 80),
    )

    assert result == output.resolve()
    frames = _read_video(output)
    assert len(frames) == 3
    assert frames[0].shape[:2] == (80, 100)
    assert [float(frame.mean()) for frame in frames] == pytest.approx([40, 60, 80], abs=4.0)


def test_postprocess_mp4_uses_browser_compatible_h264_faststart(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    if shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None:
        pytest.skip("FFmpeg tools are not installed")
    source = tmp_path / "source.avi"
    output = tmp_path / "hud.mp4"
    _write_video(source, [20, 40, 60])
    monkeypatch.setattr(video_debug, "overlay_hud", lambda frame, record: frame.copy())

    postprocess_video(source, output, [_record(index) for index in range(3)], 20.0)

    probe = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=codec_name,pix_fmt,nb_frames",
            "-of",
            "default=noprint_wrappers=1",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "codec_name=h264" in probe.stdout
    assert "pix_fmt=yuv420p" in probe.stdout
    assert "nb_frames=3" in probe.stdout
    encoded = output.read_bytes()
    assert encoded.index(b"moov") < encoded.index(b"mdat")


def test_postprocess_webm_uses_browser_compatible_vp8(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    if shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None:
        pytest.skip("FFmpeg tools are not installed")
    source = tmp_path / "source.avi"
    output = tmp_path / "hud.webm"
    _write_video(source, [20, 40, 60])
    monkeypatch.setattr(video_debug, "overlay_hud", lambda frame, record: frame.copy())

    postprocess_video(source, output, [_record(index) for index in range(3)], 20.0)

    probe = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "format=format_name:stream=codec_name,pix_fmt,nb_frames",
            "-of",
            "default=noprint_wrappers=1",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "format_name=matroska,webm" in probe.stdout
    assert "codec_name=vp8" in probe.stdout
    assert "pix_fmt=yuv420p" in probe.stdout


def test_transcode_webm_supports_raw_recording_without_hud(tmp_path: Path) -> None:
    if shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None:
        pytest.skip("FFmpeg tools are not installed")
    source = tmp_path / "source.avi"
    output = tmp_path / "raw.webm"
    _write_video(source, [20, 40, 60])

    transcode_webm(source, output)

    probe = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=codec_name,pix_fmt,nb_frames",
            "-of",
            "default=noprint_wrappers=1",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "codec_name=vp8" in probe.stdout
    assert "pix_fmt=yuv420p" in probe.stdout
    assert len(_read_video(output)) == 3
    assert output.stat().st_mode & 0o777 == 0o644


def test_postprocess_rejects_noncontiguous_records(tmp_path: Path) -> None:
    source = tmp_path / "source.avi"
    _write_video(source, [10, 20, 30])
    with pytest.raises(ValueError, match="contiguous"):
        postprocess_video(source, tmp_path / "output.avi", [_record(0), _record(2)], 20.0)


def test_postprocess_reports_missing_aligned_frame_and_removes_partial_output(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "short.avi"
    output = tmp_path / "partial.avi"
    _write_video(source, [10, 20, 30])
    monkeypatch.setattr(video_debug, "overlay_hud", lambda frame, record: frame.copy())

    with pytest.raises(RuntimeError, match=r"source_frame=3.*records_written=1/2"):
        postprocess_video(
            source,
            output,
            [_record(0), _record(1)],
            20.0,
            start_frame=2,
        )
    assert not output.exists()


def test_postprocess_reports_video_shorter_than_start_frame(tmp_path: Path) -> None:
    source = tmp_path / "short.avi"
    _write_video(source, [10, 20])
    with pytest.raises(RuntimeError, match="before start_frame=3"):
        postprocess_video(source, tmp_path / "output.avi", [_record()], 20.0, start_frame=3)
