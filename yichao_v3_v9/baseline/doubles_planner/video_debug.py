from __future__ import annotations

import math
import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Literal, Mapping

import numpy as np

try:
    import cv2 as _cv2
except (ImportError, OSError) as exc:
    _cv2 = None
    _CV2_IMPORT_ERROR: BaseException | None = exc
else:
    _CV2_IMPORT_ERROR = None


PlayerName = Literal["left", "right"]
MarkerRole = Literal["actual", "terminal", "safety"]
MarkerShape = Literal["cylinder", "sphere", "cuboid"]
CropRect = tuple[int, int, int, int]

_PLAYER_COLORS_BGR = {
    "left": (255, 210, 40),
    "right": (190, 80, 255),
}
_ROLE_COLORS_RGB = {
    "actual_left": (0.05, 0.82, 1.0),
    "actual_right": (1.0, 0.32, 0.76),
    "terminal": (0.30, 1.0, 0.38),
    "safety": (1.0, 0.55, 0.08),
}
_PROXIMITY_THRESHOLDS = {
    "base": (0.47, 0.35),
    "hand": (0.20, 0.15),
    "racket": (0.25, 0.15),
}


def _finite(value: float, name: str) -> float:
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


@dataclass(frozen=True)
class PlayerHudState:
    phase: str
    actual_base_y: float
    terminal_base_y: float
    safety_base_y: float
    role: str = "hold"

    def __post_init__(self) -> None:
        if not self.phase or not self.phase.strip():
            raise ValueError("phase must be a non-empty string")
        object.__setattr__(self, "phase", self.phase.strip())
        if not self.role or not self.role.strip():
            raise ValueError("role must be a non-empty string")
        object.__setattr__(self, "role", self.role.strip())
        for name in ("actual_base_y", "terminal_base_y", "safety_base_y"):
            object.__setattr__(self, name, _finite(getattr(self, name), name))


@dataclass(frozen=True)
class ProximityHudState:
    distance_m: float
    ttc_s: float | None

    def __post_init__(self) -> None:
        distance = _finite(self.distance_m, "distance_m")
        if distance < 0.0:
            raise ValueError("distance_m must not be negative")
        object.__setattr__(self, "distance_m", distance)
        if self.ttc_s is not None:
            ttc = _finite(self.ttc_s, "ttc_s")
            if ttc < 0.0:
                raise ValueError("ttc_s must not be negative")
            object.__setattr__(self, "ttc_s", ttc)


@dataclass(frozen=True)
class HudFrameRecord:
    frame_index: int
    sim_time_s: float
    strategy: str
    shot_index: int | None
    shot_count: int
    hitter: PlayerName | None
    left: PlayerHudState
    right: PlayerHudState
    base: ProximityHudState
    hand: ProximityHudState
    racket: ProximityHudState
    safety_active: bool = False
    safety_emergency: bool = False
    relay_stage: str = "idle"
    commit_requested: bool = False
    warnings: tuple[str, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if isinstance(self.frame_index, bool) or not isinstance(self.frame_index, int):
            raise TypeError("frame_index must be an integer")
        if self.frame_index < 0:
            raise ValueError("frame_index must not be negative")
        object.__setattr__(self, "sim_time_s", _finite(self.sim_time_s, "sim_time_s"))
        if self.sim_time_s < 0.0:
            raise ValueError("sim_time_s must not be negative")
        if not self.strategy or not self.strategy.strip():
            raise ValueError("strategy must be a non-empty string")
        object.__setattr__(self, "strategy", self.strategy.strip())
        if isinstance(self.shot_count, bool) or not isinstance(self.shot_count, int):
            raise TypeError("shot_count must be an integer")
        if self.shot_count < 0:
            raise ValueError("shot_count must not be negative")
        if self.shot_index is not None:
            if isinstance(self.shot_index, bool) or not isinstance(self.shot_index, int):
                raise TypeError("shot_index must be an integer or None")
            if not 0 <= self.shot_index < self.shot_count:
                raise ValueError("shot_index must identify one of shot_count shots")
        if self.hitter not in {None, "left", "right"}:
            raise ValueError("hitter must be left, right, or None")
        if not self.relay_stage or not self.relay_stage.strip():
            raise ValueError("relay_stage must be a non-empty string")
        object.__setattr__(self, "relay_stage", self.relay_stage.strip())
        normalized_warnings = tuple(str(warning).strip() for warning in self.warnings)
        if any(not warning for warning in normalized_warnings):
            raise ValueError("warnings must not contain empty strings")
        object.__setattr__(self, "warnings", normalized_warnings)

    @property
    def simultaneous_hit(self) -> bool:
        return self.left.phase == "HIT" and self.right.phase == "HIT"


@dataclass(frozen=True)
class WorldMarkerSpec:
    name: str
    player: PlayerName
    role: MarkerRole
    shape: MarkerShape
    position: tuple[float, float, float]
    color_rgb: tuple[float, float, float]


def base_marker_specs(
    record: HudFrameRecord,
    base_x: Mapping[PlayerName, float] | None = None,
    *,
    ground_z: float = 0.025,
) -> tuple[WorldMarkerSpec, ...]:
    """Return Isaac-independent marker data for actual, terminal and safety base Y."""

    x_values: Mapping[PlayerName, float] = {"left": 0.0, "right": 0.0} if base_x is None else base_x
    if set(x_values) != {"left", "right"}:
        raise ValueError("base_x must contain exactly left and right")
    ground_z = _finite(ground_z, "ground_z")
    role_shape: dict[MarkerRole, MarkerShape] = {
        "actual": "cylinder",
        "terminal": "sphere",
        "safety": "cuboid",
    }
    role_height = {"actual": 0.0, "terminal": 0.045, "safety": 0.090}
    markers: list[WorldMarkerSpec] = []
    for player, state in (("left", record.left), ("right", record.right)):
        player_x = _finite(x_values[player], f"base_x[{player!r}]")
        y_values: dict[MarkerRole, float] = {
            "actual": state.actual_base_y,
            "terminal": state.terminal_base_y,
            "safety": state.safety_base_y,
        }
        for role, y_value in y_values.items():
            color_key = f"actual_{player}" if role == "actual" else role
            markers.append(
                WorldMarkerSpec(
                    name=f"{player}_{role}_base",
                    player=player,
                    role=role,
                    shape=role_shape[role],
                    position=(player_x, y_value, ground_z + role_height[role]),
                    color_rgb=_ROLE_COLORS_RGB[color_key],
                )
            )
    return tuple(markers)


def _require_cv2():
    if _cv2 is None:
        raise RuntimeError(
            "OpenCV is required for HUD rendering and video postprocessing; "
            "install the 'opencv-python' package in the active environment."
        ) from _CV2_IMPORT_ERROR
    return _cv2


def _validate_frame(frame: np.ndarray) -> None:
    if not isinstance(frame, np.ndarray):
        raise TypeError("frame must be a NumPy array")
    if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame must have uint8 BGR shape (height, width, 3)")
    if frame.shape[0] < 64 or frame.shape[1] < 96:
        raise ValueError("frame must be at least 96x64 pixels")


def _panel(cv2, image: np.ndarray, start: tuple[int, int], end: tuple[int, int], alpha: float = 0.72) -> None:
    overlay = image.copy()
    cv2.rectangle(overlay, start, end, (10, 14, 22), -1, cv2.LINE_AA)
    cv2.addWeighted(overlay, alpha, image, 1.0 - alpha, 0.0, image)
    cv2.rectangle(image, start, end, (115, 125, 143), 1, cv2.LINE_AA)


def _put_text_fit(
    cv2,
    image: np.ndarray,
    text: str,
    origin: tuple[int, int],
    max_width: int,
    scale: float,
    color: tuple[int, int, int],
    thickness: int = 1,
) -> None:
    resolved_scale = scale
    width = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, resolved_scale, thickness)[0][0]
    if width > max_width and width > 0:
        resolved_scale = max(0.22, resolved_scale * max_width / width)
    cv2.putText(
        image,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        resolved_scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def _format_ttc(ttc_s: float | None) -> str:
    return "--" if ttc_s is None else f"{ttc_s:.2f}s"


def _metric_color(name: str, state: ProximityHudState) -> tuple[int, int, int]:
    distance_threshold, ttc_threshold = _PROXIMITY_THRESHOLDS[name]
    if state.distance_m < distance_threshold or (state.ttc_s is not None and state.ttc_s < ttc_threshold):
        return (70, 85, 255)
    return (215, 225, 235)


def hud_text_lines(record: HudFrameRecord) -> tuple[str, ...]:
    shot_text = "--"
    if record.shot_index is not None:
        shot_text = f"{record.shot_index + 1:02d}/{record.shot_count:02d}"
    hitter_text = record.hitter.upper() if record.hitter is not None else "--"

    def player_lines(label: str, state: PlayerHudState) -> tuple[str, str]:
        hit_status = "ACTIVE" if state.phase == "HIT" else "inactive"
        return (
            f"{label} phase={state.phase} HIT={hit_status} role={state.role}",
            f"{label} base Y (m): actual={state.actual_base_y:+.3f} | "
            f"planner terminal command={state.terminal_base_y:+.3f} | "
            f"safety command={state.safety_base_y:+.3f}",
        )

    left_phase, left_commands = player_lines("LEFT", record.left)
    right_phase, right_commands = player_lines("RIGHT", record.right)
    if record.safety_emergency:
        safety_line = "SAFETY state=EMERGENCY"
    elif record.safety_active:
        safety_line = "SAFETY state=FILTER_ACTIVE"
    else:
        safety_line = "SAFETY state=INACTIVE"
    return (
        f"strategy={record.strategy}  stage={record.relay_stage}  "
        f"commit={'REQUESTED' if record.commit_requested else 'no'}  "
        f"shot={shot_text}  selected_hitter={hitter_text}  "
        f"t={record.sim_time_s:6.2f}s  frame={record.frame_index:05d}",
        left_phase,
        left_commands,
        right_phase,
        right_commands,
        f"BASE   distance={record.base.distance_m:.3f} m    TTC={_format_ttc(record.base.ttc_s)}",
        f"HAND   distance={record.hand.distance_m:.3f} m    TTC={_format_ttc(record.hand.ttc_s)}",
        f"RACKET distance={record.racket.distance_m:.3f} m    TTC={_format_ttc(record.racket.ttc_s)}",
        safety_line,
    )


def overlay_hud(frame: np.ndarray, record: HudFrameRecord) -> np.ndarray:
    """Return a BGR frame with a compact, camera-independent doubles debug HUD."""

    cv2 = _require_cv2()
    _validate_frame(frame)
    image = frame.copy()
    height, width = image.shape[:2]
    ui_scale = max(0.55, min(1.50, min(width / 1280.0, height / 720.0)))
    margin = max(6, int(round(13 * ui_scale)))
    padding = max(5, int(round(9 * ui_scale)))
    line_height = max(14, int(round(27 * ui_scale)))
    font_scale = 0.58 * ui_scale
    panel_width = min(width - 2 * margin, max(260, int(round(900 * ui_scale))))
    text_lines = hud_text_lines(record)
    panel_height = len(text_lines) * line_height + 2 * padding
    panel_bottom = min(height - margin, margin + panel_height)
    _panel(cv2, image, (margin, margin), (margin + panel_width, panel_bottom))
    text_x = margin + padding
    text_width = panel_width - 2 * padding
    baseline = margin + padding + line_height - max(2, int(5 * ui_scale))

    _put_text_fit(cv2, image, text_lines[0], (text_x, baseline), text_width, font_scale, (250, 250, 250), 2)

    player_rows = (
        (1, record.left, _PLAYER_COLORS_BGR["left"]),
        (3, record.right, _PLAYER_COLORS_BGR["right"]),
    )
    for line_index, state, color in player_rows:
        phase_color = (60, 70, 255) if state.phase == "HIT" else color
        row = (
            text_lines[line_index],
            phase_color,
            2,
        )
        _put_text_fit(
            cv2,
            image,
            row[0],
            (text_x, baseline + line_index * line_height),
            text_width,
            font_scale,
            row[1],
            row[2],
        )

    command_rows = (
        (2, _PLAYER_COLORS_BGR["left"]),
        (4, _PLAYER_COLORS_BGR["right"]),
    )
    for line_index, color in command_rows:
        _put_text_fit(
            cv2,
            image,
            text_lines[line_index],
            (text_x, baseline + line_index * line_height),
            text_width,
            font_scale,
            color,
            1,
        )

    metric_rows = ((5, "base", record.base), (6, "hand", record.hand), (7, "racket", record.racket))
    for line_index, name, state in metric_rows:
        metric_color = _metric_color(name, state)
        _put_text_fit(
            cv2,
            image,
            text_lines[line_index],
            (text_x, baseline + line_index * line_height),
            text_width,
            font_scale,
            metric_color,
            2 if metric_color != (215, 225, 235) else 1,
        )
    safety_color = (
        (60, 70, 255)
        if record.safety_emergency
        else (40, 220, 255)
        if record.safety_active
        else (155, 165, 180)
    )
    _put_text_fit(
        cv2,
        image,
        text_lines[8],
        (text_x, baseline + 8 * line_height),
        text_width,
        font_scale,
        safety_color,
        2,
    )

    warnings = list(record.warnings)
    if record.simultaneous_hit:
        warnings.insert(0, "SIMULTANEOUS HIT: both players are in HIT phase")
    if record.safety_emergency and "SAFETY EMERGENCY" not in warnings:
        warnings.append("SAFETY EMERGENCY")
    if warnings:
        warning_height = max(28, int(round(42 * ui_scale)))
        warning_top = max(panel_bottom + margin, height - margin - warning_height - max(30, int(46 * ui_scale)))
        warning_bottom = min(height - margin, warning_top + warning_height)
        _panel(cv2, image, (margin, warning_top), (width - margin, warning_bottom), alpha=0.82)
        warning_text = "WARNING | " + " | ".join(warnings)
        _put_text_fit(
            cv2,
            image,
            warning_text,
            (margin + padding, warning_bottom - max(7, int(10 * ui_scale))),
            width - 2 * (margin + padding),
            0.65 * ui_scale,
            (55, 70, 255),
            2,
        )

    legend_height = max(25, int(round(36 * ui_scale)))
    legend_top = height - margin - legend_height
    _panel(cv2, image, (margin, legend_top), (width - margin, height - margin), alpha=0.72)
    legend_y = legend_top + legend_height // 2
    marker_x = margin + padding + max(4, int(6 * ui_scale))
    radius = max(3, int(round(5 * ui_scale)))
    legend_items = (
        ("actual base", (40, 210, 255), "circle"),
        ("terminal command", (95, 255, 75), "cross"),
        ("safety command", (20, 140, 255), "square"),
    )
    for label, color, shape in legend_items:
        if shape == "circle":
            cv2.circle(image, (marker_x, legend_y), radius, color, -1, cv2.LINE_AA)
        elif shape == "cross":
            cv2.drawMarker(image, (marker_x, legend_y), color, cv2.MARKER_CROSS, 2 * radius + 4, 2, cv2.LINE_AA)
        else:
            cv2.rectangle(
                image,
                (marker_x - radius, legend_y - radius),
                (marker_x + radius, legend_y + radius),
                color,
                -1,
                cv2.LINE_AA,
            )
        label_width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.43 * ui_scale, 1)[0][0]
        _put_text_fit(
            cv2,
            image,
            label,
            (marker_x + radius + padding, legend_y + max(4, int(5 * ui_scale))),
            max(40, width - marker_x - 2 * margin),
            0.43 * ui_scale,
            (225, 230, 238),
        )
        marker_x += radius + padding + label_width + max(16, int(30 * ui_scale))
        if marker_x >= width - margin - 60:
            break
    return image


def _validated_crop(crop: CropRect | None, width: int, height: int) -> CropRect:
    if crop is None:
        return 0, 0, width, height
    if not isinstance(crop, tuple) or len(crop) != 4:
        raise TypeError("crop must be an (x, y, width, height) tuple or None")
    if any(isinstance(value, bool) or not isinstance(value, int) for value in crop):
        raise TypeError("crop values must be integers")
    x, y, crop_width, crop_height = crop
    if x < 0 or y < 0 or crop_width <= 0 or crop_height <= 0:
        raise ValueError("crop origin must be non-negative and dimensions must be positive")
    if x + crop_width > width or y + crop_height > height:
        raise ValueError(f"crop {crop!r} exceeds the {width}x{height} input frame")
    return crop


def _validate_record_sequence(records: tuple[HudFrameRecord, ...]) -> None:
    if not records:
        raise ValueError("records must contain at least one HUD frame")
    if any(not isinstance(record, HudFrameRecord) for record in records):
        raise TypeError("records must contain only HudFrameRecord instances")
    for previous, current in zip(records, records[1:]):
        if current.frame_index != previous.frame_index + 1:
            raise ValueError("records must have contiguous frame_index values in playback order")


def postprocess_video(
    input_path: str | Path,
    output_path: str | Path,
    records: Iterable[HudFrameRecord],
    fps: float,
    *,
    start_frame: int = 0,
    crop: CropRect | None = None,
) -> Path:
    """Overlay records on aligned frames and write exactly one output frame per record.

    ``start_frame`` skips raw frames before the first relay record. Isaac Sim
    5.1's Gym recorder emits one frame per step (no separate reset frame), so the
    doubles runner passes ``warmup_steps``. Trailing raw frames are omitted.
    ``crop`` optionally applies a spatial ``(x, y, width, height)`` crop.
    """

    cv2 = _require_cv2()
    input_file = Path(input_path).expanduser().resolve()
    output_file = Path(output_path).expanduser().resolve()
    frame_records = tuple(records)
    _validate_record_sequence(frame_records)
    fps = _finite(fps, "fps")
    if fps <= 0.0:
        raise ValueError("fps must be positive")
    if isinstance(start_frame, bool) or not isinstance(start_frame, int):
        raise TypeError("start_frame must be an integer")
    if start_frame < 0:
        raise ValueError("start_frame must not be negative")
    if not input_file.is_file():
        raise FileNotFoundError(f"input video does not exist: {input_file}")
    if input_file == output_file:
        raise ValueError("input_path and output_path must differ")
    if output_file.exists():
        raise FileExistsError(f"refusing to overwrite existing output video: {output_file}")
    output_file.parent.mkdir(parents=True, exist_ok=True)

    output_suffix = output_file.suffix.lower()
    h264_mp4 = output_suffix in {".mp4", ".m4v"}
    webm = output_suffix == ".webm"
    ffmpeg = shutil.which("ffmpeg") if h264_mp4 or webm else None
    if h264_mp4 and ffmpeg is None:
        raise RuntimeError("FFmpeg is required to create browser-compatible H.264 MP4 video")
    if webm and ffmpeg is None:
        raise RuntimeError("FFmpeg is required to create browser-compatible VP8 WebM video")
    intermediate_file: Path | None = None
    writer_file = output_file
    if h264_mp4 or webm:
        with tempfile.NamedTemporaryFile(
            prefix=f".{output_file.stem}.",
            suffix=".mp4" if h264_mp4 else ".avi",
            dir=output_file.parent,
            delete=False,
        ) as temporary:
            intermediate_file = Path(temporary.name)
        intermediate_file.unlink()
        writer_file = intermediate_file

    capture = cv2.VideoCapture(str(input_file))
    if not capture.isOpened():
        capture.release()
        raise RuntimeError(f"OpenCV could not open input video: {input_file}")
    writer = None
    output_created = False
    completed = False
    try:
        for source_index in range(start_frame):
            available, _ = capture.read()
            if not available:
                raise RuntimeError(
                    f"input video ended at frame {source_index} before start_frame={start_frame}"
                )

        crop_rect: CropRect | None = None
        for record_offset, record in enumerate(frame_records):
            source_index = start_frame + record_offset
            available, frame = capture.read()
            if not available:
                raise RuntimeError(
                    "input video is missing an aligned relay frame: "
                    f"source_frame={source_index}, record_frame={record.frame_index}, "
                    f"records_written={record_offset}/{len(frame_records)}"
                )
            if crop_rect is None:
                crop_rect = _validated_crop(crop, frame.shape[1], frame.shape[0])
                x, y, crop_width, crop_height = crop_rect
                codec = "mp4v" if h264_mp4 else "MJPG"
                writer = cv2.VideoWriter(
                    str(writer_file),
                    cv2.VideoWriter_fourcc(*codec),
                    fps,
                    (crop_width, crop_height),
                )
                output_created = True
                if not writer.isOpened():
                    raise RuntimeError(f"OpenCV could not create output video: {writer_file}")
            x, y, crop_width, crop_height = crop_rect
            if x + crop_width > frame.shape[1] or y + crop_height > frame.shape[0]:
                raise RuntimeError(
                    f"input frame {source_index} changed size and no longer satisfies crop {crop_rect!r}"
                )
            rendered = overlay_hud(frame[y : y + crop_height, x : x + crop_width], record)
            writer.write(rendered)

        completed = True
    finally:
        capture.release()
        if writer is not None:
            writer.release()
        if not completed and output_created and writer_file.exists():
            writer_file.unlink()

    if intermediate_file is not None:
        if h264_mp4:
            encoder_arguments = [
                "-c:v",
                "libx264",
                "-preset",
                "medium",
                "-crf",
                "18",
                "-profile:v",
                "baseline",
                "-level:v",
                "4.0",
                "-bf",
                "0",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
            ]
        else:
            encoder_arguments = [
                "-c:v",
                "libvpx",
                "-deadline",
                "good",
                "-cpu-used",
                "2",
                "-crf",
                "32",
                "-b:v",
                "0",
                "-pix_fmt",
                "yuv420p",
                "-row-mt",
                "1",
            ]
        try:
            result = subprocess.run(
                [
                    ffmpeg,
                    "-v",
                    "error",
                    "-n",
                    "-i",
                    str(intermediate_file),
                    "-an",
                    *encoder_arguments,
                    str(output_file),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
        finally:
            intermediate_file.unlink(missing_ok=True)
        if result.returncode != 0:
            output_file.unlink(missing_ok=True)
            detail = result.stderr.strip() or f"exit status {result.returncode}"
            raise RuntimeError(f"FFmpeg could not create H.264 output video: {detail}")

    if not output_file.is_file() or output_file.stat().st_size == 0:
        if output_file.exists():
            output_file.unlink()
        raise RuntimeError(f"OpenCV produced no output video: {output_file}")
    output_file.chmod(0o644)
    return output_file


def transcode_webm(
    input_path: str | Path,
    output_path: str | Path,
) -> Path:
    """Transcode a recorded video to a VS Code-compatible VP8 WebM file."""

    input_file = Path(input_path).expanduser().resolve()
    output_file = Path(output_path).expanduser().resolve()
    if not input_file.is_file():
        raise FileNotFoundError(f"input video does not exist: {input_file}")
    if input_file == output_file:
        raise ValueError("input_path and output_path must differ")
    if output_file.suffix.lower() != ".webm":
        raise ValueError("output_path must use the .webm suffix")
    if output_file.exists():
        raise FileExistsError(f"refusing to overwrite existing output video: {output_file}")
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("FFmpeg is required to create browser-compatible VP8 WebM video")
    output_file.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run(
        [
            ffmpeg,
            "-v",
            "error",
            "-n",
            "-i",
            str(input_file),
            "-an",
            "-c:v",
            "libvpx",
            "-deadline",
            "good",
            "-cpu-used",
            "2",
            "-crf",
            "32",
            "-b:v",
            "0",
            "-pix_fmt",
            "yuv420p",
            "-row-mt",
            "1",
            str(output_file),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        output_file.unlink(missing_ok=True)
        detail = result.stderr.strip() or f"exit status {result.returncode}"
        raise RuntimeError(f"FFmpeg could not create VP8 WebM output: {detail}")
    if not output_file.is_file() or output_file.stat().st_size == 0:
        output_file.unlink(missing_ok=True)
        raise RuntimeError(f"FFmpeg produced no output video: {output_file}")
    output_file.chmod(0o644)
    return output_file


__all__ = [
    "CropRect",
    "HudFrameRecord",
    "PlayerHudState",
    "ProximityHudState",
    "WorldMarkerSpec",
    "base_marker_specs",
    "hud_text_lines",
    "overlay_hud",
    "postprocess_video",
    "transcode_webm",
]
