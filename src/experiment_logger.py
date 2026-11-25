#!/usr/bin/env python3
"""Persistent, paper-oriented logging for MiR follower experiments.

The runtime application is intentionally kept separate from this module.  The
logger writes small, line-oriented files so an interrupted run still leaves
usable data and so the result directory can be copied directly out of Docker.
"""

from __future__ import annotations

import atexit
import csv
import json
import os
import platform
import socket
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional


def _utc_iso(timestamp: Optional[float] = None) -> str:
    value = datetime.fromtimestamp(timestamp or time.time(), tz=timezone.utc)
    return value.isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _clean_name(value: str, fallback: str) -> str:
    value = (value or fallback).strip()
    cleaned = "".join(char if char.isalnum() or char in "-_" else "-" for char in value)
    return cleaned.strip("-") or fallback


def _as_float(value: Any) -> Any:
    if value is None or value == "":
        return ""
    try:
        return float(value)
    except (TypeError, ValueError):
        return value


def _host_memory_total_bytes() -> Optional[int]:
    try:
        return int(os.sysconf("SC_PHYS_PAGES") * os.sysconf("SC_PAGE_SIZE"))
    except (AttributeError, OSError, TypeError, ValueError):
        return None


def _host_cpu_model() -> str:
    try:
        for line in Path("/proc/cpuinfo").read_text(encoding="utf-8").splitlines():
            key, separator, value = line.partition(":")
            if separator and key.strip().lower() in {"model name", "hardware"}:
                return value.strip()
    except OSError:
        pass
    return platform.processor()


def _container_memory_limit_bytes() -> Optional[int]:
    for candidate in (
        Path("/sys/fs/cgroup/memory.max"),
        Path("/sys/fs/cgroup/memory/memory.limit_in_bytes"),
    ):
        try:
            value = candidate.read_text(encoding="ascii").strip()
            if value and value != "max":
                limit = int(value)
                if 0 < limit < (1 << 60):
                    return limit
        except (OSError, ValueError):
            continue
    return None


class ExperimentLogger:
    """Write one self-contained directory for one application run."""

    FRAME_FIELDS = [
        "wall_time_utc",
        "wall_time_unix",
        "ros_time_s",
        "auto_phase_id",
        "frame_idx",
        "realsense_frame_number",
        "frame_timestamp_ms",
        "color_timestamp_ms",
        "depth_timestamp_ms",
        "frame_period_s",
        "measured_fps",
        "rgbd_aligned",
        "image_width",
        "image_height",
        "person_count",
        "all_detections_json",
        "target_id",
        "target_present",
        "target_x1",
        "target_y1",
        "target_x2",
        "target_y2",
        "target_center_x",
        "target_center_y",
        "roi_sample_count",
        "roi_nonzero_count",
        "roi_valid_count",
        "roi_zero_count",
        "roi_out_of_range_count",
        "roi_median_nonzero_m",
        "roi_median_filtered_m",
        "roi_empty",
        "depth_zero_ratio",
        "geometry_valid_count",
        "target_camera_x_m",
        "target_camera_y_m",
        "target_camera_z_m",
        "target_forward_from_camera_m",
        "target_front_gap_m",
        "target_lateral_offset_m",
        "target_height_m",
        "distance_history_len",
        "distance_history_last_m",
        "camera_depth_smoothed_m",
        "front_gap_history_len",
        "front_gap_history_last_m",
        "front_gap_smoothed_m",
        "front_gap_error_m",
        "distance_smoothed_m",
        "distance_error_m",
        "ground_truth_distance_m",
        "distance_error_to_ground_truth_m",
        "gate_state",
        "gesture_command",
        "is_tracking",
        "is_recognizing",
        "is_idle",
        "is_moving_to_goal",
        "is_rotating",
        "controller_profile_id",
        "distance_sample_valid",
        "controller_requested_linear_x_mps",
        "controller_rate_limited_linear_x_mps",
        "controller_angular_z_rps",
        "controller_shadow_active",
        "target_loss_latched",
        "target_reacquisition_stable_frames",
        "command_linear_x_mps",
        "command_angular_z_rps",
        "command_generated",
        "command_published",
        "control_mode",
        "odom_stamp_s",
        "odom_x_m",
        "odom_y_m",
        "odom_yaw_rad",
        "odom_linear_x_mps",
        "odom_linear_y_mps",
        "odom_angular_z_rps",
    ]

    COMMAND_FIELDS = [
        "wall_time_utc",
        "wall_time_unix",
        "ros_time_s",
        "auto_phase_id",
        "linear_x_mps",
        "angular_z_rps",
        "published",
        "control_mode",
        "source",
    ]

    ODOM_FIELDS = [
        "wall_time_utc",
        "wall_time_unix",
        "ros_time_s",
        "odom_stamp_s",
        "x_m",
        "y_m",
        "z_m",
        "yaw_rad",
        "linear_x_mps",
        "linear_y_mps",
        "linear_z_mps",
        "angular_z_rps",
    ]

    GROUND_TRUTH_FIELDS = [
        "wall_time_utc",
        "wall_time_unix",
        "ros_time_s",
        "frame_idx",
        "robot_person_separation_m",
        "reference_definition",
        "source",
        "quality",
        "note",
    ]

    def __init__(
        self,
        root: Optional[str] = None,
        run_id: Optional[str] = None,
        config_name: Optional[str] = None,
        scenario: Optional[str] = None,
        control_mode: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        requested_root = root or os.environ.get("EXPERIMENT_LOG_ROOT", "/app/experiments/logs")
        requested_run_id = run_id or os.environ.get("EXPERIMENT_RUN_ID", "")
        config_name = config_name or os.environ.get("EXPERIMENT_CONFIG", "runtime_baseline")
        scenario = scenario or os.environ.get("EXPERIMENT_SCENARIO", "unspecified")
        control_mode = control_mode or os.environ.get("MIR_CONTROL_MODE", "live")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_id = f"{timestamp}_{_clean_name(config_name, 'config')}_{_clean_name(scenario, 'run')}"
        self.root = Path(requested_root)
        self.run_id = _clean_name(requested_run_id, default_id)
        self.run_dir = self.root / self.run_id
        if self.run_dir.exists() and any(self.run_dir.iterdir()):
            suffix = 1
            while (self.root / f"{self.run_id}_{suffix:02d}").exists():
                suffix += 1
            self.run_id = f"{self.run_id}_{suffix:02d}"
            self.run_dir = self.root / self.run_id
        self.enabled = os.environ.get("EXPERIMENT_LOGGING", "1").lower() not in {
            "0",
            "false",
            "no",
            "off",
        }
        self.control_mode = control_mode
        self.config_name = config_name
        self.scenario = scenario
        self._lock = threading.RLock()
        self._closed = False
        self._last_frame_time: Optional[float] = None
        self._latest_odom: Dict[str, Any] = {}
        self._frame_count = 0
        self._target_frame_count = 0
        self._valid_roi_frame_count = 0
        self._command_count = 0
        self._max_abs_command_linear_mps = 0.0
        self._max_abs_command_angular_rps = 0.0
        self._published_nonzero_command_count = 0
        self._odom_count = 0
        self._event_count = 0
        self._ground_truth_count = 0
        self._last_auto_target_present: Optional[bool] = None
        self._last_auto_target_id: Optional[str] = None
        self._target_loss_started_unix: Optional[float] = None
        self._target_loss_started_in_crossing = False
        self._target_loss_event_count = 0
        self._target_reacquisition_event_count = 0
        self._target_id_change_count = 0
        self._crossing_target_loss_event_count = 0
        self._crossing_target_reacquisition_event_count = 0
        self._crossing_target_id_change_count = 0
        self._max_completed_crossing_target_loss_duration_s = 0.0
        self._max_crossing_target_reacquisition_stable_frames = 0
        self._crossing_reacquisition_stability_active = False
        self._max_completed_target_loss_duration_s = 0.0
        self._max_target_reacquisition_stable_frames = 0
        self._start_unix = time.time()
        self._files = []

        if not self.enabled:
            return

        try:
            self.run_dir.mkdir(parents=True, exist_ok=True)
            self._frame_file, self._frame_writer = self._open_csv("frames.csv", self.FRAME_FIELDS)
            self._command_file, self._command_writer = self._open_csv("commands.csv", self.COMMAND_FIELDS)
            self._odom_file, self._odom_writer = self._open_csv("odometry.csv", self.ODOM_FIELDS)
            self._event_file = self.run_dir / "events.jsonl"
            self._event_handle = self._event_file.open("a", encoding="utf-8", buffering=1)
            self._files.append(self._event_handle)
            self._create_run_templates()

            self.metadata: Dict[str, Any] = {
                "schema_version": 3,
                "run_id": self.run_id,
                "config_name": self.config_name,
                "scenario": self.scenario,
                "control_mode": self.control_mode,
                "started_at_utc": _utc_iso(self._start_unix),
                "host": {
                    "hostname": socket.gethostname(),
                    "platform": platform.platform(),
                    "architecture": platform.machine(),
                    "processor": _host_cpu_model(),
                    "cpu_count": os.cpu_count(),
                    "memory_total_bytes": _host_memory_total_bytes(),
                    "container_memory_limit_bytes": _container_memory_limit_bytes(),
                    "python": sys.version.split()[0],
                },
                "environment": {
                    "robot_ip": os.environ.get("ROBOT_IP", ""),
                    "ros_master_uri": os.environ.get("ROS_MASTER_URI", ""),
                    "experiment_log_root": str(self.root),
                },
            }
            if metadata:
                self.metadata.update(metadata)
            self._write_metadata()
            self.log_event("run_started", "Experiment logging started")
        except Exception as exc:  # Logging must never prevent robot startup.
            self.enabled = False
            print(f"[experiment_logger] disabled: {exc}", file=sys.stderr)

        atexit.register(self.close)

    def _open_csv(self, filename: str, fields):
        handle = (self.run_dir / filename).open("w", newline="", encoding="utf-8", buffering=1)
        self._files.append(handle)
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        return handle, writer

    def _create_run_templates(self) -> None:
        ground_truth_path = self.run_dir / "ground_truth.csv"
        if not ground_truth_path.exists():
            with ground_truth_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=self.GROUND_TRUTH_FIELDS)
                writer.writeheader()
        self._ground_truth_file = ground_truth_path.open(
            "a", newline="", encoding="utf-8", buffering=1
        )
        self._files.append(self._ground_truth_file)
        self._ground_truth_writer = csv.DictWriter(
            self._ground_truth_file, fieldnames=self.GROUND_TRUTH_FIELDS,
            extrasaction="ignore",
        )
        notes_path = self.run_dir / "operator_notes.md"
        if not notes_path.exists():
            notes_path.write_text(
                f"# Operator notes — {self.run_id}\n\n"
                f"- Configuration: `{self.config_name}`\n"
                f"- Scenario: `{self.scenario}`\n"
                f"- Control mode: `{self.control_mode}`\n\n"
                "## Notes\n\n",
                encoding="utf-8",
            )

    def _write_metadata(self) -> None:
        metadata_path = self.run_dir / "run_metadata.json"
        temporary_path = self.run_dir / "run_metadata.json.tmp"
        temporary_path.write_text(
            json.dumps(self.metadata, ensure_ascii=False, indent=2, default=str) + "\n",
            encoding="utf-8",
        )
        temporary_path.replace(metadata_path)

    def update_metadata(self, **values: Any) -> None:
        if not self.enabled:
            return
        with self._lock:
            self.metadata.update(values)
            try:
                self._write_metadata()
            except Exception as exc:
                self._handle_error(exc)

    def _handle_error(self, exc: Exception) -> None:
        print(f"[experiment_logger] write failed: {exc}", file=sys.stderr)
        self.enabled = False

    def log_event(self, event: str, message: str = "", data: Optional[Dict[str, Any]] = None) -> None:
        if not self.enabled:
            return
        record = {
            "wall_time_utc": _utc_iso(),
            "wall_time_unix": time.time(),
            "event": event,
            "message": message,
            "data": data or {},
        }
        with self._lock:
            try:
                self._event_handle.write(json.dumps(record, ensure_ascii=False, default=str) + "\n")
                self._event_count += 1
            except Exception as exc:
                self._handle_error(exc)

    def log_command(
        self,
        linear_x_mps: float,
        angular_z_rps: float,
        published: bool,
        ros_time_s: Optional[float] = None,
        source: str = "controller",
        auto_phase_id: str = "",
    ) -> None:
        if not self.enabled:
            return
        now = time.time()
        row = {
            "wall_time_utc": _utc_iso(now),
            "wall_time_unix": now,
            "ros_time_s": _as_float(ros_time_s),
            "auto_phase_id": auto_phase_id or "",
            "linear_x_mps": _as_float(linear_x_mps),
            "angular_z_rps": _as_float(angular_z_rps),
            "published": int(bool(published)),
            "control_mode": self.control_mode,
            "source": source,
        }
        with self._lock:
            try:
                self._command_writer.writerow(row)
                self._command_count += 1
                self._max_abs_command_linear_mps = max(
                    self._max_abs_command_linear_mps, abs(float(linear_x_mps))
                )
                self._max_abs_command_angular_rps = max(
                    self._max_abs_command_angular_rps, abs(float(angular_z_rps))
                )
                if published and (
                    abs(float(linear_x_mps)) > 1e-6
                    or abs(float(angular_z_rps)) > 1e-6
                ):
                    self._published_nonzero_command_count += 1
            except Exception as exc:
                self._handle_error(exc)

    def log_ground_truth(
        self,
        robot_person_separation_m: float,
        ros_time_s: Optional[float] = None,
        frame_idx: Optional[int] = None,
        reference_definition: str = "",
        source: str = "",
        quality: str = "manual_independent_measurement",
        note: str = "",
    ) -> None:
        """Persist one independently measured separation marker and its frame link."""
        if not self.enabled:
            return
        now = time.time()
        row = {
            "wall_time_utc": _utc_iso(now),
            "wall_time_unix": now,
            "ros_time_s": _as_float(ros_time_s),
            "frame_idx": "" if frame_idx is None else int(frame_idx),
            "robot_person_separation_m": _as_float(robot_person_separation_m),
            "reference_definition": reference_definition,
            "source": source,
            "quality": quality,
            "note": note,
        }
        with self._lock:
            try:
                self._ground_truth_writer.writerow(row)
                self._ground_truth_count += 1
                self._event_handle.write(json.dumps({
                    "wall_time_utc": row["wall_time_utc"],
                    "wall_time_unix": now,
                    "ros_time_s": row["ros_time_s"],
                    "event": "ground_truth_sample",
                    "message": "Operator recorded an independent distance marker",
                    "data": {
                        "frame_idx": row["frame_idx"],
                        "robot_person_separation_m": row["robot_person_separation_m"],
                        "reference_definition": reference_definition,
                        "source": source,
                        "quality": quality,
                        "note": note,
                    },
                }, ensure_ascii=False, default=str) + "\n")
                self._event_count += 1
            except Exception as exc:
                self._handle_error(exc)

    def log_odom(
        self,
        ros_time_s: Optional[float],
        odom_stamp_s: Optional[float],
        x_m: float,
        y_m: float,
        z_m: float,
        yaw_rad: float,
        linear_x_mps: float,
        linear_y_mps: float,
        linear_z_mps: float,
        angular_z_rps: float,
    ) -> None:
        if not self.enabled:
            return
        now = time.time()
        row = {
            "wall_time_utc": _utc_iso(now),
            "wall_time_unix": now,
            "ros_time_s": _as_float(ros_time_s),
            "odom_stamp_s": _as_float(odom_stamp_s),
            "x_m": _as_float(x_m),
            "y_m": _as_float(y_m),
            "z_m": _as_float(z_m),
            "yaw_rad": _as_float(yaw_rad),
            "linear_x_mps": _as_float(linear_x_mps),
            "linear_y_mps": _as_float(linear_y_mps),
            "linear_z_mps": _as_float(linear_z_mps),
            "angular_z_rps": _as_float(angular_z_rps),
        }
        self._latest_odom = {
            "odom_stamp_s": row["odom_stamp_s"],
            "odom_x_m": row["x_m"],
            "odom_y_m": row["y_m"],
            "odom_yaw_rad": row["yaw_rad"],
            "odom_linear_x_mps": row["linear_x_mps"],
            "odom_linear_y_mps": row["linear_y_mps"],
            "odom_angular_z_rps": row["angular_z_rps"],
        }
        with self._lock:
            try:
                self._odom_writer.writerow(row)
                self._odom_count += 1
            except Exception as exc:
                self._handle_error(exc)

    def progress_snapshot(self) -> Dict[str, Any]:
        """Return thread-safe counters used by the automatic capture gate."""
        with self._lock:
            return {
                "elapsed_s": max(0.0, time.time() - self._start_unix),
                "frames": self._frame_count,
                "target_frames": self._target_frame_count,
                "valid_roi_frames": self._valid_roi_frame_count,
                "commands": self._command_count,
                "max_abs_command_linear_mps": self._max_abs_command_linear_mps,
                "max_abs_command_angular_rps": self._max_abs_command_angular_rps,
                "published_nonzero_command_count": self._published_nonzero_command_count,
                "odometry": self._odom_count,
                "ground_truth_samples": self._ground_truth_count,
                "target_loss_events": self._target_loss_event_count,
                "target_reacquisition_events": self._target_reacquisition_event_count,
                "target_id_change_count": self._target_id_change_count,
                "max_completed_target_loss_duration_s": round(
                    self._max_completed_target_loss_duration_s, 4
                ),
                "max_target_reacquisition_stable_frames": self._max_target_reacquisition_stable_frames,
                "max_crossing_target_reacquisition_stable_frames": self._max_crossing_target_reacquisition_stable_frames,
                "crossing_target_loss_events": self._crossing_target_loss_event_count,
                "crossing_target_reacquisition_events": self._crossing_target_reacquisition_event_count,
                "crossing_target_id_change_count": self._crossing_target_id_change_count,
                "max_completed_crossing_target_loss_duration_s": round(
                    self._max_completed_crossing_target_loss_duration_s, 4
                ),
            }

    def log_frame(self, row: Dict[str, Any]) -> None:
        if not self.enabled:
            return
        now = time.time()
        previous_time = self._last_frame_time
        frame_period = now - previous_time if previous_time is not None else ""
        measured_fps = 1.0 / frame_period if frame_period and frame_period > 0 else ""
        self._last_frame_time = now
        full_row = {
            "wall_time_utc": _utc_iso(now),
            "wall_time_unix": now,
            "frame_period_s": _as_float(row.get("frame_period_s", frame_period)),
            "measured_fps": _as_float(row.get("measured_fps", measured_fps)),
            "control_mode": self.control_mode,
        }
        full_row.update(self._latest_odom)
        full_row.update(row)
        full_row.setdefault("wall_time_utc", _utc_iso(now))
        full_row.setdefault("wall_time_unix", now)
        full_row.setdefault("frame_period_s", _as_float(frame_period))
        full_row.setdefault("measured_fps", _as_float(measured_fps))
        full_row.setdefault("control_mode", self.control_mode)
        target_transition = None
        target_id_transition = None
        with self._lock:
            try:
                self._frame_writer.writerow(full_row)
                self._frame_count += 1
                try:
                    self._max_target_reacquisition_stable_frames = max(
                        self._max_target_reacquisition_stable_frames,
                        int(full_row.get("target_reacquisition_stable_frames", 0) or 0),
                    )
                except (TypeError, ValueError):
                    pass
                target_present = full_row.get("target_present", False)
                if isinstance(target_present, str):
                    target_present = target_present.strip().lower() in {"1", "true", "yes"}
                if target_present:
                    self._target_frame_count += 1
                    try:
                        valid_roi_count = int(full_row.get("roi_valid_count", 0) or 0)
                    except (TypeError, ValueError):
                        valid_roi_count = 0
                    if valid_roi_count > 0:
                        self._valid_roi_frame_count += 1
                if self.metadata.get("auto_sequence"):
                    target_present = bool(target_present)
                    in_crossing_action = str(full_row.get("auto_phase_id", "")) == "crossing_start"
                    previous_present = self._last_auto_target_present
                    loss_was_in_crossing = self._target_loss_started_in_crossing
                    if previous_present is not None and target_present != previous_present:
                        if target_present:
                            self._target_reacquisition_event_count += 1
                            if loss_was_in_crossing:
                                self._crossing_target_reacquisition_event_count += 1
                                self._crossing_reacquisition_stability_active = True
                                self._max_crossing_target_reacquisition_stable_frames = 0
                            if self._target_loss_started_unix is not None:
                                loss_duration = max(0.0, now - self._target_loss_started_unix)
                                self._max_completed_target_loss_duration_s = max(
                                    self._max_completed_target_loss_duration_s,
                                    loss_duration,
                                )
                                if loss_was_in_crossing:
                                    self._max_completed_crossing_target_loss_duration_s = max(
                                        self._max_completed_crossing_target_loss_duration_s,
                                        loss_duration,
                                    )
                            self._target_loss_started_unix = None
                            self._target_loss_started_in_crossing = False
                            target_transition = "target_reacquired"
                        else:
                            self._target_loss_event_count += 1
                            self._target_loss_started_unix = now
                            self._target_loss_started_in_crossing = in_crossing_action
                            if in_crossing_action or loss_was_in_crossing:
                                self._crossing_target_loss_event_count += 1
                                if self._crossing_reacquisition_stability_active or in_crossing_action:
                                    self._crossing_reacquisition_stability_active = False
                                    self._max_crossing_target_reacquisition_stable_frames = 0
                            target_transition = "target_lost"
                    if self._crossing_reacquisition_stability_active and target_present:
                        try:
                            stable_frames = int(
                                full_row.get("target_reacquisition_stable_frames", 0) or 0
                            )
                            self._max_crossing_target_reacquisition_stable_frames = max(
                                self._max_crossing_target_reacquisition_stable_frames,
                                stable_frames,
                            )
                        except (TypeError, ValueError):
                            pass
                    self._last_auto_target_present = target_present
                    current_target_id = str(full_row.get("target_id", "")).strip()
                    if target_present and current_target_id:
                        previous_target_id = self._last_auto_target_id
                        if previous_target_id and current_target_id != previous_target_id:
                            self._target_id_change_count += 1
                            if in_crossing_action:
                                self._crossing_target_id_change_count += 1
                            target_id_transition = (previous_target_id, current_target_id)
                        self._last_auto_target_id = current_target_id
            except Exception as exc:
                self._handle_error(exc)
        if target_transition:
            self.log_event(
                target_transition,
                "Tự ghi nhận chuyển trạng thái hiển thị của mục tiêu đã chọn",
                {
                    "frame_idx": full_row.get("frame_idx", ""),
                    "target_id": full_row.get("target_id", ""),
                    "auto_phase_id": full_row.get("auto_phase_id", ""),
                    "target_present": bool(target_present),
                },
            )
        if target_id_transition:
            self.log_event(
                "target_id_changed",
                "Mã mục tiêu thay đổi khi mục tiêu vẫn đang hiển thị",
                {
                    "previous_target_id": target_id_transition[0],
                    "target_id": target_id_transition[1],
                    "frame_idx": full_row.get("frame_idx", ""),
                    "auto_phase_id": full_row.get("auto_phase_id", ""),
                },
            )

    def close(self, status: str = "process_exit") -> None:
        if self._closed:
            return
        self._closed = True
        if not self.enabled:
            return
        end_time = time.time()
        try:
            self.log_event("run_finished", status)
            self.metadata.update(
                {
                    "ended_at_utc": _utc_iso(end_time),
                    "duration_s": round(end_time - self._start_unix, 3),
                    "status": status,
                    "counts": {
                        "frames": self._frame_count,
                        "commands": self._command_count,
                        "odometry": self._odom_count,
                        "events": self._event_count,
                        "ground_truth_samples": self._ground_truth_count,
                        "target_loss_events": self._target_loss_event_count,
                        "target_reacquisition_events": self._target_reacquisition_event_count,
                        "target_id_change_count": self._target_id_change_count,
                        "max_completed_target_loss_duration_s": round(
                            self._max_completed_target_loss_duration_s, 4
                        ),
                        "crossing_target_loss_events": self._crossing_target_loss_event_count,
                        "crossing_target_reacquisition_events": self._crossing_target_reacquisition_event_count,
                        "crossing_target_id_change_count": self._crossing_target_id_change_count,
                        "max_completed_crossing_target_loss_duration_s": round(
                            self._max_completed_crossing_target_loss_duration_s, 4
                        ),
                    },
                    "files": [
                        "run_metadata.json",
                        "frames.csv",
                        "commands.csv",
                        "odometry.csv",
                        "events.jsonl",
                        "ground_truth.csv",
                        "operator_notes.md",
                    ],
                }
            )
            self._write_metadata()
        except Exception as exc:
            print(f"[experiment_logger] finalization failed: {exc}", file=sys.stderr)
        finally:
            for handle in self._files:
                try:
                    handle.close()
                except Exception:
                    pass
            self.enabled = False

