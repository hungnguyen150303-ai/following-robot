#!/usr/bin/env python3
"""Create a compact, paper-ready summary from one experiment run directory."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


def rows(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def jsonl_rows(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    parsed = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            try:
                value = json.loads(line)
            except (ValueError, TypeError):
                continue
            if isinstance(value, dict):
                parsed.append(value)
    return parsed


def number(value: Any) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def truth(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def mean(values: Iterable[Optional[float]]) -> Optional[float]:
    numbers = [value for value in values if value is not None]
    return sum(numbers) / len(numbers) if numbers else None


def rmse(values: Iterable[Optional[float]]) -> Optional[float]:
    numbers = [value for value in values if value is not None]
    return math.sqrt(sum(value * value for value in numbers) / len(numbers)) if numbers else None


def round_value(value: Any, digits: int = 4) -> Any:
    return round(value, digits) if isinstance(value, (int, float)) else value


def percentile(values: Iterable[float], fraction: float) -> Optional[float]:
    ordered = sorted(values)
    if not ordered:
        return None
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def jerk_summary(rows_in: List[Dict[str, str]], velocity_field: str) -> Dict[str, Any]:
    """Estimate linear jerk from timestamped velocity samples; skip large gaps."""
    samples = []
    for row in rows_in:
        timestamp = number(row.get("wall_time_unix"))
        velocity = number(row.get(velocity_field))
        if timestamp is not None and velocity is not None:
            samples.append((timestamp, velocity))
    samples.sort(key=lambda item: item[0])
    accelerations = []
    for (time_a, velocity_a), (time_b, velocity_b) in zip(samples, samples[1:]):
        delta = time_b - time_a
        if 0 < delta <= 0.5:
            accelerations.append(((time_a + time_b) / 2.0, (velocity_b - velocity_a) / delta))
    jerks = []
    for (time_a, accel_a), (time_b, accel_b) in zip(accelerations, accelerations[1:]):
        delta = time_b - time_a
        if 0 < delta <= 0.5:
            jerks.append((accel_b - accel_a) / delta)
    absolute_jerks = [abs(value) for value in jerks]
    return {
        "sample_count": len(jerks),
        "mean_abs_mps3": round_value(mean(absolute_jerks)),
        "p95_abs_mps3": round_value(percentile(absolute_jerks, 0.95)),
        "max_abs_mps3": round_value(max(absolute_jerks, default=None)),
    }


def false_command_metrics(
    frame_rows: List[Dict[str, str]], setpoint_m: float, tolerance_m: float,
    command_threshold_mps: float = 0.05,
    command_field: Optional[str] = None,
) -> Dict[str, Any]:
    """Measure false reverse/hold episodes and durations on dense camera-range GT."""
    samples = []
    selected_fields = set()
    for row in frame_rows:
        timestamp = number(row.get("wall_time_unix"))
        ground_truth = number(row.get("ground_truth_distance_m"))
        selected_field = command_field
        if not selected_field:
            selected_field = (
                "controller_rate_limited_linear_x_mps"
                if truth(row.get("controller_shadow_active"))
                else "command_linear_x_mps"
            )
        command = number(row.get(selected_field))
        if timestamp is None or ground_truth is None or command is None:
            continue
        if not truth(row.get("target_present")):
            continue
        roi_valid = number(row.get("roi_valid_count"))
        roi_empty = truth(row.get("roi_empty"))
        false_backward = ground_truth > setpoint_m + tolerance_m and command < -command_threshold_mps
        false_hold = (
            abs(ground_truth - setpoint_m) > tolerance_m
            and abs(command) <= command_threshold_mps
            and truth(row.get("distance_sample_valid"))
            and not roi_empty
            and roi_valid is not None
            and roi_valid > 0
        )
        samples.append((timestamp, false_backward, false_hold))
        selected_fields.add(selected_field)
    samples.sort(key=lambda item: item[0])

    backward_duration = 0.0
    hold_duration = 0.0
    backward_episodes = 0
    hold_episodes = 0
    prior_backward = False
    prior_hold = False
    for previous, current in zip(samples, samples[1:]):
        delta = current[0] - previous[0]
        if delta <= 0 or delta > 0.5:
            prior_backward = False
            prior_hold = False
            continue
        backward, hold = previous[1], previous[2]
        if backward:
            backward_duration += delta
            if not prior_backward:
                backward_episodes += 1
        if hold:
            hold_duration += delta
            if not prior_hold:
                hold_episodes += 1
        prior_backward = backward
        prior_hold = hold
    return {
        "continuous_ground_truth_sample_count": len(samples),
        "false_backward_episode_count": backward_episodes,
        "false_backward_duration_s": round_value(backward_duration),
        "false_hold_episode_count": hold_episodes,
        "false_hold_duration_s": round_value(hold_duration),
        "command_threshold_mps": command_threshold_mps,
        "command_fields": sorted(selected_fields),
    }


def recovery_times(
    frame_rows: List[Dict[str, str]], event_rows: List[Dict[str, Any]],
    setpoint_m: float, tolerance_m: float, stable_duration_s: float = 0.5,
) -> List[float]:
    """Time from crossing_end to a sustained return of independent range to band."""
    samples = []
    for row in frame_rows:
        timestamp = number(row.get("wall_time_unix"))
        distance = number(row.get("ground_truth_distance_m"))
        if timestamp is not None and distance is not None:
            samples.append((timestamp, distance))
    samples.sort(key=lambda item: item[0])
    recoveries = []
    end_events = [
        number(row.get("wall_time_unix"))
        for row in event_rows if row.get("event") == "crossing_end"
    ]
    for event_time in (value for value in end_events if value is not None):
        stable_start = None
        previous_time = None
        for timestamp, distance in samples:
            if timestamp < event_time:
                continue
            in_band = abs(distance - setpoint_m) <= tolerance_m
            if not in_band or (previous_time is not None and timestamp - previous_time > 0.5):
                stable_start = timestamp if in_band else None
            elif stable_start is None:
                stable_start = timestamp
            previous_time = timestamp
            if stable_start is not None and timestamp - stable_start >= stable_duration_s:
                recoveries.append(timestamp - event_time)
                break
    return recoveries


def target_visibility_metrics(frame_rows: List[Dict[str, str]]) -> Dict[str, Any]:
    """Measure selected-target dropouts and ID changes from timestamped frames."""
    samples = []
    visible_ids = []
    for row in frame_rows:
        timestamp = number(row.get("wall_time_unix"))
        if timestamp is None:
            continue
        present = truth(row.get("target_present"))
        samples.append((timestamp, present))
        if present:
            target_id = str(row.get("target_id", "")).strip()
            if target_id:
                visible_ids.append(target_id)
    samples.sort(key=lambda item: item[0])

    loss_count = 0
    reacquisition_count = 0
    completed_loss_durations = []
    previous_time = None
    previous_present = None
    loss_start = None
    for timestamp, present in samples:
        if previous_time is None:
            previous_time = timestamp
            previous_present = present
            continue
        delta = timestamp - previous_time
        if delta <= 0:
            continue
        if delta > 0.5:
            # Do not infer a visibility transition through an unobserved time gap.
            previous_time = timestamp
            previous_present = present
            loss_start = timestamp if not present else None
            continue
        if previous_present and not present:
            loss_count += 1
            loss_start = timestamp
        elif not previous_present and present:
            reacquisition_count += 1
            if loss_start is not None:
                completed_loss_durations.append(max(0.0, timestamp - loss_start))
            loss_start = None
        previous_time = timestamp
        previous_present = present

    id_changes = sum(1 for first, second in zip(visible_ids, visible_ids[1:]) if first != second)
    return {
        "target_lost_event_count": loss_count,
        "target_reacquired_event_count": reacquisition_count,
        "mean_target_loss_duration_s": round_value(mean(completed_loss_durations)),
        "max_target_loss_duration_s": round_value(max(completed_loss_durations, default=None)),
        "target_id_change_count": id_changes,
        "target_id_unique_count": len(set(visible_ids)),
    }


def summarize(run_dir: Path, false_command_field: Optional[str] = None) -> Dict[str, Any]:
    annotated_frames = run_dir / "frames_annotated.csv"
    frame_rows = rows(annotated_frames if annotated_frames.exists() else run_dir / "frames.csv")
    command_rows = rows(run_dir / "commands.csv")
    odom_rows = rows(run_dir / "odometry.csv")
    ground_truth_rows = rows(run_dir / "ground_truth.csv")
    event_rows = jsonl_rows(run_dir / "events.jsonl")
    metadata = {}
    metadata_path = run_dir / "run_metadata.json"
    if metadata_path.exists():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))

    wall_times = [number(row.get("wall_time_unix")) for row in frame_rows]
    wall_times = [value for value in wall_times if value is not None]
    duration_s = wall_times[-1] - wall_times[0] if len(wall_times) >= 2 else None
    observed_fps = (len(wall_times) - 1) / duration_s if duration_s and duration_s > 0 else None

    target_rows = [row for row in frame_rows if truth(row.get("target_present"))]
    target_visibility = target_visibility_metrics(frame_rows)
    roi_empty_rows = [row for row in target_rows if truth(row.get("roi_empty"))]
    distance_errors = [number(row.get("distance_error_m")) for row in target_rows]
    ground_truth_errors = []
    for row in target_rows:
        error = number(row.get("distance_error_to_ground_truth_m"))
        if error is None:
            camera_reference = number(row.get("ground_truth_distance_m"))
            camera_estimate = number(row.get("camera_depth_smoothed_m"))
            if camera_estimate is None:
                camera_estimate = number(row.get("distance_smoothed_m"))
            if camera_reference is not None and camera_estimate is not None:
                error = camera_estimate - camera_reference
            else:
                front_reference = number(row.get("ground_truth_front_gap_m"))
                front_estimate = number(row.get("front_gap_smoothed_m"))
                if front_reference is not None and front_estimate is not None:
                    error = front_estimate - front_reference
        ground_truth_errors.append(error)
    smoothed_distances = [number(row.get("distance_smoothed_m")) for row in target_rows]
    zero_ratios = [number(row.get("depth_zero_ratio")) for row in target_rows]
    fps_values = [number(row.get("measured_fps")) for row in frame_rows]

    frames_by_index = {}
    for frame in frame_rows:
        frame_idx = number(frame.get("frame_idx"))
        if frame_idx is not None:
            frames_by_index[int(frame_idx)] = frame
    marker_errors = []
    marker_definition_counts = Counter()
    matched_marker_count = 0
    for marker in ground_truth_rows:
        definition = marker.get("reference_definition", "")
        marker_definition_counts[definition or "unspecified"] += 1
        frame_idx = number(marker.get("frame_idx"))
        measured = number(
            marker.get("robot_person_separation_m")
            or marker.get("front_edge_gap_m")
        )
        if frame_idx is None or measured is None:
            continue
        frame = frames_by_index.get(int(frame_idx))
        if frame is None:
            continue
        if definition == "camera_to_person_roi":
            estimate = number(frame.get("camera_depth_smoothed_m"))
            if estimate is None:
                estimate = number(frame.get("distance_smoothed_m"))
        elif definition == "robot_front_edge_to_person":
            estimate = number(frame.get("front_gap_smoothed_m"))
        else:
            estimate = None
        if estimate is not None:
            marker_errors.append(estimate - measured)
            matched_marker_count += 1

    # Prefer dense frame annotations when present. Otherwise report only the
    # explicitly paired manual markers; never interpolate sparse points into a
    # continuous ground-truth trajectory.
    annotated_error_values = [value for value in ground_truth_errors if value is not None]
    selected_ground_truth_errors = annotated_error_values or marker_errors

    gate_counts = Counter(row.get("gate_state", "") for row in frame_rows)
    command_speeds = [number(row.get("linear_x_mps")) for row in command_rows]
    command_speeds = [value for value in command_speeds if value is not None]
    command_angular = [number(row.get("angular_z_rps")) for row in command_rows]
    command_angular = [value for value in command_angular if value is not None]
    controller_requested = [
        number(row.get("controller_requested_linear_x_mps")) for row in frame_rows
    ]
    controller_requested = [value for value in controller_requested if value is not None]
    controller_rate_limited = [
        number(row.get("controller_rate_limited_linear_x_mps")) for row in frame_rows
    ]
    controller_rate_limited = [value for value in controller_rate_limited if value is not None]
    controller_angular = [
        number(row.get("controller_angular_z_rps")) for row in frame_rows
    ]
    controller_angular = [value for value in controller_angular if value is not None]
    controller_shadow_frames = sum(
        1 for row in frame_rows if truth(row.get("controller_shadow_active"))
    )
    reacquisition_stability = [
        number(row.get("target_reacquisition_stable_frames")) for row in frame_rows
    ]
    reacquisition_stability = [value for value in reacquisition_stability if value is not None]
    target_loss_latched_frames = sum(
        1 for row in frame_rows if truth(row.get("target_loss_latched"))
    )

    odom_path = 0.0
    previous_xy = None
    for row in odom_rows:
        x = number(row.get("x_m"))
        y = number(row.get("y_m"))
        if x is None or y is None:
            continue
        if previous_xy is not None:
            odom_path += math.hypot(x - previous_xy[0], y - previous_xy[1])
        previous_xy = (x, y)

    config_name = metadata.get("config_name", "unknown")
    scenario = metadata.get("scenario", "unknown")
    controller_parameters = metadata.get("controller_parameters", {})
    host_metadata = metadata.get("host", {})
    runtime_metadata = metadata.get("runtime", {})
    camera_metadata = metadata.get("camera", {})
    rgbd_metadata = metadata.get("rgbd_recording", {})
    setpoint_m = number(controller_parameters.get("desired_camera_distance_m")) or 1.4
    tolerance_m = number(controller_parameters.get("distance_tolerance_m")) or 0.03
    ground_truth_definition = metadata.get(
        "ground_truth_definition_planned", "camera_to_person_roi"
    )
    continuous_ground_truth_frames = sum(
        1 for row in frame_rows if number(row.get("ground_truth_distance_m")) is not None
    )
    continuous_ground_truth_coverage = (
        continuous_ground_truth_frames / len(frame_rows) if frame_rows else 0.0
    )
    has_camera_ground_truth = (
        ground_truth_definition == "camera_to_person_roi"
        and continuous_ground_truth_coverage >= 0.8
    )
    false_metrics = (
        false_command_metrics(
            frame_rows, setpoint_m, tolerance_m, command_field=false_command_field
        )
        if has_camera_ground_truth else {
            "continuous_ground_truth_sample_count": 0,
            "false_backward_episode_count": None,
            "false_backward_duration_s": None,
            "false_hold_episode_count": None,
            "false_hold_duration_s": None,
            "command_threshold_mps": 0.05,
            "command_fields": [],
        }
    )
    recovery_values = (
        recovery_times(frame_rows, event_rows, setpoint_m, tolerance_m)
        if has_camera_ground_truth else []
    )
    command_jerk = jerk_summary(command_rows, "linear_x_mps")
    odometry_jerk = jerk_summary(odom_rows, "linear_x_mps")
    return {
        "run_id": metadata.get("run_id", run_dir.name),
        "config_name": config_name,
        "scenario": scenario,
        "scenario_id": metadata.get("scenario_id", ""),
        "variant": metadata.get("scenario_variant", ""),
        "repeat_id": metadata.get("repeat_id", ""),
        "data_partition": metadata.get("data_partition", "unspecified"),
        "source": metadata.get("source", "unknown"),
        "participant_height_m": metadata.get("participant_height_m", ""),
        "walking_speed_mps": metadata.get("walking_speed_mps", ""),
        "crossing_angle_deg": metadata.get("crossing_angle_deg", ""),
        "crowd_count": metadata.get("crowd_count", ""),
        "robot_model": metadata.get("robot_model", ""),
        "host_platform": host_metadata.get("platform", ""),
        "cpu_model": host_metadata.get("processor", ""),
        "cpu_count": host_metadata.get("cpu_count", ""),
        "memory_reported_bytes": host_metadata.get("memory_total_bytes", ""),
        "container_memory_limit_bytes": host_metadata.get("container_memory_limit_bytes", ""),
        "inference_device": runtime_metadata.get("inference_device", ""),
        "gpu_name": runtime_metadata.get("gpu_name", ""),
        "gpu_total_memory_bytes": runtime_metadata.get("gpu_total_memory_bytes", ""),
        "camera_model": camera_metadata.get("name", ""),
        "camera_firmware": camera_metadata.get("firmware", ""),
        "camera_height_m": camera_metadata.get("height_m", ""),
        "camera_pitch_down_deg": camera_metadata.get("pitch_down_deg", ""),
        "robot_length_m": camera_metadata.get("robot_length_m", ""),
        "rgbd_recording_enabled": rgbd_metadata.get("enabled", False),
        "rgbd_recording_path": rgbd_metadata.get("path", ""),
        "rgbd_segment_start_unix": metadata.get("rgbd_segment_start_unix", ""),
        "rgbd_segment_end_unix": metadata.get("rgbd_segment_end_unix", ""),
        "control_mode": metadata.get("control_mode", "unknown"),
        "status": metadata.get("status", "unknown"),
        "frame_count": len(frame_rows),
        "duration_s": round_value(duration_s),
        "observed_fps": round_value(observed_fps),
        "mean_logged_fps": round_value(mean(fps_values)),
        "person_present_rate": round_value(len(target_rows) / len(frame_rows) if frame_rows else None),
        **target_visibility,
        "roi_empty_rate_when_target_present": round_value(
            len(roi_empty_rows) / len(target_rows) if target_rows else None
        ),
        "mean_depth_zero_ratio_when_target_present": round_value(mean(zero_ratios)),
        "mean_smoothed_distance_m": round_value(mean(smoothed_distances)),
        "mean_controller_distance_error_m": round_value(mean(distance_errors)),
        "controller_distance_rmse_m": round_value(rmse(distance_errors)),
        "ground_truth_error_mae_m": round_value(mean(abs(value) for value in selected_ground_truth_errors)),
        "ground_truth_error_rmse_m": round_value(rmse(selected_ground_truth_errors)),
        "ground_truth_samples": len(selected_ground_truth_errors),
        "ground_truth_manual_marker_count": len(ground_truth_rows),
        "ground_truth_matched_marker_count": matched_marker_count,
        "ground_truth_marker_definitions": dict(marker_definition_counts),
        "ground_truth_definition": ground_truth_definition,
        "continuous_ground_truth_frames": continuous_ground_truth_frames,
        "continuous_ground_truth_coverage": round_value(continuous_ground_truth_coverage),
        "continuous_camera_ground_truth_available": has_camera_ground_truth,
        "false_backward_episode_count": false_metrics["false_backward_episode_count"],
        "false_backward_duration_s": false_metrics["false_backward_duration_s"],
        "false_hold_episode_count": false_metrics["false_hold_episode_count"],
        "false_hold_duration_s": false_metrics["false_hold_duration_s"],
        "false_command_fields": false_metrics["command_fields"],
        "false_command_threshold_mps": false_metrics["command_threshold_mps"],
        "ground_truth_recovery_samples": len(recovery_values),
        "mean_ground_truth_recovery_time_s": round_value(mean(recovery_values)),
        "p95_ground_truth_recovery_time_s": round_value(percentile(recovery_values, 0.95)),
        "command_jerk": command_jerk,
        "odometry_jerk": odometry_jerk,
        "command_jerk_p95_abs_mps3": command_jerk["p95_abs_mps3"],
        "odometry_jerk_p95_abs_mps3": odometry_jerk["p95_abs_mps3"],
        "metric_definitions": {
            "false_backward": "Có người trong khung, khoảng cách camera độc lập lớn hơn mốc điều khiển cộng dung sai, lệnh vận tốc dọc nhỏ hơn -0,05 m/s.",
            "false_hold": "Có người trong khung, mẫu distance_sample_valid và ROI không rỗng, khoảng cách camera độc lập ngoài vùng dung sai và trị tuyệt đối lệnh dọc không quá 0,05 m/s.",
            "false_command_duration": "Cộng thời lượng theo các khoảng khung hình liền nhau tối đa 0,5 giây; bỏ qua khoảng mất dữ liệu dài hơn.",
            "recovery_time": "Từ mốc kết thúc cắt ngang đến khi khoảng cách camera độc lập nằm trong vùng dung sai liên tục ít nhất 0,5 giây.",
            "jerk": "Đạo hàm bậc hai của vận tốc lệnh hoặc odometry có dấu thời gian; bỏ qua khoảng lấy mẫu trên 0,5 giây.",
            "continuous_ground_truth_required_coverage": "Ít nhất 80% số khung hình phải có khoảng cách camera độc lập đã gán nhãn để tính các chỉ số động.",
        },
        "gate_state_counts": dict(gate_counts),
        "command_count": len(command_rows),
        "published_command_count": sum(1 for row in command_rows if truth(row.get("published"))),
        "max_abs_linear_command_mps": round_value(max((abs(value) for value in command_speeds), default=None)),
        "max_abs_angular_command_rps": round_value(max((abs(value) for value in command_angular), default=None)),
        "max_abs_controller_requested_linear_mps": round_value(
            max((abs(value) for value in controller_requested), default=None)
        ),
        "max_abs_controller_rate_limited_linear_mps": round_value(
            max((abs(value) for value in controller_rate_limited), default=None)
        ),
        "max_abs_controller_angular_rps": round_value(
            max((abs(value) for value in controller_angular), default=None)
        ),
        "controller_shadow_frame_count": controller_shadow_frames,
        "max_target_reacquisition_stable_frames": int(max(reacquisition_stability, default=0)),
        "target_loss_latched_frame_count": target_loss_latched_frames,
        "odometry_sample_count": len(odom_rows),
        "odometry_path_length_m": round_value(odom_path),
        "data_quality": {
            "frames_file_present": (run_dir / "frames.csv").exists(),
            "frames_source": annotated_frames.name if annotated_frames.exists() else "frames.csv",
            "commands_file_present": (run_dir / "commands.csv").exists(),
            "odometry_file_present": (run_dir / "odometry.csv").exists(),
            "events_file_present": (run_dir / "events.jsonl").exists(),
            "ground_truth_template_present": (run_dir / "ground_truth.csv").exists(),
            "ground_truth_manual_samples_present": bool(ground_truth_rows),
            "operator_notes_present": (run_dir / "operator_notes.md").exists(),
            "ground_truth_error_available": bool(selected_ground_truth_errors),
            "ground_truth_dense_frame_annotations_present": bool(annotated_error_values),
            "continuous_camera_ground_truth_available": has_camera_ground_truth,
        },
        "interpretation_note": (
            "Các mốc khoảng cách nhập tay là phép đo thưa. Lệnh lùi không phù hợp, thời gian hồi phục, "
            "độ giật và sai số liên tục cần chuỗi khoảng cách chuẩn độc lập đã đồng bộ; nhãn dày trong "
            "frames_annotated.csv được ưu tiên hơn các mốc rời rạc."
        ),
    }


def write_outputs(run_dir: Path, summary: Dict[str, Any]) -> None:
    (run_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    flat = {
        key: value
        for key, value in summary.items()
        if not isinstance(value, (dict, list))
    }
    flat["gate_state_counts_json"] = json.dumps(summary["gate_state_counts"], ensure_ascii=False, sort_keys=True)
    flat["ground_truth_marker_definitions_json"] = json.dumps(
        summary["ground_truth_marker_definitions"], ensure_ascii=False, sort_keys=True
    )
    flat["false_command_fields"] = ",".join(summary["false_command_fields"])
    flat["metric_definitions_json"] = json.dumps(
        summary["metric_definitions"], ensure_ascii=False, sort_keys=True
    )
    with (run_dir / "paper_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(flat.keys()))
        writer.writeheader()
        writer.writerow(flat)

    lines = [
        f"# Tóm tắt lượt đo: `{summary['run_id']}`",
        "",
        f"- Cấu hình: `{summary['config_name']}`",
        f"- Phần thực nghiệm: `{summary['scenario']}`",
        f"- Phân vùng dữ liệu: `{summary['data_partition']}`",
        f"- Chế độ điều khiển: `{summary['control_mode']}`",
        f"- Khung hình: {summary['frame_count']} (tốc độ quan sát {summary['observed_fps']} khung/giây)",
        f"- Tỷ lệ có người mục tiêu: {summary['person_present_rate']}",
        f"- Mất dấu / nhận lại mục tiêu: {summary['target_lost_event_count']} / {summary['target_reacquired_event_count']} lần",
        f"- Thời lượng mất dấu trung bình / dài nhất: {summary['mean_target_loss_duration_s']} / {summary['max_target_loss_duration_s']} giây",
        f"- Số lần đổi ID giữa các khung hình có mục tiêu: {summary['target_id_change_count']} (số ID riêng: {summary['target_id_unique_count']})",
        f"- Tỷ lệ vùng đo rỗng khi có người: {summary['roi_empty_rate_when_target_present']}",
        f"- Tỷ lệ độ sâu bằng 0 trung bình khi có người: {summary['mean_depth_zero_ratio_when_target_present']}",
        f"- RMSE khoảng cách camera–người so với mốc điều khiển 1,40 m: {summary['controller_distance_rmse_m']} m",
        f"- Mẫu khoảng cách chuẩn độc lập: {summary['ground_truth_samples']} "
        f"(mốc nhập tay ghép được với khung hình: {summary['ground_truth_matched_marker_count']})",
        f"- RMSE so với khoảng cách chuẩn: {summary['ground_truth_error_rmse_m']} m",
        f"- Lần lùi không phù hợp / thời lượng: {summary['false_backward_episode_count']} / {summary['false_backward_duration_s']} giây",
        f"- Lần giữ sai / thời lượng: {summary['false_hold_episode_count']} / {summary['false_hold_duration_s']} giây",
        f"- Trường lệnh dùng tính false command: {', '.join(summary['false_command_fields']) or 'chưa có ground truth dày'}",
        f"- Thời gian hồi phục trung bình sau cắt ngang: {summary['mean_ground_truth_recovery_time_s']} giây",
        f"- Phân vị 95% độ giật odometry: {summary['odometry_jerk']['p95_abs_mps3']} m/s³",
        f"- Vận tốc P yêu cầu / sau giới hạn gia tốc lớn nhất: {summary['max_abs_controller_requested_linear_mps']} / {summary['max_abs_controller_rate_limited_linear_mps']} m/s",
        f"- Khung hình có lệnh giả lập khi robot bị khóa: {summary['controller_shadow_frame_count']}",
        f"- Khung hình đang chốt dừng sau mất dấu / số khung hình nhận lại liên tiếp tối đa: {summary['target_loss_latched_frame_count']} / {summary['max_target_reacquisition_stable_frames']}",
        f"- Lệnh đã phát / tổng số lệnh: {summary['published_command_count']} / {summary['command_count']}",
        f"- Quãng đường odometry: {summary['odometry_path_length_m']} m",
        "",
        "Số khung hình theo trạng thái điều khiển:",
        "",
    ]
    for key, value in sorted(summary["gate_state_counts"].items()):
        lines.append(f"- `{key}`: {value}")
    lines.extend(["", summary["interpretation_note"], ""])
    (run_dir / "paper_summary.md").write_text("\n".join(lines), encoding="utf-8")

    metadata_path = run_dir / "run_metadata.json"
    if metadata_path.exists():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        analysis_files = ["summary.json", "paper_summary.csv", "paper_summary.md"]
        metadata["analysis_outputs"] = analysis_files
        metadata["files"] = list(dict.fromkeys(metadata.get("files", []) + analysis_files))
        temporary_path = run_dir / "run_metadata.json.tmp"
        temporary_path.write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        temporary_path.replace(metadata_path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, help="Path to one directory under experiments/logs")
    parser.add_argument(
        "--false-command-field",
        help=(
            "Explicit velocity column for annotated/replayed comparisons. "
            "By default, shadow runs use controller_rate_limited_linear_x_mps and live runs use command_linear_x_mps."
        ),
    )
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    if not run_dir.is_dir():
        parser.error(f"run directory does not exist: {run_dir}")
    summary = summarize(run_dir, false_command_field=args.false_command_field)
    write_outputs(run_dir, summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

