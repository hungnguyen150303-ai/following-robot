#!/usr/bin/env python3
"""Combine per-run summaries into a compact comparison table."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, List


FIELDS = [
    "run_id",
    "status",
    "config_name",
    "scenario",
    "scenario_id",
    "variant",
    "repeat_id",
    "data_partition",
    "source",
    "participant_height_m",
    "walking_speed_mps",
    "crossing_angle_deg",
    "crowd_count",
    "host_platform",
    "cpu_count",
    "container_memory_limit_bytes",
    "inference_device",
    "gpu_name",
    "control_mode",
    "frame_count",
    "observed_fps",
    "person_present_rate",
    "roi_empty_rate_when_target_present",
    "mean_depth_zero_ratio_when_target_present",
    "controller_distance_rmse_m",
    "ground_truth_error_mae_m",
    "ground_truth_error_rmse_m",
    "false_backward_episode_count",
    "false_backward_duration_s",
    "false_hold_episode_count",
    "false_hold_duration_s",
    "false_command_threshold_mps",
    "false_command_fields",
    "mean_ground_truth_recovery_time_s",
    "odometry_jerk_p95_abs_mps3",
    "max_abs_controller_requested_linear_mps",
    "max_abs_controller_rate_limited_linear_mps",
    "max_abs_controller_angular_rps",
    "controller_shadow_frame_count",
    "max_target_reacquisition_stable_frames",
    "target_loss_latched_frame_count",
    "published_command_count",
    "odometry_path_length_m",
]


def load_summary(run_dir: Path) -> Dict:
    summary_path = run_dir / "summary.json"
    if not summary_path.exists():
        raise FileNotFoundError(f"missing summary.json; run analyze_run.py first: {run_dir}")
    return json.loads(summary_path.read_text(encoding="utf-8"))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dirs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=Path("experiment_comparison.csv"))
    args = parser.parse_args()

    all_summaries: List[Dict] = [load_summary(path.resolve()) for path in args.run_dirs]
    invalidated = [
        summary for summary in all_summaries
        if summary.get("status") == "operator_invalidated"
    ]
    summaries = [summary for summary in all_summaries if summary not in invalidated]
    if invalidated:
        print(f"Excluded {len(invalidated)} operator-invalidated run(s).")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        for summary in summaries:
            row = {field: summary.get(field, "") for field in FIELDS}
            if isinstance(row.get("false_command_fields"), list):
                row["false_command_fields"] = ";".join(row["false_command_fields"])
            writer.writerow(row)

    markdown_path = args.output.with_suffix(".md")
    lines = [
        "| " + " | ".join(FIELDS) + " |",
        "| " + " | ".join("---" for _ in FIELDS) + " |",
    ]
    for summary in summaries:
        lines.append("| " + " | ".join(str(summary.get(field, "")) for field in FIELDS) + " |")
    markdown_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {args.output} and {markdown_path}")


if __name__ == "__main__":
    main()

