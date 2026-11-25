# E1 logging and simulation–real comparison

## 1. Audit of the current implementation

The Docker application is a complete operational follower for the current
workflow: the image contains ROS Noetic, the MiR driver packages, the YOLO
model, the `buffalo_s` InsightFace dependency and both alert sounds. The
application connects to the MiR bridge through ROS and sends velocity commands
on `/cmd_vel`. The saved runtime output also shows the bridge publishing
`/odom` and subscribing to `cmd_vel`.

The original runtime, however, was not an experiment logger. It exposed
startup and error text through Docker stdout and ROS logs only. It did not
write a frame-level table, a command table, an odometry table, a run manifest,
or a synchronized ground-truth channel. The old `logs.sh` filters are useful
for interactive debugging, but they cannot produce quantitative E1 metrics;
several filters also look for labels that the current `main.py` does not
systematically print.

The application also opens the D435 directly with `pyrealsense2` rather than
subscribing to RGB-D ROS topics. A normal ROS bag therefore does not contain the
camera stream. The application can optionally record a RealSense session file;
the separate ROS bag records robot state and command topics. The CSV logger
records frame number, color/depth timestamps and alignment status either way.

## 2. Logging method used for E1

Each launch creates a unique directory under `experiments/logs/<run_id>`. The
run manifest stores the configuration label, scenario, control mode, robot IP,
ROS master, camera request, inference device and controller constants. The
following files are written during execution:

- `frames.csv`: one row after RGB-D alignment, person detection, ROI depth
  processing, filtering, gate evaluation and command generation. It includes
  timestamps, FPS, number of people, target ID and bounding box, raw/nonzero
  ROI statistics, zero-depth ratio, filtered camera depth, deprojected target
  geometry, front-edge gap history, gate state and the command associated with
  that frame.
- `commands.csv`: every generated velocity command, including whether it was
  actually published. This makes the dry-run safety check auditable.
- `odometry.csv`: every received `/odom` message, including pose, yaw and
  velocity.
- `events.jsonl`: startup, camera, ROS and frame errors, run markers and manual
  independent-distance measurements.
- `ros_topics.bag`: optional recording of `/odom`, `/cmd_vel`, TF and MiR state.
- `_sessions/*.bag`: optional raw RealSense color/depth recording shared by the
  scenario runs in one application session.

The logger is line-oriented and flushes rows as they are written. Therefore an
interrupted run normally retains the data collected before the interruption.
The D435 range is never used as its own ground truth. Sparse measurements entered
in the GUI are timestamped in `ground_truth.csv`; continuous dynamic metrics need
an independent synchronized trace annotated in a copy of `frames.csv`.

The **E1 — Hướng dẫn và đo** tab now runs a timed queue. After the operator
registers the target on the camera, the interface starts each run, displays
timed movement cues, records phase markers, checks log coverage, retries an
incomplete capture up to three times, and advances to the next run. Each run
still gets an isolated directory and summary. A stop-session control publishes
zero velocity. The current C1 queue selects `c1_input_capture`, which is only a
capture profile; it is not any of the three offline comparison configurations.
`p_tapered` is the current shadow output. C1 and the historical zero-fallback
baseline still need exact implementations before paired replay.

The automatic gate checks technical log coverage (frame rate, command samples,
odometry, target presence and valid ROI depth). Passing this gate does not make
the run paper-ready: continuous independent camera-to-person ground truth is
still required for distance error, false-command and reaction-time metrics.

## 3. Safe pilot and experiment protocol

For every configuration and scenario, first run with
`MIR_CONTROL_MODE=dry_run`. The application may observe the real MiR bridge and
record odometry, but it suppresses both velocity publication and `move_base`
goals; it must show `published=0` in `commands.csv`. A live run is allowed only
after this check and a physical safety check.

The revised C1 collection sequence is specified in
[`E1_data_collection_plan.md`](E1_data_collection_plan.md). It schedules six
development source recordings (two per condition), then 15 held-out source
recordings (five per condition, arranged as five blocks). In both crossing
branches the target stands at 2.20 m and a second person crosses in front;
direction is balanced. MiR remains motion-locked. The capture configuration
stores raw RGB-D and the current P shadow output only; C1 is not implemented or
evaluated in this capture pass. The UI opens held-out collection only after all
six development captures pass the technical gate and the operator records a
frozen C1 version identifier.

Replay each of the 15 held-out recordings through the same three controller
profiles to produce 45 planned paired outputs, after the C1 rule and historical
zero-fallback baseline have been specified and implemented. These are planned
outputs; the capture UI does not execute replay. Keep each trial paired across profiles. An external synchronized
range trace is required for false-backward duration and valid-approach false
hold metrics. If it is unavailable, report sensor/tracking observations only.
The UI's technical log-quality gate does not confirm that the intended
occlusion occurred or provide independent ground truth.

The current `p_tapered` profile uses a 1.40 m setpoint measured from the
RealSense optical centre to the target ROI point, ±0.03 m tolerance and up to
five valid distance samples. Its proportional gain is 0.75 (m/s)/m, forward
speed is capped at 0.35 m/s, reverse speed at 0.25 m/s, acceleration at
0.35 m/s² and deceleration at 0.65 m/s². Thus the requested speed falls as the
distance error shrinks: errors of 0.40, 0.20 and 0.10 m request 0.30, 0.15 and
0.075 m/s, respectively. Inside `[1.37, 1.43] m`, invalid depth, or target
loss, the command is zero immediately. After target loss, follow control stays
latched until the registered face and valid ROI are stable for 10 consecutive
frames. The legacy `p_original` profile keeps its 0.90 m/s cap for manual
comparison and is not automatically queued.
The camera is mounted at the rear edge centreline, at 1.80 m height and 30
degrees downward; the nominal MiR100 body length used for the additional
front-edge-gap log is 0.89 m. The command clipping limit remains 1.8 m/s in the
low-level function and is recorded separately. This distinction should be
stated in the paper.

## 4. Metrics available for the paper

The generated summaries provide:

- observed frame rate and target-present rate;
- target-loss/reacquisition counts and durations, plus target-ID changes;
- ROI-empty rate and depth-zero ratio while the target is present;
- mean/r.m.s. error of the camera-to-target distance relative to the 1.40 m
  controller setpoint;
- error at manually measured points or after dense independent annotation;
- counts of `forward`, `backward`, `hold`, `invalid_depth`, `stop` and
  `no_target` gate states;
- number of commands, number actually published, actual command limits and
  odometry path length;
- requested and rate-limited controller velocity, plus the number of frames
  with shadow commands during locked crossing runs.

For crossing runs, shadow commands describe what the current controller would
request from the selected target. They are not commands sent to the robot and
must not be presented as measured MiR motion. If tracking is lost, the current
control path requests zero and waits for stable reacquisition; the target must
not be replaced by the helper. The physical robot remains stopped during C1
capture. C1's own decision cannot be inferred from these shadow commands; it
must be generated by the later paired-replay implementation.

The number of false backward commands and reaction time should be computed only
after continuous ground-truth annotation. Sparse manual points are not enough.
A false backward command is counted when synchronized ground truth is above
1.43 m and the paired replay command is below −0.05 m/s. Reaction time is
measured from the first annotated crossing of a gate threshold to the first
corresponding nonzero command. These
definitions avoid treating odometry as a distance-to-person reference.

## 5. Simulation versus real robot

The current application is not yet simulation-ready as a drop-in comparison:
it starts a physical `pyrealsense2` pipeline, the entrypoint launches the MiR
driver, and the controller publishes to the real `/cmd_vel` topic. A Gazebo
run would need a sensor/actuator boundary with the same logger interface:

```text
RGB-D source (D435 or simulated/replayed) -> perception/filter/controller
                                      -> command sink (MiR or simulated base)
                                      -> identical frames/commands/odom logs
```

For a fair comparison, keep the YOLO model, face model, ROI calculation,
filter, gate parameters and controller code fixed. Change only the sensor
backend and the command/odometry backend. Use the same run schema and the same
scenario definitions in both conditions.

| Aspect | Simulation | Real MiR + D435 |
|---|---|---|
| RGB-D availability | Usually continuous and timestamp-clean | USB drops, missing depth and alignment errors can occur |
| Person motion | Scriptable and repeatable | Human motion and occlusion vary |
| Robot response | Idealized or model-dependent | Network, safety limits and floor dynamics affect response |
| Distance noise | Often too clean unless noise is injected | Depth holes and reflective/low-texture regions are common |
| Timing | Determined by simulator clock | Camera, inference, ROS bridge and robot have separate clocks |
| Main validity risk | Sim-to-real gap | Sensor/calibration and synchronization error |

Simulation is therefore useful for controller logic, repeatability and failure
case generation, but it cannot by itself validate D435 ROI quality or MiR
command latency. Real runs are required for the final claims about depth-zero
handling, reaction time and false backward commands.

## 6. Computer configuration and expected consequence

The current Ubuntu host was measured as Ubuntu 22.04.5, Intel Core i7-10750H
(6 cores/12 threads), 7.6 GiB RAM, Docker 28.5.1 and Compose 2.40.2. At the
time of inspection, the NVIDIA driver was unavailable and no `/dev/video*`
device was present. Thus this host cannot currently produce a valid D435/GPU
real-time benchmark; the observed `RuntimeError: No device connected` is a
hardware/device pass-through failure, not evidence that the MiR bridge failed.

The image itself targets Ubuntu 20.04, CUDA 11.8 and ROS Noetic. On a target
machine with a working NVIDIA driver, USB 3 access and the camera exposed to
Docker, the GPU should be used for YOLO and InsightFace. Exact FPS must be
reported from `frames.csv`, not inferred from the requested 30 Hz timer. With
CPU-only inference, the 6-core/7.6 GiB host may still start the stack but is
likely to produce lower and more variable frame rate; this can increase the
effective perception-to-command delay. The 2 GiB swap is a safety net rather
than a performance resource.

Before collecting paper data on this host, verify:

- `nvidia-smi` works on the host and inside the container;
- the D435 appears in `lsusb` and `/dev/video*`/USB devices are passed through;
- `commands.csv` reports `published=0` in pilot mode;
- `frames.csv` has stable timestamps and nonzero odometry coverage;
- the independent ground-truth device is calibrated and time-synchronized.

## 7. Current scope boundary

The new instrumentation and folder structure are ready for the operational
baseline. The current Docker `main.py` does not implement `p_no_zero` or C1;
those labels remain blocked until their controller logic is added and verified
against the paper's definitions. This prevents a log label from being mistaken
for a completed experimental condition.

