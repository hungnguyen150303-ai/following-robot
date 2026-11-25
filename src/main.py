#!/usr/bin/env python3

import cv2
import json
import os
import sys
import numpy as np
import torch
import pyrealsense2 as rs
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from ultralytics import YOLO
from insightface.app import FaceAnalysis
import pygame
import time
import mediapipe as mp
from collections import deque
import tf
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import threading
import queue
import subprocess
from pathlib import Path as FilePath
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
from experiment_logger import ExperimentLogger
from experiment_protocol import (
    AUTO_MIN_FRAME_RATE_HZ,
    AUTO_MIN_ODOMETRY_RATE_HZ,
    AUTO_MIN_VALID_ROI_RATIO,
    get_configuration,
    get_scenario,
)
from experiment_ui import ExperimentPanel

# PyQt5 imports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QFrame, QSizePolicy, QStatusBar, QMessageBox, QTabWidget, QScrollArea,
    QSplitter,
)
from PyQt5.QtGui import QImage, QPixmap, QFont, QColor
from PyQt5.QtCore import Qt, QTimer, QPointF
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

# --- Constants ---
# The experimental setpoint is the camera-to-target distance used by the
# original controller.  The camera is mounted at the rear edge, centred on the
# robot longitudinal axis, and points 30 degrees down.  The corresponding
# MiR-front-edge gap is logged separately for safety/geometry analysis.
ROBOT_LENGTH_M = float(os.environ.get('ROBOT_LENGTH_M', '0.89'))
CAMERA_HEIGHT_M = float(os.environ.get('CAMERA_HEIGHT_M', '1.80'))
CAMERA_PITCH_DEG = float(os.environ.get('CAMERA_PITCH_DEG', '30.0'))
CAMERA_FPS = max(5, int(os.environ.get('CAMERA_FPS', '30')))
DESIRED_CAMERA_DISTANCE_M = float(
    os.environ.get('DESIRED_CAMERA_DISTANCE_M', '1.40')
)

# Backward-compatible names used by the existing controller/log analyzer.
DESIRED_DISTANCE = DESIRED_CAMERA_DISTANCE_M
DISTANCE_TOLERANCE = 0.03  # ±3cm tolerance at the camera target distance
MIN_DISTANCE = 0.5  # minimum camera-to-target distance
MAX_SPEED = 1.8
ANGULAR_SPEED = 1.5
DISTANCE_SMOOTHING = 5  # number of samples for moving average
GESTURE_CONFIRM_FRAMES_STOP = 5  # frames for stop gesture
GESTURE_CONFIRM_FRAMES_FOLLOW = 10  # frames for follow gesture
GESTURE_CONFIRM_FRAMES_PARKING = 20  # frames for parking gesture
FACE_DISTANCE_THRESHOLD = 0.4
DELAY_TIME = 3  # seconds before alert
GOAL_DISTANCE_THRESHOLD = 2.5  # meters for gesture-based goal


def camera_point_to_robot_geometry(point_m):
    """Convert a RealSense point (x-right, y-down, z-forward) to robot geometry.

    The camera optical centre is the origin of ``point_m``.  With the camera
    centred at the rear edge and pitched down, ``forward_from_camera_m`` is the
    horizontal distance in the robot-forward direction.  Subtracting the
    nominal MiR body length gives the gap from the robot front edge to the
    observed target point.
    """
    camera_x_m, camera_y_m, camera_z_m = [float(value) for value in point_m]
    pitch_rad = np.deg2rad(CAMERA_PITCH_DEG)
    forward_from_camera_m = (
        camera_z_m * np.cos(pitch_rad) - camera_y_m * np.sin(pitch_rad)
    )
    lateral_from_center_m = -camera_x_m
    height_m = (
        CAMERA_HEIGHT_M
        - camera_z_m * np.sin(pitch_rad)
        - camera_y_m * np.cos(pitch_rad)
    )
    return {
        'camera_x_m': camera_x_m,
        'camera_y_m': camera_y_m,
        'camera_z_m': camera_z_m,
        'forward_from_camera_m': float(forward_from_camera_m),
        'front_gap_m': float(forward_from_camera_m - ROBOT_LENGTH_M),
        'lateral_offset_m': float(lateral_from_center_m),
        'target_height_m': float(height_m),
    }


# --- ROS Setup ---

# ============================================================================
# Robot IP already set by run_mir.sh via environment variable
print("=" * 70)
print("🤖 MiR ROBOT PERSON FOLLOWER - STARTING")
print("=" * 70)

# Get robot IP from environment (set by run_mir.sh)
robot_ip = os.environ.get('ROBOT_IP', '192.168.12.20')

# Create QApplication for GUI
app = QApplication(sys.argv)
app.setApplicationName("Robot MiR — Theo dõi người")

print(f"📡 Robot IP: {robot_ip}")
print(f"🔗 ROS Master: http://localhost:11311 (container)")
print("=" * 70)

# ============================================================================
# CONTINUE WITH ORIGINAL ROS INIT BELOW
# ============================================================================

rospy.init_node('mir_follower')
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Experimental runs are opt-in by configuration but logging is enabled by
# default.  MIR_CONTROL_MODE=dry_run/pilot/log_only records commands without
# publishing them to /cmd_vel.
experiment_config = os.environ.get('EXPERIMENT_CONFIG', 'runtime_baseline')
control_mode = os.environ.get('MIR_CONTROL_MODE', 'live').strip().lower()
if control_mode not in {'live', 'dry_run', 'pilot', 'log_only'}:
    print(f"Unknown MIR_CONTROL_MODE={control_mode!r}; falling back to live")
    control_mode = 'live'

# Trial telemetry stays off during startup and between experimental runs.
experiment_capture_active = False
experiment_logger = ExperimentLogger(
    config_name=experiment_config,
    scenario=os.environ.get('EXPERIMENT_SCENARIO', 'unspecified'),
    control_mode=control_mode,
    metadata={
        'application': 'Robot MiR — Theo dõi người',
        'robot_model': 'MiR100',
        'camera_requested': {
            'color': '640x480 BGR8',
            'depth': '640x480 Z16',
            'fps': CAMERA_FPS,
            'alignment': 'depth aligned to color',
            'mount_position': 'rear_edge_centerline',
            'height_m': CAMERA_HEIGHT_M,
            'pitch_down_deg': CAMERA_PITCH_DEG,
            'rear_offset_m': 0.0,
        },
        'controller_parameters': {
            **get_configuration('p_tapered').get('controller', {}),
            'profile_id': 'p_tapered',
            'distance_reference': 'RealSense optical center to target ROI point',
            'desired_camera_distance_m': DESIRED_CAMERA_DISTANCE_M,
            'robot_length_m': ROBOT_LENGTH_M,
            'camera_height_m': CAMERA_HEIGHT_M,
            'camera_pitch_down_deg': CAMERA_PITCH_DEG,
            'distance_tolerance_m': DISTANCE_TOLERANCE,
            'distance_tolerance_behavior': (
                'publish_zero_linear_and_angular_velocity_inside_deadband'
            ),
            'minimum_distance_m': MIN_DISTANCE,
            'distance_history_samples': DISTANCE_SMOOTHING,
            'command_linear_clip_mps': MAX_SPEED,
            'effective_linear_cap_mps': 0.35,
            'command_angular_clip_rps': ANGULAR_SPEED,
            'roi_valid_range_m': [0.3, 6.0],
            'empty_roi_value_m': 0.0,
            'zero_depth_history_policy': 'ignored_by_p_tapered_fail_safe',
        },
        'measurement_note': (
            'distance_smoothed_m is the camera-to-target distance used by the '
            'controller; front-edge gap is logged separately from deprojected '
            'geometry. ground_truth_distance_m remains optional for independent annotation.'
        ),
    },
)
experiment_logger.log_event(
    'control_policy',
    'Velocity command policy selected',
    {'mode': control_mode, 'commands_published': control_mode == 'live'},
)


def start_scenario_logger(run_info):
    """Switch from the preflight log to one isolated directory per scenario run."""
    global experiment_logger

    previous_logger = experiment_logger
    inherited_metadata = {}
    if hasattr(previous_logger, 'metadata'):
        for key in (
            'application',
            'robot_model',
            'camera_requested',
            'controller_parameters',
            'runtime',
            'camera',
            'environment',
            'measurement_note',
            'rgbd_recording',
        ):
            if key in previous_logger.metadata:
                inherited_metadata[key] = previous_logger.metadata[key]
        previous_logger.close('preflight_replaced_by_scenario')

    scenario = get_scenario(run_info['scenario_id'])
    config = get_configuration(run_info['config_id'])
    variant = run_info.get('variant', 'unspecified')
    repeat_id = int(run_info.get('repeat_id', 1))
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    run_id = f"{timestamp}_{config['id']}_{scenario['id']}_{variant}_r{repeat_id:02d}"
    inherited_metadata.update({
        'source': run_info.get('source', 'real'),
        'operator_id': run_info.get('operator_id', ''),
        'operator_notes': run_info.get('operator_notes', ''),
        'scenario_id': scenario['id'],
        'scenario_action': scenario.get('scenario_action', ''),
        'target_role': run_info.get('target_role', ''),
        'occluder_role': run_info.get('occluder_role', ''),
        'motion_inhibited_by_protocol': bool(
            run_info.get('motion_inhibited_by_protocol', False)
        ),
        'scenario_variant': variant,
        'repeat_id': repeat_id,
        'block_id': run_info.get('block_id', ''),
        'collection_stage': run_info.get('collection_stage', ''),
        'data_partition': run_info.get('data_partition', 'unspecified'),
        'scenario_definition': scenario,
        'configuration_definition': config,
        'controller_profile_id': config.get('controller', {}).get('profile_id', config['id']),
        'controller_parameters': {
            **inherited_metadata.get('controller_parameters', {}),
            **config.get('controller', {}),
            'distance_reference': 'RealSense optical center to target ROI point',
            'desired_camera_distance_m': DESIRED_CAMERA_DISTANCE_M,
            'distance_tolerance_m': DISTANCE_TOLERANCE,
            'minimum_distance_m': MIN_DISTANCE,
            'distance_history_samples': DISTANCE_SMOOTHING,
            'command_linear_clip_mps': MAX_SPEED,
            'effective_linear_cap_mps': config.get('controller', {}).get(
                'max_forward_speed_mps', 0.35
            ),
            'command_angular_clip_rps': ANGULAR_SPEED,
        },
        'crossing_command_capture': (
            'shadow_controller_command_only; protocol locks published robot velocity to zero'
            if scenario.get('motion_inhibited') else 'published controller command'
        ),
        'ground_truth_ready': bool(run_info.get('ground_truth_ready', False)),
        'safety_checked': bool(run_info.get('safety_checked', False)),
        'participant_height_m': run_info.get('participant_height_m', ''),
        'walking_speed_mps': run_info.get('walking_speed_mps', ''),
        'walking_speed_value_type': run_info.get('walking_speed_value_type', 'planned_or_estimated'),
        'crossing_angle_deg': run_info.get('crossing_angle_deg', ''),
        'crossing_direction': run_info.get('crossing_direction', ''),
        'crowd_count': run_info.get('crowd_count', ''),
        'c1_capture_only': bool(run_info.get('c1_capture_only', False)),
        'rgbd_capture_required': bool(run_info.get('rgbd_capture_required', False)),
        'rgbd_capture_active_at_start': bool(run_info.get('rgbd_capture_active', False)),
        'rgbd_session_path': run_info.get('rgbd_session_path', ''),
        'offline_comparison_profiles': run_info.get('offline_comparison_profiles', []),
        'replay_profile_ids': run_info.get('replay_profile_ids', []),
        'replay_status': run_info.get('replay_status', 'not_planned'),
        'replay_source_capture_id': run_info.get('replay_source_capture_id', ''),
        'replay_outputs_expected': run_info.get('replay_outputs_expected', 0),
        'c1_freeze_confirmed': bool(run_info.get('c1_freeze_confirmed', False)),
        'c1_freeze_id': run_info.get('c1_freeze_id', ''),
        'ground_truth_source_planned': run_info.get('ground_truth_source', ''),
        'ground_truth_definition_planned': run_info.get('ground_truth_definition', ''),
        'rgbd_segment_start_unix': time.time(),
        'auto_sequence': bool(run_info.get('auto_sequence', False)),
        'auto_task_id': run_info.get('auto_task_id', ''),
        'auto_attempt': run_info.get('auto_attempt', 0),
        'auto_duration_s': run_info.get('auto_duration_s', ''),
        'auto_nominal_distance_m': run_info.get('auto_nominal_distance_m', ''),
        'auto_occlusion_duration_s': run_info.get('auto_occlusion_duration_s', ''),
        'paper_metrics_ready': False,
        'paper_metrics_note': (
            'Independent continuous camera-to-person ground truth must be annotated/synchronized separately.'
        ),
    })
    experiment_logger = ExperimentLogger(
        run_id=run_id,
        config_name=config['id'],
        scenario=f"{scenario['id']}_{variant}",
        control_mode=control_mode,
        metadata=inherited_metadata,
    )
    experiment_logger.log_event(
        'scenario_start',
        'Automated protocol started' if run_info.get('auto_sequence') else 'Operator started a protocol run',
        {
            'scenario_id': scenario['id'],
            'variant': variant,
            'repeat_id': repeat_id,
            'config_id': config['id'],
            'operator_id': run_info.get('operator_id', ''),
            'ros_time_s': rospy.get_time(),
        },
    )
    return experiment_logger


def odometry_callback(message):
    """Persist the raw /odom stream and retain its latest sample for frame logs."""
    if not experiment_capture_active:
        return
    try:
        orientation = message.pose.pose.orientation
        yaw = euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))[2]
        stamp_s = message.header.stamp.to_sec() if message.header.stamp else rospy.get_time()
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        experiment_logger.log_odom(
            ros_time_s=rospy.get_time(),
            odom_stamp_s=stamp_s,
            x_m=message.pose.pose.position.x,
            y_m=message.pose.pose.position.y,
            z_m=message.pose.pose.position.z,
            yaw_rad=yaw,
            linear_x_mps=linear.x,
            linear_y_mps=linear.y,
            linear_z_mps=linear.z,
            angular_z_rps=angular.z,
        )
    except Exception as exc:
        experiment_logger.log_event('odometry_error', str(exc))


rospy.Subscriber('/odom', Odometry, odometry_callback, queue_size=100)
experiment_logger.log_event('ros_ready', 'ROS node and /odom logger ready')

# --- Model Initialization ---
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Thiết bị suy luận đang dùng: {device}")
runtime_metadata = {
    'inference_device': str(device),
    'cuda_available': bool(torch.cuda.is_available()),
    'torch_version': torch.__version__,
    'cuda_version': torch.version.cuda,
}
if torch.cuda.is_available():
    runtime_metadata['gpu_name'] = torch.cuda.get_device_name(0)
    runtime_metadata['gpu_total_memory_bytes'] = torch.cuda.get_device_properties(0).total_memory
experiment_logger.update_metadata(runtime=runtime_metadata)
yolo_model = YOLO("/app/assets/models/yolo11s.pt").to(device)

# Initialize FaceAnalysis
face_app = FaceAnalysis(name="buffalo_s", allowed_modules=['detection', 'recognition'])
face_app.prepare(ctx_id=0 if torch.cuda.is_available() else -1, det_size=(640, 640))

# --- MediaPipe Hands ---
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

# --- RealSense Setup ---
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, CAMERA_FPS)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, CAMERA_FPS)
rgbd_recording_enabled = os.environ.get('EXPERIMENT_RECORD_RGBD', '0').strip().lower() in {
    '1', 'true', 'yes', 'on'
}
rgbd_recording_path = ''
if rgbd_recording_enabled:
    try:
        sessions_dir = FilePath(os.environ.get('EXPERIMENT_LOG_ROOT', '/app/experiments/logs')) / '_sessions'
        sessions_dir.mkdir(parents=True, exist_ok=True)
        rgbd_recording_path = str(
            sessions_dir / f"realsense_{time.strftime('%Y%m%d_%H%M%S')}_{os.getpid()}.bag"
        )
        config.enable_record_to_file(rgbd_recording_path)
    except Exception as exc:
        rgbd_recording_path = ''
        experiment_logger.log_event('rgbd_recording_setup_error', repr(exc))
try:
    profile = pipeline.start(config)
except Exception as exc:
    if rgbd_recording_path:
        experiment_logger.log_event('rgbd_recording_start_error', repr(exc))
        try:
            pipeline.stop()
        except Exception:
            pass
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, CAMERA_FPS)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, CAMERA_FPS)
        rgbd_recording_path = ''
        try:
            profile = pipeline.start(config)
        except Exception as fallback_exc:
            experiment_logger.log_event('camera_start_error', repr(fallback_exc))
            experiment_logger.close('camera_start_error')
            raise
    else:
        experiment_logger.log_event('camera_start_error', repr(exc))
        experiment_logger.close('camera_start_error')
        raise
color_sensor = profile.get_device().first_color_sensor()
color_sensor.set_option(rs.option.enable_auto_exposure, 1)  # Bật auto-exposure
color_sensor.set_option(rs.option.enable_auto_white_balance, 1)  # Bật auto-white balance
align = rs.align(rs.stream.color)  # Align depth and color streams
os.environ['EXPERIMENT_RECORD_RGBD_ACTIVE'] = '1' if rgbd_recording_path else '0'
os.environ['EXPERIMENT_RECORD_RGBD_PATH'] = rgbd_recording_path
experiment_logger.update_metadata(rgbd_recording={
    'enabled': bool(rgbd_recording_path),
    'path': rgbd_recording_path,
    'scope': 'shared application session; scenario start/end timestamps are in each run metadata',
})
try:
    camera_device = profile.get_device()
    experiment_logger.update_metadata(camera={
        'name': camera_device.get_info(rs.camera_info.name),
        'serial_number': camera_device.get_info(rs.camera_info.serial_number),
        'firmware': camera_device.get_info(rs.camera_info.firmware_version),
        'depth_scale_m': camera_device.first_depth_sensor().get_depth_scale(),
        'mount_position': 'rear_edge_centerline',
        'height_m': CAMERA_HEIGHT_M,
        'pitch_down_deg': CAMERA_PITCH_DEG,
        'robot_length_m': ROBOT_LENGTH_M,
    })
    experiment_logger.log_event('camera_ready', 'RealSense pipeline started')
except Exception as exc:
    experiment_logger.log_event('camera_metadata_error', str(exc))

# --- Global Variables ---
person_list = []
target_id = None
target_encodings = []  # List to store three face embeddings
encoding_step = 0  # 0: initial, 1: left, 2: right, 3: done
is_tracking = False
is_recognizing = False
lost_time = None
current_frame = None
distance_history = deque(maxlen=DISTANCE_SMOOTHING)
front_gap_history = deque(maxlen=DISTANCE_SMOOTHING)
gesture_command = None
gesture_frames = 0
current_gesture = None
robot_pose = None  # (x, y, theta)
person_map_pos = None
map_data = None
map_resolution = 0.05
map_origin_x = 0
map_origin_y = 0
last_command_linear_x = 0.0
last_command_angular_z = 0.0
last_command_published = False

# --- Sound Setup ---
pygame.mixer.init()
alert_sound = pygame.mixer.Sound("/app/assets/alert.mp3")
alert_sound.set_volume(0.8)
alert2_sound = pygame.mixer.Sound("/app/assets/alert2.mp3")
alert2_sound.set_volume(0.8)
is_sound_playing = False
is_alert2_playing = False

# --- TF Listener ---
tf_listener = tf.TransformListener()

# Queue for thread-safe communication
map_queue = queue.Queue()

class MapCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig, self.ax = plt.subplots(figsize=(width, height), dpi=dpi)
        self.fig.patch.set_facecolor('#e0e0e0')  # Màu nền xám
        self.ax.set_facecolor('#e0e0e0')
        super().__init__(self.fig)
        self.setParent(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.updateGeometry()

        # Variables for zoom and pan
        self.current_xlim = None
        self.current_ylim = None
        self.panning = False
        self.pan_start = None
        self.app = parent

        # Connect events
        self.mpl_connect('scroll_event', self.on_scroll)
        self.mpl_connect('button_press_event', self.on_button_press)
        self.mpl_connect('motion_notify_event', self.on_motion)
        self.mpl_connect('button_release_event', self.on_button_release)

    def draw_map(self):
        global map_data, map_resolution, map_origin_x, map_origin_y, robot_pose, person_map_pos
        if map_data is None or map_resolution == 0:
            return
        try:
            self.ax.clear()
            self.ax.set_aspect('equal')
            self.ax.imshow(map_data, cmap='gray', origin='lower', extent=[0, map_data.shape[1], 0, map_data.shape[0]])

            # Draw robot
            if robot_pose:
                robot_width_px = 0.88 / map_resolution
                robot_length_px = 0.55 / map_resolution

                map_x = (robot_pose[0] - map_origin_x) / map_resolution
                map_y = (robot_pose[1] - map_origin_y) / map_resolution

                rect = plt.Rectangle((map_x - robot_width_px / 2, map_y - robot_length_px / 2),
                                     robot_width_px, robot_length_px,
                                     angle=np.degrees(robot_pose[2]), rotation_point='center', color='#3498db',
                                     alpha=0.8)
                self.ax.add_patch(rect)

                arrow_length = robot_length_px * 0.8
                dx = arrow_length * np.cos(robot_pose[2])
                dy = arrow_length * np.sin(robot_pose[2])
                self.ax.arrow(map_x, map_y, dx, dy, head_width=robot_width_px * 0.4, head_length=robot_length_px * 0.3,
                              fc='red', ec='red')

            # Draw person
            if person_map_pos and is_tracking and not self.app.is_moving_to_goal:
                px = (person_map_pos[0] - map_origin_x) / map_resolution
                py = (person_map_pos[1] - map_origin_y) / map_resolution
                self.ax.plot(px, py, 'go', markersize=10)

                if robot_pose:
                    rx = (robot_pose[0] - map_origin_x) / map_resolution
                    ry = (robot_pose[1] - map_origin_y) / map_resolution
                    self.ax.plot([rx, px], [ry, py], 'g--', linewidth=1)

            # Draw goal
            if self.app.goal_position and self.app.is_moving_to_goal:
                gx = (self.app.goal_position[0] - map_origin_x) / map_resolution
                gy = (self.app.goal_position[1] - map_origin_y) / map_resolution
                self.ax.plot(gx, gy, 'ro', markersize=10)

            # Draw path
            if self.app.path_points and self.app.is_moving_to_goal:
                path_x = [(x - map_origin_x) / map_resolution for x, y in self.app.path_points]
                path_y = [(y - map_origin_y) / map_resolution for x, y in self.app.path_points]
                self.ax.plot(path_x, path_y, 'b-', linewidth=1)

            # Draw parking spot
            if self.app.parking_position:
                grid_x = (self.app.parking_position[0] - map_origin_x) / map_resolution
                grid_y = (self.app.parking_position[1] - map_origin_y) / map_resolution
                self.ax.plot(grid_x, grid_y, 'yo', markersize=10)
                if self.app.parking_orientation:
                    quaternion = (0, 0, self.app.parking_orientation[0], self.app.parking_orientation[1])
                    euler = euler_from_quaternion(quaternion)
                    yaw = euler[2]
                    arrow_length = 10
                    dx = arrow_length * np.cos(yaw)
                    dy = arrow_length * np.sin(yaw)
                    self.ax.arrow(grid_x, grid_y, dx, dy, head_width=5, head_length=5, fc='yellow', ec='yellow')

            # Draw rotation target
            if self.app.rotation_target:
                rx = (self.app.rotation_target[0] - map_origin_x) / map_resolution
                ry = (self.app.rotation_target[1] - map_origin_y) / map_resolution
                self.ax.plot(rx, ry, 'r*', markersize=15)
                if robot_pose:
                    rxx = (robot_pose[0] - map_origin_x) / map_resolution
                    ryy = (robot_pose[1] - map_origin_y) / map_resolution
                    self.ax.plot([rxx, rx], [ryy, ry], 'r-', linewidth=2)
                    # Draw arrow
                    angle = np.arctan2(ry - ryy, rx - rxx)
                    arrow_length = 20
                    dx = arrow_length * np.cos(angle)
                    dy = arrow_length * np.sin(angle)
                    self.ax.arrow(rxx, ryy, dx, dy, head_width=10, head_length=10, fc='red', ec='red')

            # Zoom and pan logic
            if gesture_command == "stop":
                if self.current_xlim is None or self.current_ylim is None:
                    self.current_xlim = [0, map_data.shape[1]]
                    self.current_ylim = [0, map_data.shape[0]]
            else:
                if gesture_command in ["follow", "parking"] and robot_pose:
                    robot_map_x = (robot_pose[0] - map_origin_x) / map_resolution
                    robot_map_y = (robot_pose[1] - map_origin_y) / map_resolution
                    w = map_data.shape[1]
                    h = map_data.shape[0]
                    zoom_factor = 2
                    view_w = w / zoom_factor
                    view_h = h / zoom_factor
                    x_min = robot_map_x - view_w / 2
                    x_max = robot_map_x + view_w / 2
                    y_min = robot_map_y - view_h / 2
                    y_max = robot_map_y + view_h / 2

                    if x_min < 0:
                        x_max -= x_min
                        x_min = 0
                    if x_max > w:
                        x_min -= (x_max - w)
                        x_max = w
                    if y_min < 0:
                        y_max -= y_min
                        y_min = 0
                    if y_max > h:
                        y_min -= (y_max - h)
                        y_max = h

                    self.current_xlim = [x_min, x_max]
                    self.current_ylim = [y_min, y_max]
                else:
                    self.current_xlim = [0, map_data.shape[1]]
                    self.current_ylim = [0, map_data.shape[0]]

            self.ax.set_xlim(self.current_xlim)
            self.ax.set_ylim(self.current_ylim)
            self.ax.set_xticks([])
            self.ax.set_yticks([])
            self.draw()

        except Exception as e:
            rospy.logerr(f"Map drawing error: {str(e)}")

    def on_scroll(self, event):
        global gesture_command
        if gesture_command == "stop":
            if event.inaxes != self.ax:
                return
            x_mouse = event.xdata
            y_mouse = event.ydata
            if x_mouse is None or y_mouse is None:
                return

            if event.button == 'up':
                zoom_factor = 1.2
            else:
                zoom_factor = 0.8

            x_min, x_max = self.current_xlim
            y_min, y_max = self.current_ylim
            w = x_max - x_min
            h = y_max - y_min

            r_x = (x_mouse - x_min) / w
            r_y = (y_mouse - y_min) / h

            original_w = map_data.shape[1]
            original_h = map_data.shape[0]

            new_w = w / zoom_factor
            new_h = h / zoom_factor

            if new_w < original_w / 5:
                new_w = original_w / 5
                new_h = original_h / 5
            elif new_w > original_w:
                new_w = original_w
                new_h = original_h

            x_min_new = x_mouse - r_x * new_w
            x_max_new = x_min_new + new_w
            y_min_new = y_mouse - r_y * new_h
            y_max_new = y_min_new + new_h

            if x_min_new < 0:
                x_max_new -= x_min_new
                x_min_new = 0
            if x_max_new > original_w:
                x_min_new -= (x_max_new - original_w)
                x_max_new = original_w
            if y_min_new < 0:
                y_max_new -= y_min_new
                y_min_new = 0
            if y_max_new > original_h:
                y_min_new -= (y_max_new - original_h)
                y_max_new = original_h

            self.current_xlim = [x_min_new, x_max_new]
            self.current_ylim = [y_min_new, y_max_new]
            self.draw_map()

    def on_button_press(self, event):
        global gesture_command, target_id
        if gesture_command == "stop" and event.button == 1 and event.inaxes == self.ax:
            self.panning = True
            self.pan_start = (event.xdata, event.ydata)
            # Handle parking mode clicks
            if self.app.parking_mode:
                self.app.handle_parking_click(event.xdata, event.ydata)
            # Handle rotation target clicks
        if self.app.rotation_mode and not target_id:
            self.app.handle_rotation_click(event.xdata, event.ydata)

    def on_motion(self, event):
        global gesture_command
        if self.panning and gesture_command == "stop" and event.inaxes == self.ax:
            dx = event.xdata - self.pan_start[0]
            dy = event.ydata - self.pan_start[1]
            x_min, x_max = self.current_xlim
            y_min, y_max = self.current_ylim

            new_x_min = x_min - dx
            new_x_max = x_max - dx
            new_y_min = y_min - dy
            new_y_max = y_max - dy

            original_w = map_data.shape[1]
            original_h = map_data.shape[0]

            if new_x_min < 0:
                new_x_max = new_x_max - new_x_min
                new_x_min = 0
            if new_x_max > original_w:
                new_x_min = new_x_min - (new_x_max - original_w)
                new_x_max = original_w
            if new_y_min < 0:
                new_y_max = new_y_max - new_y_min
                new_y_min = 0
            if new_y_max > original_h:
                new_y_min = new_y_min - (new_y_max - original_h)
                new_y_max = original_h

            self.current_xlim = [new_x_min, new_x_max]
            self.current_ylim = [new_y_min, new_y_max]
            self.pan_start = (event.xdata, event.ydata)
            self.draw_map()

    def on_button_release(self, event):
        if event.button == 1:  # Đổi từ chuột phải (3) sang chuột trái (1)
            self.panning = False
            self.pan_start = None


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MiR100 — Theo dõi người và điều hướng")
        self.setGeometry(100, 100, 1200, 800)

        # Initialize navigation variables
        self.goal_position = None
        self.is_moving_to_goal = False
        self.path_points = []
        self.last_person_goal = None
        self.parking_mode = False
        self.parking_click_count = 0
        self.parking_position = None
        self.parking_orientation = None
        self.is_registering_face = False  # Biến mới để theo dõi trạng thái đăng ký khuôn mặt
        self.is_idle = True  # Biến mới để theo dõi trạng thái chờ đăng ký khuôn mặt
        self.experiment_frame_idx = 0
        self.command_generated_this_frame = False
        self.experiment_run_active = False
        self.current_experiment_run_id = None
        self.current_experiment_scenario_id = None
        self.experiment_motion_inhibited = True
        self.motion_block_event_active = False
        self.reposition_control_active = False
        self.active_controller_config_id = 'p_tapered'
        self.target_loss_latched = False
        self.target_reacquisition_stable_frames = 0
        self.controller_distance_sample_valid = False
        self.controller_linear_target_mps = 0.0
        self.controller_linear_limited_mps = 0.0
        self.controller_angular_target_rps = 0.0
        self.controller_last_update_monotonic = None

        # Rotation variables
        self.rotation_mode = False
        self.rotation_target = None
        self.is_rotating = False
        self.target_yaw = 0.0

        # Create main widget and layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget)
        self.main_splitter = QSplitter(Qt.Horizontal)
        self.main_layout.addWidget(self.main_splitter)

        # Left panel (camera and controls)
        self.left_panel = QFrame()
        self.left_panel.setFrameShape(QFrame.StyledPanel)
        self.left_panel.setMinimumWidth(600)
        self.left_panel.setStyleSheet("background-color: #A9A9A9;")
        self.main_splitter.addWidget(self.left_panel)
        self.left_layout = QVBoxLayout(self.left_panel)
        self.camera_panel_title = QLabel("CAMERA — THEO DÕI VÀ KHOẢNG CÁCH")
        self.camera_panel_title.setAlignment(Qt.AlignCenter)
        self.camera_panel_title.setStyleSheet(
            "font-size:14pt;font-weight:bold;color:#16324f;padding:4px;"
        )
        self.left_layout.addWidget(self.camera_panel_title)

        # Camera display
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setStyleSheet("background-color: #66CDAA;")
        self.left_layout.addWidget(self.video_label)

        # Status labels
        self.status_label = QLabel("Nhìn thẳng vào camera và nhấp chuột vào người")
        self.status_label.setStyleSheet("font-size: 12pt; color: white;")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.left_layout.addWidget(self.status_label)
        self.face_status_label = QLabel("Trạng thái nhận diện: Chưa đăng ký khuôn mặt")
        self.face_status_label.setStyleSheet("font-size: 11pt; color: white;")
        self.face_status_label.setAlignment(Qt.AlignCenter)
        self.left_layout.addWidget(self.face_status_label)
        self.distance_label = QLabel("Khoảng cách từ camera: --.-- m")
        self.distance_label.setStyleSheet("font-size: 12pt; font-weight: bold; color: white;")
        self.distance_label.setAlignment(Qt.AlignCenter)
        self.left_layout.addWidget(self.distance_label)
        self.gesture_label = QLabel("Cử chỉ: --")
        self.gesture_label.setStyleSheet("font-size: 11pt; color: white;")
        self.gesture_label.setAlignment(Qt.AlignCenter)
        self.left_layout.addWidget(self.gesture_label)

        # Nút "Hủy theo dõi"
        self.cancel_tracking_btn = QPushButton("Hủy theo dõi")
        self.cancel_tracking_btn.setStyleSheet(
            "QPushButton {"
            "   background-color: #e74c3c;"
            "   color: white;"
            "   border: none;"
            "   padding: 8px;"
            "   font-weight: bold;"
            "   border-radius: 4px;"
            "}"
            "QPushButton:hover {"
            "   background-color: #c0392b;"
            "}"
        )
        self.cancel_tracking_btn.clicked.connect(self.cancel_tracking)
        self.left_layout.addWidget(self.cancel_tracking_btn)

        # Right panel (map)
        self.right_panel = QFrame()
        self.right_panel.setFrameShape(QFrame.StyledPanel)
        self.main_splitter.addWidget(self.right_panel)
        self.main_splitter.setStretchFactor(0, 1)
        self.main_splitter.setStretchFactor(1, 1)
        self.main_splitter.setSizes([700, 700])
        self.right_outer_layout = QVBoxLayout(self.right_panel)
        self.right_tabs = QTabWidget()
        self.right_outer_layout.addWidget(self.right_tabs)

        self.map_tab = QWidget()
        self.right_layout = QVBoxLayout(self.map_tab)
        self.right_tabs.addTab(self.map_tab, "Bản đồ / robot")

        # Map canvas
        self.map_canvas = MapCanvas(self)
        self.right_layout.addWidget(self.map_canvas)

        # Rotation status label
        self.rotation_status_label = QLabel("Chưa chọn hướng quay")
        self.rotation_status_label.setStyleSheet("font-size: 13pt; color: purple;")
        self.rotation_status_label.setAlignment(Qt.AlignCenter)
        self.right_layout.addWidget(self.rotation_status_label)

        # Rotation button
        self.rotate_btn = QPushButton("Quay robot")
        self.rotate_btn.setStyleSheet(
            "QPushButton {"
            "   background-color: #9b59b6;"
            "   color: white;"
            "   border: none;"
            "   padding: 8px;"
            "   font-weight: bold;"
            "   border-radius: 4px;"
            "}"
            "QPushButton:hover {"
            "   background-color: #8e44ad;"
            "}"
            "QPushButton:disabled {"
            "   background-color: #bdc3c7;"
            "}"
        )
        self.rotate_btn.clicked.connect(self.activate_rotation_mode)
        self.right_layout.addWidget(self.rotate_btn)

        # Parking status label
        self.parking_status_label = QLabel("Chưa có vị trí đỗ xe")
        self.parking_status_label.setStyleSheet("font-size: 13pt; color: red;")
        self.parking_status_label.setAlignment(Qt.AlignCenter)
        self.right_layout.addWidget(self.parking_status_label)

        # Parking button
        self.parking_btn = QPushButton("Đỗ xe")
        self.parking_btn.setStyleSheet(
            "QPushButton {"
            "   background-color: #3498db;"
            "   color: white;"
            "   border: none;"
            "   padding: 8px;"
            "   font-weight: bold;"
            "   border-radius: 4px;"
            "}"
            "QPushButton:hover {"
            "   background-color: #2980b9;"
            "}"
        )
        self.parking_btn.clicked.connect(self.start_parking_mode)
        self.right_layout.addWidget(self.parking_btn)

        # Operator-facing E1 experiment collection panel
        self.experiment_panel = ExperimentPanel(control_mode, self)
        self.experiment_panel.run_start_requested.connect(self.start_experiment_run)
        self.experiment_panel.run_stop_requested.connect(self.stop_experiment_run)
        self.experiment_panel.event_requested.connect(self.mark_experiment_event)
        self.experiment_panel.ground_truth_requested.connect(self.log_ground_truth_sample)
        self.experiment_panel.motion_inhibit_requested.connect(
            self.set_experiment_motion_inhibited
        )
        self.experiment_panel.reposition_control_requested.connect(
            self.set_reposition_control_active
        )
        self.experiment_scroll = QScrollArea()
        self.experiment_scroll.setWidgetResizable(True)
        self.experiment_scroll.setWidget(self.experiment_panel)
        self.right_tabs.addTab(self.experiment_scroll, "HƯỚNG DẪN ĐO — E1")
        self.right_tabs.setCurrentWidget(self.experiment_scroll)

        # Set up timers
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(30)  # ~30 FPS
        self.position_timer = QTimer()
        self.position_timer.timeout.connect(self.update_position_continuously)
        self.position_timer.start(500)
        self.map_timer = QTimer()
        self.map_timer.timeout.connect(self.check_map_queue)
        self.map_timer.start(100)
        self.rotation_timer = QTimer()
        self.rotation_timer.timeout.connect(self.update_rotation)
        self.rotation_timer.setInterval(100)  # 10 Hz

        # ROS Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback_threaded)
        rospy.Subscriber('/move_base_node/SBPLLatticePlanner/plan', Path, self.path_callback)

        # Connect video label click event
        self.video_label.mousePressEvent = self.on_video_click

        # Start ROS spin thread
        self.ros_thread = threading.Thread(target=rospy.spin, daemon=True)
        self.ros_thread.start()

    def start_experiment_run(self, run_info):
        """Create an isolated log folder and reset only experiment bookkeeping."""
        global experiment_capture_active
        global distance_history, front_gap_history
        global last_command_linear_x, last_command_angular_z, last_command_published
        if self.experiment_run_active:
            return
        logger = start_scenario_logger(run_info)
        self.active_controller_config_id = run_info.get('config_id', 'p_tapered')
        self.experiment_run_active = True
        experiment_capture_active = True
        self.current_experiment_run_id = logger.run_id
        self.current_experiment_scenario_id = run_info.get('scenario_id')
        self.experiment_frame_idx = 0
        self.command_generated_this_frame = False
        distance_history.clear()
        front_gap_history.clear()
        last_command_linear_x = 0.0
        last_command_angular_z = 0.0
        last_command_published = False
        self.controller_linear_target_mps = 0.0
        self.controller_linear_limited_mps = 0.0
        self.controller_angular_target_rps = 0.0
        self.controller_last_update_monotonic = None
        self.target_reacquisition_stable_frames = 0
        self.experiment_panel.set_run_directory(str(logger.run_dir))
        self.experiment_panel.status_label.setText(
            f"Đang ghi kịch bản {run_info['scenario_id']} — {logger.run_id}"
        )

    def set_experiment_motion_inhibited(self, inhibited):
        self.experiment_motion_inhibited = bool(inhibited)
        if not self.experiment_motion_inhibited:
            self.motion_block_event_active = False
        if self.experiment_motion_inhibited:
            self.send_velocity_command(0, 0)
            self._suppress_occlusion_navigation_goal("motion_lock_enabled")

    def _occlusion_motion_lock_active(self):
        panel = getattr(self, 'experiment_panel', None)
        task = getattr(panel, 'current_task', None) if panel else None
        scenario_id = (
            self.current_experiment_scenario_id
            or (task.get('scenario_id', '') if task else '')
        )
        return bool(
            self.experiment_motion_inhibited
            and get_scenario(scenario_id).get('motion_inhibited', False)
        )

    def _suppress_occlusion_navigation_goal(self, reason):
        if not self._occlusion_motion_lock_active():
            return False
        try:
            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            client.cancel_all_goals()
        except Exception as exc:
            experiment_logger.log_event('move_base_cancel_error', str(exc))
        self.is_moving_to_goal = False
        self.goal_position = None
        self.path_points = []
        experiment_logger.log_event(
            'move_base_goal_suppressed',
            'Kịch bản che khuất khóa chuyển động robot và hủy lệnh điều hướng',
            {'reason': reason, 'scenario_id': self.current_experiment_scenario_id or ''},
        )
        return True

    def set_reposition_control_active(self, active):
        active = bool(active)
        if active:
            panel = self.experiment_panel
            current_task = panel.current_task
            scenario = get_scenario(current_task.get('scenario_id', '')) if current_task else {}
            c1_between_runs = bool(
                scenario.get('c1_capture_only')
                and panel.auto_state in {
                    'waiting_for_rgbd', 'waiting_for_target', 'waiting_for_follow',
                    'waiting_for_stable_distance', 'transition'
                }
            )
            if (
                control_mode != 'live'
                or self.experiment_run_active
                or not current_task
                or (
                    current_task.get('scenario_id') != 'E1_robot_approach'
                    and not c1_between_runs
                )
            ):
                return
            self.reposition_control_active = True
            self.experiment_motion_inhibited = False
            return

        self.reposition_control_active = False
        self.experiment_motion_inhibited = True
        self.send_velocity_command(0, 0, source='reposition_control_stop')

    def mark_experiment_event(self, event_name, run_info):
        if not self.experiment_run_active:
            return
        experiment_logger.log_event(
            event_name,
            'Automatic protocol phase marker' if run_info.get('auto_sequence') else 'Operator marked protocol event',
            {
                'scenario_id': run_info.get('scenario_id', ''),
                'variant': run_info.get('variant', ''),
                'repeat_id': run_info.get('repeat_id', ''),
                'operator_id': run_info.get('operator_id', ''),
                'operator_notes': run_info.get('operator_notes', ''),
                'auto_task_id': run_info.get('auto_task_id', ''),
                'auto_attempt': run_info.get('auto_attempt', ''),
                'auto_phase_id': run_info.get('auto_phase_id', ''),
                'auto_goal_reached': run_info.get('auto_goal_reached'),
                'auto_goal_distance_m': run_info.get('auto_goal_distance_m'),
                'auto_goal_tolerance_m': run_info.get('auto_goal_tolerance_m'),
                'auto_goal_stability_frames': run_info.get('auto_goal_stability_frames'),
                'auto_goal_final_distance_m': run_info.get('auto_goal_final_distance_m'),
                'auto_start_gate': run_info.get('auto_start_gate', {}),
                'auto_skip_scenario': bool(run_info.get('auto_skip_scenario', False)),
                'ros_time_s': rospy.get_time(),
            },
        )

    def log_ground_truth_sample(self, separation_m, run_info):
        """Store a manually measured range with the current ROS/frame timestamp."""
        if not self.experiment_run_active:
            return
        frame_idx = self.experiment_frame_idx - 1 if self.experiment_frame_idx else None
        reference_definition = run_info.get('ground_truth_definition', '')
        source = run_info.get('ground_truth_source', '')
        note = run_info.get('ground_truth_note', '')
        ros_time_s = rospy.get_time()
        experiment_logger.log_ground_truth(
            separation_m,
            ros_time_s=ros_time_s,
            frame_idx=frame_idx,
            reference_definition=reference_definition,
            source=source,
            note=note,
        )

    def stop_experiment_run(self, run_info):
        global experiment_capture_active
        if not self.experiment_run_active:
            return
        # Stop the robot before closing the CSV/event handles.
        self.send_velocity_command(0, 0)
        quality = None
        if run_info.get('auto_sequence'):
            snapshot = experiment_logger.progress_snapshot()
            expected_s = max(1.0, float(run_info.get('auto_duration_s', 1.0) or 1.0))
            scenario = get_scenario(run_info.get('scenario_id', ''))
            elapsed_s = max(snapshot['elapsed_s'], 1e-6)
            target_ratio = snapshot['target_frames'] / max(snapshot['frames'], 1)
            valid_roi_ratio = snapshot['valid_roi_frames'] / max(snapshot['target_frames'], 1)
            reasons = []
            if scenario.get('requires_rgbd_recording') and not run_info.get('rgbd_capture_active'):
                reasons.append('thiếu ghi RGB-D gốc để phát lại')
            if scenario.get('c1_capture_only') and not run_info.get('safety_checked'):
                reasons.append('chưa xác nhận kiểm tra an toàn')
            if scenario.get('c1_capture_only') and not run_info.get('ground_truth_ready'):
                reasons.append('chưa xác nhận ground truth độc lập liên tục/đồng bộ')
            if snapshot['elapsed_s'] < expected_s * 0.90:
                reasons.append('thời lượng ghi chưa đủ')
            if snapshot['frames'] / elapsed_s < AUTO_MIN_FRAME_RATE_HZ:
                reasons.append('thiếu khung hình')
            if snapshot['commands'] / elapsed_s < AUTO_MIN_FRAME_RATE_HZ * 0.6:
                reasons.append('thiếu mẫu lệnh')
            if snapshot['odometry'] / elapsed_s < AUTO_MIN_ODOMETRY_RATE_HZ:
                reasons.append('thiếu odometry')
            minimum_target_ratio = float(scenario.get('auto_min_target_presence_ratio', 0.4))
            if target_ratio < minimum_target_ratio:
                reasons.append('mục tiêu xuất hiện quá ít')
            minimum_target_loss_s = scenario.get('auto_min_target_loss_duration_s')
            if minimum_target_loss_s is not None:
                minimum_target_loss_s = float(minimum_target_loss_s)
                completed_loss = (
                    snapshot.get('crossing_target_loss_events', 0) >= 1
                    and snapshot.get('crossing_target_reacquisition_events', 0) >= 1
                    and snapshot.get('max_completed_crossing_target_loss_duration_s', 0.0)
                    >= minimum_target_loss_s
                )
                if not completed_loss:
                    reasons.append(
                        f'chưa ghi nhận mất dấu ≥{minimum_target_loss_s:.1f} giây rồi nhận lại mục tiêu'
                    )
                maximum_target_loss_s = scenario.get('auto_max_target_loss_duration_s')
                if (
                    maximum_target_loss_s is not None
                    and snapshot.get('max_completed_crossing_target_loss_duration_s', 0.0)
                    > float(maximum_target_loss_s)
                ):
                    reasons.append(
                        f'thời lượng mất dấu vượt {float(maximum_target_loss_s):.1f} giây'
                    )
                minimum_reacquisition_frames = int(
                    scenario.get('auto_min_target_reacquisition_stability_frames', 0) or 0
                )
                stable_reacquisition_frames = snapshot.get(
                    'max_crossing_target_reacquisition_stable_frames', 0
                )
                if stable_reacquisition_frames < minimum_reacquisition_frames:
                    reasons.append(
                        f'mục tiêu chưa ổn định đủ {minimum_reacquisition_frames} khung hình sau khi nhận lại'
                    )
            if scenario.get('auto_require_tracking_continuity_during_crossing'):
                crossing_losses = snapshot.get('crossing_target_loss_events', 0)
                crossing_id_changes = snapshot.get('crossing_target_id_change_count', 0)
                if crossing_losses or crossing_id_changes:
                    reasons.append(
                        'mục tiêu bị mất dấu hoặc đổi ID trong nhánh yêu cầu theo dõi liên tục'
                    )
            if valid_roi_ratio < AUTO_MIN_VALID_ROI_RATIO:
                reasons.append('ROI có độ sâu hợp lệ quá ít')
            if scenario.get('motion_inhibited'):
                if (
                    snapshot.get('max_abs_command_linear_mps', 0.0) > 1e-6
                    or snapshot.get('max_abs_command_angular_rps', 0.0) > 1e-6
                    or snapshot.get('published_nonzero_command_count', 0) > 0
                ):
                    reasons.append('kịch bản khóa robot nhưng log có lệnh chuyển động khác 0')
            if (
                scenario['id'] == 'E1_robot_approach'
                and not run_info.get('auto_goal_reached', False)
            ):
                reasons.append('MiR chưa ổn định tại khoảng cách đích 1,4 m')
            if (
                scenario['id'] != 'E1_calibration'
                and control_mode != 'live'
                and not scenario.get('allow_dry_run_capture', False)
            ):
                reasons.append('không phải lượt robot thật')
            quality = {
                'passed': not reasons,
                'reasons': reasons,
                'duration_s': round(snapshot['elapsed_s'], 3),
                'expected_duration_s': expected_s,
                'frames': snapshot['frames'],
                'frames_per_s': round(snapshot['frames'] / elapsed_s, 3),
                'target_frames': snapshot['target_frames'],
                'target_presence_ratio': round(target_ratio, 4),
                'target_loss_events': snapshot.get('target_loss_events', 0),
                'target_reacquisition_events': snapshot.get('target_reacquisition_events', 0),
                'target_id_change_count': snapshot.get('target_id_change_count', 0),
                'crossing_target_loss_events': snapshot.get('crossing_target_loss_events', 0),
                'crossing_target_reacquisition_events': snapshot.get(
                    'crossing_target_reacquisition_events', 0
                ),
                'crossing_target_id_change_count': snapshot.get(
                    'crossing_target_id_change_count', 0
                ),
                'max_completed_crossing_target_loss_duration_s': snapshot.get(
                    'max_completed_crossing_target_loss_duration_s', 0.0
                ),
                'max_completed_target_loss_duration_s': snapshot.get(
                    'max_completed_target_loss_duration_s', 0.0
                ),
                'max_target_reacquisition_stable_frames': snapshot.get(
                    'max_target_reacquisition_stable_frames', 0
                ),
                'max_crossing_target_reacquisition_stable_frames': snapshot.get(
                    'max_crossing_target_reacquisition_stable_frames', 0
                ),
                'valid_roi_frames': snapshot['valid_roi_frames'],
                'valid_roi_ratio_when_target_present': round(valid_roi_ratio, 4),
                'commands': snapshot['commands'],
                'max_abs_command_linear_mps': snapshot.get('max_abs_command_linear_mps', 0.0),
                'max_abs_command_angular_rps': snapshot.get('max_abs_command_angular_rps', 0.0),
                'published_nonzero_command_count': snapshot.get('published_nonzero_command_count', 0),
                'odometry': snapshot['odometry'],
                'odometry_per_s': round(snapshot['odometry'] / elapsed_s, 3),
                'ground_truth_samples': snapshot['ground_truth_samples'],
                'auto_goal_reached': run_info.get('auto_goal_reached'),
                'auto_goal_distance_m': run_info.get('auto_goal_distance_m'),
                'auto_goal_tolerance_m': run_info.get('auto_goal_tolerance_m'),
                'auto_goal_stability_frames': run_info.get('auto_goal_stability_frames'),
                'auto_goal_final_distance_m': run_info.get('auto_goal_final_distance_m'),
            }
            experiment_logger.update_metadata(
                auto_capture_quality=quality,
                paper_metrics_ready=False,
                paper_metrics_note=(
                    'The technical capture gate does not replace independent continuous ground-truth annotation.'
                ),
            )
            experiment_logger.log_event(
                'auto_capture_quality',
                'Automatic log sufficiency check',
                quality,
            )
        experiment_logger.log_event(
            'scenario_aborted' if run_info.get('auto_abort') else 'scenario_end',
            'Automatic session aborted by operator' if run_info.get('auto_abort') else 'Automatic protocol duration ended',
            {
                'scenario_id': run_info.get('scenario_id', ''),
                'variant': run_info.get('variant', ''),
                'repeat_id': run_info.get('repeat_id', ''),
                'operator_id': run_info.get('operator_id', ''),
                'auto_task_id': run_info.get('auto_task_id', ''),
                'auto_skip_scenario': bool(run_info.get('auto_skip_scenario', False)),
                'ros_time_s': rospy.get_time(),
            },
        )
        experiment_logger.update_metadata(rgbd_segment_end_unix=time.time())
        run_dir = experiment_logger.run_dir
        if run_info.get('auto_abort'):
            final_status = 'scenario_aborted'
        elif quality is not None and not quality.get('passed'):
            final_status = 'scenario_incomplete'
        else:
            final_status = 'scenario_completed'
        experiment_logger.close(final_status)
        experiment_capture_active = False
        analyzer_path = FilePath(__file__).resolve().parents[1] / 'experiments' / 'scripts' / 'analyze_run.py'
        try:
            subprocess.run(
                [sys.executable, str(analyzer_path), str(run_dir)],
                check=True,
                capture_output=True,
                text=True,
                timeout=30,
            )
            self.experiment_panel.status_label.setText(
                f"Đã lưu nhật ký và tạo bản tổng hợp: {run_dir}"
            )
        except (OSError, subprocess.SubprocessError) as exc:
            rospy.logwarn("Could not generate E1 summary for %s: %s", run_dir, exc)
            self.experiment_panel.status_label.setText(
                f"Đã lưu nhật ký; chưa tạo được bản tổng hợp. Thư mục: {run_dir}"
            )
        self.experiment_run_active = False
        self.current_experiment_run_id = None
        self.current_experiment_scenario_id = None
        self.active_controller_config_id = 'p_tapered'
        self.experiment_panel.set_run_quality(quality or {'passed': True})
        self.experiment_panel.refresh_progress()

    def activate_rotation_mode(self):
        """Kích hoạt chế độ chọn hướng quay cho robot"""
        if is_tracking:
            QMessageBox.warning(self, "Lỗi", "Không thể quay robot khi đang theo dõi người!")
            return
        if self.is_moving_to_goal:
            QMessageBox.warning(self, "Lỗi", "Không thể quay robot khi đang di chuyển đến mục tiêu!")
            return
        if self.is_rotating:
            QMessageBox.warning(self, "Lỗi", "Robot đang trong quá trình quay!")
            return

        self.rotation_mode = True
        self.rotation_status_label.setText("Vui lòng chọn hướng quay trên bản đồ")
        self.rotation_status_label.setStyleSheet("font-size: 13pt; color: purple;")

    def handle_rotation_click(self, x, y):
        """Xử lý sự kiện nhấp chuột để chọn hướng quay"""
        if x is not None and y is not None:
            # Chuyển đổi tọa độ pixel sang tọa độ bản đồ
            x_map = x * map_resolution + map_origin_x
            y_map = y * map_resolution + map_origin_y
            self.rotation_target = (x_map, y_map)
            self.rotation_mode = False
            self.start_rotation()

    def start_rotation(self):
        """Bắt đầu quá trình quay robot sau 3 giây trễ"""
        if robot_pose is None or self.rotation_target is None:
            return

        # Tính góc quay mục tiêu
        rx, ry, current_yaw = robot_pose
        tx, ty = self.rotation_target

        # Tính vector từ robot đến mục tiêu
        dx = tx - rx
        dy = ty - ry

        # Tính góc quay mục tiêu (radians)
        self.target_yaw = np.arctan2(dy, dx)

        # Hiển thị thông báo chuẩn bị quay
        self.rotation_status_label.setText("Chuẩn bị quay robot sau 5 giây...")
        self.rotation_status_label.setStyleSheet("font-size: 12pt; color: purple;")

        # Trì hoãn 5 giây trước khi quay
        QTimer.singleShot(5000, self.execute_rotation)

    def execute_rotation(self):
        """Thực hiện quá trình quay robot"""
        if robot_pose is None or self.rotation_target is None:
            return

        # Bắt đầu quay
        self.is_rotating = True
        self.rotation_status_label.setText("Đang quay robot")
        self.rotation_status_label.setStyleSheet("font-size: 12pt; color: purple;")
        self.rotation_timer.start()

    def update_rotation(self):
        """Cập nhật quá trình quay robot với tốc độ bằng một nửa tối đa"""
        if not self.is_rotating or robot_pose is None:
            self.rotation_timer.stop()
            self.rotation_status_label.setText("Chưa chọn hướng quay")
            self.rotation_status_label.setStyleSheet("font-size: 13pt; color: purple;")
            return

        rx, ry, current_yaw = robot_pose

        # Tính sai số góc
        error = self.target_yaw - current_yaw

        # Đưa sai số về khoảng [-pi, pi]
        while error > np.pi:
            error -= 2 * np.pi
        while error < -np.pi:
            error += 2 * np.pi

        # Ngưỡng dừng (5 độ)
        if abs(error) < np.radians(5):
            self.send_velocity_command(0, 0)
            self.is_rotating = False
            self.rotation_target = None
            self.rotation_timer.stop()
            self.rotation_status_label.setText("Hoàn tất quay")
            self.rotation_status_label.setStyleSheet("font-size: 13pt; color: green;")
            QTimer.singleShot(2000, lambda: self.rotation_status_label.setText("Chưa chọn hướng quay"))
            QTimer.singleShot(2000, lambda: self.rotation_status_label.setStyleSheet("font-size: 13pt; color: purple;"))
            return

        # Điều khiển P
        Kp = 0.75
        angular_speed = Kp * error

        # Giới hạn tốc độ góc bằng một nửa tốc độ tối đa
        max_angular_speed = ANGULAR_SPEED / 2  # Giảm tốc độ quay xuống một nửa
        angular_speed = np.clip(angular_speed, -max_angular_speed, max_angular_speed)

        # Gửi lệnh quay
        self.send_velocity_command(0, angular_speed)

    def map_callback_threaded(self, msg):
        try:
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            data = np.array(msg.data).reshape((height, width))
            map_queue.put((data, resolution, origin_x, origin_y))
        except Exception as e:
            rospy.logerr(f"Map processing error: {str(e)}")

    def check_map_queue(self):
        try:
            while not map_queue.empty():
                data, resolution, origin_x, origin_y = map_queue.get_nowait()
                self.process_map_data(data, resolution, origin_x, origin_y)
        except queue.Empty:
            pass

    def process_map_data(self, data, resolution, origin_x, origin_y):
        global map_data, map_resolution, map_origin_x, map_origin_y
        map_data = data
        map_resolution = resolution
        map_origin_x = origin_x
        map_origin_y = origin_y
        self.map_canvas.draw_map()

    def start_parking_mode(self):
        self.parking_mode = True
        self.parking_click_count = 0
        self.parking_status_label.setText("Vui lòng chọn vị trí đỗ xe")
        self.parking_status_label.setStyleSheet("font-size: 13pt; color: red;")

    def handle_parking_click(self, x, y):
        if self.parking_mode and x is not None and y is not None:
            if self.parking_click_count == 0:
                x_parking = x * map_resolution + map_origin_x
                y_parking = y * map_resolution + map_origin_y
                self.parking_position = (x_parking, y_parking)
                self.parking_click_count = 1
                self.parking_status_label.setText("Đã xác nhận vị trí đỗ xe, vui lòng chọn hướng đỗ xe")
                self.parking_status_label.setStyleSheet("font-size: 13pt; color: red;")
            elif self.parking_click_count == 1:
                direction_x = x * map_resolution + map_origin_x
                direction_y = y * map_resolution + map_origin_y
                z_parking, w_parking = self.calculate_orientation(self.parking_position, (direction_x, direction_y))
                self.parking_orientation = (z_parking, w_parking)
                self.parking_mode = False
                self.parking_click_count = 0
                self.parking_status_label.setText("Đã xác nhận vị trí và hướng đỗ xe")
                self.parking_status_label.setStyleSheet("font-size: 13pt; color: green;")
                self.map_canvas.draw_map()

    def calculate_orientation(self, start, end):
        orientation = np.arctan2(end[1] - start[1], end[0] - start[0])
        z = np.sin(orientation / 2)
        w = np.cos(orientation / 2)
        return z, w

    def path_callback(self, path_msg):
        if self.is_moving_to_goal:
            try:
                self.path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]
                self.map_canvas.draw_map()
            except Exception as e:
                rospy.logerr(f"Error in path_callback: {str(e)}")

    def send_velocity_command(self, linear_x=0, angular_z=0, source='controller'):
        global last_command_linear_x, last_command_angular_z, last_command_published
        requested_linear_x = float(linear_x)
        requested_angular_z = float(angular_z)
        if self.experiment_motion_inhibited and (
            abs(requested_linear_x) > 1e-6 or abs(requested_angular_z) > 1e-6
        ):
            if not self.motion_block_event_active:
                experiment_logger.log_event(
                    'motion_command_blocked',
                    'Lệnh vận tốc khác 0 bị chặn do khóa chuyển động của giao thức',
                    {
                        'requested_linear_x_mps': requested_linear_x,
                        'requested_angular_z_rps': requested_angular_z,
                        'source': source,
                        'scenario_id': self.current_experiment_scenario_id or '',
                    },
                )
                self.motion_block_event_active = True
            linear_x = 0.0
            angular_z = 0.0
            source = 'protocol_motion_lock_blocked'
        cmd = Twist()
        cmd.linear.x = np.clip(linear_x, -MAX_SPEED, MAX_SPEED)
        cmd.angular.z = np.clip(angular_z, -ANGULAR_SPEED, ANGULAR_SPEED)
        should_publish = control_mode == 'live'
        if should_publish:
            cmd_vel_pub.publish(cmd)
        last_command_linear_x = float(cmd.linear.x)
        last_command_angular_z = float(cmd.angular.z)
        last_command_published = should_publish
        self.command_generated_this_frame = True
        auto_phase_id = ""
        if self.experiment_run_active:
            auto_phase_id = getattr(
                getattr(self, 'experiment_panel', None), 'current_phase_id', ''
            )
            experiment_logger.log_command(
                linear_x_mps=last_command_linear_x,
                angular_z_rps=last_command_angular_z,
                published=should_publish,
                ros_time_s=rospy.get_time(),
                source=source,
                auto_phase_id=auto_phase_id,
            )

    def get_depth_roi_stats(self, depth_frame, box):
        """Return raw depth quality and the front-edge distance estimate.

        ``get_distance`` is a camera-frame depth value.  For this setup the
        camera is at the MiR rear edge, so the controller must use the
        horizontal forward coordinate after deprojection and camera-pitch
        compensation, then subtract the MiR body length.
        """
        x1, y1, x2, y2 = box
        center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
        roi_size = min(x2 - x1, y2 - y1) // 4
        samples = []
        valid_points = []
        try:
            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        except Exception:
            intrinsics = None
        for dx in range(-roi_size, roi_size, 5):
            for dy in range(-roi_size, roi_size, 5):
                px = center_x + dx
                py = center_y + dy
                if 0 <= px < 640 and 0 <= py < 480:
                    try:
                        depth_m = float(depth_frame.get_distance(px, py))
                        samples.append(depth_m)
                        if intrinsics is not None and 0.3 < depth_m < 6.0:
                            point = rs.rs2_deproject_pixel_to_point(
                                intrinsics, [float(px), float(py)], depth_m
                            )
                            valid_points.append(camera_point_to_robot_geometry(point))
                    except Exception:
                        continue
        nonzero = [value for value in samples if value > 0.0]
        valid = [value for value in samples if 0.3 < value < 6.0]
        zero_count = sum(1 for value in samples if value <= 0.0)
        out_of_range_count = len(samples) - len(valid) - zero_count
        front_gap_values = [point['front_gap_m'] for point in valid_points]
        forward_values = [point['forward_from_camera_m'] for point in valid_points]
        lateral_values = [point['lateral_offset_m'] for point in valid_points]
        target_height_values = [point['target_height_m'] for point in valid_points]
        camera_x_values = [point['camera_x_m'] for point in valid_points]
        camera_y_values = [point['camera_y_m'] for point in valid_points]
        camera_z_values = [point['camera_z_m'] for point in valid_points]
        return {
            'sample_count': len(samples),
            'nonzero_count': len(nonzero),
            'valid_count': len(valid),
            'zero_count': zero_count,
            'out_of_range_count': max(0, out_of_range_count),
            'median_nonzero_m': float(np.median(nonzero)) if nonzero else '',
            'median_filtered_m': float(np.median(valid)) if valid else 0.0,
            'empty': not bool(valid),
            'zero_ratio': float(zero_count / len(samples)) if samples else 1.0,
            'geometry_valid_count': len(valid_points),
            'camera_x_m': float(np.median(camera_x_values)) if camera_x_values else '',
            'camera_y_m': float(np.median(camera_y_values)) if camera_y_values else '',
            'camera_z_m': float(np.median(camera_z_values)) if camera_z_values else '',
            'forward_from_camera_m': float(np.median(forward_values)) if forward_values else 0.0,
            'front_gap_m': float(np.median(front_gap_values)) if front_gap_values else 0.0,
            'lateral_offset_m': float(np.median(lateral_values)) if lateral_values else '',
            'target_height_m': float(np.median(target_height_values)) if target_height_values else '',
        }

    def get_smoothed_distance(self, depth_frame, box):
        return self.get_depth_roi_stats(depth_frame, box)['median_filtered_m']

    def face_recognition_handler(self, frame, person):
        global target_id, is_tracking, is_recognizing, lost_time, is_sound_playing
        x1, y1, x2, y2 = person['box']
        box_x_mid = (x1 + x2) // 2
        face_x_mid = None

        box_height = y2 - y1
        y_mid = y1 + int(box_height * 0.5)
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(640, x2)
        y_mid = min(480, y_mid)
        face_region = frame[y1:y_mid, x1:x2]
        if face_region.size == 0:
            return False
        face_region_rgb = cv2.cvtColor(face_region, cv2.COLOR_BGR2RGB)
        faces = face_app.get(face_region_rgb)

        if len(faces) >= 5:
            return False
        if len(faces) == 0:
            return False

        max_area_face = None
        max_area = 0
        for face in faces:
            bbox = face.bbox.astype(int)
            top, right, bottom, left = bbox[1], bbox[2], bbox[3], bbox[0]
            face_area = (right - left) * (bottom - top)
            if face_area > max_area:
                max_area = face_area
                max_area_face = face

        if max_area_face is None:
            return False

        face_embedding = max_area_face.embedding
        face_bbox = max_area_face.bbox.astype(int)
        face_x_mid = x1 + (face_bbox[0] + face_bbox[2]) // 2

        X_MID_THRESHOLD = 80
        if face_x_mid is None or abs(face_x_mid - box_x_mid) > X_MID_THRESHOLD:
            return False

        for encoding in target_encodings:
            distance = 1 - np.dot(face_embedding, encoding) / (
                    np.linalg.norm(face_embedding) * np.linalg.norm(encoding))
            if distance < FACE_DISTANCE_THRESHOLD:
                target_id = person['id']
                is_tracking = True
                is_recognizing = False
                lost_time = None
                self.face_status_label.setText("Đã xác nhận danh tính")
                self.face_status_label.setStyleSheet("font-size: 11pt; color: green;")
                if is_sound_playing:
                    alert_sound.stop()
                    is_sound_playing = False
                return True
        return False

    def process_frame(self, color_frame, depth_frame):
        global target_id, person_list, is_tracking, is_recognizing, lost_time, current_frame, is_sound_playing
        color_image = np.asanyarray(color_frame.get_data())
        current_frame = color_image.copy()
        results = yolo_model.track(color_image, persist=True, verbose=False, conf=0.5)
        person_list = []
        for result in results:
            for box in result.boxes:
                if int(box.cls) != 0:
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                box_area = (x2 - x1) * (y2 - y1)
                if box_area < 10000:
                    continue
                person_id = f"ID{int(box.id)}" if box.id is not None else "ID?"
                roi_stats = self.get_depth_roi_stats(depth_frame, (x1, y1, x2, y2))
                distance = roi_stats['median_filtered_m']
                person_list.append({
                    'id': person_id,
                    'box': (x1, y1, x2, y2),
                    'center': ((x1 + x2) // 2, (y1 + y2) // 2),
                    'distance': distance,
                    'front_gap_m': roi_stats.get('front_gap_m', 0.0),
                    'roi_stats': roi_stats,
                    'is_target': (person_id == target_id)
                })

        if not self.is_moving_to_goal and not self.is_rotating:
            if is_tracking:
                if target_id not in [p['id'] for p in person_list]:
                    self.target_loss_latched = True
                    self.target_reacquisition_stable_frames = 0
                    distance_history.clear()
                    front_gap_history.clear()
                    is_tracking = False
                    is_recognizing = True
                    target_id = None
                    lost_time = time.time()
                    self.face_status_label.setText("Mất dấu! Đang nhận diện...")
                    self.face_status_label.setStyleSheet("font-size: 11pt; color: red;")
            elif is_recognizing and target_encodings:
                for person in sorted(person_list, key=lambda x: x['distance']):
                    if person['distance'] < 2.5:
                        if self.face_recognition_handler(color_image, person):
                            break
                if gesture_command == "follow" and lost_time and (
                        time.time() - lost_time) > DELAY_TIME and not is_sound_playing:
                    alert_sound.play(loops=-1)
                    is_sound_playing = True

        if target_id and not self.is_moving_to_goal and not self.is_rotating:
            target_person = next((p for p in person_list if p['id'] == target_id), None)
            if target_person:
                self.detect_gestures(color_image, target_person['box'], depth_frame)
        return color_image

    def detect_gestures(self, image, target_box, depth_frame):
        global gesture_command, gesture_frames, current_gesture
        if self.is_moving_to_goal or self.is_rotating:
            return

        x1, y1, x2, y2 = target_box
        box_height = y2 - y1
        y_top = y1
        y_bottom = y1 + box_height // 2
        roi = image[max(0, y_top):min(480, y_bottom), max(0, x1):min(640, x2)]
        if roi.size == 0:
            return
        roi_rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
        results = hands.process(roi_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                fingers_up = self.count_fingers(hand_landmarks)

                if fingers_up == 3:
                    gesture = "stop"
                    confirm_frames = GESTURE_CONFIRM_FRAMES_STOP
                elif fingers_up == 2:
                    gesture = "follow"
                    confirm_frames = GESTURE_CONFIRM_FRAMES_FOLLOW
                elif fingers_up == 5:
                    gesture = "parking"
                    confirm_frames = GESTURE_CONFIRM_FRAMES_PARKING
                else:
                    gesture = None
                    confirm_frames = 0

                if gesture:
                    if gesture == current_gesture:
                        gesture_frames += 1
                        if gesture_frames >= confirm_frames:
                            if gesture != "parking":
                                gesture_command = gesture
                            gesture_frames = 0
                            if gesture == "parking" and self.parking_position is not None and self.parking_orientation is not None:
                                target_person = next((p for p in person_list if p['id'] == target_id), None)
                                if target_person and gesture_command == "stop" and target_person['distance'] < 2.0:
                                    gesture_command = "parking"
                                    threading.Thread(target=self.send_goal_parking,
                                                     args=(self.parking_position[0], self.parking_position[1],
                                                           self.parking_orientation[0], self.parking_orientation[1]),
                                                     daemon=True).start()
                            elif gesture == "follow" and not self.is_moving_to_goal:
                                target_person = next((p for p in person_list if p['id'] == target_id), None)
                                if target_person and target_person['distance'] > GOAL_DISTANCE_THRESHOLD:
                                    person_pos = self.get_person_map_position(depth_frame, target_person['box'])
                                    if person_pos:
                                        self.goal_position = person_pos
                                        self.last_person_goal = person_pos
                                        threading.Thread(target=self.send_goal, args=(person_pos[0], person_pos[1]),
                                                         daemon=True).start()
                    else:
                        current_gesture = gesture
                        gesture_frames = 1

    def count_fingers(self, hand_landmarks):
        tip_ids = [4, 8, 12, 16, 20]
        fingers = []
        if hand_landmarks.landmark[tip_ids[0]].x < hand_landmarks.landmark[tip_ids[0] - 1].x:
            fingers.append(1)
        else:
            fingers.append(0)
        for id in range(1, 5):
            if hand_landmarks.landmark[tip_ids[id]].y < hand_landmarks.landmark[tip_ids[id] - 2].y:
                fingers.append(1)
            else:
                fingers.append(0)
        return sum(fingers)

    def _active_controller_profile(self):
        config = get_configuration(self.active_controller_config_id)
        profile = config.get('controller', {})
        if not profile:
            profile = get_configuration('p_tapered').get('controller', {})
        return profile

    def _reset_controller_speed(self):
        self.controller_linear_target_mps = 0.0
        self.controller_linear_limited_mps = 0.0
        self.controller_angular_target_rps = 0.0
        self.controller_last_update_monotonic = time.monotonic()

    def _calculate_follow_command(self, distance_m, center_x, profile):
        """Return proportional linear/angular commands and retain slew state."""
        distance_error = float(distance_m) - DESIRED_DISTANCE
        deadband_m = float(profile.get('deadband_m', DISTANCE_TOLERANCE))
        if abs(distance_error) <= deadband_m or distance_m <= MIN_DISTANCE:
            self._reset_controller_speed()
            return 0.0, 0.0, distance_error

        kp = float(profile.get('linear_kp_mps_per_m', 0.75))
        forward_cap = float(profile.get('max_forward_speed_mps', 0.35))
        reverse_cap = float(profile.get('max_reverse_speed_mps', 0.25))
        target_speed = float(np.clip(kp * distance_error, -reverse_cap, forward_cap))
        self.controller_linear_target_mps = target_speed

        accel_limit = profile.get('acceleration_limit_mps2')
        decel_limit = profile.get('deceleration_limit_mps2')
        if accel_limit is None or decel_limit is None:
            limited_speed = target_speed
        else:
            now = time.monotonic()
            if self.controller_last_update_monotonic is None:
                dt = 0.1
            else:
                dt = min(0.25, max(0.0, now - self.controller_last_update_monotonic))
            self.controller_last_update_monotonic = now
            previous_speed = self.controller_linear_limited_mps
            if previous_speed * target_speed < 0.0:
                # Never slew directly from forward to reverse (or vice versa).
                limited_speed = 0.0
            else:
                rate = float(accel_limit if abs(target_speed) > abs(previous_speed) else decel_limit)
                max_delta = rate * dt
                limited_speed = previous_speed + float(
                    np.clip(target_speed - previous_speed, -max_delta, max_delta)
                )
        self.controller_linear_limited_mps = limited_speed

        angular_z = 0.0
        if abs(distance_error) > deadband_m:
            angular_z = -ANGULAR_SPEED * ((float(center_x) - 320.0) / 320.0)
        return limited_speed, angular_z, distance_error

    def control_robot(self):
        global target_id, distance_history, front_gap_history, gesture_command

        profile = self._active_controller_profile()
        target_person = None
        if target_id and person_list:
            target_person = next((p for p in person_list if p['id'] == target_id), None)

        distance_sample_valid = False
        raw_distance_too_close = False
        if target_person:
            try:
                current_camera_depth = float(target_person.get('distance', 0.0))
                roi_stats = target_person.get('roi_stats', {})
                valid_count = int(roi_stats.get('valid_count', 0) or 0)
                distance_sample_valid = (
                    np.isfinite(current_camera_depth)
                    and 0.3 <= current_camera_depth <= 6.0
                    and valid_count > 0
                    and not roi_stats.get('empty', True)
                )
                raw_distance_too_close = (
                    distance_sample_valid and current_camera_depth <= MIN_DISTANCE
                )
            except (TypeError, ValueError):
                current_camera_depth = 0.0
                roi_stats = {}
            if distance_sample_valid:
                distance_history.append(current_camera_depth)
                current_front_gap = target_person.get('front_gap_m', 0.0)
                if np.isfinite(current_front_gap) and current_front_gap > 0:
                    front_gap_history.append(current_front_gap)
            smoothed_distance = np.mean(distance_history) if distance_history else None
            if smoothed_distance is None:
                self.distance_label.setText("Khoảng cách từ camera: --.-- m (độ sâu lỗi)")
            else:
                self.distance_label.setText(f"Khoảng cách từ camera: {smoothed_distance:.2f}m")
        else:
            smoothed_distance = None
            self.distance_label.setText("Khoảng cách từ camera: --.-- m")
        self.controller_distance_sample_valid = distance_sample_valid
        if target_person and not distance_sample_valid:
            if smoothed_distance is None:
                self.distance_label.setText("Khoảng cách từ camera: --.-- m (mẫu lỗi)")
            else:
                self.distance_label.setText(
                    f"Khoảng cách đã lọc: {smoothed_distance:.2f} m — mẫu mới lỗi, robot dừng"
                )

        if self.target_loss_latched:
            if is_tracking and target_person and distance_sample_valid:
                self.target_reacquisition_stable_frames += 1
            else:
                self.target_reacquisition_stable_frames = 0
            required_stable_frames = 10
            if self.target_reacquisition_stable_frames >= required_stable_frames:
                self.target_loss_latched = False
                experiment_logger.log_event(
                    'target_reacquisition_stable',
                    'Mục tiêu đã được nhận diện ổn định; cho phép tính lại lệnh theo dõi',
                    {
                        'stable_frames': self.target_reacquisition_stable_frames,
                        'target_id': target_id or '',
                    },
                )

        angular_z = 0.0
        can_calculate_follow = bool(
            smoothed_distance is not None
            and distance_sample_valid
            and not raw_distance_too_close
            and target_person is not None
            and target_id
            and is_tracking
            and not self.target_loss_latched
            and not self.is_idle
            and not self.is_registering_face
            and not self.is_moving_to_goal
            and gesture_command != 'stop'
        )
        active_scenario = get_scenario(self.current_experiment_scenario_id or '')
        is_crossing_shadow = bool(
            self.experiment_motion_inhibited
            and active_scenario.get('record_shadow_controller', False)
        )

        if self.is_rotating or self.rotation_mode:
            return
        if self.is_moving_to_goal:
            self.status_label.setText("Đang di chuyển đến mục tiêu")
            self.status_label.setStyleSheet("font-size: 12pt; color: blue;")
            self.gesture_label.setText("Cử chỉ: --")
            self.gesture_label.setStyleSheet("font-size: 11pt; color: white;")
            self.send_velocity_command(0, 0)
            return
        if self.is_registering_face and encoding_step in [0, 1, 2]:
            self.send_velocity_command(0, 0)
            return
        if self.is_idle or gesture_command == 'stop':
            self._reset_controller_speed()
            self.send_velocity_command(0, 0)
            if gesture_command == 'stop':
                self.status_label.setText("ĐANG TẠM DỪNG")
                self.status_label.setStyleSheet("font-size: 12pt; color: orange;")
            return

        if gesture_command == 'follow':
            self.gesture_label.setText("Cử chỉ: THEO DÕI (2 ngón)")
            self.gesture_label.setStyleSheet("font-size: 11pt; color: green;")

        if can_calculate_follow:
            cx, _ = target_person['center']
            shadow_linear, angular_z, distance_error = self._calculate_follow_command(
                smoothed_distance, cx, profile
            )
            self.controller_linear_target_mps = float(
                np.clip(
                    float(profile.get('linear_kp_mps_per_m', 0.75)) * distance_error,
                    -float(profile.get('max_reverse_speed_mps', 0.25)),
                    float(profile.get('max_forward_speed_mps', 0.35)),
                )
                if abs(distance_error) > float(profile.get('deadband_m', DISTANCE_TOLERANCE))
                and smoothed_distance > MIN_DISTANCE else 0.0
            )
            self.controller_linear_limited_mps = shadow_linear
            self.controller_angular_target_rps = angular_z
            if distance_error > float(profile.get('deadband_m', DISTANCE_TOLERANCE)):
                self.status_label.setText("Đang tiến — tốc độ giảm theo sai lệch khoảng cách")
                self.status_label.setStyleSheet("font-size: 12pt; color: green;")
            elif distance_error < -float(profile.get('deadband_m', DISTANCE_TOLERANCE)):
                self.status_label.setText("Đang lùi — tốc độ giảm theo sai lệch khoảng cách")
                self.status_label.setStyleSheet("font-size: 12pt; color: red;")
            else:
                self.status_label.setText("Giữ khoảng cách trong vùng ±3 cm")
                self.status_label.setStyleSheet("font-size: 12pt; color: blue;")
        else:
            self._reset_controller_speed()
            shadow_linear = 0.0
            if self.target_loss_latched:
                self.status_label.setText(
                    f"Mất dấu — dừng; chờ nhận diện ổn định "
                    f"({self.target_reacquisition_stable_frames}/10 khung hình)"
                )
                self.status_label.setStyleSheet("font-size: 12pt; color: red;")
            elif not target_person or not is_tracking:
                self.status_label.setText("Chưa nhận lại đúng mục tiêu — robot dừng")
                self.status_label.setStyleSheet("font-size: 12pt; color: red;")
            elif not distance_sample_valid:
                self.status_label.setText("Độ sâu không hợp lệ — robot dừng an toàn")
                self.status_label.setStyleSheet("font-size: 12pt; color: red;")
            elif raw_distance_too_close:
                self.status_label.setText("Quá gần — dừng robot an toàn")
                self.status_label.setStyleSheet("font-size: 12pt; color: red;")
            elif smoothed_distance is not None and smoothed_distance <= MIN_DISTANCE:
                self.status_label.setText("Quá gần — dừng robot an toàn")
                self.status_label.setStyleSheet("font-size: 12pt; color: red;")

        if self.experiment_motion_inhibited:
            # Crossing trials log the counterfactual controller output while
            # keeping the physical MiR stopped for the entire pass.
            if not (is_crossing_shadow and can_calculate_follow):
                self._reset_controller_speed()
                angular_z = 0.0
            self.send_velocity_command(0, 0, source='protocol_motion_lock')
            if is_crossing_shadow:
                self.status_label.setText(
                    "Robot đang khóa đứng yên — lệnh điều khiển giả lập được ghi riêng"
                )
            return

        self.send_velocity_command(shadow_linear, angular_z)

    def log_experiment_frame(self, color_frame, depth_frame, image):
        """Write one row after perception and command generation for this frame."""
        if not self.experiment_run_active:
            return
        global target_id, person_list, is_tracking, is_recognizing, gesture_command
        profile = self._active_controller_profile()
        target_person = next((p for p in person_list if p['id'] == target_id), None)
        target_stats = target_person.get('roi_stats', {}) if target_person else {}
        camera_depth_smoothed = float(np.mean(distance_history)) if distance_history else ''
        smoothed_front_gap = float(np.mean(front_gap_history)) if front_gap_history else ''
        distance_error = ''
        if camera_depth_smoothed != '':
            distance_error = float(camera_depth_smoothed - DESIRED_DISTANCE)

        if self.is_idle:
            gate_state = 'idle'
        elif self.is_moving_to_goal:
            gate_state = 'goal_navigation'
        elif self.is_rotating:
            gate_state = 'rotating'
        elif not target_person:
            gate_state = 'no_target'
        elif gesture_command == 'stop':
            gate_state = 'stop'
        elif target_stats.get('empty', True):
            gate_state = 'invalid_depth'
        elif camera_depth_smoothed != '' and camera_depth_smoothed > DESIRED_DISTANCE + DISTANCE_TOLERANCE:
            gate_state = 'forward'
        elif camera_depth_smoothed != '' and camera_depth_smoothed < DESIRED_DISTANCE - DISTANCE_TOLERANCE:
            gate_state = 'backward'
        else:
            gate_state = 'hold'

        def frame_value(frame, method_name):
            try:
                return getattr(frame, method_name)()
            except Exception:
                return ''

        detections = []
        for person in person_list:
            x1, y1, x2, y2 = person['box']
            stats = person.get('roi_stats', {})
            detections.append({
                'id': person['id'],
                'bbox': [x1, y1, x2, y2],
                'camera_depth_m': person.get('distance', ''),
                'front_gap_m': stats.get('front_gap_m', ''),
                'lateral_offset_m': stats.get('lateral_offset_m', ''),
                'roi_empty': stats.get('empty', ''),
            })

        image_height, image_width = image.shape[:2]
        target_box = target_person.get('box') if target_person else ('', '', '', '')
        center = target_person.get('center') if target_person else ('', '')
        row = {
            'ros_time_s': rospy.get_time(),
            'auto_phase_id': (
                getattr(self.experiment_panel, 'current_phase_id', '')
                if self.experiment_run_active else ''
            ),
            'frame_idx': self.experiment_frame_idx,
            'realsense_frame_number': frame_value(color_frame, 'get_frame_number'),
            'frame_timestamp_ms': frame_value(color_frame, 'get_timestamp'),
            'color_timestamp_ms': frame_value(color_frame, 'get_timestamp'),
            'depth_timestamp_ms': frame_value(depth_frame, 'get_timestamp'),
            'rgbd_aligned': True,
            'image_width': image_width,
            'image_height': image_height,
            'person_count': len(person_list),
            'all_detections_json': json.dumps(detections, ensure_ascii=False, default=str),
            'target_id': target_id or '',
            'target_present': bool(target_person),
            'target_x1': target_box[0],
            'target_y1': target_box[1],
            'target_x2': target_box[2],
            'target_y2': target_box[3],
            'target_center_x': center[0],
            'target_center_y': center[1],
            'roi_sample_count': target_stats.get('sample_count', ''),
            'roi_nonzero_count': target_stats.get('nonzero_count', ''),
            'roi_valid_count': target_stats.get('valid_count', ''),
            'roi_zero_count': target_stats.get('zero_count', ''),
            'roi_out_of_range_count': target_stats.get('out_of_range_count', ''),
            'roi_median_nonzero_m': target_stats.get('median_nonzero_m', ''),
            'roi_median_filtered_m': target_stats.get('median_filtered_m', ''),
            'roi_empty': target_stats.get('empty', ''),
            'depth_zero_ratio': target_stats.get('zero_ratio', ''),
            'geometry_valid_count': target_stats.get('geometry_valid_count', ''),
            'target_camera_x_m': target_stats.get('camera_x_m', ''),
            'target_camera_y_m': target_stats.get('camera_y_m', ''),
            'target_camera_z_m': target_stats.get('camera_z_m', ''),
            'target_forward_from_camera_m': target_stats.get('forward_from_camera_m', ''),
            'target_front_gap_m': target_stats.get('front_gap_m', ''),
            'target_lateral_offset_m': target_stats.get('lateral_offset_m', ''),
            'target_height_m': target_stats.get('target_height_m', ''),
            'distance_history_len': len(distance_history),
            'distance_history_last_m': distance_history[-1] if distance_history else '',
            'camera_depth_smoothed_m': camera_depth_smoothed,
            'front_gap_history_len': len(front_gap_history),
            'front_gap_history_last_m': front_gap_history[-1] if front_gap_history else '',
            'front_gap_smoothed_m': smoothed_front_gap,
            'front_gap_error_m': '',
            # Keep the old columns for analyzer compatibility.  They retain
            # the original camera-distance controller reference.
            'distance_smoothed_m': camera_depth_smoothed,
            'distance_error_m': distance_error,
            'ground_truth_distance_m': '',
            'distance_error_to_ground_truth_m': '',
            'gate_state': gate_state,
            'gesture_command': gesture_command or '',
            'is_tracking': bool(is_tracking),
            'is_recognizing': bool(is_recognizing),
            'is_idle': bool(self.is_idle),
            'is_moving_to_goal': bool(self.is_moving_to_goal),
            'is_rotating': bool(self.is_rotating),
            'controller_profile_id': profile.get('profile_id', self.active_controller_config_id),
            'distance_sample_valid': self.controller_distance_sample_valid,
            'controller_requested_linear_x_mps': self.controller_linear_target_mps,
            'controller_rate_limited_linear_x_mps': self.controller_linear_limited_mps,
            'controller_angular_z_rps': self.controller_angular_target_rps,
            'controller_shadow_active': bool(
                self.experiment_motion_inhibited
                and get_scenario(self.current_experiment_scenario_id or '')
                .get('record_shadow_controller', False)
            ),
            'target_loss_latched': bool(self.target_loss_latched),
            'target_reacquisition_stable_frames': self.target_reacquisition_stable_frames,
            'command_linear_x_mps': last_command_linear_x,
            'command_angular_z_rps': last_command_angular_z,
            'command_generated': bool(self.command_generated_this_frame),
            'command_published': bool(last_command_published),
            'control_mode': control_mode,
        }
        experiment_logger.log_frame(row)
        self.experiment_frame_idx += 1

    def update_gui(self):
        try:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                return
            color_image = self.process_frame(color_frame, depth_frame)

            # Calculate person_map_pos
            global person_map_pos
            if is_tracking and target_id and not self.is_moving_to_goal and not self.is_rotating:
                target_person = next((p for p in person_list if p['id'] == target_id), None)
                if target_person:
                    person_map_pos = self.get_person_map_position(depth_frame, target_person['box'])
                else:
                    person_map_pos = None
            else:
                person_map_pos = None

            # Draw bounding boxes
            if is_tracking and target_id:
                target_person = next((p for p in person_list if p['id'] == target_id), None)
                if target_person:
                    x1, y1, x2, y2 = target_person['box']
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(color_image, "MỤC TIÊU", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            # Convert to QImage and display
            rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.video_label.setPixmap(pixmap)
            self.command_generated_this_frame = False
            tracked_person = None
            if is_tracking and target_id:
                tracked_person = next(
                    (person for person in person_list if person['id'] == target_id),
                    None,
                )
            target_distance_m = (
                tracked_person['distance']
                if tracked_person and tracked_person['distance'] > 0.0
                else None
            )
            self.experiment_panel.update_operator_state(
                target_selected=bool(is_tracking and target_id),
                follow_enabled=bool(is_tracking and target_id and gesture_command == "follow" and not self.is_idle),
                target_distance_m=target_distance_m,
            )
            self.control_robot()
            self.experiment_panel.update_controller_safety_state(
                is_tracking=is_tracking,
                loss_latched=self.target_loss_latched,
                stable_frames=self.target_reacquisition_stable_frames,
            )
            self.log_experiment_frame(color_frame, depth_frame, color_image)
        except Exception as e:
            print(f"Error: {e}")
            experiment_logger.log_event('frame_error', str(e))

    def on_video_click(self, event):
        global target_id, target_encodings, encoding_step, is_tracking, is_recognizing, gesture_command, target_brightness, target_histogram
        if current_frame is None:
            return
        x = event.pos().x()
        y = event.pos().y()

        # Adjust for label centering
        label_size = self.video_label.size()
        frame_width, frame_height = 640, 480
        offset_x = (label_size.width() - frame_width) // 2
        offset_y = (label_size.height() - frame_height) // 2
        x -= offset_x
        y -= offset_y
        if x < 0 or y < 0 or x >= frame_width or y >= frame_height:
            return

        for person in person_list:
            x1, y1, x2, y2 = person['box']
            if x1 <= x <= x2 and y1 <= y <= y2:
                self.is_registering_face = True  # Bật trạng thái đăng ký khuôn mặt
                self.is_idle = False  # Tắt trạng thái chờ khi bắt đầu đăng ký
                face_region = current_frame[y1:y2, x1:x2]
                face_region_rgb = cv2.cvtColor(face_region, cv2.COLOR_BGR2RGB)
                faces = face_app.get(face_region_rgb)
                if len(faces) == 0:
                    self.face_status_label.setText("Không tìm thấy khuôn mặt, vui lòng thử lại")
                    self.face_status_label.setStyleSheet("font-size: 11pt; color: red;")
                    self.is_registering_face = False
                    return

                # Chọn khuôn mặt có diện tích lớn nhất
                max_area_face = None
                max_area = 0
                for face in faces:
                    bbox = face.bbox.astype(int)
                    area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
                    if area > max_area:
                        max_area = area
                        max_area_face = face
                if max_area_face is None:
                    self.is_registering_face = False
                    return

                face_embedding = max_area_face.embedding

                if encoding_step == 0:
                    target_encodings.append(face_embedding)
                    encoding_step = 1
                    self.status_label.setText("Quay mặt sang trái (~45°) và nhấp chuột vào người")
                    self.status_label.setStyleSheet("font-size: 12pt; color: yellow;")
                    self.face_status_label.setText("Hoàn thành bước 1/3")
                    self.face_status_label.setStyleSheet("font-size: 11pt; color: yellow;")
                elif encoding_step == 1:
                    target_encodings.append(face_embedding)
                    encoding_step = 2
                    self.status_label.setText("Quay mặt sang phải (~45°) và nhấp chuột vào người")
                    self.status_label.setStyleSheet("font-size: 12pt; color: yellow;")
                    self.face_status_label.setText("Hoàn thành bước 2/3")
                    self.face_status_label.setStyleSheet("font-size: 11pt; color: yellow;")
                elif encoding_step == 2:
                    target_encodings.append(face_embedding)
                    encoding_step = 3
                    target_id = person['id']
                    is_tracking = True
                    self.is_registering_face = False  # Tắt trạng thái đăng ký sau khi hoàn tất
                    self.is_idle = False  # Đảm bảo robot không ở trạng thái chờ sau khi đăng ký
                    gesture_command = "stop"
                    self.face_status_label.setText("Hoàn thành bước 3/3")
                    self.face_status_label.setStyleSheet("font-size: 11pt; color: green;")
                    self.status_label.setText("Đã đăng ký khuôn mặt — robot đang tạm dừng")
                    self.status_label.setStyleSheet("font-size: 12pt; color: green;")
                    QTimer.singleShot(2000, lambda: self.face_status_label.setText("Đã đăng ký khuôn mặt"))
                    QTimer.singleShot(2000,
                                      lambda: self.face_status_label.setStyleSheet("font-size: 11pt; color: green;"))
                break

    def cancel_tracking(self):
        global target_id, encoding_step, gesture_command, target_encodings, is_tracking, is_recognizing, is_sound_playing
        target_id = None
        encoding_step = 0
        gesture_command = "stop"
        target_encodings = []
        is_tracking = False
        is_recognizing = False
        self.is_registering_face = False  # Đặt lại trạng thái đăng ký khuôn mặt
        self.is_idle = True  # Đặt robot vào trạng thái chờ
        if is_sound_playing:
            alert_sound.stop()
            is_sound_playing = False
        self.send_velocity_command(0, 0)
        self.status_label.setText("Nhìn thẳng vào camera và nhấp chuột vào người")
        self.status_label.setStyleSheet("font-size: 12pt; color: white;")
        self.face_status_label.setText("Chưa đăng ký khuôn mặt")
        self.face_status_label.setStyleSheet("font-size: 11pt; color: white;")
        self.gesture_label.setText("Cử chỉ: --")
        self.gesture_label.setStyleSheet("font-size: 11pt; color: white;")
        # Reset rotation if active
        if self.is_rotating:
            self.rotation_timer.stop()
            self.is_rotating = False
            self.rotation_target = None
            self.rotation_status_label.setText("Chưa chọn hướng quay")
            self.rotation_status_label.setStyleSheet("font-size: 13pt; color: purple;")

    def update_position_continuously(self):
        global robot_pose
        if not rospy.is_shutdown():
            try:
                (trans, rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                robot_pose = (trans[0], trans[1], euler_from_quaternion(rot)[2])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            self.map_canvas.draw_map()

    def get_person_map_position(self, depth_frame, box):
        global robot_pose
        if robot_pose is None or depth_frame is None:
            return None
        try:
            x1, y1, x2, y2 = box
            center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
            distance = depth_frame.get_distance(center_x, center_y)
            if distance <= 0:
                return None
            fov = 69
            angle = (center_x - 320) * (fov / 640)
            angle_rad = np.radians(angle)
            z_cam = distance * np.cos(angle_rad)
            x_cam = distance * np.sin(angle_rad)
            x_rel = z_cam
            y_rel = -x_cam
            robot_x, robot_y, robot_theta = robot_pose
            map_x = robot_x + x_rel * np.cos(robot_theta) - y_rel * np.sin(robot_theta)
            map_y = robot_y + x_rel * np.sin(robot_theta) + y_rel * np.cos(robot_theta)
            return (map_x, map_y)
        except Exception as e:
            rospy.logerr(f"Person position calculation error: {str(e)}")
            return None

    def send_goal(self, x, y):
        global gesture_command, is_alert2_playing
        if self._suppress_occlusion_navigation_goal('send_goal'):
            gesture_command = 'stop'
            return
        if control_mode != 'live':
            experiment_logger.log_event(
                'move_base_goal_suppressed',
                'Dry-run control mode prevented a navigation goal',
                {'x_m': x, 'y_m': y, 'control_mode': control_mode},
            )
            self.is_moving_to_goal = False
            self.goal_position = None
            gesture_command = 'stop'
            return
        self.is_moving_to_goal = True
        while self.is_moving_to_goal and not rospy.is_shutdown():
            try:
                client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                client.wait_for_server()
                if self._suppress_occlusion_navigation_goal('send_goal_before_publish'):
                    gesture_command = 'stop'
                    return
                if not self.is_moving_to_goal:
                    break
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = x
                goal.target_pose.pose.position.y = y
                z, w = self.calculate_orientation(robot_pose[:2], (x, y))
                goal.target_pose.pose.orientation.z = z
                goal.target_pose.pose.orientation.w = w
                client.send_goal(goal)
                client.wait_for_result()
                state = client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Robot reached the goal")
                    self.is_moving_to_goal = False
                    self.goal_position = None
                    self.path_points = []
                    gesture_command = "stop"
                    if is_alert2_playing:
                        alert2_sound.stop()
                        is_alert2_playing = False
                    self.map_canvas.draw_map()
                    break
                else:
                    rospy.logwarn("Failed to reach goal, state: %d. Retrying in 1.5 seconds...", state)
                    if not pygame.mixer.get_busy():
                        alert2_sound.play(loops=0)
                        is_alert2_playing = True
                    rospy.sleep(2.0)
            except Exception as e:
                rospy.logerr(f"Error sending goal: {str(e)}")
                rospy.sleep(1)

    def send_goal_parking(self, x_parking, y_parking, z_parking, w_parking):
        global is_alert2_playing, gesture_command
        if self._suppress_occlusion_navigation_goal('send_goal_parking'):
            gesture_command = 'stop'
            return
        if control_mode != 'live':
            experiment_logger.log_event(
                'parking_goal_suppressed',
                'Dry-run control mode prevented a parking goal',
                {
                    'x_m': x_parking,
                    'y_m': y_parking,
                    'control_mode': control_mode,
                },
            )
            self.is_moving_to_goal = False
            self.goal_position = None
            gesture_command = 'stop'
            return
        self.is_moving_to_goal = True
        self.goal_position = (x_parking, y_parking)
        while self.is_moving_to_goal and not rospy.is_shutdown():
            try:
                client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                client.wait_for_server()
                if self._suppress_occlusion_navigation_goal('send_goal_parking_before_publish'):
                    gesture_command = 'stop'
                    return
                if not self.is_moving_to_goal:
                    break
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = x_parking
                goal.target_pose.pose.position.y = y_parking
                goal.target_pose.pose.orientation.z = z_parking
                goal.target_pose.pose.orientation.w = w_parking
                client.send_goal(goal)
                client.wait_for_result()
                state = client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Robot reached the parking spot")
                    self.is_moving_to_goal = False
                    self.goal_position = None
                    self.path_points = []
                    gesture_command = "stop"
                    if is_alert2_playing:
                        alert2_sound.stop()
                        is_alert2_playing = False
                    self.map_canvas.draw_map()
                    break
                else:
                    rospy.logwarn("Failed to reach parking spot, state: %d. Retrying in 1.5 seconds...", state)
                    if not pygame.mixer.get_busy():
                        alert2_sound.play(loops=0)
                        is_alert2_playing = True
                    rospy.sleep(2.0)
            except Exception as e:
                rospy.logerr(f"Error sending parking goal: {str(e)}")
                rospy.sleep(1)

    def closeEvent(self, event):
        global experiment_capture_active
        self.send_velocity_command(0, 0)
        if self.experiment_run_active:
            experiment_logger.log_event(
                'scenario_aborted',
                'GUI closed before operator ended the protocol run',
                {'run_id': self.current_experiment_run_id},
            )
            self.experiment_run_active = False
        experiment_capture_active = False
        pipeline.stop()
        if is_sound_playing:
            alert_sound.stop()
        if is_alert2_playing:
            alert2_sound.stop()
        experiment_logger.close('gui_closed')
        rospy.signal_shutdown("GUI closed")
        event.accept()


if __name__ == "__main__":
    # app already created in get_robot_ip()
    window = MainWindow()
    window.showMaximized()
    exit_code = app.exec_()
    experiment_logger.close('application_exit')
    sys.exit(exit_code)

