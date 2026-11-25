"""Qt operator panel for repeatable E1 data collection."""

import os
import json
import math
import time
from pathlib import Path

from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from experiment_protocol import (
    AUTO_MAX_ATTEMPTS,
    AUTO_MIN_FRAME_RATE_HZ,
    AUTO_MIN_ODOMETRY_RATE_HZ,
    AUTO_MIN_VALID_ROI_RATIO,
    AUTO_PREPARE_SECONDS,
    AUTO_RESET_SECONDS,
    AUTO_START_DISTANCE_TOLERANCE_M,
    AUTO_START_STABILITY_FRAMES,
    CONFIGURATIONS,
    E1_CAPTURE_SCENARIO_IDS,
    E1_COLLECTION_STAGES,
    E1_REPLAY_PROFILE_IDS,
    SCENARIOS,
    get_configuration,
    get_scenario,
)


EVENT_LABELS = {
    "scenario_start": "Bắt đầu lần đo",
    "scenario_end": "Kết thúc lần đo",
    "calibration_point": "Ghi điểm hiệu chuẩn",
    "crossing_start": "Bắt đầu cắt ngang",
    "crossing_end": "Kết thúc cắt ngang",
    "approach_start": "Bắt đầu tiếp cận thật",
    "approach_end": "Kết thúc tiếp cận thật",
    "robot_approach_start": "MiR bắt đầu tự tiến tới",
    "robot_approach_end": "MiR đạt khoảng cách đích",
    "robot_approach_timeout": "MiR chưa tới khoảng cách đích đúng hạn",
    "retreat_start": "Bắt đầu lùi ra",
    "retreat_end": "Kết thúc lùi ra",
    "hold_start": "Bắt đầu giữ khoảng cách",
    "hold_end": "Kết thúc giữ khoảng cách",
    "depth_fault_start": "Độ sâu lỗi bắt đầu",
    "depth_fault_end": "Độ sâu lỗi kết thúc",
    "target_lost": "Mục tiêu mất",
    "target_reacquired": "Mục tiêu nhận lại",
    "target_id_changed": "Mã người theo dõi thay đổi",
    "operator_note": "Ghi chú của người vận hành",
}


class ExperimentPanel(QWidget):
    """Run a timed E1 queue and show the operator each physical action."""

    run_start_requested = pyqtSignal(dict)
    run_stop_requested = pyqtSignal(dict)
    event_requested = pyqtSignal(str, dict)
    ground_truth_requested = pyqtSignal(float, dict)
    motion_inhibit_requested = pyqtSignal(bool)
    reposition_control_requested = pyqtSignal(bool)

    def __init__(self, control_mode, parent=None):
        super().__init__(parent)
        self.control_mode = control_mode
        self.run_active = False
        self.run_started_at = None
        self.operator_target_selected = False
        self.operator_follow_enabled = False
        self.auto_state = "waiting_for_target"
        self.auto_countdown = 0
        self.operator_distance_m = None
        self.auto_stability_frames = 0
        self.auto_stability_samples = []
        self.current_start_gate = None
        self.auto_queue = []
        self.auto_index = 0
        self.current_task = None
        self.current_run_info = None
        self.current_run_quality = None
        self.current_phase_id = ""
        self.phase_events_emitted = set()
        self.robot_approach_motion_released = False
        self.robot_approach_distance_history = []
        self.robot_approach_stable_frames = 0
        self.robot_approach_completed = False
        self.robot_approach_final_distance_m = None
        self.reposition_control_active = False
        self.completed_task_ids = set()
        self.exhausted_task_ids = set()
        self.skipped_scenarios = {}
        self.all_tasks = []
        self.active_task_ids = set()
        self.collection_stage = "development"
        self._last_collection_stage_index = 0
        self.evaluation_started = False
        self.collection_stage_combo = QComboBox()
        for stage in E1_COLLECTION_STAGES:
            self.collection_stage_combo.addItem(stage["label"], stage["id"])
        self.freeze_c1_check = QCheckBox(
            "Đã gán nhãn tập hiệu chỉnh, triển khai/hiệu chỉnh và khóa C1; không chỉnh trên tập đánh giá"
        )
        self.freeze_c1_id_edit = QLineEdit()
        self.freeze_c1_id_edit.setPlaceholderText("Mã phiên bản C1/tham số đã khóa")
        self._load_auto_history_and_queue()

        mode_description = {
            "live": "ĐANG PHÁT LỆNH tới MiR",
            "dry_run": "CHỈ GHI NHẬT KÝ — không phát lệnh chuyển động",
            "pilot": "CHỈ GHI NHẬT KÝ — không phát lệnh chuyển động",
            "log_only": "CHỈ GHI NHẬT KÝ — không phát lệnh chuyển động",
        }.get(control_mode, "Chưa xác định")
        self.mode_label = QLabel(f"Chế độ điều khiển: {mode_description}")
        self.mode_label.setStyleSheet(
            "font-weight:bold;color:#b02a37;" if control_mode == "live"
            else "font-weight:bold;color:#198754;"
        )

        self.config_combo = QComboBox()
        for config in CONFIGURATIONS:
            self.config_combo.addItem(
                config["label"],
                config["id"],
            )
        self.config_combo.currentIndexChanged.connect(self._refresh_description)

        self.scenario_combo = QComboBox()
        for scenario in SCENARIOS:
            self.scenario_combo.addItem(scenario["title"], scenario["id"])
        self.scenario_combo.currentIndexChanged.connect(self._scenario_changed)

        self.variant_edit = QLineEdit()
        self.variant_edit.setPlaceholderText("Ví dụ: cat_01_trai-phai_1p4m")
        self.repeat_spin = QSpinBox()
        self.repeat_spin.setRange(1, 99)
        self.repeat_spin.setValue(1)
        self.operator_edit = QLineEdit()
        self.operator_edit.setPlaceholderText("Tên người thực hiện đo")
        self.notes_edit = QLineEdit()
        self.notes_edit.setPlaceholderText("Ánh sáng, nền, bất thường, can thiệp...")

        self.participant_height_spin = QDoubleSpinBox()
        self.participant_height_spin.setRange(0.5, 2.5)
        self.participant_height_spin.setDecimals(2)
        self.participant_height_spin.setSingleStep(0.01)
        self.participant_height_spin.setValue(1.76)
        self.walk_speed_spin = QDoubleSpinBox()
        self.walk_speed_spin.setRange(0.0, 2.0)
        self.walk_speed_spin.setDecimals(2)
        self.walk_speed_spin.setSingleStep(0.05)
        self.walk_speed_spin.setSuffix(" m/s")
        self.crossing_angle_spin = QDoubleSpinBox()
        self.crossing_angle_spin.setRange(0.0, 180.0)
        self.crossing_angle_spin.setDecimals(0)
        self.crossing_angle_spin.setSuffix("°")
        self.crossing_angle_spin.setValue(90.0)
        self.crowd_count_spin = QSpinBox()
        self.crowd_count_spin.setRange(1, 50)
        self.crowd_count_spin.setValue(1)

        self.safety_check = QCheckBox("Đã kiểm tra vùng chạy và nút dừng khẩn cấp")
        self.ground_truth_check = QCheckBox(
            "Đã chuẩn bị khoảng cách chuẩn độc lập liên tục, đồng bộ với RGB-D"
        )
        self.safety_check.toggled.connect(self._prerequisite_changed)
        self.ground_truth_check.toggled.connect(self._prerequisite_changed)
        self.collection_stage_combo.currentIndexChanged.connect(
            self._collection_stage_changed
        )
        self.freeze_c1_check.toggled.connect(self._refresh_stage_controls)
        self.freeze_c1_id_edit.textChanged.connect(self._refresh_stage_controls)

        self.reference_distance_spin = QDoubleSpinBox()
        self.reference_distance_spin.setRange(0.0, 20.0)
        self.reference_distance_spin.setDecimals(3)
        self.reference_distance_spin.setSingleStep(0.05)
        self.reference_distance_spin.setSuffix(" m")
        self.reference_distance_spin.setSpecialValueText("Nhập khoảng cách")
        self.reference_source_combo = QComboBox()
        for source_id, source_label in (
            ("synchronized_video", "Video ngoài đồng bộ"),
            ("laser_rangefinder", "Thước laser"),
            ("floor_marks", "Mốc sàn đã đo"),
            ("other_independent", "Nguồn độc lập khác"),
        ):
            self.reference_source_combo.addItem(source_label, source_id)
        self.reference_source_combo.setCurrentIndex(
            self.reference_source_combo.findData("synchronized_video")
        )
        self.reference_definition_combo = QComboBox()
        for definition_id, definition_label in (
            ("camera_to_person_roi", "Tâm camera → điểm ROI trên người"),
            ("robot_front_edge_to_person", "Mép trước MiR → điểm trên người"),
        ):
            self.reference_definition_combo.addItem(definition_label, definition_id)
        self.reference_note_edit = QLineEdit()
        self.reference_note_edit.setPlaceholderText("Điểm đo, sai số, ghi chú...")
        self.reference_button = QPushButton("Ghi mốc khoảng cách độc lập")
        self.reference_button.clicked.connect(self._ground_truth_clicked)
        self.reference_button.setEnabled(False)

        self.description_label = QLabel()
        self.description_label.setWordWrap(True)
        self.description_label.setStyleSheet("color: #174a67;")
        self.preparation_edit = QTextEdit()
        self.preparation_edit.setReadOnly(True)
        self.preparation_edit.setMaximumHeight(90)
        self.procedure_edit = QTextEdit()
        self.procedure_edit.setReadOnly(True)
        self.procedure_edit.setMaximumHeight(150)
        self.ground_truth_label = QLabel()
        self.ground_truth_label.setWordWrap(True)
        self.ground_truth_label.setStyleSheet("color: #174a67;")
        self.events_label = QLabel()
        self.events_label.setWordWrap(True)
        self.events_label.setStyleSheet("color: #555;")
        self.metrics_label = QLabel()
        self.metrics_label.setWordWrap(True)
        self.metrics_label.setStyleSheet("color: #555;")

        self.progress_label = QLabel()
        self.progress_label.setWordWrap(True)
        self.progress_label.setStyleSheet("color: #174a67;")
        self.next_action_label = QLabel()
        self.next_action_label.setWordWrap(True)
        self.next_action_label.setStyleSheet("font-weight:bold;color:#198754;")
        self.auto_task_label = QLabel()
        self.auto_task_label.setWordWrap(True)
        self.auto_task_label.setStyleSheet("font-size:16pt;font-weight:bold;color:#16324f;")
        self.auto_phase_label = QLabel()
        self.auto_phase_label.setWordWrap(True)
        self.auto_phase_label.setStyleSheet("font-size:15pt;font-weight:bold;color:#b02a37;")
        self.controller_safety_label = QLabel("Trạng thái an toàn của bộ điều khiển sẽ hiện tại đây.")
        self.controller_safety_label.setWordWrap(True)
        self.controller_safety_label.setStyleSheet("font-size:11pt;font-weight:bold;color:#0b5ed7;")
        self.auto_countdown_label = QLabel()
        self.auto_countdown_label.setStyleSheet("font-size:14pt;font-weight:bold;color:#174a67;")
        self.abort_button = QPushButton("Dừng phiên tự động và phát lệnh dừng robot")
        self.abort_button.setStyleSheet("background:#dc3545;color:white;font-weight:bold;padding:8px;")
        self.abort_button.clicked.connect(self.abort_auto_session)
        self.skip_scenario_button = QPushButton("Bỏ qua kịch bản này — giữ nguyên log đã lưu")
        self.skip_scenario_button.setStyleSheet(
            "background:#f0ad4e;color:#222;font-weight:bold;padding:8px;"
        )
        self.skip_scenario_button.clicked.connect(self.skip_current_scenario)
        self.skipped_scenario_combo = QComboBox()
        self.restore_scenario_button = QPushButton("Khôi phục kịch bản đã bỏ qua")
        self.restore_scenario_button.clicked.connect(self.restore_skipped_scenario)
        restore_layout = QHBoxLayout()
        self.skipped_scenario_label = QLabel("Kịch bản đã bỏ qua:")
        restore_layout.addWidget(self.skipped_scenario_label)
        restore_layout.addWidget(self.skipped_scenario_combo, 1)
        restore_layout.addWidget(self.restore_scenario_button)
        self.skip_help_label = QLabel(
            "Bỏ qua sẽ giữ log các lượt đã lưu và loại các lượt còn lại của kịch bản khỏi hàng đợi. "
            "Có thể khôi phục kịch bản sau."
        )
        self.skip_help_label.setWordWrap(True)
        self.skip_help_label.setStyleSheet("color:#6c5b20;")
        self.progress_refresh_button = QPushButton("Cập nhật tiến độ")
        self.progress_refresh_button.clicked.connect(self.refresh_progress)

        form = QFormLayout()
        form.addRow("Cấu hình thực nghiệm:", self.config_combo)
        form.addRow("Kịch bản đo:", self.scenario_combo)
        form.addRow("Mã đường đi/biến thể:", self.variant_edit)
        form.addRow("Lần lặp:", self.repeat_spin)
        form.addRow("Người thực hiện:", self.operator_edit)
        form.addRow("Chiều cao người (kể cả giày):", self.participant_height_spin)
        form.addRow("Tốc độ đi bộ đặt ra/ước lượng:", self.walk_speed_spin)
        form.addRow("Góc đường đi so với trục robot:", self.crossing_angle_spin)
        form.addRow("Số người trong cảnh:", self.crowd_count_spin)
        form.addRow("Điều kiện và ghi chú:", self.notes_edit)

        checks = QHBoxLayout()
        checks.addWidget(self.safety_check)
        checks.addWidget(self.ground_truth_check)

        self.start_button = QPushButton("Bắt đầu lần đo")
        self.start_button.setStyleSheet("background:#198754;color:white;font-weight:bold;padding:6px;")
        self.start_button.clicked.connect(self._start_clicked)
        self.stop_button = QPushButton("Kết thúc lần đo")
        self.stop_button.setStyleSheet("background:#dc3545;color:white;font-weight:bold;padding:6px;")
        self.stop_button.clicked.connect(self._stop_clicked)
        self.stop_button.setEnabled(False)

        self.event_combo = QComboBox()
        for event_id, event_label in EVENT_LABELS.items():
            if event_id not in {"scenario_start", "scenario_end"}:
                self.event_combo.addItem(event_label, event_id)
        self.event_button = QPushButton("Đánh dấu mốc")
        self.event_button.clicked.connect(self._event_clicked)
        self.event_button.setEnabled(False)

        run_buttons = QHBoxLayout()
        run_buttons.addWidget(self.start_button)
        run_buttons.addWidget(self.stop_button)
        run_buttons.addWidget(self.event_combo)
        run_buttons.addWidget(self.event_button)

        reference_box = QGroupBox("Khoảng cách chuẩn độc lập")
        reference_layout = QGridLayout(reference_box)
        reference_layout.addWidget(QLabel("Giá trị:"), 0, 0)
        reference_layout.addWidget(self.reference_distance_spin, 0, 1)
        reference_layout.addWidget(QLabel("Nguồn đo:"), 0, 2)
        reference_layout.addWidget(self.reference_source_combo, 0, 3)
        reference_layout.addWidget(QLabel("Quy chiếu:"), 1, 0)
        reference_layout.addWidget(self.reference_definition_combo, 1, 1, 1, 3)
        reference_layout.addWidget(self.reference_note_edit, 2, 0, 1, 3)
        reference_layout.addWidget(self.reference_button, 2, 3)
        self.reference_help_label = QLabel(
            "Mỗi lần bấm sẽ lưu một mốc cùng thời gian và khung hình gần nhất. Mốc thủ công phù hợp để kiểm tra khoảng cách; "
            "muốn tính lệnh lùi sai, thời gian hồi phục và độ giật cần chuỗi khoảng cách chuẩn liên tục, đồng bộ từ video/hệ đo ngoài."
        )
        self.reference_help_label.setWordWrap(True)
        self.reference_help_label.setStyleSheet("color:#8a3b12;")
        reference_layout.addWidget(self.reference_help_label, 3, 0, 1, 4)

        self.status_label = QLabel("Chưa bắt đầu lần đo")
        self.status_label.setStyleSheet("font-weight:bold;color:#6c757d;")
        self.elapsed_label = QLabel("Thời gian: 00:00")
        self.run_dir_label = QLabel("Thư mục nhật ký: --")
        self.run_dir_label.setWordWrap(True)
        self.reposition_control_button = QPushButton(
            "Tạm dừng đo — bật điều khiển robot bình thường"
        )
        self.reposition_control_button.setCheckable(True)
        self.reposition_control_button.setStyleSheet(
            "background:#6c757d;color:white;font-weight:bold;padding:8px;"
        )
        self.reposition_control_button.toggled.connect(
            self._reposition_control_toggled
        )
        self.reposition_control_button.setEnabled(False)
        self.reposition_help_label = QLabel(
            "Giữa các lượt, bấm nút để tạm dừng ghi log và điều khiển robot như bình thường để đưa robot về mốc. "
            "Robot không tự khóa chỉ vì đã tới 2,2 m: khi đã về vị trí, bấm nút lần nữa để khóa robot; "
            "sau đó camera–ROI ở 2,2 m ±0,05 m đủ 10 khung hình liên tiếp thì lượt tiếp theo tự bắt đầu. "
            "Chỉ di chuyển khi lối đi trống và nút dừng khẩn cấp trong tầm với."
        )
        self.reposition_help_label.setWordWrap(True)
        self.reposition_help_label.setStyleSheet("color:#174a67;")
        self.reposition_status_label = QLabel(
            "Chưa bật điều khiển thường trong lúc đặt lại vị trí."
        )
        self.reposition_status_label.setWordWrap(True)

        instructions = QGroupBox("Cách thực hiện phép đo")
        instructions_layout = QVBoxLayout(instructions)
        instructions_layout.addWidget(self.description_label)
        instructions_layout.addWidget(QLabel("Chuẩn bị:"))
        instructions_layout.addWidget(self.preparation_edit)
        instructions_layout.addWidget(QLabel("Các bước thực hiện:"))
        instructions_layout.addWidget(self.procedure_edit)
        instructions_layout.addWidget(QLabel("Cách lấy khoảng cách chuẩn:"))
        instructions_layout.addWidget(self.ground_truth_label)

        record_box = QGroupBox("Thông tin cần ghi")
        record_layout = QVBoxLayout(record_box)
        record_layout.addWidget(self.events_label)
        record_layout.addWidget(self.metrics_label)

        progress_box = QGroupBox("Tiến độ thu thập dữ liệu")
        progress_layout = QVBoxLayout(progress_box)
        progress_layout.addWidget(self.progress_label)
        progress_layout.addWidget(self.next_action_label)

        automatic_box = QGroupBox("Thu thập tự động — làm theo hướng dẫn đang hiện")
        automatic_layout = QVBoxLayout(automatic_box)
        automatic_layout.addWidget(QLabel("Giai đoạn thu dữ liệu:"))
        automatic_layout.addWidget(self.collection_stage_combo)
        automatic_layout.addWidget(self.freeze_c1_check)
        automatic_layout.addWidget(self.freeze_c1_id_edit)
        automatic_layout.addWidget(self.auto_task_label)
        automatic_layout.addWidget(self.auto_phase_label)
        automatic_layout.addWidget(self.controller_safety_label)
        automatic_layout.addWidget(self.auto_countdown_label)
        automatic_layout.addWidget(self.status_label)
        automatic_layout.addWidget(self.elapsed_label)
        automatic_layout.addWidget(self.run_dir_label)
        automatic_layout.addWidget(self.reposition_control_button)
        automatic_layout.addWidget(self.reposition_help_label)
        automatic_layout.addWidget(self.reposition_status_label)
        automatic_layout.addWidget(self.skip_scenario_button)
        automatic_layout.addWidget(self.skip_help_label)
        automatic_layout.addLayout(restore_layout)
        automatic_layout.addWidget(self.abort_button)

        root = QVBoxLayout(self)
        root.addWidget(self.mode_label)
        root.addWidget(automatic_box)
        root.addWidget(instructions)
        root.addWidget(record_box)
        root.addWidget(progress_box)

        self.elapsed_timer = QTimer(self)
        self.elapsed_timer.timeout.connect(self._update_elapsed)
        self.progress_timer = QTimer(self)
        self.progress_timer.timeout.connect(self.refresh_progress)
        self.progress_timer.start(2000)
        self.auto_timer = QTimer(self)
        self.auto_timer.timeout.connect(self._tick_auto_sequence)
        self.auto_timer.start(1000)
        self._apply_scenario_defaults()
        self._refresh_stage_controls()
        self._refresh_description()
        self.refresh_progress()
        self.motion_inhibit_requested.emit(True)
        self._prepare_current_task()
        self._update_skip_controls()

    def _load_auto_history_and_queue(self):
        log_root = Path(os.environ.get("EXPERIMENT_LOG_ROOT", "/app/experiments/logs"))
        self.queue_state_path = log_root / "_auto_queue_state.json"
        try:
            state = json.loads(self.queue_state_path.read_text(encoding="utf-8"))
            if not isinstance(state, dict):
                raise ValueError("auto queue state must be a JSON object")
            saved_freeze_id = str(state.get("c1_freeze_id", "")).strip()
            saved_freeze_confirmed = bool(state.get("c1_freeze_confirmed", False))
            if (
                state.get("collection_stage") == "held_out_evaluation"
                and saved_freeze_confirmed
                and saved_freeze_id
                and self._stage_is_complete("development")
            ):
                self.collection_stage = "held_out_evaluation"
                self.evaluation_started = bool(state.get("evaluation_started", False))
                self.freeze_c1_check.setChecked(True)
                self.freeze_c1_id_edit.setText(saved_freeze_id)
                self._last_collection_stage_index = 1
                self.collection_stage_combo.setCurrentIndex(1)
            saved_skips = state.get("skipped_scenarios", {})
            if isinstance(saved_skips, dict):
                self.skipped_scenarios = {
                    str(scenario_id): dict(details) if isinstance(details, dict)
                    else {"reason": str(details)}
                    for scenario_id, details in saved_skips.items()
                }
                legacy_crossing_skip = self.skipped_scenarios.pop("E1_crossing", None)
                if legacy_crossing_skip is not None:
                    for scenario_id in ("E1_crossing_visible", "E1_crossing_occlusion"):
                        self.skipped_scenarios.setdefault(
                            scenario_id,
                            {
                                **legacy_crossing_skip,
                                "reason": (
                                    "Kế thừa trạng thái bỏ qua kịch bản cắt ngang cũ; "
                                    "khôi phục riêng từng nhánh khi sẵn sàng đo."
                                ),
                            },
                        )
        except (OSError, ValueError, TypeError):
            self.skipped_scenarios = {}
            self.collection_stage = "development"
            self.evaluation_started = False
        completed = set()
        attempts = {}
        if log_root.is_dir():
            for run_dir in log_root.iterdir():
                metadata_path = run_dir / "run_metadata.json"
                if not run_dir.is_dir() or not metadata_path.is_file():
                    continue
                try:
                    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
                except (OSError, ValueError, TypeError):
                    continue
                if not isinstance(metadata, dict):
                    continue
                task_id = metadata.get("auto_task_id")
                if not task_id:
                    continue
                try:
                    attempt = int(metadata.get("auto_attempt", 0) or 0)
                except (TypeError, ValueError):
                    attempt = 0
                attempts[task_id] = max(attempts.get(task_id, 0), attempt)
                quality = metadata.get("auto_capture_quality", {})
                if not isinstance(quality, dict):
                    quality = {}
                if (
                    metadata.get("collection_stage") == "held_out_evaluation"
                    and metadata.get("auto_task_id")
                ):
                    self.collection_stage = "held_out_evaluation"
                    self.evaluation_started = True
                    self.freeze_c1_check.setChecked(bool(metadata.get("c1_freeze_confirmed")))
                    self.freeze_c1_id_edit.setText(str(metadata.get("c1_freeze_id", "")))
                    self._last_collection_stage_index = 1
                    self.collection_stage_combo.setCurrentIndex(1)
                if metadata.get("status") == "scenario_completed" and quality.get("passed"):
                    completed.add(task_id)

        self.task_attempt_history = attempts
        self.completed_task_history = completed
        self.all_tasks = self._build_collection_tasks(self.collection_stage, attempts)
        self.all_task_ids = {task["task_id"] for task in self.all_tasks}
        self.completed_task_ids = completed & self.all_task_ids
        self.exhausted_task_ids = {
            task["task_id"] for task in self.all_tasks
            if task["task_id"] not in self.completed_task_ids
            and attempts.get(task["task_id"], 0) >= AUTO_MAX_ATTEMPTS
        }
        self._rebuild_auto_queue()

    def _build_collection_tasks(self, stage_id, attempts=None):
        """Build one source capture per scenario and block, independent of replay profiles."""
        attempts = attempts or {}
        is_development = stage_id == "development"
        block_count = 2 if is_development else 5
        direction_sequence = (
            ["trái sang phải", "phải sang trái"]
            if is_development
            else ["trái sang phải", "phải sang trái", "trái sang phải", "phải sang trái", "trái sang phải"]
        )
        occlusion_durations = [0.5, 1.5] if is_development else [0.5, 0.75, 1.0, 1.25, 1.5]
        config_id = "c1_input_capture"
        tasks = []
        for block_index in range(block_count):
            for scenario_id in E1_CAPTURE_SCENARIO_IDS:
                scenario = get_scenario(scenario_id)
                repeat_id = block_index + 1
                condition = {
                    "duration_s": int(scenario["duration_s"]),
                    "nominal_distance_m": None,
                    "start_distance_m": scenario.get("auto_start_distance_m"),
                    "walking_speed_mps": scenario.get("suggested_walking_speed_mps", 0.0),
                    "crossing_angle_deg": 90.0,
                    "crossing_direction": "",
                    "occlusion_duration_s": None,
                }
                if scenario_id == "E1_C1_crossing_visible":
                    direction = direction_sequence[block_index]
                    condition.update({
                        "crossing_direction": direction,
                        "walking_speed_mps": 0.4,
                        "variant": f"partial_b{repeat_id:02d}_{'lr' if direction == 'trái sang phải' else 'rl'}",
                    })
                elif scenario_id == "E1_C1_crossing_occlusion":
                    direction = direction_sequence[block_index]
                    duration_s = occlusion_durations[block_index]
                    condition.update({
                        "crossing_direction": direction,
                        "walking_speed_mps": 0.4,
                        "occlusion_duration_s": duration_s,
                        "variant": f"full_b{repeat_id:02d}_{'lr' if direction == 'trái sang phải' else 'rl'}_occ{str(duration_s).replace('.', 'p')}s",
                    })
                else:
                    condition["walking_speed_mps"] = 0.3
                    condition["variant"] = f"approach_b{repeat_id:02d}_2m_to_1m_0p3mps"

                block_id = f"{'HC' if is_development else 'DG'}_B{repeat_id:02d}"
                task_id = f"E1::{stage_id}::{scenario_id}::{block_id}"
                tasks.append({
                    "task_id": task_id,
                    "config_id": config_id,
                    "scenario_id": scenario_id,
                    "variant": condition["variant"],
                    "repeat_id": repeat_id,
                    "block_id": block_id,
                    "collection_stage": stage_id,
                    "data_partition": stage_id,
                    "duration_s": condition["duration_s"],
                    "nominal_distance_m": condition["nominal_distance_m"],
                    "start_distance_m": condition["start_distance_m"],
                    "attempt": attempts.get(task_id, 0) + 1,
                    "walking_speed_mps": condition["walking_speed_mps"],
                    "crossing_angle_deg": condition["crossing_angle_deg"],
                    "crossing_direction": condition["crossing_direction"],
                    "occlusion_duration_s": condition["occlusion_duration_s"],
                    "replay_profile_ids": list(E1_REPLAY_PROFILE_IDS),
                })
        return tasks

    def _rebuild_auto_queue(self):
        self.active_task_ids = {
            task["task_id"] for task in self.all_tasks
            if task["scenario_id"] not in self.skipped_scenarios
        }
        self.auto_queue = [
            task for task in self.all_tasks
            if task["task_id"] not in self.completed_task_ids
            and task["task_id"] not in self.exhausted_task_ids
            and task["scenario_id"] not in self.skipped_scenarios
        ]

    def _stage_expected_task_ids(self, stage_id):
        return {
            task["task_id"]
            for task in self._build_collection_tasks(stage_id)
        }

    def _stage_is_complete(self, stage_id):
        expected = self._stage_expected_task_ids(stage_id)
        if not expected:
            return False
        log_root = Path(os.environ.get("EXPERIMENT_LOG_ROOT", "/app/experiments/logs"))
        completed = set()
        if not log_root.is_dir():
            return False
        for run_dir in log_root.iterdir():
            metadata_path = run_dir / "run_metadata.json"
            if not run_dir.is_dir() or not metadata_path.is_file():
                continue
            try:
                metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            except (OSError, ValueError, TypeError):
                continue
            if not isinstance(metadata, dict):
                continue
            quality = metadata.get("auto_capture_quality", {})
            if not isinstance(quality, dict):
                continue
            task_id = metadata.get("auto_task_id", "")
            if (
                metadata.get("collection_stage") == stage_id
                and task_id in expected
                and metadata.get("status") == "scenario_completed"
                and quality.get("passed")
                and metadata.get("rgbd_capture_active_at_start")
                and metadata.get("ground_truth_ready")
            ):
                completed.add(task_id)
        return expected.issubset(completed)

    def _refresh_stage_controls(self, *_args):
        if not hasattr(self, "freeze_c1_check"):
            return
        development_complete = self._stage_is_complete("development")
        editing_development = self.collection_stage == "development"
        can_edit_freeze = development_complete and editing_development
        self.freeze_c1_check.setEnabled(can_edit_freeze)
        self.freeze_c1_id_edit.setEnabled(can_edit_freeze)
        self.collection_stage_combo.setEnabled(not self.evaluation_started)
        if not development_complete and self.freeze_c1_check.isChecked():
            self.freeze_c1_check.blockSignals(True)
            self.freeze_c1_check.setChecked(False)
            self.freeze_c1_check.blockSignals(False)

    def _collection_stage_changed(self, index):
        requested_stage = self.collection_stage_combo.itemData(index)
        if not requested_stage or requested_stage == self.collection_stage:
            return
        if self.run_active or self.auto_state in {"countdown", "running", "transition"}:
            self.collection_stage_combo.blockSignals(True)
            self.collection_stage_combo.setCurrentIndex(self._last_collection_stage_index)
            self.collection_stage_combo.blockSignals(False)
            return
        if requested_stage == "held_out_evaluation":
            if not self._stage_is_complete("development"):
                self.status_label.setText(
                    "Chưa mở tập đánh giá: cần đủ 6 bản ghi hiệu chỉnh đạt kiểm tra kỹ thuật."
                )
                self.collection_stage_combo.blockSignals(True)
                self.collection_stage_combo.setCurrentIndex(self._last_collection_stage_index)
                self.collection_stage_combo.blockSignals(False)
                return
            if not self.freeze_c1_check.isChecked() or not self.freeze_c1_id_edit.text().strip():
                self.status_label.setText(
                    "Chưa mở tập đánh giá: xác nhận đã khóa C1 và nhập mã phiên bản/tham số."
                )
                self.collection_stage_combo.blockSignals(True)
                self.collection_stage_combo.setCurrentIndex(self._last_collection_stage_index)
                self.collection_stage_combo.blockSignals(False)
                return

        self.collection_stage = requested_stage
        self._last_collection_stage_index = index
        try:
            self._save_auto_queue_state()
        except OSError as exc:
            self.collection_stage = (
                "development" if requested_stage == "held_out_evaluation" else "held_out_evaluation"
            )
            self._last_collection_stage_index = 0 if self.collection_stage == "development" else 1
            self.collection_stage_combo.blockSignals(True)
            self.collection_stage_combo.setCurrentIndex(self._last_collection_stage_index)
            self.collection_stage_combo.blockSignals(False)
            self.status_label.setText(f"Không lưu được trạng thái giai đoạn đo: {exc}")
            self._refresh_stage_controls()
            return
        self._load_auto_history_and_queue()
        self.auto_index = 0
        self.auto_state = "waiting_for_target"
        if not self.auto_timer.isActive():
            self.auto_timer.start(1000)
        self._refresh_stage_controls()
        self._prepare_current_task()
        self.refresh_progress()

    def _collection_prerequisites_ready(self):
        rgbd_active = os.environ.get("EXPERIMENT_RECORD_RGBD_ACTIVE", "0") == "1"
        return bool(
            rgbd_active
            and self.safety_check.isChecked()
            and self.ground_truth_check.isChecked()
        )

    def _prerequisite_changed(self, *_args):
        if self.run_active or not self.current_task:
            return
        scenario = get_scenario(self.current_task["scenario_id"])
        if not scenario.get("c1_capture_only"):
            return
        if self._collection_prerequisites_ready():
            if self.auto_state == "waiting_for_prerequisites":
                self._prepare_current_task()
        else:
            self.motion_inhibit_requested.emit(True)
            if self.auto_state in {"countdown", "waiting_for_stable_distance"}:
                self._reset_start_stability()
            self.auto_state = "waiting_for_prerequisites"
            self._update_prerequisites_prompt()

    def _update_prerequisites_prompt(self):
        missing = []
        if os.environ.get("EXPERIMENT_RECORD_RGBD_ACTIVE", "0") != "1":
            missing.append("bật lưu RGB-D và khởi động lại ứng dụng")
        if not self.safety_check.isChecked():
            missing.append("đánh dấu đã kiểm tra an toàn và nút dừng khẩn cấp")
        if not self.ground_truth_check.isChecked():
            missing.append("chuẩn bị khoảng cách chuẩn độc lập liên tục, đồng bộ với camera")
        self.auto_phase_label.setText("Chưa bắt đầu đo — robot đang bị khóa.")
        self.auto_countdown_label.setText("Cần hoàn tất: " + "; ".join(missing) + ".")
        self.status_label.setText("Đang chờ điều kiện bắt buộc trước khi ghi")
        self._update_manual_reposition_button()

    def _save_auto_queue_state(self):
        self.queue_state_path.parent.mkdir(parents=True, exist_ok=True)
        state = {
            "schema_version": 1,
            "skipped_scenarios": self.skipped_scenarios,
            "collection_stage": self.collection_stage,
            "evaluation_started": self.evaluation_started,
            "c1_freeze_confirmed": self.freeze_c1_check.isChecked(),
            "c1_freeze_id": self.freeze_c1_id_edit.text().strip(),
        }
        temporary_path = self.queue_state_path.with_name(
            self.queue_state_path.name + ".tmp"
        )
        temporary_path.write_text(
            json.dumps(state, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary_path.replace(self.queue_state_path)

    def _update_skip_controls(self):
        if not hasattr(self, "skip_scenario_button"):
            return
        if self.current_task:
            scenario = get_scenario(self.current_task["scenario_id"])
            self.skip_scenario_button.setText(
                f"Bỏ qua kịch bản này — {scenario['title']}"
            )
            self.skip_scenario_button.setEnabled(
                self.current_task["scenario_id"] not in self.skipped_scenarios
                and self.auto_state != "complete"
            )
        else:
            self.skip_scenario_button.setEnabled(False)

        selected_id = self.skipped_scenario_combo.currentData()
        self.skipped_scenario_combo.clear()
        for scenario in SCENARIOS:
            if scenario["id"] in self.skipped_scenarios:
                self.skipped_scenario_combo.addItem(scenario["title"], scenario["id"])
        if selected_id:
            index = self.skipped_scenario_combo.findData(selected_id)
            if index >= 0:
                self.skipped_scenario_combo.setCurrentIndex(index)
        has_skipped = self.skipped_scenario_combo.count() > 0
        self.skipped_scenario_label.setVisible(has_skipped)
        self.skipped_scenario_combo.setEnabled(has_skipped)
        self.skipped_scenario_combo.setVisible(has_skipped)
        self.restore_scenario_button.setEnabled(has_skipped and not self.run_active)
        self.restore_scenario_button.setVisible(has_skipped)

    def skip_current_scenario(self):
        if not self.current_task:
            return
        self._stop_reposition_controls(
            "Đã khóa robot khi chuyển kịch bản; lượt đo chưa bắt đầu."
        )
        scenario_id = self.current_task["scenario_id"]
        if scenario_id in self.skipped_scenarios:
            return
        scenario = get_scenario(scenario_id)
        self.skipped_scenarios[scenario_id] = {
            "skipped_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "reason": "Chưa thể thực hiện trong buổi đo; các lượt đã lưu được giữ nguyên.",
            "after_task_id": self.current_task["task_id"],
        }
        try:
            self._save_auto_queue_state()
        except OSError as exc:
            self.skipped_scenarios.pop(scenario_id, None)
            self.status_label.setText(f"Không lưu được trạng thái bỏ qua: {exc}")
            self._update_skip_controls()
            return

        if self.run_active:
            self.run_active = False
            self.run_started_at = None
            self.elapsed_timer.stop()
            self.motion_inhibit_requested.emit(True)
            self.run_stop_requested.emit(self._current_info({
                "auto_abort": True,
                "auto_skip_scenario": True,
            }))

        self.motion_inhibit_requested.emit(True)
        self.auto_index = 0
        self._rebuild_auto_queue()
        self._update_skip_controls()
        if not self.auto_queue:
            self._finish_auto_session()
            return

        self.auto_state = "transition"
        self.auto_countdown = AUTO_RESET_SECONDS
        self._select_current_task()
        next_scenario = get_scenario(self.current_task["scenario_id"])
        self.auto_phase_label.setText(
            f"Đã bỏ qua {scenario['title']}; các log cũ vẫn được giữ. "
            f"Kịch bản tiếp theo: {next_scenario['title']}."
        )
        self.auto_countdown_label.setText(
            f"Tự chuyển sau {AUTO_RESET_SECONDS} giây; có thể khôi phục kịch bản đã bỏ qua."
        )
        self.status_label.setText(f"Đã bỏ qua kịch bản: {scenario['title']}")
        if not self.auto_timer.isActive():
            self.auto_timer.start(1000)
        self.refresh_progress()

    def restore_skipped_scenario(self):
        if self.run_active:
            return
        scenario_id = self.skipped_scenario_combo.currentData()
        if not scenario_id or scenario_id not in self.skipped_scenarios:
            return
        skipped_entry = self.skipped_scenarios.pop(scenario_id)
        try:
            self._save_auto_queue_state()
        except OSError as exc:
            self.skipped_scenarios[scenario_id] = skipped_entry
            self.status_label.setText(f"Không lưu được trạng thái khôi phục: {exc}")
            self._update_skip_controls()
            return

        self.auto_index = 0
        self._rebuild_auto_queue()
        self._update_skip_controls()
        if not self.auto_queue:
            self._finish_auto_session()
            return
        self.auto_state = "transition"
        self.auto_countdown = AUTO_RESET_SECONDS
        self._select_current_task()
        restored = get_scenario(scenario_id)
        self.auto_phase_label.setText(
            f"Đã khôi phục {restored['title']}; kịch bản còn lượt chưa hoàn thành sẽ được xếp lại."
        )
        self.auto_countdown_label.setText(
            f"Tự chuẩn bị lại hàng đợi sau {AUTO_RESET_SECONDS} giây."
        )
        self.status_label.setText(f"Đã khôi phục kịch bản: {restored['title']}")
        if not self.auto_timer.isActive():
            self.auto_timer.start(1000)
        self.refresh_progress()

    def _current_info(self, extra=None):
        if not self.current_task:
            return {}
        scenario = get_scenario(self.current_task["scenario_id"])
        info = {
            "config_id": self.current_task["config_id"],
            "scenario_id": self.current_task["scenario_id"],
            "variant": self.current_task["variant"],
            "repeat_id": self.current_task["repeat_id"],
            "block_id": self.current_task["block_id"],
            "collection_stage": self.current_task["collection_stage"],
            "data_partition": self.current_task.get("data_partition", "unspecified"),
            "operator_id": os.environ.get("EXPERIMENT_OPERATOR_ID", "operator"),
            "operator_notes": "Kịch bản tự động theo thời gian; người vận hành làm theo hướng dẫn trên giao diện.",
            "safety_checked": self.safety_check.isChecked(),
            "ground_truth_ready": self.ground_truth_check.isChecked(),
            "participant_height_m": float(os.environ.get("EXPERIMENT_PARTICIPANT_HEIGHT_M", "1.76")),
            "walking_speed_mps": self.current_task.get(
                "walking_speed_mps", scenario.get("suggested_walking_speed_mps", 0.0)
            ),
            "walking_speed_value_type": "planned_or_estimated",
            "crossing_angle_deg": self.current_task.get(
                "crossing_angle_deg", scenario.get("crossing_angle_deg", 90.0)
            ),
            "crossing_direction": self.current_task.get("crossing_direction", ""),
            "crowd_count": int(scenario.get("crowd_count", 1)),
            "scenario_action": scenario.get("scenario_action", ""),
            "target_role": scenario.get("target_role", "tracked_person"),
            "occluder_role": scenario.get("occluder_role", ""),
            "ground_truth_source": self.reference_source_combo.currentData(),
            "ground_truth_definition": self.reference_definition_combo.currentData(),
            "source": "real",
            "control_mode": self.control_mode,
            "auto_sequence": True,
            "auto_task_id": self.current_task["task_id"],
            "auto_attempt": self.current_task["attempt"],
            "auto_duration_s": self.current_task["duration_s"],
            "auto_nominal_distance_m": self.current_task["nominal_distance_m"],
            "auto_occlusion_duration_s": self.current_task.get("occlusion_duration_s"),
            "replay_profile_ids": list(self.current_task["replay_profile_ids"]),
            "replay_status": "pending_offline_replay",
            "replay_source_capture_id": self.current_task["task_id"],
            "replay_outputs_expected": len(self.current_task["replay_profile_ids"]),
            "c1_freeze_confirmed": self.freeze_c1_check.isChecked(),
            "c1_freeze_id": self.freeze_c1_id_edit.text().strip(),
            "auto_phase_id": self.current_phase_id,
            "motion_inhibited_by_protocol": bool(scenario.get("motion_inhibited", False)),
            "c1_capture_only": bool(scenario.get("c1_capture_only", False)),
            "rgbd_capture_required": bool(scenario.get("requires_rgbd_recording", False)),
            "rgbd_capture_active": os.environ.get("EXPERIMENT_RECORD_RGBD_ACTIVE", "0") == "1",
            "rgbd_session_path": os.environ.get("EXPERIMENT_RECORD_RGBD_PATH", ""),
            "offline_comparison_profiles": scenario.get("offline_comparison_profiles", []),
            "auto_goal_reached": (
                self.robot_approach_completed
                if self.current_task["scenario_id"] == "E1_robot_approach" else None
            ),
            "auto_goal_distance_m": scenario.get("auto_goal_distance_m"),
            "auto_goal_tolerance_m": scenario.get("auto_goal_tolerance_m"),
            "auto_goal_stability_frames": scenario.get("auto_goal_stability_frames"),
            "auto_goal_final_distance_m": self.robot_approach_final_distance_m,
            "auto_start_gate": dict(self.current_start_gate or {
                "passed": False,
                "target_distance_m": self._required_start_distance_m(),
                "tolerance_m": AUTO_START_DISTANCE_TOLERANCE_M,
                "required_consecutive_frames": AUTO_START_STABILITY_FRAMES,
                "observed_consecutive_frames": self.auto_stability_frames,
                "distance_reference": "camera_to_target_roi",
            }),
        }
        if extra:
            info.update(extra)
        return info

    def _required_start_distance_m(self):
        if self.current_task and self.current_task.get("start_distance_m") is not None:
            return float(self.current_task["start_distance_m"])
        if self.current_task and self.current_task["scenario_id"] == "E1_calibration":
            return float(self.current_task["nominal_distance_m"])
        if self.current_task:
            scenario = get_scenario(self.current_task["scenario_id"])
            if scenario.get("auto_start_distance_m") is not None:
                return float(scenario["auto_start_distance_m"])
        return float(os.environ.get("DESIRED_CAMERA_DISTANCE_M", "1.40"))

    def _distance_is_in_start_range(self, distance_m):
        if distance_m is None:
            return False
        try:
            value = float(distance_m)
        except (TypeError, ValueError):
            return False
        return (
            math.isfinite(value)
            and value > 0.0
            and abs(value - self._required_start_distance_m())
            <= AUTO_START_DISTANCE_TOLERANCE_M
        )

    def _reset_start_stability(self):
        self.auto_stability_frames = 0
        self.auto_stability_samples = []
        self.current_start_gate = None

    def _begin_distance_stability_gate(self):
        self.motion_inhibit_requested.emit(True)
        self._reset_start_stability()
        self.auto_state = "waiting_for_stable_distance"
        target = self._required_start_distance_m()
        tolerance = AUTO_START_DISTANCE_TOLERANCE_M
        if self.current_task and self.current_task.get("scenario_id") == "E1_robot_approach":
            prompt = (
                f"Cổng bắt đầu lượt: khoảng cách camera–ROI phải ở {target:.2f} m "
                f"±{tolerance:.2f} m. Đây là mốc xuất phát 2,2 m; ±0,03 m là ngưỡng dừng "
                "tại đích 1,4 m. "
            )
        else:
            prompt = (
                f"Đưa người tới {target:.2f} m theo số đo camera, trong vùng "
                f"±{tolerance:.2f} m; "
            )
        self.auto_phase_label.setText(
            prompt + f"giữ yên cho tới khi đạt {AUTO_START_STABILITY_FRAMES} khung hình liên tiếp."
        )
        self.auto_countdown_label.setText(
            f"Ổn định khoảng cách: 0/{AUTO_START_STABILITY_FRAMES} khung hình"
        )
        self.status_label.setText("Đang chờ khoảng cách ổn định trước lượt đo")
        self._update_manual_reposition_button()

    def _record_start_distance_sample(self, distance_m):
        if not self._distance_is_in_start_range(distance_m):
            self.auto_stability_frames = 0
            self.auto_stability_samples = []
            return False
        value = float(distance_m)
        self.auto_stability_samples.append(value)
        self.auto_stability_samples = self.auto_stability_samples[-AUTO_START_STABILITY_FRAMES:]
        self.auto_stability_frames = min(
            self.auto_stability_frames + 1,
            AUTO_START_STABILITY_FRAMES,
        )
        return True

    def _start_distance_gate_summary(self):
        samples = list(self.auto_stability_samples)
        return {
            "passed": True,
            "distance_reference": "camera_to_target_roi",
            "target_distance_m": self._required_start_distance_m(),
            "tolerance_m": AUTO_START_DISTANCE_TOLERANCE_M,
            "required_consecutive_frames": AUTO_START_STABILITY_FRAMES,
            "observed_consecutive_frames": self.auto_stability_frames,
            "sample_mean_m": sum(samples) / len(samples) if samples else None,
            "sample_min_m": min(samples) if samples else None,
            "sample_max_m": max(samples) if samples else None,
            "samples_m": samples,
        }

    def update_operator_state(self, target_selected, follow_enabled, target_distance_m=None):
        self.operator_target_selected = bool(target_selected)
        self.operator_follow_enabled = bool(follow_enabled)
        self.operator_distance_m = target_distance_m

        if self.auto_state == "waiting_for_prerequisites":
            if self._collection_prerequisites_ready():
                self._prepare_current_task()
            else:
                self.motion_inhibit_requested.emit(True)
                self._update_manual_reposition_button()
            return

        if self.reposition_control_active:
            # The operator has paused data collection to reposition the robot.
            # Keep this mode active until they explicitly toggle it off; do not
            # let the distance gate start a trial while the robot is moving.
            self._update_manual_reposition_button()
            return

        if self.run_active:
            if (
                self.current_task
                and self.current_task["scenario_id"] == "E1_robot_approach"
                and self.current_phase_id == "robot_approach"
            ):
                self._update_robot_approach_goal(target_distance_m)
            self._update_manual_reposition_button()
            return
        if self.auto_state in {"running", "transition", "complete", "aborted"}:
            self._update_manual_reposition_button()
            return

        if self.auto_state in {"countdown", "waiting_for_stable_distance"}:
            if not self.operator_target_selected:
                self.motion_inhibit_requested.emit(True)
                self._reset_start_stability()
                self.auto_state = "waiting_for_target"
                self.auto_phase_label.setText(
                    "Mục tiêu chưa được chọn; đang chờ đăng ký người trên camera."
                )
                self.auto_countdown_label.setText("Tiến trình vị trí ổn định đã đặt lại.")
                self._update_manual_reposition_button()
                return
        if self.auto_state == "countdown":
            if not self._distance_is_in_start_range(target_distance_m):
                self._begin_distance_stability_gate()
                self._update_manual_reposition_button()
                return
            self._update_manual_reposition_button()
            return

        if self.auto_state == "waiting_for_target" and self.operator_target_selected:
            self._prepare_current_task()
        elif self.auto_state == "waiting_for_follow" and self.operator_follow_enabled:
            self._prepare_current_task()
        elif self.auto_state == "waiting_for_stable_distance":
            in_range = self._record_start_distance_sample(target_distance_m)
            current_text = (
                f"{float(target_distance_m):.2f} m" if target_distance_m is not None else "chưa có số đo hợp lệ"
            )
            self.auto_countdown_label.setText(
                f"Đang đo {current_text} — ổn định: "
                f"{self.auto_stability_frames}/{AUTO_START_STABILITY_FRAMES} khung hình liên tiếp"
            )
            if self.auto_stability_frames >= AUTO_START_STABILITY_FRAMES:
                self.current_start_gate = self._start_distance_gate_summary()
                self.auto_countdown_label.setText(
                    f"Đã đủ {AUTO_START_STABILITY_FRAMES} khung hình ổn định — bắt đầu đo ngay."
                )
                self._start_current_run()
            elif not in_range:
                self.auto_phase_label.setText(
                    f"Khoảng cách cần ở {self._required_start_distance_m():.2f} m "
                    f"±{AUTO_START_DISTANCE_TOLERANCE_M:.2f} m. Đứng yên trong vùng này "
                    "để bộ đếm 10 khung hình liên tiếp hoàn tất."
                )
        self._update_manual_reposition_button()

    def update_controller_safety_state(self, is_tracking, loss_latched, stable_frames):
        task = self.current_task or {}
        scenario = get_scenario(task.get("scenario_id", "")) if task else {}
        action = scenario.get("scenario_action")
        if scenario.get("c1_capture_only"):
            self.controller_safety_label.setVisible(True)
            if loss_latched or not is_tracking:
                message = (
                    "ĐANG THU DỮ LIỆU ĐẦU VÀO — mục tiêu đang mất dấu/được nhận lại; "
                    "robot khóa ở 0. C1 chưa chạy trong lượt này."
                )
                color = "#b02a37"
            else:
                message = (
                    "CHỈ THU RGB-D VÀ LỆNH P GIẢ LẬP HIỆN TẠI. C1 chưa được áp dụng; "
                    "robot thật giữ lệnh 0. Sẽ so sánh các luật trên cùng dữ liệu sau khi triển khai."
                )
                color = "#0b5ed7"
            self.controller_safety_label.setText(message)
            self.controller_safety_label.setStyleSheet(
                f"font-size:11pt;font-weight:bold;color:{color};"
            )
            return
        if action not in {"crossing_no_loss", "crossing_occlusion"}:
            self.controller_safety_label.clear()
            self.controller_safety_label.setVisible(False)
            return
        self.controller_safety_label.setVisible(True)
        if loss_latched:
            if action == "crossing_no_loss":
                message = (
                    "NHÁNH A KHÔNG ĐẠT: mục tiêu đã mất dấu. Bộ điều khiển dừng, "
                    "không bám người hỗ trợ; giữ log và lượt này sẽ được đo lại. "
                    f"Chờ nhận lại ổn định {stable_frames}/10 khung hình. Robot đang khóa."
                )
            else:
                message = (
                    "MẤT DẤU: phản ứng an toàn là dừng lệnh tiến/quay, không bám người hỗ trợ. "
                    f"Chờ nhận lại đúng mục tiêu ổn định {stable_frames}/10 khung hình. Robot đang khóa."
                )
            self.controller_safety_label.setText(message)
            self.controller_safety_label.setStyleSheet("font-size:11pt;font-weight:bold;color:#b02a37;")
        elif is_tracking and stable_frames >= 10:
            self.controller_safety_label.setText(
                "Đã nhận lại mục tiêu ổn định 10 khung hình; bộ điều khiển mới tính lại lệnh P "
                "theo khoảng cách và vị trí mục tiêu. Lệnh chỉ ghi giả lập; robot vẫn khóa đứng yên."
            )
            self.controller_safety_label.setStyleSheet("font-size:11pt;font-weight:bold;color:#198754;")
        elif is_tracking:
            if action == "crossing_no_loss":
                message = (
                    "MỤC TIÊU CÒN THEO DÕI: nếu tracker vẫn gắn đúng người đã chọn, bộ P tiếp tục "
                    "tính tiến/lùi theo sai lệch khoảng cách và quay theo lệch tâm ảnh. Hệ thống không "
                    "tự né người đi ngang. Đây là lệnh giả lập; robot thật đang khóa ở 0."
                )
            else:
                message = (
                    "MỤC TIÊU CÒN THEO DÕI: bộ P tính tiến/lùi theo sai lệch khoảng cách "
                    "và quay theo lệch tâm ảnh. Nếu mục tiêu mất dấu, lệnh dừng; sau đó chỉ "
                    "tính lại khi nhận diện ổn định 10 khung hình. Lệnh ghi giả lập; robot đang khóa."
                )
            self.controller_safety_label.setText(message)
            self.controller_safety_label.setStyleSheet("font-size:11pt;font-weight:bold;color:#0b5ed7;")
        else:
            self.controller_safety_label.setText(
                "Chưa thấy đúng mục tiêu đã đăng ký: robot phải giữ dừng, không chạy theo người đi ngang. "
                "Sau khi nhận diện lại cần 10 khung hình ổn định mới tính lệnh tiếp."
            )
            self.controller_safety_label.setStyleSheet("font-size:11pt;font-weight:bold;color:#b02a37;")

    def _update_manual_reposition_button(self):
        if not hasattr(self, "reposition_control_button"):
            return
        scenario = (
            get_scenario(self.current_task.get("scenario_id", ""))
            if self.current_task else {}
        )
        is_reposition_task = bool(
            self.current_task
            and self.current_task.get("scenario_id") == "E1_robot_approach"
        )
        self.reposition_control_button.setVisible(
            is_reposition_task and not self.run_active
        )
        self.reposition_help_label.setVisible(is_reposition_task)
        self.reposition_status_label.setVisible(is_reposition_task)
        self.reposition_control_button.setEnabled(
            self.reposition_control_active or self._normal_reposition_control_available()
        )

    def _normal_reposition_control_available(self):
        if self.control_mode != "live" or not self.current_task:
            return False
        scenario = get_scenario(self.current_task["scenario_id"])
        is_reposition_task = self.current_task["scenario_id"] == "E1_robot_approach"
        if not is_reposition_task:
            return False
        allowed_auto_states = {"waiting_for_stable_distance"}
        if (
            self.run_active
            or self.auto_state not in allowed_auto_states
        ):
            return False
        return (
            self.operator_target_selected
            and self.operator_follow_enabled
        )

    def _reposition_control_toggled(self, checked):
        if checked:
            if not self._normal_reposition_control_available():
                self.reposition_control_button.blockSignals(True)
                self.reposition_control_button.setChecked(False)
                self.reposition_control_button.blockSignals(False)
                return
            self.reposition_control_active = True
            self.reposition_control_button.setText(
                "TẮT ĐIỀU KHIỂN THƯỜNG — tiếp tục đo"
            )
            self.reposition_status_label.setText(
                "Đang tạm dừng thu thập; điều khiển theo dõi robot hoạt động bình thường. "
                "Di chuyển về vị trí xuất phát; khi đã sẵn sàng, bấm nút lần nữa để khóa robot và tiếp tục đo."
            )
            self.next_action_label.setText(
                "Đang tạm dừng ghi nhận số liệu. Dùng điều khiển THEO DÕI như bình thường để đưa robot về vị trí xuất phát. "
                "Khi sẵn sàng, bấm nút lần nữa; robot sẽ khóa và chỉ bắt đầu lượt tiếp theo sau khi camera–ROI ổn định ở 2,20 m ±0,05 m trong 10 khung hình."
            )
            self.reposition_control_requested.emit(True)
        else:
            lock_message = (
                "Đã khóa robot; bộ đếm đo tiếp tục. Cần ở mốc camera–ROI 2,2 m ±0,05 m "
                f"trong {AUTO_START_STABILITY_FRAMES} khung hình liên tiếp để tự bắt đầu lượt kế tiếp."
            )
            self._stop_reposition_control(lock_message)
        self._update_manual_reposition_button()

    def _stop_reposition_control(self, message):
        if not self.reposition_control_active:
            return
        self.reposition_control_active = False
        self.reposition_control_button.blockSignals(True)
        self.reposition_control_button.setChecked(False)
        self.reposition_control_button.setText(
            "Tạm dừng đo — bật điều khiển robot bình thường"
        )
        self.reposition_control_button.blockSignals(False)
        self.reposition_control_requested.emit(False)
        self.reposition_status_label.setText(message)
        self._update_manual_reposition_button()

    def _stop_reposition_controls(self, message):
        if self.reposition_control_active:
            self._stop_reposition_control(message)

    def _update_robot_approach_goal(self, target_distance_m):
        scenario = get_scenario("E1_robot_approach")
        try:
            distance_m = float(target_distance_m)
        except (TypeError, ValueError):
            distance_m = 0.0
        if not math.isfinite(distance_m) or distance_m <= 0.0:
            self.robot_approach_distance_history = []
            self.robot_approach_stable_frames = 0
            return

        self.robot_approach_distance_history.append(distance_m)
        self.robot_approach_distance_history = self.robot_approach_distance_history[-5:]
        filtered_distance_m = sum(self.robot_approach_distance_history) / len(
            self.robot_approach_distance_history
        )
        goal_m = float(scenario["auto_goal_distance_m"])
        tolerance_m = float(scenario["auto_goal_tolerance_m"])
        if abs(filtered_distance_m - goal_m) <= tolerance_m:
            self.robot_approach_stable_frames += 1
        else:
            self.robot_approach_stable_frames = 0

        required_frames = int(scenario["auto_goal_stability_frames"])
        self.auto_countdown_label.setText(
            f"Khoảng cách camera–ROI: {filtered_distance_m:.2f} m; "
            f"đích {goal_m:.2f} m ±{tolerance_m:.2f} m — "
            f"ổn định {self.robot_approach_stable_frames}/{required_frames} khung hình"
        )
        if self.robot_approach_stable_frames < required_frames:
            return

        self.robot_approach_completed = True
        self.robot_approach_final_distance_m = filtered_distance_m
        self._emit_phase_event("robot_approach_end")
        self.current_phase_id = "hold"
        self.motion_inhibit_requested.emit(True)
        self.auto_phase_label.setText(
            "MiR đã tới khoảng cách đích. Đứng yên; robot đã bị khóa và đang ghi đoạn giữ."
        )
        self.auto_countdown_label.setText(
            f"Khoảng cách cuối: {filtered_distance_m:.3f} m; tiếp tục đứng yên đến hết lượt."
        )

    def _select_current_task(self):
        if self.auto_index >= len(self.auto_queue):
            self.current_task = None
            return False
        self.current_task = self.auto_queue[self.auto_index]
        self._reset_start_stability()
        config_index = self.config_combo.findData(self.current_task["config_id"])
        scenario_index = self.scenario_combo.findData(self.current_task["scenario_id"])
        if config_index >= 0:
            self.config_combo.setCurrentIndex(config_index)
        if scenario_index >= 0:
            self.scenario_combo.setCurrentIndex(scenario_index)
        self.variant_edit.setText(self.current_task["variant"])
        self.repeat_spin.setValue(self.current_task["repeat_id"])
        self._refresh_description()
        task_number = min(
            len(self.all_task_ids),
            len(self.completed_task_ids & self.all_task_ids)
            + len(self.exhausted_task_ids & self.all_task_ids)
            + 1,
        )
        total = len(self.all_task_ids)
        config = get_configuration(self.current_task["config_id"])
        scenario = get_scenario(self.current_task["scenario_id"])
        partition_label = {
            "development": "TẬP HIỆU CHỈNH C1 — không tính là đánh giá cuối",
            "held_out_evaluation": "TẬP ĐÁNH GIÁ GIỮ RIÊNG — không dùng chỉnh ngưỡng C1",
        }.get(self.current_task.get("data_partition"), "")
        self.auto_task_label.setText(
            f"{task_number}/{total} — {config['label']}<br>{scenario['title']} — "
            f"{self.current_task['variant']} — "
            f"khối {self.current_task['block_id']} — lần thử {self.current_task['attempt']}"
            + (f"<br><b>{partition_label}</b>" if partition_label else "")
        )
        self._update_skip_controls()
        return True

    def _prepare_current_task(self):
        if not self._select_current_task():
            self._finish_auto_session()
            return
        self.motion_inhibit_requested.emit(True)
        if not self._collection_prerequisites_ready():
            self.auto_state = "waiting_for_prerequisites"
            self._update_prerequisites_prompt()
            return
        if not self.operator_target_selected:
            self.auto_state = "waiting_for_target"
            self.auto_phase_label.setText("Hãy đăng ký/chọn người cần theo dõi trên hình camera.")
            self.auto_countdown_label.setText("Sau đó quá trình ghi sẽ bắt đầu tự động.")
            self.status_label.setText("Đang chờ chọn người trên camera")
            return
        if self.current_task["scenario_id"] != "E1_calibration" and not self.operator_follow_enabled:
            self.auto_state = "waiting_for_follow"
            self.auto_phase_label.setText("Hãy thực hiện cử chỉ THEO DÕI để robot sẵn sàng cho lượt này.")
            self.auto_countdown_label.setText("Khi chế độ theo dõi bật, sẽ tự đếm ngược rồi bắt đầu ghi.")
            self.status_label.setText("Đang chờ bật theo dõi cho lượt chuyển động")
            return
        self._begin_distance_stability_gate()

    def _begin_prepare_countdown(self):
        self._stop_reposition_controls(
            "Đã khóa điều khiển đặt lại; bắt đầu chuẩn bị lượt đo."
        )
        self.motion_inhibit_requested.emit(True)
        self.auto_state = "countdown"
        self.auto_countdown = AUTO_PREPARE_SECONDS
        self.auto_phase_label.setText(
            self._preparation_prompt()
            + f" Đã đạt {AUTO_START_STABILITY_FRAMES} khung hình liên tiếp trong "
            f"±{AUTO_START_DISTANCE_TOLERANCE_M:.2f} m; giữ vị trí ổn định trong lúc đếm ngược."
        )
        self.auto_countdown_label.setText(f"Tự bắt đầu sau {self.auto_countdown} giây")
        self.status_label.setText("Đang chuẩn bị lượt đo tự động")

    def _preparation_prompt(self):
        task = self.current_task or {}
        if task.get("scenario_id") == "E1_calibration":
            distance = task.get("nominal_distance_m")
            return (
                f"Đưa người tới mốc camera–ROI gần {distance:.1f} m, "
                "đứng yên khi kiểm tra ổn định, rồi giữ vị trí trong 15 giây ghi đo. Robot được khóa đứng yên."
            )
        scenario = get_scenario(task.get("scenario_id", ""))
        if scenario.get("scenario_action") == "c1_crossing":
            return (
                "Một người đã đăng ký bắt đầu ở camera–ROI 1,65 m rồi đi cắt ngang theo "
                f"điều kiện {task.get('crossing_angle_deg', 90):.0f}°, "
                f"{task.get('walking_speed_mps', 0.0):.1f} m/s, "
                f"hướng {task.get('crossing_direction', 'đang chọn')}. Robot khóa đứng yên; "
                "đang thu RGB-D đầu vào, chưa chạy C1."
            )
        if scenario.get("scenario_action") == "c1_approach_control":
            return (
                "Một người đã đăng ký đi thẳng từ 2,0 m về khoảng 1,0 m với tốc độ 0,30 m/s. "
                "Robot khóa đứng yên; lượt này kiểm tra dữ liệu thay đổi khoảng cách thật, chưa chạy C1."
            )
        if scenario.get("scenario_action") == "crossing_no_loss":
            return (
                "Mục tiêu đứng yên ở 2,20 m; người hỗ trợ đi phía trước, chỉ che một phần. "
                f"Đi {task.get('crossing_direction', 'theo hướng đang chọn')} ở "
                f"khoảng {task.get('walking_speed_mps', 0.4):.1f} m/s. Robot khóa; C1 chưa chạy."
            )
        if scenario.get("scenario_action") == "crossing_occlusion":
            return (
                "Mục tiêu đứng yên ở 2,20 m; người hỗ trợ chuẩn bị che kín "
                f"{task.get('occlusion_duration_s', 0.5):.2f} giây rồi rời khung. "
                f"Hướng {task.get('crossing_direction', 'đang chọn')}, "
                f"khoảng {task.get('walking_speed_mps', 0.4):.1f} m/s. Robot khóa; C1 chưa chạy."
            )
        if task.get("scenario_id") == "E1_robot_approach":
            return (
                "Đứng yên tại mốc camera–ROI 2,2 m, hướng mặt về robot; "
                "không di chuyển khi MiR tự tiến tới mốc 1,4 m."
            )
        return "Đứng ở mốc xuất phát; chuẩn bị đi thẳng về phía robot khi giao diện báo."

    def _start_current_run(self):
        if not self._collection_prerequisites_ready():
            self.auto_state = "waiting_for_prerequisites"
            self.motion_inhibit_requested.emit(True)
            self._update_prerequisites_prompt()
            return
        if self.collection_stage == "held_out_evaluation" and not self.evaluation_started:
            self.evaluation_started = True
            try:
                self._save_auto_queue_state()
            except OSError as exc:
                self.status_label.setText(
                    f"Đang ghi bản ghi đánh giá; không lưu được trạng thái khóa: {exc}"
                )
            self._refresh_stage_controls()
        self.run_active = True
        self.auto_state = "running"
        self.run_started_at = time.time()
        self.current_phase_id = "baseline"
        self.phase_events_emitted = set()
        self.robot_approach_motion_released = False
        self.robot_approach_distance_history = []
        self.robot_approach_stable_frames = 0
        self.robot_approach_completed = False
        self.robot_approach_final_distance_m = None
        self.elapsed_timer.start(500)
        scenario_id = self.current_task["scenario_id"]
        self.motion_inhibit_requested.emit(
            scenario_id in {"E1_calibration", "E1_robot_approach"}
            or bool(get_scenario(scenario_id).get("motion_inhibited", False))
        )
        self.current_run_info = self._current_info()
        self.current_run_quality = None
        rgbd_recording_active = os.environ.get("EXPERIMENT_RECORD_RGBD_ACTIVE", "0") == "1"
        self.status_label.setText(
            "ĐANG GHI — các mốc và dừng lượt sẽ tự động"
            if rgbd_recording_active else
            "ĐANG GHI CSV/JSON — không lưu video RGB-D gốc"
        )
        self.status_label.setStyleSheet("font-weight:bold;color:#198754;")
        self._update_skip_controls()
        self.run_start_requested.emit(self.current_run_info)
        if scenario_id == "E1_calibration":
            self.current_phase_id = "calibration_point"
            self.event_requested.emit("calibration_point", self._current_info())

    def _emit_phase_event(self, event_id):
        if event_id in self.phase_events_emitted:
            return
        self.phase_events_emitted.add(event_id)
        self.current_phase_id = event_id
        self.event_requested.emit(event_id, self._current_info())

    def _update_run_phase(self, elapsed_s):
        task = self.current_task
        scenario_id = task["scenario_id"]
        scenario = get_scenario(scenario_id)
        if scenario_id == "E1_calibration":
            self.auto_phase_label.setText(
                f"Giữ tại mốc {task['nominal_distance_m']:.1f} m — robot đứng yên; "
                "không cần bấm nút nào."
            )
            self.auto_countdown_label.setText(
                f"Còn {max(0, int(task['duration_s'] - elapsed_s))} giây"
            )
            return

        if scenario_id == "E1_robot_approach":
            action_start = int(scenario.get("auto_action_start_s", 5))
            if elapsed_s < action_start:
                self.current_phase_id = "baseline"
                prompt = (
                    f"Đứng yên tại mốc 2,2 m — đang ghi nền; MiR bị khóa "
                    f"({action_start - int(elapsed_s)} giây)."
                )
            elif self.robot_approach_completed:
                self.current_phase_id = "hold"
                prompt = (
                    "MiR đã tới đích 1,4 m và đang bị khóa. Người tiếp tục đứng yên "
                    "đến khi lượt đo kết thúc."
                )
            else:
                if not self.robot_approach_motion_released:
                    self.motion_inhibit_requested.emit(False)
                    self.robot_approach_motion_released = True
                    self._emit_phase_event("robot_approach_start")
                self.current_phase_id = "robot_approach"
                prompt = (
                    "MiR TỰ TIẾN TỚI — người đứng yên tại chỗ; không lùi, không đi theo robot. "
                    "Robot sẽ tự dừng khi ổn định ở 1,4 m ±0,03 m."
                )
            self.auto_phase_label.setText(prompt)
            self.auto_countdown_label.setText(
                f"Thời gian lượt: {int(elapsed_s)}/{task['duration_s']} giây"
            )
            return

        action_start = int(scenario.get("auto_action_start_s", 5))
        action_end = int(scenario.get("auto_action_end_s", task["duration_s"] - 10))
        if elapsed_s < action_start:
            self.current_phase_id = "baseline"
            if scenario.get("scenario_action") in {
                "crossing_no_loss", "crossing_occlusion", "c1_crossing"
            }:
                prompt = (
                    f"Đang ghi nền ({action_start - int(elapsed_s)} giây); "
                    "giữ người tại mốc xuất phát, robot khóa đứng yên."
                )
            else:
                prompt = f"Đứng yên — đang ghi đoạn nền ({action_start - int(elapsed_s)} giây)."
        elif elapsed_s < action_end:
            if scenario.get("scenario_action") == "crossing_no_loss":
                self._emit_phase_event("crossing_start")
                prompt = (
                    "BẮT ĐẦU CẮT NGANG — người hỗ trợ đi một lần "
                    f"{task.get('crossing_direction', 'theo hướng đang chọn')} ở "
                    f"{task.get('walking_speed_mps', 0.4):.1f} m/s, chỉ che một phần; "
                    "mục tiêu đứng yên ở 2,20 m. Robot khóa; C1 chưa chạy."
                )
            elif scenario.get("scenario_action") == "crossing_occlusion":
                self._emit_phase_event("crossing_start")
                prompt = (
                    "BẮT ĐẦU CHE KHUẤT — đi "
                    f"{task.get('crossing_direction', 'theo hướng đang chọn')} ở "
                    f"{task.get('walking_speed_mps', 0.4):.1f} m/s, che kín mục tiêu "
                    f"{task.get('occlusion_duration_s', 0.5):.2f} giây "
                    "rồi rời đi; robot khóa, C1 chưa chạy."
                )
            elif scenario.get("scenario_action") == "c1_crossing":
                self._emit_phase_event("crossing_start")
                prompt = (
                    "BẮT ĐẦU CROSSING — người mục tiêu đi ngang một lần theo hướng "
                    f"{task.get('crossing_direction', 'đang chọn')} ở "
                    f"{task.get('walking_speed_mps', 0.0):.1f} m/s; robot đứng yên, "
                    "tiếp tục ghi dữ liệu RGB-D."
                )
            elif scenario.get("scenario_action") == "c1_approach_control":
                self._emit_phase_event("approach_start")
                prompt = (
                    "BẮT ĐẦU TIẾN LẠI — đi thẳng từ 2,0 m tới 1,0 m ở khoảng "
                    "0,30 m/s (khoảng 3,3 giây), sau đó đứng yên; "
                    "robot khóa, chỉ ghi dữ liệu đầu vào."
                )
            else:
                self._emit_phase_event("approach_start")
                prompt = "BẮT ĐẦU TIẾN LẠI — đi thẳng về phía robot theo tốc độ đã thống nhất."
        else:
            if scenario.get("scenario_action") in {
                "crossing_no_loss", "crossing_occlusion", "c1_crossing"
            }:
                self._emit_phase_event("crossing_end")
                if scenario.get("scenario_action") == "crossing_occlusion":
                    prompt = (
                        "KẾT THÚC CHE KHUẤT — người hỗ trợ rời khỏi trục nhìn; "
                        "người mục tiêu đứng yên để ghi nhận nhận lại đủ 10 khung hình."
                    )
                elif scenario.get("scenario_action") == "c1_crossing":
                    prompt = (
                        "KẾT THÚC CROSSING — người dừng ngoài trục nhìn và giữ yên; "
                        "robot tiếp tục khóa đến khi lượt tự kết thúc."
                    )
                else:
                    prompt = "KẾT THÚC CẮT NGANG — người hỗ trợ dừng ngoài trục nhìn; mục tiêu tiếp tục đứng yên."
            else:
                self._emit_phase_event("approach_end")
                prompt = "KẾT THÚC TIẾN LẠI — dừng tại điểm cuối và giữ nguyên vị trí."
        self.auto_phase_label.setText(prompt)
        self.auto_countdown_label.setText(
            f"Thời gian lượt: {int(elapsed_s)}/{task['duration_s']} giây"
        )

    def _tick_auto_sequence(self):
        if self.auto_state == "countdown":
            self.auto_countdown -= 1
            self.auto_countdown_label.setText(f"Tự bắt đầu sau {max(0, self.auto_countdown)} giây")
            if self.auto_countdown <= 0:
                self._start_current_run()
        elif self.auto_state == "running" and self.run_started_at is not None:
            elapsed_s = max(0.0, time.time() - self.run_started_at)
            self._update_run_phase(elapsed_s)
            if elapsed_s >= self.current_task["duration_s"]:
                if (
                    self.current_task["scenario_id"] == "E1_robot_approach"
                    and not self.robot_approach_completed
                ):
                    self._emit_phase_event("robot_approach_timeout")
                    self.motion_inhibit_requested.emit(True)
                    self.current_phase_id = "timeout"
                    self.auto_phase_label.setText(
                        "Hết thời gian nhưng MiR chưa ổn định tại 1,4 m; lượt sẽ được ghi là chưa đạt."
                    )
                self._finish_current_run()
        elif self.auto_state == "transition":
            if self.reposition_control_active:
                self.auto_countdown_label.setText(
                    "Đang tạm dừng phiên để điều khiển robot; bấm nút khóa điều khiển thường khi đã về vị trí."
                )
                return
            self.auto_countdown -= 1
            self.auto_countdown_label.setText(
                f"Chuẩn bị vị trí/lượt tiếp theo — tự tiếp tục sau {max(0, self.auto_countdown)} giây"
            )
            if self.auto_countdown <= 0:
                self.auto_state = "waiting_for_target"
                self._prepare_current_task()

    def set_run_quality(self, quality):
        self.current_run_quality = dict(quality or {})

    def _finish_current_run(self, aborted=False):
        if not self.run_active:
            return
        self.run_active = False
        self.elapsed_timer.stop()
        self.motion_inhibit_requested.emit(True)
        self._update_skip_controls()
        info = self._current_info({"auto_abort": bool(aborted)})
        self.run_stop_requested.emit(info)
        if aborted:
            self.auto_timer.stop()
            self.auto_state = "aborted"
            self.auto_phase_label.setText("Đã dừng phiên tự động theo yêu cầu.")
            self.auto_countdown_label.setText("Lệnh vận tốc 0 đã được phát; dữ liệu lượt dở được giữ lại.")
            self.status_label.setText("Phiên thu thập đã dừng")
            return

        quality = self.current_run_quality or {"passed": False, "reasons": ["Không nhận được kết quả kiểm tra log"]}
        if quality.get("passed"):
            self.completed_task_ids.add(self.current_task["task_id"])
            self.auto_index += 1
            self.status_label.setText("Đủ log kỹ thuật — tự chuyển sang lượt kế tiếp")
        elif self.current_task["attempt"] < AUTO_MAX_ATTEMPTS:
            self.current_task["attempt"] += 1
            reasons = ", ".join(quality.get("reasons", []))
            self.status_label.setText(f"Log chưa đủ ({reasons}) — tự ghi lại lượt này")
        else:
            self.exhausted_task_ids.add(self.current_task["task_id"])
            self.auto_index += 1
            self.status_label.setText("Log chưa đủ sau 3 lần — đánh dấu thiếu và tự chuyển tiếp")

        self.refresh_progress()
        self._refresh_stage_controls()
        if self.auto_index >= len(self.auto_queue):
            self._finish_auto_session()
            return
        self.auto_state = "transition"
        self.auto_countdown = AUTO_RESET_SECONDS
        self._select_current_task()
        self.motion_inhibit_requested.emit(True)
        self.auto_phase_label.setText(self._preparation_prompt())
        self.auto_countdown_label.setText(
            f"Chuẩn bị vị trí/lượt tiếp theo — tự tiếp tục sau {AUTO_RESET_SECONDS} giây"
        )

    def abort_auto_session(self):
        if self.auto_state in {"aborted", "complete"}:
            return
        self._stop_reposition_controls(
            "Đã khóa robot khi dừng phiên; lượt đo chưa bắt đầu."
        )
        self.motion_inhibit_requested.emit(True)
        if self.run_active:
            self._finish_current_run(aborted=True)
            return
        self.auto_timer.stop()
        self.auto_state = "aborted"
        self.auto_phase_label.setText("Đã dừng phiên tự động theo yêu cầu.")
        self.auto_countdown_label.setText("Phiên chưa bắt đầu hoặc không có lượt đang ghi.")
        self.status_label.setText("Phiên thu thập đã dừng")

    def _finish_auto_session(self):
        self._stop_reposition_controls(
            "Đã khóa robot khi kết thúc hàng đợi đo."
        )
        self.motion_inhibit_requested.emit(True)
        self.auto_state = "complete"
        self.auto_timer.stop()
        self.auto_task_label.setText("Đã kết thúc giai đoạn thu bản ghi nguồn")
        passed_count = len(self.completed_task_ids & self.all_task_ids)
        if self.collection_stage == "development":
            if self._stage_is_complete("development"):
                completion_text = (
                    "Đã thu đủ 6 bản ghi hiệu chỉnh. Hãy gán nhãn và triển khai/hiệu chỉnh C1 ngoại tuyến "
                    "(ứng dụng thu dữ liệu hiện chưa chạy C1); sau đó nhập mã phiên bản/tham số đã khóa, "
                    "xác nhận ô khóa C1 và chọn giai đoạn đánh giá giữ riêng."
                )
            else:
                completion_text = (
                    f"Hàng đợi hiệu chỉnh tạm dừng/kết thúc: đạt {passed_count}/6 bản ghi. "
                    "Cần khôi phục các kịch bản bị bỏ qua hoặc xử lý lượt lỗi trước khi mở tập đánh giá."
                )
            next_text = (
                "Bản ghi hiệu chỉnh không dùng làm kết quả đánh giá cuối. Sau khi chuyển giai đoạn, "
                "thu 15 bản ghi mới; không dùng chúng để chỉnh C1."
            )
        else:
            if self._stage_is_complete("held_out_evaluation"):
                completion_text = (
                    "Đã thu đủ 15 bản ghi đánh giá giữ riêng. Mỗi bản ghi được dành cho phát lại ghép cặp qua 3 cấu hình; "
                    "kỳ vọng 45 đầu ra, hiện chưa được tạo trong giao diện thu dữ liệu."
                )
            else:
                completion_text = (
                    f"Hàng đợi đánh giá tạm dừng/kết thúc: đạt {passed_count}/15 bản ghi nguồn. "
                    "Giữ riêng tập này; không dùng để chỉnh C1."
                )
            next_text = (
                "Cần chạy công cụ phát lại sau khi 3 cấu hình A/B/C được cài đặt và khóa; "
                "đối chiếu cùng bản ghi, khoảng cách chuẩn liên tục và nhãn sự kiện độc lập."
            )
        self.auto_phase_label.setText(completion_text)
        self.auto_countdown_label.setText(next_text)
        skipped_titles = [
            get_scenario(scenario_id)["title"]
            for scenario_id in self.skipped_scenarios
            if any(item["id"] == scenario_id for item in SCENARIOS)
        ]
        if skipped_titles:
            self.auto_phase_label.setText(
                "Đã bỏ qua (vẫn giữ log cũ): " + ", ".join(skipped_titles)
            )
        if self.exhausted_task_ids:
            self.status_label.setText(
                f"Đã kết thúc hàng đợi; {len(self.exhausted_task_ids)} lượt không đủ log sau 3 lần thử."
            )
        else:
            self.status_label.setText("Hoàn tất phiên tự động")
        self._update_skip_controls()
        self._refresh_stage_controls()
        self.refresh_progress()

    def _apply_scenario_defaults(self):
        scenario = get_scenario(self.scenario_combo.currentData())
        self.walk_speed_spin.setValue(scenario.get("suggested_walking_speed_mps", 0.0))

    def _scenario_changed(self):
        self._apply_scenario_defaults()
        self._refresh_description()

    def _refresh_description(self):
        scenario = get_scenario(self.scenario_combo.currentData())
        config = get_configuration(self.config_combo.currentData())
        desired_distance_m = float(
            os.environ.get('DESIRED_CAMERA_DISTANCE_M', '1.40')
        )
        current_task_matches = bool(
            self.current_task and self.current_task.get("scenario_id") == scenario["id"]
        )
        scenario_start_distance_m = float(
            self.current_task.get("start_distance_m")
            if current_task_matches and self.current_task.get("start_distance_m") is not None
            else scenario.get("auto_start_distance_m", desired_distance_m)
        )
        if scenario.get("c1_capture_only"):
            protocol_motion_note = (
                "<b>CHỈ THU DỮ LIỆU ĐẦU VÀO: robot khóa đứng yên; C1 chưa chạy và chưa được so sánh. "
                "Lệnh P hiện tại chỉ là đầu ra giả lập.</b><br>"
            )
        elif scenario.get("motion_inhibited"):
            protocol_motion_note = (
                "<b>Robot bị khóa đứng yên; lệnh thật giữ 0, lệnh P giả lập được ghi riêng. "
                "Nếu mất dấu, bộ điều khiển dừng và không đuổi theo người hỗ trợ.</b><br>"
            )
        else:
            protocol_motion_note = ""
        distance_tolerance_m = 0.03
        tolerance_min_m = desired_distance_m - distance_tolerance_m
        tolerance_max_m = desired_distance_m + distance_tolerance_m
        rgbd_recording = os.environ.get(
            "EXPERIMENT_RECORD_RGBD_ACTIVE",
            os.environ.get("EXPERIMENT_RECORD_RGBD", "0"),
        ).strip().lower() in {
            "1", "true", "yes", "on"
        }
        rgbd_status = (
            "ĐANG lưu ảnh màu và độ sâu vào tệp phiên trong experiments/logs/_sessions (dung lượng lớn)."
            if rgbd_recording else
            "Chỉ lưu bảng CSV và thông tin từng khung hình; chưa lưu ảnh màu/độ sâu gốc. Bật EXPERIMENT_RECORD_RGBD=1 trước khi mở ứng dụng nếu quy trình cần tệp ảnh gốc."
        )
        stage_note = (
            "Giai đoạn hiện tại: 6 bản ghi hiệu chỉnh (2 mỗi nhánh), dùng để gán nhãn và hiệu chỉnh C1 ngoại tuyến."
            if self.collection_stage == "development"
            else "Giai đoạn hiện tại: 15 bản ghi đánh giá giữ riêng (5 mỗi nhánh); không dùng để chỉnh C1. "
                 "Mục tiêu là phát lại 15 bản ghi qua 3 cấu hình để có 45 kết quả ghép cặp."
        )
        self.description_label.setText(
            f"<b>{scenario['title']}</b><br>{scenario['objective']}<br>"
            f"Cấu hình đang chọn: <b>{config['label']}</b> — {config['description']}<br>"
            f"Mốc điều khiển: <b>{desired_distance_m:.2f} m</b> "
            "theo số đo độ sâu D435 từ tâm camera đến vùng người; đây là mốc đang dùng. "
            f"Bộ điều khiển dùng trung bình trượt tối đa 5 mẫu hợp lệ; trong vùng dung sai ±{distance_tolerance_m:.2f} m "
            f"([{tolerance_min_m:.2f}, {tolerance_max_m:.2f}] m), robot nhận lệnh vận tốc 0 cả tiến/lùi và quay "
            f"(ví dụ {desired_distance_m + 0.01:.2f} m nằm trong vùng này). Robot có thể trôi nhẹ do quán tính khi dừng. Mất dấu hoặc độ sâu lỗi sẽ phát lệnh dừng ngay.<br>"
            "Khe hở mép trước MiR–người được ghi riêng dưới dạng ước lượng hình học, không phải khoảng cách chuẩn độc lập.<br>"
            f"MiR100 dài {float(os.environ.get('ROBOT_LENGTH_M', '0.89')):.2f} m; camera ở mép sau/trục giữa, "
            "cao 1,80 m và chúc xuống 30°; "
            f"thời lượng lượt tự động {self.current_task['duration_s'] if self.current_task and self.current_task['scenario_id'] == scenario['id'] else scenario['duration_s']} giây; mục tiêu "
            f"{scenario['minimum_repetitions']}–{scenario['maximum_repetitions']} lượt đo.<br>"
            f"Trước lượt này cần ổn định tại {scenario_start_distance_m:.2f} m trong {AUTO_START_STABILITY_FRAMES} khung hình liên tiếp.<br>"
            + protocol_motion_note
            + f"<b>Ảnh màu–độ sâu:</b> {rgbd_status}<br>"
            + f"<b>Thiết kế hàng đợi:</b> {stage_note} Phát lại chưa chạy trong giao diện thu dữ liệu."
        )
        self.preparation_edit.setPlainText("\n".join(
            f"{index}. {step}" for index, step in enumerate(scenario.get("preparation", []), 1)
        ))
        self.procedure_edit.setPlainText("\n".join(
            f"{index}. {step}" for index, step in enumerate(scenario.get("measurement_procedure", scenario["operator_steps"]), 1)
        ))
        self.ground_truth_label.setText(scenario.get("ground_truth_method", "Chưa có hướng dẫn về khoảng cách chuẩn độc lập."))
        event_labels = [EVENT_LABELS.get(event, event) for event in scenario["required_events"]]
        event_text = "Mốc chính tự ghi: " + " → ".join(event_labels)
        optional_events = scenario.get("optional_events", [])
        if optional_events:
            optional_labels = [EVENT_LABELS.get(event, event) for event in optional_events]
            event_text += "; ghi thêm khi xảy ra: " + ", ".join(optional_labels)
        self.events_label.setText(event_text)
        minimum_target_ratio = float(scenario.get("auto_min_target_presence_ratio", 0.4))
        self.metrics_label.setText(
            "Thông số cần quan sát: " + ", ".join(scenario["metrics"]) +
            f".<br>Mã điều kiện gợi ý: <code>{scenario['variant_hint']}</code>. "
            "Lệnh, khung hình, ROI, sự kiện và odometry được ghi tự động. "
            f"Trước mỗi lượt, khoảng cách camera phải ổn định trong ±{AUTO_START_DISTANCE_TOLERANCE_M:.2f} m "
            f"đủ {AUTO_START_STABILITY_FRAMES} khung hình liên tiếp tại mốc xuất phát "
            f"{scenario_start_distance_m:.2f} m của kịch bản này. "
            f"Cổng đủ log yêu cầu mục tiêu xuất hiện ≥{minimum_target_ratio:.0%}, ROI hợp lệ ≥{AUTO_MIN_VALID_ROI_RATIO:.0%}, "
            f"FPS ≥{AUTO_MIN_FRAME_RATE_HZ:.0f} và odometry ≥{AUTO_MIN_ODOMETRY_RATE_HZ:.0f} Hz. "
            "Khoảng cách chuẩn liên tục chưa được tạo tự động; cần đo/đồng bộ độc lập để tính các chỉ số paper."
        )
        self.refresh_progress()

    def _completed_run_counts(self):
        """Count completed protocol runs that meet the scenario logging checks."""
        log_root = Path(os.environ.get("EXPERIMENT_LOG_ROOT", "/app/experiments/logs"))
        counts = {(config["id"], scenario["id"]): 0
                  for config in CONFIGURATIONS for scenario in SCENARIOS}
        if not log_root.is_dir():
            return counts
        for run_dir in log_root.iterdir():
            if not run_dir.is_dir():
                continue
            metadata_path = run_dir / "run_metadata.json"
            try:
                metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            except (OSError, ValueError, TypeError):
                continue
            try:
                schema_version = int(metadata.get("schema_version", 0) or 0)
            except (TypeError, ValueError):
                continue
            if schema_version < 3:
                continue
            if metadata.get("status") != "scenario_completed":
                continue
            if int(metadata.get("counts", {}).get("frames", 0) or 0) <= 0:
                continue
            config_id = metadata.get("config_name", "")
            scenario_name = metadata.get("scenario", "")
            control_mode = metadata.get("control_mode", "")
            for scenario in SCENARIOS:
                if scenario_name == scenario["id"] or scenario_name.startswith(scenario["id"] + "_"):
                    if scenario["id"] == "E1_calibration":
                        if int(metadata.get("counts", {}).get("ground_truth_samples", 0) or 0) < 3:
                            break
                    elif control_mode != "live" or not metadata.get("ground_truth_ready", False):
                        break
                    key = (config_id, scenario["id"])
                    if key in counts:
                        counts[key] += 1
                    break
        return counts

    def refresh_progress(self):
        """Show automatic queue progress and the next operator action."""
        all_ids = getattr(self, "all_task_ids", set())
        total = len(all_ids)
        completed = len(self.completed_task_ids & all_ids)
        exhausted = len(self.exhausted_task_ids & all_ids)
        skipped_titles = [
            get_scenario(scenario_id)["title"]
            for scenario_id in self.skipped_scenarios
            if any(item["id"] == scenario_id for item in SCENARIOS)
        ]
        skipped_text = (
            " Kịch bản đã bỏ qua (log cũ vẫn giữ): " + ", ".join(skipped_titles) + "."
            if skipped_titles else ""
        )
        stage_name = (
            "hiệu chỉnh (6 bản ghi)"
            if self.collection_stage == "development"
            else "đánh giá giữ riêng (15 bản ghi)"
        )
        self.progress_label.setText(
            f"Giai đoạn <b>{stage_name}</b> — đủ log kỹ thuật: <b>{completed}/{total}</b> bản ghi nguồn; "
            f"không đạt sau 3 lần thử: <b>{exhausted}</b>. "
            "Mỗi lượt tự kiểm tra khung hình, ROI, lệnh và odometry trước khi chuyển tiếp. "
            "Mỗi bản ghi nguồn được thu một lần; phát lại qua 3 cấu hình là bước ngoại tuyến sau đó."
            + skipped_text
        )
        if self.auto_state == "waiting_for_prerequisites":
            message = (
                "Robot đang khóa. Đánh dấu kiểm tra an toàn và chuẩn bị khoảng cách chuẩn độc lập liên tục; "
                "RGB-D phải được bật trước khi mở ứng dụng. Đủ ba điều kiện thì giao diện tự tiếp tục."
            )
        elif self.auto_state == "waiting_for_target":
            message = (
                "Việc duy nhất cần làm lúc này: đăng ký/chọn người trên hình camera. "
                "Sau đó chương trình tự chạy hàng đợi, tự ghi mốc và tự chuyển lượt."
            )
        elif self.auto_state == "waiting_for_follow":
            message = (
                "Bật THEO DÕI một lần để bắt đầu chuẩn bị. Sau đó đứng yên tại mốc xuất phát; "
                "đủ 10 khung hình ổn định thì chương trình tự đếm ngược và đo."
            )
        elif self.auto_state == "waiting_for_stable_distance":
            message = (
                f"Đứng yên trong vùng {self._required_start_distance_m():.2f} m "
                f"±{AUTO_START_DISTANCE_TOLERANCE_M:.2f} m cho đủ "
                f"{AUTO_START_STABILITY_FRAMES} khung hình liên tiếp."
            )
        elif self.auto_state == "complete":
            if self.collection_stage == "development":
                message = (
                    "Tập hiệu chỉnh đã hoàn tất. Khóa C1 và nhập mã phiên bản để mở tập đánh giá; "
                    "không đưa 6 bản ghi này vào kết quả đánh giá cuối."
                    if self._stage_is_complete("development") else
                    "Tập hiệu chỉnh chưa đủ 6 bản ghi hợp lệ; xem lại lượt thiếu hoặc khôi phục kịch bản đã bỏ qua."
                )
            else:
                message = (
                    "Đã có 15 bản ghi nguồn. Sau thu thập, phát lại từng bản ghi qua 3 cấu hình để tạo 45 đầu ra ghép cặp."
                    if self._stage_is_complete("held_out_evaluation") else
                    "Tập đánh giá chưa đủ 15 bản ghi; giữ nguyên các bản ghi và hoàn tất lượt còn thiếu."
                )
        elif self.auto_state == "aborted":
            message = "Phiên đã dừng; dữ liệu các lượt trước vẫn được giữ lại."
        else:
            message = "Các mốc, thời lượng, lưu log, kiểm tra độ đủ và chuyển kịch bản đều tự động."
        self.next_action_label.setText(message)

    def _collect_info(self):
        return {
            "config_id": self.config_combo.currentData(),
            "scenario_id": self.scenario_combo.currentData(),
            "variant": self.variant_edit.text().strip() or "unspecified",
            "repeat_id": self.repeat_spin.value(),
            "operator_id": self.operator_edit.text().strip(),
            "operator_notes": self.notes_edit.text().strip(),
            "safety_checked": self.safety_check.isChecked(),
            "ground_truth_ready": self.ground_truth_check.isChecked(),
            "participant_height_m": self.participant_height_spin.value(),
            "walking_speed_mps": self.walk_speed_spin.value(),
            "walking_speed_value_type": "planned_or_estimated",
            "crossing_angle_deg": self.crossing_angle_spin.value(),
            "crowd_count": self.crowd_count_spin.value(),
            "ground_truth_source": self.reference_source_combo.currentData(),
            "ground_truth_definition": self.reference_definition_combo.currentData(),
            "source": "real",
            "control_mode": self.control_mode,
        }

    def _start_clicked(self):
        info = self._collect_info()
        config = get_configuration(info["config_id"])
        if not info["operator_id"]:
            QMessageBox.warning(self, "Thiếu thông tin", "Nhập tên người thực hiện đo trước khi bắt đầu.")
            return
        if not info["safety_checked"]:
            QMessageBox.warning(self, "Chưa kiểm tra an toàn", "Cần xác nhận vùng chạy và nút dừng khẩn cấp.")
            return
        if self.control_mode == "live" and not info["ground_truth_ready"]:
            QMessageBox.warning(
                self,
                "Thiếu mốc khoảng cách",
                "Chạy thật cần nguồn đo độc lập hoặc video ngoài đồng bộ. Không dùng số đo D435 làm khoảng cách chuẩn độc lập.",
            )
            return
        if not config["implemented"]:
            allowed = {
                item.strip() for item in os.environ.get("E1_IMPLEMENTED_CONFIGS", "p_tapered").split(",")
                if item.strip()
            }
            if info["config_id"] not in allowed:
                QMessageBox.warning(
                    self,
                    "Cấu hình chưa sẵn sàng",
                    f"{config['label']} mới là nhãn kế hoạch, chưa được xác nhận trong code. "
                    "Hãy triển khai/xác nhận cấu hình rồi đặt E1_IMPLEMENTED_CONFIGS.",
                )
                return
        scenario = get_scenario(info["scenario_id"])
        self.run_active = True
        self.run_started_at = time.time()
        self._set_controls_enabled(False)
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.event_button.setEnabled(True)
        self.reference_button.setEnabled(True)
        self.status_label.setText("Đang ghi lần đo — thực hiện đúng hướng dẫn")
        self.status_label.setStyleSheet("font-weight:bold;color:#198754;")
        self.elapsed_timer.start(500)
        self.run_start_requested.emit(info)
        self.motion_inhibit_requested.emit(
            info["scenario_id"] == "E1_calibration"
            or bool(scenario.get("motion_inhibited", False))
        )

    def _stop_clicked(self):
        if not self.run_active:
            return
        info = self._collect_info()
        self.run_active = False
        self.elapsed_timer.stop()
        self._set_controls_enabled(True)
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.event_button.setEnabled(False)
        self.reference_button.setEnabled(False)
        self.status_label.setText("Đã kết thúc lần đo — hãy kiểm tra thư mục nhật ký")
        self.status_label.setStyleSheet("font-weight:bold;color:#6c757d;")
        self.run_stop_requested.emit(info)

    def _event_clicked(self):
        if not self.run_active:
            return
        event_name = self.event_combo.currentData()
        event_label = self.event_combo.currentText()
        self.event_requested.emit(event_name, self._collect_info())
        self.status_label.setText(f"Đã đánh dấu: {event_label}")

    def _ground_truth_clicked(self):
        if not self.run_active:
            return
        distance_m = self.reference_distance_spin.value()
        if distance_m <= 0.0:
            QMessageBox.warning(self, "Thiếu số đo", "Nhập khoảng cách từ nguồn độc lập trước khi ghi mốc.")
            return
        info = self._collect_info()
        info["ground_truth_source"] = self.reference_source_combo.currentData()
        info["ground_truth_definition"] = self.reference_definition_combo.currentData()
        info["ground_truth_note"] = self.reference_note_edit.text().strip()
        self.ground_truth_requested.emit(distance_m, info)
        self.status_label.setText(f"Đã ghi mốc độc lập: {distance_m:.3f} m")

    def _set_controls_enabled(self, enabled):
        for widget in (
            self.config_combo,
            self.scenario_combo,
            self.variant_edit,
            self.repeat_spin,
            self.operator_edit,
            self.participant_height_spin,
            self.walk_speed_spin,
            self.crossing_angle_spin,
            self.crowd_count_spin,
            self.notes_edit,
            self.safety_check,
            self.ground_truth_check,
            self.reference_source_combo,
            self.reference_definition_combo,
        ):
            widget.setEnabled(enabled)

    def _update_elapsed(self):
        if self.run_started_at is None:
            return
        elapsed = max(0, int(time.time() - self.run_started_at))
        self.elapsed_label.setText(f"Thời gian: {elapsed // 60:02d}:{elapsed % 60:02d}")

    def set_run_directory(self, run_dir):
        self.run_dir_label.setText(f"Thư mục nhật ký: {run_dir}")

