"""Structured E1 configurations and operator-facing scenario definitions.

The controller setpoint refers to the RealSense camera-to-target distance used
by the original implementation.  The estimated gap from the MiR front edge to
the target is logged separately because the camera is mounted at the rear edge.
"""

from copy import deepcopy


AUTO_CALIBRATION_DISTANCES_M = (1.0, 1.4, 2.0)
AUTO_PREPARE_SECONDS = 10
AUTO_RESET_SECONDS = 10
AUTO_START_STABILITY_FRAMES = 10
AUTO_START_DISTANCE_TOLERANCE_M = 0.05
AUTO_MAX_ATTEMPTS = 3
AUTO_MIN_FRAME_RATE_HZ = 5.0
AUTO_MIN_ODOMETRY_RATE_HZ = 1.0
AUTO_MIN_VALID_ROI_RATIO = 0.5
E1_REPLAY_PROFILE_IDS = (
    "baseline_p_zero_fallback",
    "p_drop_invalid_depth",
    "p_c1_range_integrity",
)
E1_COLLECTION_STAGES = (
    {"id": "development", "label": "Giai đoạn 1 — 6 bản ghi hiệu chỉnh C1"},
    {"id": "held_out_evaluation", "label": "Giai đoạn 2 — 15 bản ghi đánh giá giữ riêng"},
)
E1_CAPTURE_SCENARIO_IDS = (
    "E1_C1_crossing_visible",
    "E1_C1_crossing_occlusion",
    "E1_C1_approach_control",
)


CONFIGURATIONS = [
    {
        "id": "c1_input_capture",
        "label": "Thu dữ liệu đầu vào C1 — chưa áp dụng C1",
        "description": (
            "Chỉ ghi dữ liệu RGB-D và đầu ra P giả lập để phân tích ghép cặp sau này. "
            "C1 chưa được chạy; MiR bị khóa đứng yên trong lượt thu dữ liệu."
        ),
        "implemented": True,
        "auto_queue_enabled": True,
        "controller": {
            "profile_id": "c1_input_capture",
            "linear_kp_mps_per_m": 0.75,
            "max_forward_speed_mps": 0.35,
            "max_reverse_speed_mps": 0.25,
            "acceleration_limit_mps2": 0.35,
            "deceleration_limit_mps2": 0.65,
            "deadband_m": 0.03,
            "invalid_depth_policy": "zero_command_and_do_not_update_distance_history",
            "capture_only": True,
        },
    },
    {
        "id": "p_tapered",
        "label": "P giảm tốc theo sai lệch — mặc định",
        "description": "P = 0,75 (m/s)/m; tiến tối đa 0,35 m/s, lùi 0,25 m/s, tăng tốc 0,35 m/s², giảm tốc 0,65 m/s². Bỏ mẫu độ sâu lỗi; trong ±3 cm hoặc mất mục tiêu thì dừng ngay.",
        "implemented": True,
        "auto_queue_enabled": False,
        "controller": {
            "profile_id": "p_tapered",
            "linear_kp_mps_per_m": 0.75,
            "max_forward_speed_mps": 0.35,
            "max_reverse_speed_mps": 0.25,
            "acceleration_limit_mps2": 0.35,
            "deceleration_limit_mps2": 0.65,
            "deadband_m": 0.03,
            "invalid_depth_policy": "zero_command_and_do_not_update_distance_history",
        },
    },
    {
        "id": "p_original",
        "label": "P cũ — đối chứng thủ công",
        "description": (
            "Giữ hệ số và trần tốc độ P cũ (0,90 m/s), nhưng bộ xử lý độ sâu hiện tại "
            "đã bỏ mẫu ROI lỗi khỏi lịch sử. Chưa tái tạo đúng baseline cũ có đưa số 0 vào bộ lọc."
        ),
        "implemented": True,
        "auto_queue_enabled": False,
        "controller": {
            "profile_id": "p_original",
            "linear_kp_mps_per_m": 1.8,
            "max_forward_speed_mps": 0.9,
            "max_reverse_speed_mps": 0.9,
            "acceleration_limit_mps2": None,
            "deceleration_limit_mps2": None,
            "deadband_m": 0.03,
            "invalid_depth_policy": "zero_command_and_do_not_update_distance_history",
        },
    },
    {
        "id": "p_no_zero",
        "label": "P bỏ mẫu vùng đo lỗi",
        "description": "Giữ luật P có dấu, bỏ số đo độ sâu lỗi trong vùng quan tâm và giữ an toàn khi dữ liệu cũ hết hạn.",
        "implemented": False,
    },
    {
        "id": "p_c1",
        "label": "P có C1 — kiểm tra độ tin cậy khoảng cách",
        "description": "Bộ điều khiển P có dấu với cổng C1 kiểm tra tính toàn vẹn khoảng cách; cần triển khai riêng.",
        "implemented": False,
    },
]


SCENARIOS = [
    {
        "id": "E1_calibration",
        "title": "E1-0 — Kiểm tra khoảng cách và hiệu chuẩn",
        "objective": "Kiểm tra ảnh màu–độ sâu, độ ổn định khoảng cách với người đứng yên và cách đo độc lập.",
        "recommended_repetitions": 3,
        "minimum_repetitions": 3,
        "maximum_repetitions": 3,
        "auto_queue_enabled": False,
        "duration_s": 60,
        "auto_min_target_presence_ratio": 0.7,
        "suggested_walking_speed_mps": 0.0,
        "variant_hint": "cal_3muc_1p0_1p4_2p0",
        "preparation": [
            "Đỗ robot trên sàn phẳng, giữ robot đứng yên và dùng cùng một người/mục tiêu.",
            "Chuẩn bị thước laser hoặc mốc sàn đã đo độc lập; xác định rõ điểm tham chiếu trên người.",
            "Dọn trống vùng xung quanh robot và giữ nút dừng khẩn cấp trong tầm với.",
            "Ghi điều kiện ánh sáng, khoảng cách tham chiếu và xác nhận ảnh màu/độ sâu không bị mất khung.",
        ],
        "measurement_procedure": [
            "Giao diện tự chuyển qua các mốc camera–người khoảng 1,0 m; 1,4 m; 2,0 m, mỗi mốc giữ 15 giây và lặp ba lượt.",
            "Khi dòng hướng dẫn đổi mốc, di chuyển người tới dấu sàn đã đo độc lập rồi đứng yên đến hết đếm ngược.",
            "Robot được khóa đứng yên trong phần hiệu chuẩn; không cần bấm nút ghi hay đánh dấu sự kiện.",
            "Các giá trị 1,0/1,4/2,0 m là mốc bố trí; không được coi là khoảng cách chuẩn nếu chưa đo độc lập.",
        ],
        "ground_truth_method": "Khoảng cách chuẩn phải đến từ thước laser/mốc đo ngoài, không lấy lại từ D435. Ghi rõ phép đo là tâm camera–điểm trên người hay mép robot–người.",
        "operator_steps": [
            "Đứng yên tại ba mốc khoảng cách 1,0; 1,4; 2,0 m.",
            "Mỗi mốc giữ khoảng 15 giây và ghi giá trị chuẩn độc lập.",
            "Đây là dữ liệu hiệu chuẩn, không gộp vào đánh giá người cắt ngang/tiến lại gần.",
        ],
        "required_events": ["scenario_start", "calibration_point", "scenario_end"],
        "metrics": ["trung vị độ sâu vùng quan tâm", "độ phân tán khoảng cách", "tỷ lệ vùng đo rỗng", "tỷ lệ số đo độ sâu bằng 0", "tốc độ khung hình"],
    },
    {
        "id": "E1_C1_crossing_visible",
        "title": "C1-1A — Người cắt ngang, mục tiêu vẫn được theo dõi",
        "scenario_action": "crossing_no_loss",
        "objective": (
            "Đo nhiễu khoảng cách khi người hỗ trợ đi qua trước mục tiêu nhưng hệ thống vẫn theo dõi được mục tiêu. "
            "Robot khóa đứng yên; cùng dữ liệu RGB-D sẽ được phát lại qua P gốc, P bỏ mẫu 0 và P+C1."
        ),
        "recommended_repetitions": 5,
        "minimum_repetitions": 2,
        "maximum_repetitions": 5,
        "development_repetitions_per_condition": 1,
        "duration_s": 25,
        "auto_action_start_s": 5,
        "auto_action_end_s": 12,
        "auto_start_distance_m": 2.2,
        "auto_min_target_presence_ratio": 0.25,
        "auto_queue_enabled": True,
        "auto_conditions": [
            {"variant": "partial_90deg_0p4mps_trai_phai", "walking_speed_mps": 0.4,
             "crossing_angle_deg": 90.0, "crossing_direction": "trái sang phải"},
            {"variant": "partial_90deg_0p4mps_phai_trai", "walking_speed_mps": 0.4,
             "crossing_angle_deg": 90.0, "crossing_direction": "phải sang trái"},
        ],
        "motion_inhibited": True,
        "record_shadow_controller": True,
        "c1_capture_only": True,
        "requires_rgbd_recording": True,
        "offline_comparison_profiles": [
            "A: P baseline zero-fallback (chỉ chạy nếu tái tạo đúng mã gốc)",
            "B: P loại mẫu depth 0/không hợp lệ",
            "C: P + cổng C1 kiểm tra tính toàn vẹn khoảng cách (sau khi triển khai/khóa C1)",
        ],
        "allow_dry_run_capture": True,
        "crowd_count": 2,
        "target_role": "stationary_tracked_person",
        "occluder_role": "partial_crossing_helper",
        "variant_hint": "C1_muc_tieu_2p2m_che_mot_phan",
        "preparation": [
            "Cần hai người: mục tiêu đã đăng ký đứng yên ở camera–ROI 2,20 m; người hỗ trợ đi qua phía trước theo vạch sàn đã đánh dấu.",
            "Đặt đường người hỗ trợ gần camera hơn mục tiêu (khoảng 1,9–2,0 m), để đi qua ROI nhưng chỉ che một phần. Dùng cùng người, đường đi và tốc độ 0,40 m/s; đổi chiều theo hướng hiện trên giao diện.",
            "Robot bị khóa lệnh chuyển động trong mọi lượt. C1 chưa chạy; phiên này chỉ lưu RGB-D và lệnh P giả lập hiện tại.",
            "Bật lưu RGB-D trước khi mở ứng dụng. Chuẩn bị video ngoài đồng bộ hoặc nguồn đo độc lập để xác nhận mục tiêu đứng yên và canh thời gian.",
        ],
        "measurement_procedure": [
            "Đứng ở mốc camera–ROI 2,20 m; chờ 10 khung hình liên tiếp trong ±0,05 m rồi đứng yên ghi nền 5 giây.",
            "Khi giao diện báo BẮT ĐẦU CẮT NGANG, người hỗ trợ đi qua một lần trong 90° ở khoảng 0,40 m/s. Chỉ che một phần; mục tiêu không di chuyển.",
            "Khi giao diện báo KẾT THÚC CẮT NGANG, người hỗ trợ đứng ngoài trục nhìn; mục tiêu giữ nguyên vị trí đến khi hết lượt.",
            "Giai đoạn hiệu chỉnh có 2 lượt (mỗi chiều 1 lượt); giai đoạn đánh giá giữ riêng có 5 lượt (chiều trái→phải, phải→trái, trái→phải, phải→trái, trái→phải). Giao diện tự xếp theo khối; không xóa lượt nếu không xuất hiện lỗi khoảng cách.",
            "Đủ log kỹ thuật không đồng nghĩa với đã xảy ra che khuất phù hợp. Gắn nhãn sau khi xem video/RGB-D; giữ cả các lần không đạt kiểu che dự kiến.",
        ],
        "ground_truth_method": (
            "Mục tiêu đứng yên tại mốc đã đo độc lập; dùng video ngoài đồng bộ để xác nhận vị trí và thời điểm. "
            "Nếu mục tiêu dịch chuyển, cần truy vết khoảng cách liên tục từ thiết bị độc lập; không dùng D435 làm khoảng cách chuẩn."
        ),
        "operator_steps": [
            "Một người đứng yên ở 2,20 m; người thứ hai cắt ngang phía trước rồi rời khỏi trục camera.",
            "Hiệu chỉnh: 2 lượt, mỗi chiều 1 lượt. Đánh giá giữ riêng: 5 lượt theo chiều giao diện chỉ định, cân bằng 3/2 giữa trái→phải và phải→trái; góc 90°, tốc độ gần 0,40 m/s.",
            "MiR phải đứng yên; lệnh thật trong commands.csv bằng 0. Lệnh bóng trong frames.csv không phải chuyển động thật.",
        ],
        "required_events": ["scenario_start", "crossing_start", "crossing_end", "scenario_end"],
        "metrics": [
            "tỷ lệ khung hình mục tiêu còn được theo dõi và số lần đổi ID",
            "tỷ lệ ROI rỗng/depth-zero và các bước nhảy khoảng cách",
            "sai số khoảng cách so với tham chiếu độc lập",
            "lệnh lùi bóng khi khoảng cách chuẩn vẫn lớn hơn 1,43 m",
            "độ trễ và tỷ lệ giữ nhầm của C1 khi phát lại cùng lượt",
        ],
    },
    {
        "id": "E1_C1_crossing_occlusion",
        "title": "C1-1B — Người cắt ngang làm mất dấu tạm thời",
        "scenario_action": "crossing_occlusion",
        "objective": (
            "Đo riêng trường hợp người hỗ trợ che kín mục tiêu gây mất dấu rồi nhận lại, và phân biệt phản ứng dừng do mất dấu "
            "với phản ứng C1 trước một khoảng cách bất thường. Robot khóa đứng yên; C1 chưa chạy trực tiếp."
        ),
        "recommended_repetitions": 5,
        "minimum_repetitions": 2,
        "maximum_repetitions": 5,
        "development_repetitions_per_condition": 1,
        "duration_s": 25,
        "auto_action_start_s": 5,
        "auto_action_end_s": 12,
        "auto_start_distance_m": 2.2,
        "auto_min_target_presence_ratio": 0.25,
        "auto_queue_enabled": True,
        "auto_conditions": [
            {"variant": "full_90deg_0p4mps_trai_phai", "walking_speed_mps": 0.4,
             "crossing_angle_deg": 90.0, "crossing_direction": "trái sang phải"},
            {"variant": "full_90deg_0p4mps_phai_trai", "walking_speed_mps": 0.4,
             "crossing_angle_deg": 90.0, "crossing_direction": "phải sang trái"},
        ],
        "motion_inhibited": True,
        "record_shadow_controller": True,
        "c1_capture_only": True,
        "requires_rgbd_recording": True,
        "offline_comparison_profiles": [
            "A: P baseline zero-fallback (chỉ chạy nếu tái tạo đúng mã gốc)",
            "B: P loại mẫu depth 0/không hợp lệ",
            "C: P + cổng C1 kiểm tra tính toàn vẹn khoảng cách (sau khi triển khai/khóa C1)",
        ],
        "allow_dry_run_capture": True,
        "crowd_count": 2,
        "target_role": "stationary_tracked_person",
        "occluder_role": "full_crossing_helper",
        "variant_hint": "C1_muc_tieu_2p2m_che_khuat_tam_thoi",
        "preparation": [
            "Cần hai người: mục tiêu đã đăng ký đứng yên ở camera–ROI 2,20 m; người hỗ trợ đi trên vạch gần camera hơn để che kín mục tiêu.",
            "Tập trước để thời gian che theo mục tiêu của từng lượt: giai đoạn hiệu chỉnh dùng 0,5 và 1,5 giây; giai đoạn đánh giá dùng lần lượt 0,5; 0,75; 1,0; 1,25; 1,5 giây. Sau đó người hỗ trợ rời hẳn khỏi trục nhìn. Dùng cùng người, đường đi và tốc độ 0,40 m/s.",
            "Robot bị khóa lệnh chuyển động. C1 chưa chạy; chỉ thu RGB-D và lệnh bóng. Không để robot đuổi theo người hỗ trợ.",
            "Bật lưu RGB-D và ghi video ngoài đồng bộ. Giữ mục tiêu tại mốc; xác nhận bằng video hệ thống nhận lại đúng người, không chỉ dựa vào target ID.",
        ],
        "measurement_procedure": [
            "Đưa mục tiêu đến 2,20 m; chờ đủ 10 khung hình ổn định trong ±0,05 m rồi ghi nền 5 giây.",
            "Khi giao diện báo BẮT ĐẦU CHE KHUẤT, người hỗ trợ đi ngang một lần và che kín mục tiêu đúng thời lượng giao diện đang chỉ định rồi đi khỏi khung hình.",
            "Khi giao diện báo KẾT THÚC CHE KHUẤT, mục tiêu tiếp tục đứng yên; chờ nhận diện ổn định 10 khung hình hoặc hết lượt.",
            "Giai đoạn hiệu chỉnh gồm 2 lượt (một lượt che 0,5 giây và một lượt 1,5 giây, hai chiều đối nhau). Giai đoạn đánh giá gồm 5 lượt với thời lượng che lần lượt 0,5; 0,75; 1,0; 1,25; 1,5 giây và chiều được cân bằng 3/2. Lưu mọi lượt có log kỹ thuật; ghi riêng nếu lần che không làm mất dấu hoặc chưa nhận lại được.",
            "Không dùng tiêu chí tự động để loại các lượt không tạo đúng mất dấu: tỷ lệ thành công che khuất/nhận lại là kết quả cần báo cáo, không phải điều kiện xóa dữ liệu.",
        ],
        "ground_truth_method": (
            "Mục tiêu đứng yên tại mốc độc lập; video ngoài đồng bộ xác nhận khoảng cách và khoảng che. "
            "Nếu mục tiêu dịch chuyển, phải có chuỗi khoảng cách độc lập theo thời gian."
        ),
        "operator_steps": [
            "Mục tiêu giữ nguyên ở 2,20 m; người hỗ trợ che kín đúng thời lượng ghi trên giao diện rồi rời khỏi vùng nhìn.",
            "Thực hiện các khối theo giao diện. Robot thật luôn đứng yên; khi mất dấu, lệnh điều khiển giả lập hiện tại phải về 0.",
            "Ghi thời điểm mất dấu, nhận lại, ID trước/sau và xác nhận đúng danh tính từ video ngoài.",
        ],
        "required_events": ["scenario_start", "crossing_start", "crossing_end", "scenario_end"],
        "optional_events": ["target_lost", "target_reacquired", "target_id_changed"],
        "metrics": [
            "tỷ lệ che khuất tạo mất dấu, thời lượng mất dấu và thời gian nhận lại",
            "ID theo dõi trước/sau và xác nhận nhận đúng mục tiêu",
            "ROI rỗng/độ sâu bằng 0 và bước nhảy khoảng cách khi bị che",
            "lệnh lùi bóng sai theo khoảng cách chuẩn và quyết định C1 khi phát lại dữ liệu",
            "thời gian C1 giữ nhầm trong giai đoạn mục tiêu thật đang ổn định",
        ],
    },
    {
        "id": "E1_C1_approach_control",
        "title": "C1-2 — Đối chứng người tiến lại gần (không chặn nhầm)",
        "scenario_action": "c1_approach_control",
        "objective": (
            "Kiểm tra C1 có phân biệt được khoảng cách thật đang giảm hay không. "
            "Đây là đối chứng dương cho thay đổi khoảng cách hợp lệ; robot vẫn khóa và chỉ thu dữ liệu đầu vào."
        ),
        "recommended_repetitions": 5,
        "minimum_repetitions": 2,
        "maximum_repetitions": 5,
        "development_repetitions": 2,
        "duration_s": 20,
        "auto_action_start_s": 5,
        "auto_action_end_s": 9,
        "auto_start_distance_m": 2.0,
        "auto_min_target_presence_ratio": 0.7,
        "auto_queue_enabled": True,
        "motion_inhibited": True,
        "record_shadow_controller": True,
        "c1_capture_only": True,
        "requires_rgbd_recording": True,
        "offline_comparison_profiles": [
            "A: P baseline zero-fallback (chỉ chạy nếu tái tạo đúng mã gốc)",
            "B: P loại mẫu depth 0/không hợp lệ",
            "C: P + cổng C1 kiểm tra tính toàn vẹn khoảng cách (sau khi triển khai/khóa C1)",
        ],
        "allow_dry_run_capture": True,
        "suggested_walking_speed_mps": 0.3,
        "crowd_count": 1,
        "target_role": "approaching_tracked_person",
        "variant_hint": "C1_doi_chung_tien_lai_2p0_den_1p0_tocdo0p3",
        "preparation": [
            "Một người đã đăng ký đứng trước camera–ROI ở khoảng 2,0 m; robot giữ nguyên vị trí và khóa lệnh chuyển động.",
            "Bật lưu RGB-D và chuẩn bị ghi khoảng cách chuẩn độc lập liên tục. C1 chưa chạy trong phiên này; dữ liệu sẽ được phát lại sau.",
        ],
        "measurement_procedure": [
            "Chờ khoảng cách camera–ROI ở 2,0 m ±0,05 m đủ 10 khung hình; ghi nền 5 giây.",
            "Khi giao diện báo BẮT ĐẦU TIẾN LẠI, người đi thẳng khoảng 1,0 m từ 2,0 m tới gần 1,0 m với tốc độ khoảng 0,30 m/s (mất khoảng 3–4 giây), rồi đứng yên.",
            "Khi báo KẾT THÚC TIẾN LẠI, tiếp tục đứng yên đến hết lượt. Robot không di chuyển; lệnh giả lập không phải chuyển động thật.",
            "Giai đoạn hiệu chỉnh có 2 lượt; giai đoạn đánh giá có 5 lượt. Mỗi lượt đi từ 2,0 m đến 1,0 m với cùng đường đi và tốc độ. Không loại lượt vì bộ lọc chặn hoặc cho phép lệnh; đó là kết quả cần phân tích.",
        ],
        "ground_truth_method": (
            "Ghi liên tục khoảng cách camera–cùng điểm ROI trên người bằng nguồn độc lập đồng bộ; "
            "không dùng odometry hay depth của D435 làm khoảng cách chuẩn."
        ),
        "operator_steps": [
            "Người tiến lại gần; robot bị khóa đứng yên.",
            "Kiểm tra khi phát lại dữ liệu rằng khoảng cách chuẩn giảm thật và C1 không giữ sai lệnh theo dõi.",
        ],
        "required_events": ["scenario_start", "approach_start", "approach_end", "scenario_end"],
        "metrics": [
            "tỷ lệ/thời lượng C1 ép lệnh về 0 khi mục tiêu còn được theo dõi, khoảng cách chuẩn đang thay đổi hợp lệ và sai lệch ngoài ±0,03 m",
            "sai số khoảng cách và độ trễ phát hiện biến thiên thật",
            "lệnh P giả lập trước/sau C1; đối chiếu phản ứng đúng phía quanh điểm đặt 1,40 m",
            "tỷ lệ ROI hợp lệ, độ sâu bằng 0 và tính liên tục theo dõi",
        ],
    },
    {
        "id": "E1_crossing_visible",
        "title": "Tham khảo ngoài C1 — Người hỗ trợ cắt ngang, không mất dấu",
        "scenario_action": "crossing_no_loss",
        "objective": (
            "Đo độ ổn định theo dõi khi người mục tiêu đứng yên ở 2,2 m và người hỗ trợ đi cắt ngang "
            "phía trước nhưng không che kín mục tiêu. Robot đứng yên; log thêm lệnh điều khiển giả lập."
        ),
        "recommended_repetitions": 5,
        "minimum_repetitions": 5,
        "maximum_repetitions": 10,
        "auto_queue_enabled": False,
        "duration_s": 35,
        "auto_action_start_s": 5,
        "auto_action_end_s": 12,
        "auto_start_distance_m": 2.2,
        "auto_min_target_presence_ratio": 0.85,
        "auto_require_tracking_continuity_during_crossing": True,
        "motion_inhibited": True,
        "record_shadow_controller": True,
        "allow_dry_run_capture": True,
        "suggested_walking_speed_mps": 0.4,
        "crowd_count": 2,
        "target_role": "stationary_tracked_person",
        "occluder_role": "crossing_helper",
        "variant_hint": "cat_ngang_khong_mat_dau_goc90_tocdo0p4_muctieu2p2_robot_dung",
        "preparation": [
            "Cần hai người: người mục tiêu đã đăng ký đứng yên tại mốc camera–ROI 2,2 m; người hỗ trợ đi cắt ngang phía trước mục tiêu.",
            "Đặt đường người hỗ trợ cách camera khoảng 1,9–2,0 m. Cho đi qua vùng ảnh của mục tiêu nhưng chỉ che một phần/ngắn; không che kín để mất dấu.",
            "Robot bị khóa vận tốc 0 trong cả lượt. Giao diện ghi riêng vận tốc điều khiển giả lập để biết robot sẽ ra lệnh gì nếu được phép chạy; không phát lệnh đó tới MiR.",
            "Giữ nguyên hai người, khoảng cách, hướng 90°, tốc độ khoảng 0,40 m/s và ánh sáng giữa các lượt.",
        ],
        "measurement_procedure": [
            "Người mục tiêu đứng yên tại mốc 2,2 m. Giao diện chỉ bắt đầu khi khoảng cách ổn định 10 khung hình liên tiếp.",
            "Trong 5 giây đầu, cả hai đứng yên để ghi đoạn nền; robot giữ vận tốc 0.",
            "Khi hiện BẮT ĐẦU CẮT NGANG, người hỗ trợ đi ngang trái→phải một lần ở tốc độ khoảng 0,40 m/s, che một phần mục tiêu nhưng không che kín.",
            "Khi hiện KẾT THÚC CẮT NGANG, người hỗ trợ dừng ngoài trục nhìn; người mục tiêu tiếp tục đứng yên đến khi lượt kết thúc.",
            "Lượt chỉ đạt nhánh này nếu mục tiêu không mất dấu và không đổi ID trong pha cắt ngang. Nếu bị mất dấu, giữ log và giao diện tự xếp lượt đo lại; không xóa dữ liệu lỗi.",
        ],
        "ground_truth_method": "Ưu tiên video cạnh bên có mốc sàn đã đo hoặc thiết bị đo ngoài đồng bộ, ghi khoảng cách tâm camera–cùng điểm ROI trên người. Không dùng số đo D435 làm khoảng cách chuẩn.",
        "operator_steps": [
            "Mỗi lượt người hỗ trợ đi ngang đúng một lần; người mục tiêu luôn đứng yên ở 2,2 m.",
            "Thu tối thiểu 5 lượt đạt điều kiện không mất dấu; nếu đủ thời gian tăng lên 10. Đổi chiều trái/phải thành biến thể riêng.",
            "Ghi target_present, is_tracking, target_id, ROI hợp lệ, khoảng cách mục tiêu và lệnh vận tốc giả lập; lệnh thực phát tới robot phải luôn bằng 0.",
        ],
        "required_events": ["scenario_start", "crossing_start", "crossing_end", "scenario_end"],
        "metrics": ["tỷ lệ khung hình còn mục tiêu", "số lần mất dấu/nhận lại", "đổi ID theo dõi", "dao động ROI và khoảng cách", "lệnh điều khiển giả lập khi mục tiêu còn được theo dõi"],
    },
    {
        "id": "E1_crossing_occlusion",
        "title": "Tham khảo ngoài C1 — Che kín rồi nhận lại mục tiêu",
        "scenario_action": "crossing_occlusion",
        "objective": (
            "Đo thời gian mất dấu và nhận lại người mục tiêu, thay đổi ID theo dõi, "
            "độ sâu ROI và lệnh điều khiển giả lập khi người hỗ trợ che kín mục tiêu tạm thời."
        ),
        "recommended_repetitions": 5,
        "minimum_repetitions": 5,
        "maximum_repetitions": 10,
        "auto_queue_enabled": False,
        "duration_s": 35,
        "auto_action_start_s": 5,
        "auto_action_end_s": 12,
        "auto_start_distance_m": 2.2,
        "auto_min_target_presence_ratio": 0.25,
        "auto_min_target_loss_duration_s": 0.5,
        "auto_max_target_loss_duration_s": 1.5,
        "auto_min_target_reacquisition_stability_frames": AUTO_START_STABILITY_FRAMES,
        "motion_inhibited": True,
        "record_shadow_controller": True,
        "allow_dry_run_capture": True,
        "suggested_walking_speed_mps": 0.4,
        "crowd_count": 2,
        "target_role": "stationary_tracked_person",
        "occluder_role": "crossing_helper",
        "variant_hint": "cat_che_khuat_mat_dau_goc90_tocdo0p4_d2p2_robot_dung",
        "preparation": [
            "Cần hai người: người mục tiêu đã đăng ký/đang theo dõi đứng yên tại mốc camera–ROI 2,2 m; người hỗ trợ đi cắt ngang.",
            "Đặt vạch người hỗ trợ cách tâm camera khoảng 2,0 m (xấp xỉ 1,1 m trước mép robot), để người này che mục tiêu khoảng 0,5–1,5 giây rồi rời khỏi trục nhìn.",
            "Robot bị khóa lệnh vận tốc 0; lệnh điều hướng mới bị chặn và mục tiêu đang chạy bị hủy. Lệnh điều khiển giả lập vẫn được ghi; không phát lệnh tới MiR.",
            "Giữ cùng vị trí, hướng cắt, ánh sáng và hai người trong các lượt ghép cặp. Không đổi người mục tiêu giữa lượt.",
        ],
        "measurement_procedure": [
            "Người mục tiêu đứng yên tại mốc 2,2 m. Chờ giao diện xác nhận khoảng cách ổn định 10 khung hình liên tiếp; robot giữ vận tốc 0 và không chạy goal trong suốt lượt.",
            "Trong 5 giây đầu, cả hai đứng yên để ghi đoạn nền.",
            "Khi hiện BẮT ĐẦU CẮT NGANG, người hỗ trợ đi qua vạch một lần để che kín mục tiêu khoảng 0,5–1,5 giây; người mục tiêu không di chuyển.",
            "Khi hiện KẾT THÚC CẮT NGANG, người hỗ trợ đứng hẳn ngoài trục camera–mục tiêu; người mục tiêu tiếp tục đứng yên đến hết lượt để ghi nhận quá trình nhận lại.",
            "Giao diện tự ghi thời điểm mục tiêu biến mất/xuất hiện lại và thay đổi target_id. Lượt đạt điều kiện khi mục tiêu mất dấu liên tục ít nhất 0,5 giây, xuất hiện lại và duy trì nhận diện hợp lệ 10 khung hình liên tiếp.",
            "Khi mất dấu, bộ điều khiển phát lệnh dừng và giữ dừng; không chuyển sang theo người hỗ trợ. Sau khi nhận lại đủ 10 khung hình ổn định, bộ điều khiển mới tính lại lệnh giả lập. Robot vẫn bị khóa đứng yên đến hết lượt.",
        ],
        "ground_truth_method": "Người mục tiêu đứng yên; dùng video ngoài đồng bộ để xác nhận khoảng cách camera–mục tiêu không đổi và đánh dấu khoảng bị che. Không coi số đo D435 hoặc odometry là khoảng cách chuẩn.",
        "operator_steps": [
            "Mỗi lượt có đúng một lần che khuất; người hỗ trợ đi cùng hướng và tốc độ khoảng 0,40 m/s.",
            "Thu tối thiểu 5 lượt có che khuất ≥0,5 giây, nhận lại và ổn định 10 khung hình; nếu đủ thời gian tăng lên 10.",
            "Ghi thời gian mất dấu, thời gian nhận lại, target_id trước/sau, ROI rỗng, khoảng cách chuẩn, lệnh giả lập và can thiệp an toàn. Người ghi chép xác nhận bằng hình ảnh ngoài xem hệ thống nhận lại đúng người hay nhầm người hỗ trợ; target_present và ID đơn lẻ không chứng minh danh tính thật. Robot đứng yên; không diễn giải lệnh giả lập thành chuyển động thực.",
        ],
        "required_events": ["scenario_start", "crossing_start", "crossing_end", "scenario_end"],
        "optional_events": ["target_lost", "target_reacquired", "target_id_changed"],
        "metrics": ["thời lượng mất dấu", "thời gian xuất hiện lại", "target_id trước/sau", "xác nhận ngoài việc nhận đúng người hay nhầm người hỗ trợ", "ROI rỗng/độ sâu lỗi", "khoảng cách camera–mục tiêu khi bị che"],
    },
    {
        "id": "E1_approach",
        "title": "E1-2 — Người tiến lại gần",
        "objective": "Đối chứng với tình huống khoảng cách camera–người thực sự giảm, để phân biệt lệnh lùi phù hợp và không phù hợp.",
        "recommended_repetitions": 5,
        "minimum_repetitions": 5,
        "maximum_repetitions": 10,
        "auto_queue_enabled": False,
        "duration_s": 40,
        "auto_action_start_s": 5,
        "auto_action_end_s": 30,
        "auto_start_distance_m": 2.0,
        "auto_min_target_presence_ratio": 0.5,
        "suggested_walking_speed_mps": 0.3,
        "variant_hint": "tienlai_01_tocdo0p3_tu2p0den1p0",
        "preparation": [
            "Đưa robot về vị trí/hướng đã đánh dấu; người bắt đầu tại mốc camera–ROI khoảng 2,0 m, tốc độ đặt ra 0,30 m/s.",
            "Dọn trống đường robot chạy và giữ nút dừng khẩn cấp trong tầm với.",
            "Giữ cùng người, đường đi, tốc độ và điều kiện môi trường trong cả năm lượt.",
        ],
        "measurement_procedure": [
            "Đứng tại điểm xuất phát đã đo; giữ 5 giây trước khi bắt đầu.",
            "Khi hiện dòng BẮT ĐẦU TIẾN LẠI, đi thẳng về phía robot trong 25 giây với tốc độ đã thống nhất.",
            "Khi hiện dòng KẾT THÚC TIẾN LẠI, dừng tại điểm cuối và giữ 10 giây; giao diện tự kết thúc lượt.",
            "Bộ điều khiển sẽ theo dõi trong pha hành động; chỉ can thiệp để dừng an toàn khi cần.",
        ],
        "ground_truth_method": "Ghi độc lập khoảng cách theo thời gian từ camera tới cùng vùng đo trên người hoặc dùng video ngoài có thước/mốc sàn; nêu rõ quy chiếu và sai số.",
        "operator_steps": [
            "Cho người tiến gần robot theo cùng đường và tốc độ đã quy định.",
            "Lặp tối thiểu 5 lần; thay đổi khoảng cách hoặc tốc độ theo các điều kiện đã định trước.",
            "So sánh thời gian hồi phục và lệnh lùi với tình huống cắt ngang ghép cặp.",
        ],
        "required_events": ["scenario_start", "approach_start", "approach_end", "scenario_end"],
        "metrics": ["lệnh lùi phù hợp/không phù hợp", "thời gian hồi phục", "sai số khoảng cách", "giữ sai", "độ giật vận tốc"],
    },
    {
        "id": "E1_robot_approach",
        "title": "E1-3 — Người đứng yên ở 2,2 m, MiR tự tiến tới",
        "objective": (
            "Đo khả năng MiR tự phát lệnh tiến khi người đứng yên xa hơn điểm đặt; "
            "tách chuyển động của robot khỏi chuyển động của người."
        ),
        "recommended_repetitions": 5,
        "minimum_repetitions": 5,
        "maximum_repetitions": 10,
        "auto_queue_enabled": False,
        "duration_s": 20,
        "auto_action_start_s": 5,
        "auto_start_distance_m": 2.2,
        "auto_goal_distance_m": 1.4,
        "auto_goal_tolerance_m": 0.03,
        "auto_goal_stability_frames": 10,
        "auto_min_target_presence_ratio": 0.5,
        "suggested_walking_speed_mps": 0.0,
        "variant_hint": "nguoi_dungyen_2p2m_mir_tien_1p4m_20261121",
        "preparation": [
            "Đánh dấu vị trí người đứng yên sao cho khoảng cách tâm camera–ROI ban đầu là 2,2 m (mốc 1,4 m cộng 0,8 m).",
            "Để trống lối tiến của robot, dùng cùng người và giữ nút dừng khẩn cấp trong tầm với.",
            "Bật THEO DÕI; robot bị khóa trong 5 giây đầu, sau đó tự tiến khi giao diện báo. Người không di chuyển.",
        ],
        "measurement_procedure": [
            "Đứng yên tại mốc camera–ROI 2,2 m, hướng mặt về robot; khi khoảng cách ổn định 10 khung hình liên tiếp, giao diện tự đếm ngược.",
            "Giữ nguyên vị trí trong 5 giây nền khi robot đang bị khóa.",
            "Khi giao diện báo MiR TỰ TIẾN TỚI, tiếp tục đứng yên; không lùi, không tiến theo robot.",
            "Robot tự chạy đến khi khoảng cách camera–ROI ổn định ở 1,4 m ±0,03 m trong 10 khung hình; giao diện khóa robot và ghi pha giữ đến hết lượt.",
            "Giữa hai lượt, log lượt đo tạm dừng. Bật điều khiển thường nếu cần đưa robot về vị trí xuất phát; khi khoảng cách đạt 2,2 m ±0,05 m đủ 10 khung hình, robot tự khóa và lượt kế tự đếm ngược.",
            "Khi robot về vùng camera–ROI 2,2 m ±0,05 m đủ 10 khung hình liên tiếp, robot tự khóa và lượt kế tiếp tự đếm ngược để bắt đầu ghi.",
            "Nếu robot tiến quá gần, có vật cản hoặc chuyển động bất thường, dừng an toàn ngay; không cố hoàn thành lượt.",
        ],
        "ground_truth_method": (
            "Đánh dấu vị trí cố định của người và mốc camera–ROI ban đầu 2,2 m; dùng odometry để ghi quãng tiến của MiR. "
            "Nếu có, đối chiếu bằng video ngoài/thước laser độc lập; không dùng riêng số D435 làm khoảng cách chuẩn."
        ),
        "operator_steps": [
            "Người đứng yên suốt lượt; robot tự tiến từ mốc 2,2 m tới 1,4 m.",
            "Giữa các lượt, log lượt đo đang tạm dừng. Dùng nút tạm dừng đo để bật lại điều khiển robot bình thường trong lúc đưa robot về vị trí xuất phát.",
            "Giữ người đứng yên khi camera đo trong vùng 2,2 m ±0,05 m; sau 10 khung hình liên tiếp, giao diện tự chuẩn bị lượt tiếp theo.",
            "Lặp 5 lần cùng vị trí và hướng; không để người lùi ra xa trong lúc robot tiến.",
            "So sánh tỷ lệ lệnh tiến, quãng đường odometry, thời gian tới điểm đặt và sai lệch cuối.",
        ],
        "required_events": ["scenario_start", "robot_approach_start", "robot_approach_end", "scenario_end"],
        "metrics": [
            "tỷ lệ lệnh tiến khi người đứng yên ngoài điểm đặt",
            "quãng đường MiR tiến theo odometry",
            "thời gian MiR tới điểm đặt 1,4 m",
            "độ lệch khoảng cách cuối so với 1,4 m",
            "độ giật vận tốc",
        ],
    },
]


def get_scenario(scenario_id):
    for scenario in SCENARIOS:
        if scenario["id"] == scenario_id:
            return deepcopy(scenario)
    return deepcopy(SCENARIOS[0])


def get_configuration(config_id):
    for config in CONFIGURATIONS:
        if config["id"] == config_id:
            return deepcopy(config)
    return deepcopy(CONFIGURATIONS[0])

