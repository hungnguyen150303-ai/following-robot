# Kế hoạch đo E1 — C1 kiểm tra tính toàn vẹn khoảng cách

## Mục tiêu và phạm vi

Đóng góp chính dự kiến là **C1: cổng kiểm tra tính hợp lệ/toàn vẹn của chuỗi khoảng cách trước khi bộ điều khiển P dùng nó**. Phép thử cốt lõi hỏi hai điều:

1. Khi người khác cắt ngang làm ROI/depth bị nhiễu, C1 có giảm lệnh lùi sai trong khi người mục tiêu thực tế vẫn ở xa hơn điểm đặt không?
2. Khi khoảng cách thật sự giảm vì mục tiêu tiến lại gần, C1 có cho phép phản ứng theo dõi hợp lệ hay giữ robot quá lâu không?

Giai đoạn hiện tại **chỉ thu RGB-D và lệnh P giả lập; C1 chưa được triển khai/chạy và MiR bị khóa đứng yên**. Vì thế các lượt này tạo dữ liệu đầu vào để replay ghép cặp, chưa phải kết quả chứng minh C1 hoặc chuyển động MiR.

## Thiết kế chính

Giữ cố định camera/robot, người mục tiêu, ánh sáng, vạch đi, tốc độ và thông số perception/controller. Chỉ thay biến được nêu trong từng nhánh. Mỗi bản ghi RGB-D sau đó được phát lại qua cùng ba profile: (A) baseline P có zero-fallback **chỉ khi tái tạo được đúng mã baseline**, (B) P loại mẫu 0/không hợp lệ, (C) P+C1. Cùng một lượt được dùng cho cả ba profile để so sánh ghép cặp.

| Nhánh | Bố trí và hành động | Số lần tối thiểu | Vai trò |
|---|---|---:|---|
| C1-1A: cắt ngang không mất dấu | Mục tiêu đứng yên ở camera–ROI 2,20 m. Người hỗ trợ đi phía trước, che một phần, góc 90°, khoảng 0,40 m/s. Development: 1 lượt mỗi chiều. Evaluation: 5 lượt, chiều luân phiên trái→phải/phải→trái (3/2). | 7 | Kiểm tra nhiễu depth/range khi tracker còn giữ mục tiêu; có 5 bản ghi held-out. |
| C1-1B: cắt ngang mất dấu tạm thời | Cùng mốc 2,20 m. Người hỗ trợ che kín rồi rời vùng nhìn. Development: 2 lượt, mục tiêu 0,5 và 1,5 giây. Evaluation: 5 lượt mục tiêu 0,5; 0,75; 1,0; 1,25; 1,5 giây, chiều cân bằng 3/2. | 7 | Tách trường hợp target loss/reacquisition khỏi lỗi range khi vẫn còn target; có 5 bản ghi held-out. |
| C1-2: mục tiêu tiến lại gần | Một người đi thẳng từ khoảng 2,00 m tới 1,00 m ở khoảng 0,30 m/s; robot đứng yên. Development: 2 lượt. Evaluation: 5 lượt. | 7 | Đối chứng dương: khoảng cách thay đổi là thật, C1 không nên chặn nhầm. |

Thiết kế đã chọn thu **21 bản ghi nguồn**: 6 development (2 cho mỗi nhánh) và 15 evaluation giữ riêng (5 cho mỗi nhánh). Các lượt evaluation được tổ chức thành 5 block; mỗi block có một lượt của từng nhánh. Robot bị khóa trong mọi lượt. Mỗi bản ghi evaluation sẽ được replay qua 3 profile, tạo tối đa **45 đầu ra ghép cặp** (15 × 3); đây là số đầu ra dự kiến, chưa phải kết quả đã tạo. Hàng đợi cân bằng chiều cắt ngang. Không dùng 6 bản ghi development để báo kết quả test cuối. Mỗi lượt crossing dài 25 giây; mỗi lượt tiến lại gần dài 20 giây. Năm bản ghi mỗi nhánh là quy mô đánh giá ban đầu, không mặc nhiên bảo đảm đủ lực thống kê; báo phân bố/khoảng tin cậy và tăng số bản ghi nếu sự kiện lỗi ít hoặc kết quả dao động lớn.

Hai nhánh crossing cần **hai người**. Nếu hôm đo chưa có người hỗ trợ, bỏ qua hai nhánh đó và chỉ thu nhánh tiến lại gần; không thay bằng việc cho chính người mục tiêu đi ngang vì khi đó khoảng cách thật cũng đổi. Bố trí thời gian khoảng 30–45 phút cho toàn hàng đợi, chưa tính lắp vạch và kiểm tra đồng bộ.

### Quy tắc giữ dữ liệu

- Giữ mọi lượt có log kỹ thuật, kể cả lượt không tạo ra lỗi depth, không che khuất đủ, mất dấu ngoài dự kiến hoặc không nhận lại được. Gắn nhãn kết quả sự kiện sau khi xem video/RGB-D; không âm thầm xóa lượt bất lợi.
- Với C1-1A, ghi riêng mục tiêu có bị mất dấu/đổi ID ngoài ý muốn hay không. Với C1-1B, ghi riêng việc che có tạo mất dấu và nhận lại thành công hay không.
- Chỉ đánh dấu **không dùng được về kỹ thuật** khi mất tệp RGB-D/CSV, lệch đồng hồ không khôi phục được, camera/ROS dừng lâu hoặc robot có lệnh khác 0 trong nhánh khóa. Giữ thư mục gốc và ghi lý do loại trong bảng phân tích.
- Không tự loại lượt evaluation khi che khuất không tạo mất dấu như mong muốn. Giữ bản ghi, gắn nhãn sự kiện thực tế và nếu cần thêm lượt thì ghi rõ lý do/số lượt bổ sung; không thay âm thầm kết quả bất lợi.

## Chuẩn bị trước buổi đo

1. Chốt định nghĩa C1 trong paper/code trước khi so sánh: đầu vào kiểm tra, ngưỡng/logic, thời gian giữ trạng thái, điều kiện cho phép lại, cùng cách xử lý depth 0/ROI rỗng. Không chọn ngưỡng trên tập test cuối.
2. Chọn trước nguồn ground truth độc lập và cách đồng bộ thời gian. Tối thiểu cần video ngoài đã hiệu chuẩn hoặc thiết bị đo độc lập có thể xác định khoảng cách camera–cùng điểm ROI theo thời gian. Nếu chỉ có các mốc thước rời rạc, không được tính thời lượng lệnh lùi sai hay độ trễ theo frame.
3. Đánh dấu mốc mục tiêu 2,20 m và đường đi người hỗ trợ gần camera hơn mục tiêu (khoảng 1,9–2,0 m). Tập tốc độ 0,40 m/s; với nhánh che kín, căn thời lượng che 0,5–1,5 giây.
4. Bật lưu RGB-D **trước khi mở ứng dụng** bằng `EXPERIMENT_RECORD_RGBD=1`. Tab thực nghiệm phải báo đang ghi RGB-D. Nếu chưa báo, không bắt đầu hàng đợi C1.
5. Đồng bộ video ngoài với timestamp camera/log bằng một dấu mốc dễ nhìn/nghe ở đầu mỗi phiên; ghi độ lệch đồng hồ và sai số đồng bộ. Một tệp RealSense có thể bao phủ nhiều lượt; giữ nguyên tệp và metadata của từng lượt.
6. Kiểm tra trước khi thu: MiR ở vùng an toàn, nút dừng khẩn cấp trong tầm với, điều khiển bị khóa, `commands.csv` chỉ có lệnh thật bằng 0. Không dùng lệnh shadow làm bằng chứng robot đã di chuyển.

**Điều kiện dừng phân tích:** nếu chưa có ground truth liên tục/đồng bộ, vẫn có thể thống kê tỷ lệ ROI rỗng, zero-depth, range nhảy và tracking loss; chưa được kết luận C1 giảm lệnh lùi sai theo khoảng cách thật.

## Dữ liệu phải lưu

Giữ nguyên thư mục run và tệp phiên RGB-D, tối thiểu gồm:

- `frames.csv`: timestamp/frame, trạng thái target/ID/ROI, depth hợp lệ và zero, range thô/lọc, trạng thái gate, lệnh P giả lập và pha kịch bản.
- `commands.csv`: mọi lệnh được tạo và cờ có publish hay không; nhánh hiện tại phải khóa lệnh MiR.
- `odometry.csv`, `events.jsonl`, `run_metadata.json`, `summary.json`.
- Tệp RealSense RGB-D gốc trong `experiments/logs/_sessions/` và video ngoài đồng bộ.
- CSV ground truth do phân tích/đo độc lập tạo: timestamp, khoảng cách camera–điểm ROI mục tiêu, nguồn đo, độ không chắc chắn, trạng thái mục tiêu/che khuất và ghi chú đồng bộ.
- Bảng nhãn theo từng lượt: loại nhánh thực tế, hướng/tốc độ thực đo nếu có, có tạo zero/ROI rỗng/range jump không, mất dấu/nhận lại/đổi ID, lý do loại kỹ thuật nếu có.

Khi phân tích lượt khóa robot, dùng lệnh **shadow** ở `controller_rate_limited_linear_x_mps`; `command_linear_x_mps` là lệnh thật đang bị khóa về 0. `analyze_run.py` tự chọn shadow khi `controller_shadow_active=true`. Khi phát lại một cột lệnh riêng cho từng profile, chọn cột đó bằng `--false-command-field` để không nhầm zero do khóa robot thành false hold.

Không dùng D435 vừa làm đầu vào vừa làm ground truth. Khoảng cách đích 1,40 m được đo từ tâm quang học camera tới điểm ROI người; mốc 2,20 m dùng để bố trí thí nghiệm. Sai số ±0,03 m của bộ điều khiển không phải sai số đo độc lập của camera.

## Phân tích và chỉ số paper

### Chỉ số chính

- **False-backward duration/rate:** thời lượng hoặc số đợt có lệnh tuyến tính lùi dưới −0,05 m/s trong khi ground truth lớn hơn 1,43 m. Báo theo từng lượt, rồi tổng hợp trung vị/trung bình và khoảng tin cậy; đơn vị độc lập là lượt, không phải mỗi frame coi như mẫu độc lập.
- **False reverse reduction:** so sánh A/B/C trên đúng cùng bản ghi, kèm chênh lệch ghép cặp và khoảng tin cậy.
- **Valid-approach false hold:** tỷ lệ/thời lượng C1 ép lệnh về 0 trong khi mục tiêu vẫn được theo dõi, range ground truth thay đổi liên tục/hợp lệ, và sai lệch thật nằm ngoài vùng chết ±0,03 m. So sánh với lệnh P ở cùng frame; loại trừ mất dấu, dữ liệu depth lỗi được gán nhãn và đoạn nằm trong vùng chết. Như vậy không tính nhầm lệnh giữ đúng quanh 1,40 m là lỗi.

### Chỉ số phụ

- Tỷ lệ ROI rỗng, zero-depth, range jump; sai số range so với ground truth (MAE/RMSE và phân vị).
- Mất dấu, đổi ID, tỷ lệ nhận lại đúng người và thời gian hồi phục; báo riêng nhánh C1-1A/C1-1B.
- Số quyết định C1 đúng/sai theo nhãn sự kiện độc lập, false hold/false allow, thời gian C1 giữ trạng thái.
- FPS và độ trễ frame-to-command; lệnh shadow là đầu ra bộ điều khiển, không phải quãng đường MiR.

Dùng các lượt được gắn nhãn development để chốt ngưỡng C1, quy tắc gán nhãn range jump và cách gộp một chuỗi lệnh thành một “đợt”. Sau đó khóa mã/tham số trước khi mở các lượt `held_out_evaluation`; không dùng lại test để chỉnh ngưỡng. Nếu phải chỉnh sau khi xem test, cần thu một tập test mới.

## Khi nào cần thử robot thật hoặc mô phỏng?

Thu RGB-D với MiR khóa đủ để kiểm tra **đầu vào C1 và lệnh phản thực** khi replay; không chứng minh MiR phản ứng cơ học an toàn hơn. Nếu paper tuyên bố hiệu quả điều khiển trên robot thật, sau khi C1 được cài đặt và qua kiểm tra replay mới bổ sung lượt live tốc độ thấp, có giám sát an toàn, dùng các kịch bản đã định trước. Mô phỏng là phần bổ sung cho độ lặp lại/động lực học bộ điều khiển; không thay thế dữ liệu D435 thật để đánh giá nhiễu depth.

Kịch bản che khuất hai người là phần chính vì giúp phân biệt **cắt ngang nhưng vẫn giữ dấu** với **cắt ngang làm mất dấu**. Kịch bản tiến lại gần là đối chứng bắt buộc để tránh một C1 “an toàn” bằng cách luôn chặn lệnh.
