# MiR Robot Person Follower

**Hệ thống theo dõi và điều khiển robot MiR bằng AI và cử chỉ tay**

## 🚀 Khởi động nhanh

### Cách 1: Script tương tác (Khuyến nghị)
```bash
chmod +x mir.sh
./mir.sh
```

Menu tương tác sẽ hiện ra với các lựa chọn:
- 🚀 Start application
- 🛑 Stop application  
- 📊 View logs (interactive)
- 🔍 Show status
- 🐚 Enter container shell
- 🔄 Restart
- 🗑️ Clean & rebuild

### Cách 2: Chạy trực tiếp
```bash
chmod +x run_mir.sh
./run_mir.sh
```

Chỉ cần 1 lệnh và hệ thống sẽ tự động:
- Build Docker image (lần đầu ~20-40 phút)
- Cấu hình GPU, camera, ROS
- Khởi động ứng dụng với GUI

### 📊 Xem logs realtime (Terminal mới)

**Cách 1: Script tương tác**
```bash
./logs.sh
```

Chọn chế độ xem:
1. Tất cả logs
2. Chỉ Gestures
3. Chỉ Tracking
4. Chỉ Navigation
5. Chỉ Errors/Warnings
6. Logs khởi động
7. Tùy chỉnh pattern
8. Vào shell container

**Cách 2: Lệnh trực tiếp**
```bash
# Xem tất cả logs
docker-compose logs -f

# Hoặc xem logs của container chính
docker logs -f mir_follower_app

# Lọc logs theo từ khóa
docker-compose logs -f | grep "GESTURE\|TARGET\|NAVIGATION"
```

**Logs sẽ hiển thị:**
- ✅ **Khởi động**: Camera device info, GPU info, models loading
- 🔍 **Detection**: Person detected, face recognition, gesture detected
- 🎯 **Tracking**: Target found/lost, following distance
- 🤖 **Robot**: Navigation goals, velocity commands, rotation status
- ⚠️ **Warnings**: Camera errors, ROS connection issues

### 🐚 Vào container để debug

```bash
# Cách 1: Dùng script
./mir.sh
# Chọn option 5: Enter container shell

# Cách 2: Lệnh trực tiếp
docker exec -it mir_follower_app bash

# Trong container bạn có thể:
rostopic list                    # Xem ROS topics
rostopic echo /cmd_vel           # Xem velocity commands
rosnode list                     # Xem ROS nodes
python3 -c "import torch; print(torch.cuda.is_available())"  # Test GPU
```

## 📋 Yêu cầu hệ thống

- **Ubuntu 20.04** (hoặc tương thích)
- **NVIDIA GPU** với CUDA support
- **Docker** + **Docker Compose** + **nvidia-docker2**
- **Intel RealSense Camera** (D435, D455, hoặc tương đương)
- **MiR Robot** đã kết nối mạng

### Cài đặt dependencies

```bash
# Docker
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER

# Docker Compose (nếu chưa có)
# Cách 1: Docker Compose Plugin (khuyến nghị)
sudo apt-get update
sudo apt-get install docker-compose-plugin

# Cách 2: Docker Compose standalone (nếu cách 1 không work)
sudo apt-get install docker-compose

# Kiểm tra Docker Compose đã cài chưa
docker compose version || docker-compose version

# NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
    sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker

# Thêm user vào video group (cho camera)
sudo usermod -aG video $USER

# Logout và login lại để áp dụng thay đổi
```

**Lưu ý:** Scripts tự động phát hiện và dùng `docker compose` hoặc `docker-compose` tùy phiên bản.

## ⚙️ Cấu hình Robot IP

Mặc định robot IP là `192.168.12.20`. Để thay đổi:

1. Chạy `./run_mir.sh` và nhập IP khi hộp thoại xuất hiện; hoặc
2. đặt biến môi trường `export ROBOT_IP=192.168.12.20` trước khi chạy Docker Compose.

## 🎮 Cách sử dụng

### 1. Đăng ký khuôn mặt (lần đầu)

Sau khi GUI xuất hiện:

1. **Bước 1**: Nhìn thẳng camera → nhấn **Register Face Step 1**
2. **Bước 2**: Quay mặt sang trái → nhấn **Register Face Step 2**  
3. **Bước 3**: Quay mặt sang phải → nhấn **Register Face Step 3**

✅ Xong! Hệ thống sẽ nhận diện bạn tự động.

### 2. Điều khiển bằng cử chỉ tay

Đưa tay lên trước camera với các cử chỉ sau:

| Cử chỉ | Số ngón tay | Chức năng |
|--------|-------------|-----------|
| ✌️ **2 ngón** | Ngón trỏ + ngón giữa | **BẮT ĐẦU THEO DÕI** |
| 🤟 **3 ngón** | Ngón cái + trỏ + giữa | **DỪNG LẠI** |
| ✋ **5 ngón** | Tất cả ngón tay | **ĐỖ XE** (về vị trí đã lưu) |

**Lưu ý quan trọng**: 
- Giữ cử chỉ **ổn định 1-2 giây** để hệ thống xác nhận
- Cử chỉ 2 ngón: Robot sẽ theo dõi bạn ở khoảng cách 1.4m
- Cử chỉ 3 ngón: Robot dừng ngay lập tức
- Cử chỉ 5 ngón: Robot tự động quay về vị trí đã lưu (khi dừng < 2m)

### 3. Điều hướng thủ công (khi STOP)

Khi robot đang DỪNG (sau cử chỉ 3 ngón):
- Click chuột trái vào **bản đồ** để chọn điểm đích
- Robot sẽ tự động di chuyển đến điểm đó

### 4. Xoay 360° tìm kiếm

Nhấn nút **Start Rotation** để robot xoay 360° tìm người bị mất.

## 📊 Giao diện GUI

**Màn hình chính gồm:**
- **Video stream**: Hiển thị camera với bounding boxes
- **Thông tin tracking**: ID, khoảng cách, trạng thái
- **Map 2D**: Bản đồ robot và vị trí người
- **Nút điều khiển**: Register Face, Start Rotation, Reset Target

**Status colors:**
- 🟢 **Xanh lá**: Đang theo dõi (TRACKING)
- 🔴 **Đỏ**: Dừng (STOP)
- 🔵 **Xanh dương**: Đang di chuyển đến mục tiêu
- 🟡 **Vàng**: Đang xoay tìm kiếm

## 🛠️ Troubleshooting

### Camera không hoạt động
```bash
# Kiểm tra camera
lsusb | grep Intel

# Nếu không thấy, thử cắm lại USB 3.0
# Kiểm tra quyền
ls -l /dev/bus/usb/*/*
```

### GPU không được nhận
```bash
# Kiểm tra NVIDIA driver
nvidia-smi

# Kiểm tra Docker có GPU runtime
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu20.04 nvidia-smi
```

### ROS Master không kết nối
```bash
# Ping robot
ping 192.168.12.20

# Kiểm tra ROS topics
export ROS_MASTER_URI=http://192.168.12.20:11311
rostopic list

# Nếu không kết nối được:
# - Kiểm tra robot đã bật chưa
# - Kiểm tra IP trong app/config.py
# - Kiểm tra firewall
```

### Không có âm thanh
```bash
# Khởi động lại PulseAudio
pulseaudio --kill
pulseaudio --start

# Kiểm tra output devices
pactl list sinks short
```

### Build Docker lỗi
```bash
# Xóa cache và build lại
docker compose down -v
docker system prune -a -f
./run_mir.sh
```

**Lỗi thường gặp & giải pháp:**

1. **Python packages installation failed**
   - Đã fix: Cài packages theo layers để tránh conflicts
   - PyQt5 dùng system packages thay vì pip

2. **RealSense build failed (exit code: 2)**
   - Đã fix: Dùng pre-built packages từ Intel repo

3. **Out of memory during build**
   ```bash
   # Tăng Docker memory lên 8GB+
   # Docker Desktop → Settings → Resources → Memory
   ```

4. **Network timeout**
   ```bash
   # Thử lại build
   ./run_mir.sh
   ```

### Lỗi: docker-compose command not found
```bash
# Cài Docker Compose Plugin
sudo apt-get update
sudo apt-get install docker-compose-plugin

# Hoặc cài standalone version
sudo apt-get install docker-compose

# Kiểm tra
docker compose version
```

**Lưu ý:** Scripts tự động detect và dùng `docker compose` (v2) hoặc `docker-compose` (v1).

## 📁 Cấu trúc project

```
robot_mir/
├── app/                    # Python modules
│   ├── config.py          # Cấu hình hệ thống
│   ├── camera.py          # RealSense camera handler
│   ├── detection.py       # YOLO + Face + Gesture detection
│   ├── control.py         # Robot control logic
│   └── __init__.py
├── src/
│   └── main.py            # Main application với GUI
├── assets/                # Audio files (optional)
├── Dockerfile             # Docker image definition
├── docker-compose.yml     # Docker orchestration
├── requirements.txt       # Python dependencies
├── run_mir.sh            # Main startup script
└── README.md             # This file
```

## 🔧 Tính năng nâng cao

### Thêm âm thanh (optional)

Tạo các file audio để robot thông báo trạng thái:

```bash
# Cài đặt pyttsx3 để tạo audio
pip3 install pyttsx3

# Tạo file âm thanh
python3 << 'EOF'
import pyttsx3, os
engine = pyttsx3.init()
engine.setProperty('rate', 150)
os.makedirs('assets', exist_ok=True)

messages = {
    'start': 'Bắt đầu theo dõi',
    'stop': 'Dừng lại',
    'lost': 'Đã mất mục tiêu',
    'found': 'Đã tìm thấy mục tiêu',
    'parking': 'Đang quay về vị trí đỗ xe'
}

for name, text in messages.items():
    engine.save_to_file(text, f'assets/{name}.mp3')
engine.runAndWait()
EOF
```

### Tùy chỉnh tham số

Chỉnh sửa `app/config.py` để thay đổi:
- **DESIRED_DISTANCE**: Khoảng cách theo dõi (mặc định 1.4m)
- **MAX_SPEED**: Tốc độ tối đa robot (mặc định 1.8 m/s)
- **GESTURE_CONFIRM_FRAMES**: Số frame xác nhận cử chỉ
- **FACE_DISTANCE_THRESHOLD**: Ngưỡng nhận diện khuôn mặt

### Xem logs

```bash
# Xem logs realtime
docker-compose logs -f

# Xem logs cụ thể
docker logs mir_follower_app

# Lọc logs theo level
docker-compose logs -f | grep "ERROR\|WARN"

# Lọc logs theo chức năng
docker-compose logs -f | grep "GESTURE"      # Cử chỉ
docker-compose logs -f | grep "TARGET"       # Tracking
docker-compose logs -f | grep "NAVIGATION"   # Di chuyển
docker-compose logs -f | grep "CAMERA"       # Camera
docker-compose logs -f | grep "GPU"          # GPU info
```

**Các log patterns quan trọng:**
- 📷 `CAMERA`: Thông tin camera device, frames
- 🔍 `YOLO MODEL LOADED`: YOLO đã load xong
- 👤 `FACE RECOGNIZER`: InsightFace status
- ✋ `GESTURE RECOGNIZER`: MediaPipe hands
- 🤖 `ROBOT CONTROLLER`: Robot commands
- 🎯 `NAVIGATION GOAL`: Đích di chuyển
- ⚠️ `TARGET LOST`: Mất người theo dõi
- ✅ `TARGET FOUND`: Tìm thấy lại
- 🛑 `GESTURE CONFIRMED: STOP`: Cử chỉ dừng
- ✌️ `GESTURE CONFIRMED: FOLLOW`: Cử chỉ theo dõi
- ✋ `GESTURE CONFIRMED: PARKING`: Cử chỉ đỗ xe

## 🎯 Workflow điển hình

**Setup ban đầu (1 lần):**
```bash
# 1. Cài đặt dependencies (nếu chưa có)
curl -fsSL https://get.docker.com | sh
sudo apt install nvidia-docker2
sudo usermod -aG docker $USER
# Logout và login lại

# 2. Clone hoặc copy project
cd ~/robot_mir
```

**Sử dụng hàng ngày:**

**Terminal 1 - Chạy app:**
```bash
./mir.sh          # Menu tương tác
# Hoặc
./run_mir.sh      # Chạy trực tiếp
```

**Terminal 2 - Monitor logs:**
```bash
./logs.sh         # Menu chọn loại logs
# Hoặc
docker-compose logs -f | grep "GESTURE\|TARGET"
```

**Quy trình hoàn chỉnh:**
1. **Khởi động**: `./run_mir.sh` hoặc `./mir.sh`
2. **Monitor**: `./logs.sh` (terminal mới) để theo dõi
3. **Đợi GUI xuất hiện** (~30s sau khi build xong)
4. **Đăng ký mặt**: 3 bước như hướng dẫn
5. **Bắt đầu tracking**: Giơ 2 ngón tay ✌️
6. **Robot theo dõi**: Ở khoảng cách 1.4m
7. **Dừng lại**: Giơ 3 ngón tay 🤟
8. **Điều hướng**: Click chuột vào map (khi đang dừng)
9. **Đỗ xe**: Giơ 5 ngón tay ✋ (robot về vị trí ban đầu)
10. **Thoát**: Nhấn `Ctrl+C` trong terminal, chọn Stop trong `./mir.sh`

**Debug khi có vấn đề:**
```bash
./logs.sh                           # Chọn option 5 (Errors)
docker-compose logs | grep ERROR    # Xem lỗi
docker exec -it mir_follower_app bash  # Vào container
```

## 📝 Technical Stack

- **ROS Noetic** - Robot Operating System (communication via topics)
- **PyTorch** - Deep learning framework
- **YOLO11** - Object detection
- **InsightFace** - Face recognition
- **MediaPipe** - Hand gesture recognition
- **Intel RealSense SDK** - Depth camera (pre-built packages)
- **PyQt5** - GUI framework (system packages)
- **Docker** - Containerization
- **CUDA 11.8** - GPU acceleration

**Note:** MiR robot SDK không cần cài vì app điều khiển qua ROS topics (`/cmd_vel`) thay vì MiR API.

## 🧪 E1 experiment logging

Mỗi lần chạy app tạo một thư mục riêng trong `experiments/logs/` gồm:

- `run_metadata.json`: cấu hình, scenario, host, camera và tham số điều khiển;
- `frames.csv`: một dòng cho mỗi frame RGB-D sau perception và control;
- `commands.csv`: mọi lệnh vận tốc và cờ lệnh có thực sự được publish;
- `odometry.csv`: stream `/odom`;
- `events.jsonl`: sự kiện khởi động và lỗi;
- các file `summary.*` sau khi chạy analyzer.

Pilot an toàn, không phát `/cmd_vel` tới MiR:

```bash
export EXPERIMENT_CONFIG=p_original
export EXPERIMENT_SCENARIO=crossing_01
export EXPERIMENT_RUN_ID=20261117_p_original_crossing_01
export MIR_CONTROL_MODE=dry_run
docker compose up --build
```

Chỉ dùng `MIR_CONTROL_MODE=live` sau khi kiểm tra `commands.csv` của pilot có
`published=0`. Cách tổ chức, ghi ROS bag và tạo bảng dùng trong paper nằm ở
[`experiments/README.md`](experiments/README.md); báo cáo phương pháp nằm ở
[`experiments/paper/E1_logging_and_sim_real_comparison.md`](experiments/paper/E1_logging_and_sim_real_comparison.md).

Khi GUI mở, tab **E1 — Hướng dẫn và đo** tự xếp hàng các lượt của cấu hình đã
triển khai. Đăng ký người cần theo dõi trên hình camera; sau đó giao diện tự
đếm ngược, hiện từng thao tác, ghi log, kiểm tra độ đủ kỹ thuật và chuyển lượt.
Không còn nút bắt đầu từng lượt. Có nút dừng phiên tự động để dừng robot khi cần.
Hiện chỉ `p_original` được triển khai; `p_no_zero` và `p_c1` sẽ không tự chạy.

## 🐛 Known Issues

- **First build**: Docker image ~7.5GB, build 12-20 phút lần đầu
- **RealSense SDK**: Dùng pre-built packages từ Intel (nhanh và ổn định)
- **Python packages**: Cài theo layers để tránh conflicts
- **PyQt5**: Dùng system packages (python3-pyqt5)
- **Camera warming up**: 5-10s đầu camera có thể bị lag
- **ROS connection**: Cần đảm bảo robot và laptop cùng subnet
- **Face registration**: Cần ánh sáng tốt để đăng ký chính xác

## 📞 Support

Nếu gặp vấn đề:
1. Kiểm tra phần **Troubleshooting** ở trên
2. Xem logs: `docker-compose logs -f`
3. Kiểm tra GPU: `nvidia-smi`
4. Kiểm tra camera: `lsusb | grep Intel`
5. Kiểm tra ROS: `rostopic list`

## 📜 License

MIT License - Free to use and modify

---

**Version**: 2.0  
**Last Updated**: October 2025  
**Tested on**: Ubuntu 20.04 + NVIDIA RTX 3060 + RealSense D435
