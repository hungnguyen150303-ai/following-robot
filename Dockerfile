# MiR Robot Person Follower - Dockerfile
# Based on ROS Noetic with NVIDIA GPU support

FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Ho_Chi_Minh

# Set up locales
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install ROS Noetic
RUN apt-get update && apt-get install -y --no-install-recommends \
    lsb-release \
    gnupg2 \
    curl \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-noetic-desktop-full \
        ros-noetic-move-base \
        ros-noetic-move-base-msgs \
        ros-noetic-navigation \
        ros-noetic-tf \
        ros-noetic-tf2 \
        ros-noetic-actionlib \
        ros-noetic-geometry-msgs \
        ros-noetic-sensor-msgs \
        ros-noetic-nav-msgs \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        build-essential \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install system dependencies (combined layer, cleaned immediately)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    git \
    cmake \
    pkg-config \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    gfortran \
    openexr \
    libatlas-base-dev \
    libtbb2 \
    libtbb-dev \
    libdc1394-22-dev \
    libopenexr-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    wget \
    unzip \
    pulseaudio \
    pulseaudio-utils \
    alsa-utils \
    libasound2-dev \
    libqt5widgets5 \
    libqt5gui5 \
    libqt5core5a \
    libqt5svg5 \
    qt5-default \
    python3-pyqt5 \
    python3-pyqt5.qtsvg \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libusb-1.0-0-dev \
    libglfw3-dev \
    libssl-dev \
    software-properties-common \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install Intel RealSense SDK (combined layer, cleaned immediately)
# Use the current HTTPS keyring method; the old apt-key/keyserver method is
# unreliable and the RealSense repository signing key has changed.
RUN mkdir -p /etc/apt/keyrings \
    && curl -fsSL https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
       | gpg --dearmor -o /etc/apt/keyrings/librealsenseai.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/librealsense.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Upgrade pip
RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel

# Fix importlib_metadata for catkin compatibility
RUN pip3 install --no-cache-dir "importlib-metadata<5.0" \
    && rm -rf ~/.cache/pip

# Install PyTorch with CUDA support first (base dependency)
RUN pip3 install --no-cache-dir \
    torch==2.0.1 \
    torchvision==0.15.2 \
    --index-url https://download.pytorch.org/whl/cu118 \
    && rm -rf ~/.cache/pip

# Install core scientific packages
RUN pip3 install --no-cache-dir \
    numpy==1.24.3 \
    scipy==1.10.1 \
    scikit-learn==1.3.1 \
    matplotlib==3.7.5 \
    Pillow==10.0.1 \
    && rm -rf ~/.cache/pip

# Install OpenCV (no version conflicts)
RUN pip3 install --no-cache-dir \
    opencv-python==4.8.1.78 \
    opencv-contrib-python==4.8.1.78 \
    && rm -rf ~/.cache/pip

# Install RealSense Python bindings
RUN pip3 install --no-cache-dir pyrealsense2==2.54.1.5216 \
    && rm -rf ~/.cache/pip

# Ubuntu/ROS provides psutil as a distutils package.  Install a pip wheel
# without trying to uninstall that system package; otherwise pip aborts with
# "uninstall-distutils-installed-package" when resolving Ultralytics.
RUN pip3 install --no-cache-dir --ignore-installed "psutil==5.9.8" \
    && rm -rf ~/.cache/pip

# Install AI/ML packages (Ultralytics >= 8.3.0 for YOLO11 support)
RUN pip3 install --no-cache-dir \
    "ultralytics>=8.3.0" \
    onnxruntime-gpu==1.16.0 \
    && rm -rf ~/.cache/pip

# Install MediaPipe (may downgrade some packages, that's ok)
RUN pip3 install --no-cache-dir mediapipe==0.10.5 \
    && rm -rf ~/.cache/pip

# Install InsightFace with dependencies
RUN pip3 install --no-cache-dir \
    insightface==0.7.3 \
    onnx \
    && rm -rf ~/.cache/pip

# Install utilities (PyQt5 already installed via system packages)
RUN pip3 install --no-cache-dir \
    pygame==2.5.2 \
    tqdm \
    pyyaml \
    && rm -rf ~/.cache/pip

# Set up working directory
WORKDIR /app

# Create assets directory for sounds (before copying code)
RUN mkdir -p /app/assets

# Set up ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install additional ROS packages for MiR robot
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-robot-localization \
    ros-noetic-teb-local-planner \
    ros-noetic-hector-gazebo-plugins \
    ros-noetic-rospy-message-converter \
    python3-websocket \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* \
    || true

# Install additional dependencies for MiR driver
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-tools \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Create catkin workspace and build only essential MiR packages (driver + msgs)
RUN mkdir -p /root/catkin_ws/src \
    && cd /root/catkin_ws/src \
    && git clone https://github.com/dfki-ric/mir_robot.git \
    && cd /root/catkin_ws \
    && bash -c "source /opt/ros/noetic/setup.bash && catkin build mir_driver mir_msgs sdc21x0 mir_actions -DCMAKE_BUILD_TYPE=Release" \
    && echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc \
    && rm -rf /root/catkin_ws/build /root/catkin_ws/.catkin_tools /root/catkin_ws/src/*/.git

# Copy requirements.txt first (changes rarely)
COPY requirements.txt /app/

# Copy main application code LAST (changes most frequently)
# Only this layer rebuilds when you edit main.py
COPY src/ /app/src/

# Set up environment variables
ENV PYTHONPATH=/app:$PYTHONPATH
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_HOSTNAME=localhost

# GPU support
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics

# Display support
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV QT_DEBUG_PLUGINS=0

# Audio support
ENV PULSE_SERVER=unix:/run/user/1000/pulse/native

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "/app/src/main.py"]
