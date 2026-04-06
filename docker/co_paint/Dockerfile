FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------
# 1. 기본 시스템 + locale
# -----------------------------
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# -----------------------------
# 2. 기본 도구 + 개발 환경
# -----------------------------
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# 3. ROS2 repository 추가
# -----------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# -----------------------------
# 4. ROS2 Humble Desktop
# -----------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# 5. ROS 개발 도구 + rosbridge (통합)
# -----------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosbridge-suite \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# 6. rosdep 초기화
# -----------------------------
RUN rosdep init && rosdep update

# -----------------------------
# 7. Gazebo (시뮬레이션)
# -----------------------------
# RUN apt-get update && apt-get install -y \
#    gazebo \
#    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# 8. rosbridge launch 파일 복사
# ----------1.-------------------
COPY launch/rosbridge.launch.py /launch/rosbridge.launch.py

# -----------------------------
# 9. 워크스페이스
# -----------------------------
WORKDIR /workspace

EXPOSE 9090

CMD ["/bin/bash", "-c", \
    "source /opt/ros/humble/setup.bash && \
     ros2 launch /launch/rosbridge.launch.py & \
     bash"]