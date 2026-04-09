# CO-Paint-ugv
ROS2-based UGV system for the CO-Paint project, including navigation, control, and communication modules.


## Getting Started
### 1. Clone 전 Git 설정 (Windows 사용자 필수)

Windows 환경에서 클론하기 전에 아래 명령어를 먼저 실행하세요.
```bash
git config --global core.autocrlf input
```

> Windows는 Ubuntu와 달리 기본적으로 줄바꿈 문자를 CRLF로 변환하여 Docker 컨테이너나 Ubuntu에서 문제가 발생하는 것을 방지 함.

## Control code 의존성
### 1. MicroXrce-dds
https://github.com/eProsima/Micro-XRCE-DDS

### 2. PX4 autopilot
https://github.com/PX4/PX4-Autopilot
버전 1.16 필수
서브모듈 모두 가져와야함

### 3. px4_msgs
```bash
mkdir -p ~/px4_msgs_ws/src
cd ~/px4_msgs_ws/src
git clone -b release/1.16 https://github.com/PX4/px4_msgs.git
cd ~/px4_msgs_ws
colcon build
source ~/px4_msgs_ws/install/setup.bash
```
