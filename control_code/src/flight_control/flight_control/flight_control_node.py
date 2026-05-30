#!/usr/bin/env python3
"""
Flight Control Node (UAV) - CO-Paint
=====================================
픽스호크(/fmu/in/...)로 명령을 퍼블리시하는 '유일한' 노드.

마스터 노드의 고수준 명령(TAKEOFF/PAINT/LAND)과 Planner의 경로(nav_msgs/Path)를
받아 px4_msgs/TrajectorySetpoint 로 변환해 20Hz로 픽스호크에 송신한다.

설계 원칙:
- 단일 노드, MultiThreadedExecutor + ReentrantCallbackGroup
- 논블로킹: 콜백 내 sleep / while True 금지. 모든 진행은 20Hz 타이머에서 인덱스로 처리.
- FSM: STANDBY -> TAKEOFF -> HOVER -> PAINT -> LAND -> STANDBY
- 좌표계: 모든 setpoint/odometry 는 NED.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Bool
from nav_msgs.msg import Path

from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleOdometry, VehicleStatus, VehicleLandDetected,
)


# FSM 상태
STANDBY = 'STANDBY'   # 지상 대기. setpoint 미송신.
TAKEOFF = 'TAKEOFF'   # 이륙 고도까지 상승.
HOVER   = 'HOVER'     # 경로 대기 / 호버.
PAINT   = 'PAINT'     # 경로 추종.
LAND    = 'LAND'      # Z만 천천히 하강 후 disarm.


def quat_to_yaw_ned(qx, qy, qz, qw):
    """ROS Path pose 의 quaternion(ENU 기준 가정) yaw -> NED yaw.

    Planner가 NED 좌표로 Path를 만든다면 그대로 yaw를 뽑으면 되지만,
    ROS 관례상 Path가 ENU로 올 수 있어 부호를 마스터/Planner와 합의해야 한다.
    여기서는 'Path가 이미 NED frame' 이라는 전제로 표준 yaw를 추출한다.
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class FlightControlNode(Node):
    def __init__(self):
        super().__init__('flight_control_node')

        # ---- 파라미터 (인터페이스 이름/튜닝값은 launch에서 덮어쓰기) ----
        self.declare_parameter('mission_cmd_topic', '/flight_control/mission_cmd')
        self.declare_parameter('trajectory_topic', '/flight_control/trajectory')
        self.declare_parameter('status_topic', '/flight_control/status')
        self.declare_parameter('landed_confirm_topic', '/flight_control/landed_confirm')
        self.declare_parameter('takeoff_altitude', 1.5)     # m (상승 높이, 양수)
        self.declare_parameter('cruise_speed', 0.3)         # m/s (도색 이동 속도)
        self.declare_parameter('accept_radius', 0.15)       # m (waypoint 도달 판정)
        self.declare_parameter('land_descent_rate', 0.2)    # m/s (착륙 하강 속도)
        self.declare_parameter('offboard_warmup_sec', 0.6)  # offboard 진입 전 더미 송신 시간

        self.takeoff_alt = float(self.get_parameter('takeoff_altitude').value)
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.accept_radius = float(self.get_parameter('accept_radius').value)
        self.land_descent_rate = float(self.get_parameter('land_descent_rate').value)
        self.warmup_sec = float(self.get_parameter('offboard_warmup_sec').value)

        # ---- QoS ----
        # 픽스호크 통신: BEST_EFFORT (기존 검증된 프로파일)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # 마스터 노드 명령/상태: RELIABLE + TRANSIENT_LOCAL (명령 유실 방지)
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        cb = ReentrantCallbackGroup()

        # ---- 픽스호크 발신 ----
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)

        # ---- 픽스호크 수신 ----
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_cb, px4_qos, callback_group=cb)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.status_cb, px4_qos, callback_group=cb)
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self.land_detected_cb, px4_qos, callback_group=cb)

        # ---- 마스터 노드 통신 ----
        self.create_subscription(
            String, self.get_parameter('mission_cmd_topic').value,
            self.mission_cmd_cb, cmd_qos, callback_group=cb)
        self.create_subscription(
            Path, self.get_parameter('trajectory_topic').value,
            self.trajectory_cb, cmd_qos, callback_group=cb)
        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, cmd_qos)
        self.landed_confirm_pub = self.create_publisher(
            Bool, self.get_parameter('landed_confirm_topic').value, cmd_qos)

        # ---- 상태 변수 ----
        self.state = STANDBY
        self.curr = [0.0, 0.0, 0.0]   # NED 위치
        self.curr_yaw = 0.0
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.landed = True

        # 목표 setpoint (NED)
        self.target = [0.0, 0.0, -self.takeoff_alt]
        self.target_yaw = 0.0

        # offboard 진입 절차용
        self.offboard_requested = False
        self.warmup_count = 0
        self.warmup_ticks = int(self.warmup_sec / 0.05)  # 20Hz 기준 틱 수

        # 경로 추종용
        self.path = []            # [(x, y, z, yaw), ...]
        self.wp_index = 0

        # 착륙용
        self.land_z = 0.0

        # ---- 20Hz 메인 타이머 ----
        self.create_timer(0.05, self.control_loop, callback_group=cb)

        self.get_logger().info('Flight Control Node started. state=STANDBY')

    # ================= 콜백 =================
    def odom_cb(self, msg: VehicleOdometry):
        self.curr = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
        self.curr_yaw = quat_to_yaw_ned(msg.q[1], msg.q[2], msg.q[3], msg.q[0])

    def status_cb(self, msg: VehicleStatus):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state

    def land_detected_cb(self, msg: VehicleLandDetected):
        self.landed = msg.landed

    def mission_cmd_cb(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'Received mission_cmd: {cmd} (state={self.state})')

        if cmd == 'TAKEOFF':
            self._handle_takeoff()
        elif cmd == 'PAINT':
            self._handle_paint()
        elif cmd == 'LAND':
            self._handle_land()
        else:
            self.get_logger().warn(f'Unknown command ignored: {cmd}')

    def trajectory_cb(self, msg: Path):
        # 경로는 언제든 받아서 캐싱만. 추종 시작은 PAINT 명령에서.
        pts = []
        for ps in msg.poses:
            p = ps.pose.position
            q = ps.pose.orientation
            pts.append((float(p.x), float(p.y), float(p.z),
                        quat_to_yaw_ned(q.x, q.y, q.z, q.w)))
        self.path = pts
        self.get_logger().info(f'Trajectory received: {len(pts)} waypoints')

    # ================= FSM 전이 핸들러 =================
    def _handle_takeoff(self):
        # STANDBY 에서만 이륙 허용 (방어 코드)
        if self.state != STANDBY:
            self.get_logger().warn(f'TAKEOFF ignored: must be in STANDBY (now {self.state})')
            return
        # 현재 XY 유지하고 고도만 상승
        self.target = [self.curr[0], self.curr[1], -self.takeoff_alt]
        self.target_yaw = self.curr_yaw
        self.offboard_requested = False
        self.warmup_count = 0
        self._set_state(TAKEOFF)

    def _handle_paint(self):
        if self.state != HOVER:
            self.get_logger().warn(f'PAINT ignored: must be in HOVER (now {self.state})')
            return
        if not self.path:
            self.get_logger().warn('PAINT ignored: no trajectory received')
            return
        self.wp_index = 0
        self._set_state(PAINT)

    def _handle_land(self):
        if self.state not in (HOVER, PAINT, TAKEOFF):
            self.get_logger().warn(f'LAND ignored: not airborne (now {self.state})')
            return
        # XY 현재 위치 고정, Z는 현재 고도부터 점진 하강
        self.target = [self.curr[0], self.curr[1], self.curr[2]]
        self.target_yaw = self.curr_yaw
        self.land_z = self.curr[2]
        self._set_state(LAND)

    def _set_state(self, new_state):
        self.state = new_state
        self.get_logger().info(f'State -> {new_state}')
        self._publish_status()

    def _publish_status(self):
        self.status_pub.publish(String(data=self.state))

    # ================= 20Hz 제어 루프 =================
    def control_loop(self):
        if self.state == STANDBY:
            return  # 지상 대기: 아무것도 쏘지 않음

        # offboard 모드에서는 항상 heartbeat + setpoint 송신
        self._publish_offboard_mode()

        if self.state == TAKEOFF:
            self._loop_takeoff()
        elif self.state == HOVER:
            self._loop_hover()
        elif self.state == PAINT:
            self._loop_paint()
        elif self.state == LAND:
            self._loop_land()

        self._publish_setpoint()

    def _loop_takeoff(self):
        # 1) 더미 setpoint warmup -> 2) ARM -> 3) OFFBOARD 전환
        if not self.offboard_requested:
            self.warmup_count += 1
            if self.warmup_count == self.warmup_ticks:
                self._arm()
                self._set_offboard()
                self.offboard_requested = True
            return
        # 이륙 고도 도달하면 HOVER
        if abs(self.curr[2] - self.target[2]) < self.accept_radius:
            self._set_state(HOVER)

    def _loop_hover(self):
        pass  # target 유지 (이미 setpoint 송신됨)

    def _loop_paint(self):
        if self.wp_index >= len(self.path):
            self.get_logger().info('Trajectory complete -> HOVER')
            self._set_state(HOVER)
            return
        wx, wy, wz, wyaw = self.path[self.wp_index]
        self.target = [wx, wy, wz]
        self.target_yaw = wyaw
        # 도달 판정 -> 다음 waypoint
        dx, dy, dz = wx - self.curr[0], wy - self.curr[1], wz - self.curr[2]
        if math.sqrt(dx * dx + dy * dy + dz * dz) < self.accept_radius:
            self.wp_index += 1

    def _loop_land(self):
        # Z만 점진 하강 (1틱 = 0.05s)
        self.land_z += self.land_descent_rate * 0.05
        self.target = [self.target[0], self.target[1], self.land_z]
        # 착륙 감지되면 disarm 후 STANDBY
        if self.landed:
            self._disarm()
            self.landed_confirm_pub.publish(Bool(data=True))
            self.get_logger().info('Landed & disarmed. landed_confirm=True')
            self.path = []
            self.wp_index = 0
            self._set_state(STANDBY)

    # ================= 픽스호크 송신 =================
    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self.offboard_pub.publish(msg)

    def _publish_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [float(self.target[0]), float(self.target[1]), float(self.target[2])]
        msg.yaw = float(self.target_yaw)
        msg.timestamp = self._now_us()
        self.setpoint_pub.publish(msg)

    def _send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self.command_pub.publish(msg)

    def _arm(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def _disarm(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def _set_offboard(self):
        # param1=1 (custom mode), param2=6 (PX4 offboard)
        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None):
    rclpy.init(args=args)
    node = FlightControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()