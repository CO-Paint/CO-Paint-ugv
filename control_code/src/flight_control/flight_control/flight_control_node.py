#!/usr/bin/env python3
"""
Flight Control Node (UAV) - CO-Paint
=====================================
픽스호크(/fmu/in/...)로 명령을 퍼블리시하는 '유일한' 노드.

마스터 노드의 고수준 명령을 받아 px4_msgs/TrajectorySetpoint 로 변환해
20Hz로 픽스호크에 송신한다.

설계 원칙:
- 단일 노드, MultiThreadedExecutor + ReentrantCallbackGroup
- 논블로킹: 콜백 내 sleep / while True 금지. 모든 진행은 20Hz 타이머에서 인덱스로 처리.
- 좌표계: 모든 setpoint/odometry 는 NED.

명령 명세 (master_node.py 합의 기준):
- TAKEOFF               : 현재 XY 유지, takeoff_altitude 까지 상승. 완료 시 TAKEOFF_OK 발행.
- PAINT                 : 캐싱된 trajectory 추종. 완료 시 PAINT_DONE 발행.
- ALIGN_FOR_LAND:x,y    : XY 만 갱신, Z 유지. 피드백 없음.
- ALIGN_FOR_LAND:x,y,z  : (x,y,z) 로 이동. 피드백 없음.
- START_AUTO_LAND       : Z 만 천천히 하강. landed 감지 시 LANDED_CONFIRM 발행 + disarm.
- EMERGENCY             : 픽스호크 자체 자동 LAND 모드로 전환. (모든 상태에서 수용)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from nav_msgs.msg import Path

from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleOdometry, VehicleStatus, VehicleLandDetected,
)


# ── FSM 상태 ─────────────────────────────────────────
STANDBY   = 'STANDBY'     # 지상 대기. setpoint 미송신.
TAKEOFF   = 'TAKEOFF'     # 이륙 고도까지 상승.
HOVER     = 'HOVER'       # 호버, 다음 명령 대기.
PAINTING  = 'PAINTING'    # 경로 추종.
ALIGN     = 'ALIGN'       # 착륙 정렬용 XY/Z 이동.
AUTO_LAND = 'AUTO_LAND'   # Z 만 점진 하강.
EMERGENCY = 'EMERGENCY'   # 자동 LAND 모드 전환 후 손 뗌.

# ── 외부 발행용 status 문자열 (master_node 가 기다리는 값) ──
STATUS_TAKEOFF_OK     = 'TAKEOFF_OK'
STATUS_PAINT_DONE     = 'PAINT_DONE'
STATUS_LANDED_CONFIRM = 'LANDED_CONFIRM'


def quat_to_yaw(qx, qy, qz, qw):
    """quaternion -> yaw (라디안). Path 가 NED frame 이라는 전제."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class FlightControlNode(Node):
    def __init__(self):
        super().__init__('flight_control_node')

        # ---- 파라미터 ----
        self.declare_parameter('mission_cmd_topic', '/flight_control/mission_cmd')
        self.declare_parameter('trajectory_topic',  '/flight_control/trajectory')
        self.declare_parameter('status_topic',      '/flight_control/status')
        self.declare_parameter('takeoff_altitude',  1.5)    # m, 양수
        self.declare_parameter('accept_radius',     0.15)   # m, waypoint 도달 판정
        self.declare_parameter('land_descent_rate', 0.2)    # m/s
        self.declare_parameter('offboard_warmup_sec', 0.6)

        self.takeoff_alt       = float(self.get_parameter('takeoff_altitude').value)
        self.accept_radius     = float(self.get_parameter('accept_radius').value)
        self.land_descent_rate = float(self.get_parameter('land_descent_rate').value)
        self.warmup_sec        = float(self.get_parameter('offboard_warmup_sec').value)

        # ---- QoS ----
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
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

        # ---- 상태 변수 ----
        self.state = STANDBY
        self.curr = [0.0, 0.0, 0.0]   # NED 위치
        self.curr_yaw = 0.0
        self.landed = True

        # 목표 setpoint (NED)
        self.target = [0.0, 0.0, -self.takeoff_alt]
        self.target_yaw = 0.0

        # offboard 진입 절차용
        self.offboard_requested = False
        self.warmup_count = 0
        self.warmup_ticks = int(self.warmup_sec / 0.05)

        # 경로 추종용
        self.path = []          # [(x, y, z, yaw), ...]
        self.wp_index = 0

        # 자동 착륙용
        self.land_z = 0.0

        # ---- 20Hz 메인 타이머 ----
        self.create_timer(0.05, self.control_loop, callback_group=cb)

        self.get_logger().info('Flight Control Node started. state=STANDBY')

    # ================= 콜백 =================
    def odom_cb(self, msg: VehicleOdometry):
        self.curr = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
        self.curr_yaw = quat_to_yaw(msg.q[1], msg.q[2], msg.q[3], msg.q[0])

    def status_cb(self, msg: VehicleStatus):
        # 현재 사용 안 함. 필요시 arming_state/nav_state 활용 가능.
        pass

    def land_detected_cb(self, msg: VehicleLandDetected):
        self.landed = msg.landed

    def mission_cmd_cb(self, msg: String):
        raw = msg.data.strip()
        cmd_upper = raw.upper()
        self.get_logger().info(f'Received mission_cmd: {raw} (state={self.state})')

        # EMERGENCY: 모든 상태에서 수용
        if cmd_upper == 'EMERGENCY':
            self._handle_emergency()
            return

        if cmd_upper == 'TAKEOFF':
            self._handle_takeoff()
        elif cmd_upper == 'PAINT':
            self._handle_paint()
        elif cmd_upper.startswith('ALIGN_FOR_LAND'):
            self._handle_align(raw)
        elif cmd_upper == 'START_AUTO_LAND':
            self._handle_auto_land()
        else:
            self.get_logger().warn(f'Unknown command ignored: {raw}')

    def trajectory_cb(self, msg: Path):
        pts = []
        for ps in msg.poses:
            p = ps.pose.position
            q = ps.pose.orientation
            pts.append((float(p.x), float(p.y), float(p.z),
                        quat_to_yaw(q.x, q.y, q.z, q.w)))
        self.path = pts
        self.get_logger().info(f'Trajectory received: {len(pts)} waypoints')

    # ================= 명령 핸들러 =================
    def _handle_takeoff(self):
        if self.state != STANDBY:
            self.get_logger().warn(f'TAKEOFF ignored: must be in STANDBY (now {self.state})')
            return
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
        self._set_state(PAINTING)

    def _handle_align(self, raw_cmd: str):
        """ALIGN_FOR_LAND:x,y  또는  ALIGN_FOR_LAND:x,y,z"""
        if self.state not in (HOVER, PAINTING, ALIGN):
            self.get_logger().warn(f'ALIGN_FOR_LAND ignored: not airborne (now {self.state})')
            return
        try:
            payload = raw_cmd.split(':', 1)[1]
            parts = [float(v) for v in payload.split(',')]
        except (IndexError, ValueError) as e:
            self.get_logger().error(f'ALIGN_FOR_LAND parse error: {raw_cmd} ({e})')
            return

        if len(parts) == 2:
            x, y = parts
            z = self.target[2]   # Z 유지
        elif len(parts) == 3:
            x, y, z = parts
        else:
            self.get_logger().error(f'ALIGN_FOR_LAND expects 2 or 3 args, got {len(parts)}')
            return

        self.target = [x, y, z]
        self.target_yaw = self.curr_yaw
        if self.state != ALIGN:
            self._set_state(ALIGN)
        self.get_logger().info(f'ALIGN target=({x:.2f}, {y:.2f}, {z:.2f})')

    def _handle_auto_land(self):
        if self.state not in (HOVER, ALIGN, PAINTING):
            self.get_logger().warn(f'START_AUTO_LAND ignored: not airborne (now {self.state})')
            return
        self.target = [self.curr[0], self.curr[1], self.curr[2]]
        self.target_yaw = self.curr_yaw
        self.land_z = self.curr[2]
        self._set_state(AUTO_LAND)

    def _handle_emergency(self):
        """EMERGENCY: 픽스호크 자체 자동 LAND 모드로 전환. offboard 송신 중단."""
        self.get_logger().error('EMERGENCY received -> switching to PX4 NAV_LAND mode')
        self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self._set_state(EMERGENCY)

    # ================= FSM 전이 =================
    def _set_state(self, new_state):
        self.state = new_state
        self.get_logger().info(f'State -> {new_state}')

    # ================= 20Hz 제어 루프 =================
    def control_loop(self):
        # STANDBY, EMERGENCY 는 offboard 송신 안 함
        if self.state in (STANDBY, EMERGENCY):
            return

        self._publish_offboard_mode()

        if self.state == TAKEOFF:
            self._loop_takeoff()
        elif self.state == HOVER:
            pass
        elif self.state == PAINTING:
            self._loop_painting()
        elif self.state == ALIGN:
            pass
        elif self.state == AUTO_LAND:
            self._loop_auto_land()

        self._publish_setpoint()

    def _loop_takeoff(self):
        if not self.offboard_requested:
            self.warmup_count += 1
            if self.warmup_count == self.warmup_ticks:
                self._arm()
                self._set_offboard()
                self.offboard_requested = True
            return
        if abs(self.curr[2] - self.target[2]) < self.accept_radius:
            self._set_state(HOVER)
            self.status_pub.publish(String(data=STATUS_TAKEOFF_OK))
            self.get_logger().info(f'TAKEOFF complete -> HOVER (published {STATUS_TAKEOFF_OK})')

    def _loop_painting(self):
        if self.wp_index >= len(self.path):
            self._set_state(HOVER)
            self.status_pub.publish(String(data=STATUS_PAINT_DONE))
            self.get_logger().info(f'Trajectory complete -> HOVER (published {STATUS_PAINT_DONE})')
            return
        wx, wy, wz, wyaw = self.path[self.wp_index]
        self.target = [wx, wy, wz]
        self.target_yaw = wyaw
        dx = wx - self.curr[0]
        dy = wy - self.curr[1]
        dz = wz - self.curr[2]
        if math.sqrt(dx * dx + dy * dy + dz * dz) < self.accept_radius:
            self.wp_index += 1

    def _loop_auto_land(self):
        # Z만 점진 하강. NED 라 Z=0 이 지면, 음수가 공중.
        self.land_z += self.land_descent_rate * 0.05
        if self.land_z > 0.0:
            self.land_z = 0.0
        self.target = [self.target[0], self.target[1], self.land_z]
        if self.landed:
            self._disarm()
            self.status_pub.publish(String(data=STATUS_LANDED_CONFIRM))
            self.get_logger().info(f'Landed & disarmed (published {STATUS_LANDED_CONFIRM})')
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
