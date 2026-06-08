"""
master_node.py  ─  CO-PAINT UGV-UAV 협업 도색 시스템 마스터 노드
================================================================

[확정된 시스템 구성]
  GCS PC  (192.168.53.5) : master_node, FAST-LIO, Web UI, rosbridge
  UAV RPi (192.168.53.2) : MicroXRCE-DDS, 카메라/라이다 드라이버,
                           fast_lio_to_px4.py, Flight Control Node
  UGV NUC (192.168.53.4) : UGV SLAM(FAST-LIO+Velodyne), UGV 제어,
                           ArUco 착륙보조 노드
  공유기 (ROS_DOMAIN_ID=53)

[확정된 SLAM 데이터 흐름]
  UAV:
    RPi: /livox/lidar + /livox/imu
      → GCS PC: FAST-LIO → /Odometry
      → fast_lio_to_px4.py (좌표계 변환)
      → /fmu/in/vehicle_visual_odometry (→ PX4 EKF2)
    최종 융합 위치: /fmu/out/vehicle_odometry

  UGV:
    NUC: Velodyne → FAST-LIO → /odom (Nav2 표준)

[핵심 원칙]
  1. 마스터는 /fmu/in/* 에 절대 직접 접근하지 않는다.
  2. 비동기 처리: 서비스 call_async() + 콜백, 콜백 내 sleep 금지.
  3. MultiThreadedExecutor + CallbackGroup 분리.
  4. set_home = 서비스 호출 없음. INIT 시점 /Odometry 현재값을 저장.

[FSM 상태]
  STANDBY → MAPPING → ZONE_SETUP → PLANNING
          → INIT → TAKEOFF → PAINTING → LANDING → RTL → DONE
  (any) → EMERGENCY

[착륙 시퀀스 - 확정]
  ① Master → Flight Control: UAV를 uav_home XY로 이동 (init 좌표)
  ② Master → UGV: ugv_home XY로 복귀
  ③ UAV+UGV가 init XY 근처 수렴 → ArUco 마커 감지
  ④ /landing/start_auto_land = True → 자동착륙 노드 활성화

[확정된 인터페이스]
  ✅ UGV odometry: /odom (Nav2, nav_param.yaml 기준)
  ✅ UGV 이동 명령: /goal_pose (PoseStamped) → Nav2 처리
  ✅ UGV ArUco 추적: /landing/start_auto_land (Bool)
  ✅ 경로계획: /planner/generate_path (custom_msgs/GeneratePath)
  ✅ 비행 제어: /flight_control/mission_cmd (String)

[Web UI 명령]
  ros2 topic pub --once /ui/command std_msgs/String 'data: "START_MAPPING"'
  ros2 topic pub --once /ui/command std_msgs/String 'data: "CAPTURE"'
  ros2 topic pub --once /ui/command std_msgs/String 'data: "STOP_MAPPING"'
  ros2 topic pub --once /ui/command std_msgs/String 'data: "INIT"'
  ros2 topic pub --once /ui/command std_msgs/String 'data: "TAKEOFF"'
  ros2 topic pub --once /ui/command std_msgs/String 'data: "START_PAINT"'
  ros2 topic pub --once /ui/command std_msgs/String 'data: "LAND"'
  ros2 topic pub --once /ui/command std_msgs/String 'data: "RTL"'
  ros2 topic pub --once /ui/command std_msgs/String 'data: "EMERGENCY"'
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import json, os, cv2, math
from enum import Enum, auto
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional

from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_msgs.srv import GeneratePath


# ══════════════════════════════════════════════════════════
#  FSM 상태
# ══════════════════════════════════════════════════════════

class State(Enum):
    STANDBY    = "STANDBY"
    MAPPING    = "MAPPING"      # 수동 조종 + 이미지/좌표 캡처
    ZONE_SETUP = "ZONE_SETUP"   # UI 구역 입력 대기
    PLANNING   = "PLANNING"     # 경로계획 서비스 응답 대기
    INIT       = "INIT"         # 이륙 전 좌표 원점 저장
    TAKEOFF    = "TAKEOFF"      # 이륙 명령 → 완료 대기
    PAINTING   = "PAINTING"     # 도색 비행 중
    LANDING    = "LANDING"      # 착륙 시퀀스 (XY 정렬 → ArUco)
    RTL        = "RTL"          # UGV 초기 위치 복귀
    DONE       = "DONE"
    EMERGENCY  = "EMERGENCY"


# 각 명령이 유효한 상태 목록
VALID_TRANSITIONS: dict[str, list[State]] = {
    "START_MAPPING": [State.STANDBY],
    "CAPTURE":       [State.MAPPING],
    "STOP_MAPPING":  [State.MAPPING],
    "INIT":          [State.ZONE_SETUP, State.PLANNING, State.MAPPING],
    "TAKEOFF":       [State.INIT],
    "START_PAINT":   [State.TAKEOFF, State.PAINTING],
    "LAND":          [State.PAINTING, State.TAKEOFF],
    "EMERGENCY":     list(State),
}


# ══════════════════════════════════════════════════════════
#  홈 포인트 (INIT 시 저장)
# ══════════════════════════════════════════════════════════

@dataclass
class HomePoint:
    """INIT 시점에 저장하는 UAV/UGV 시작 좌표"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    saved: bool = False

    def save_from_odom(self, odom: Odometry):
        p = odom.pose.pose.position
        self.x, self.y, self.z = p.x, p.y, p.z
        self.saved = True

    def dist_xy(self, odom: Odometry) -> float:
        p = odom.pose.pose.position
        return math.sqrt((p.x - self.x)**2 + (p.y - self.y)**2)

    def to_dict(self) -> dict:
        return {'x': round(self.x, 4), 'y': round(self.y, 4),
                'z': round(self.z, 4), 'saved': self.saved}


# ══════════════════════════════════════════════════════════
#  마스터 노드
# ══════════════════════════════════════════════════════════

class MasterNode(Node):

    # 착륙 XY 정렬 허용 오차 (m)
    LAND_ALIGN_TOL  = 0.4
    LAND_HOVER_ALT  = 1.5    # ArUco 감지 고도 (m, ENU 양수=위)
    ARUCO_TIMEOUT   = 30.0   # ArUco 못 찾을 시 강제 착륙 타임아웃 (초)

    def __init__(self):
        super().__init__('master_node')

        # ── 파라미터 ──
        self.declare_parameter('capture_dir',    '/tmp/copaint_capture')
        self.declare_parameter('uav_odom_topic', '/Odometry')
        # UGV odometry 토픽명: 팀원 확인 후 launch 인자로 덮어씀
        self.declare_parameter('ugv_odom_topic', '/odom')
        self.declare_parameter('land_align_tol', self.LAND_ALIGN_TOL)

        self.capture_dir    = self.get_parameter('capture_dir').value
        uav_odom_topic      = self.get_parameter('uav_odom_topic').value
        ugv_odom_topic      = self.get_parameter('ugv_odom_topic').value
        self.land_align_tol = self.get_parameter('land_align_tol').value

        os.makedirs(self.capture_dir, exist_ok=True)

        # ── 콜백 그룹 ──
        self.cb_ui      = MutuallyExclusiveCallbackGroup()   # UI 명령
        self.cb_service = ReentrantCallbackGroup()            # 서비스 (call_async)
        self.cb_sensor  = MutuallyExclusiveCallbackGroup()   # 센서 데이터

        # ── QoS ──
        qos_cmd = QoSProfile(                         # 명령: 신뢰성 보장
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_sensor = QoSProfile(                      # 센서: 최신값만
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # ── FSM 상태 ──
        self.state   = State.STANDBY
        self.state_t = self.get_clock().now()

        # ── 홈 포인트 (INIT 시 저장) ──
        self.uav_home = HomePoint()
        self.ugv_home = HomePoint()

        # ── 데이터 캐시 ──
        self.uav_odom:   Optional[Odometry] = None
        self.ugv_odom:   Optional[Odometry] = None
        self.latest_img: Optional[Image]    = None
        self.capture_cnt: int = 0
        self.trajectory: Optional[Path]     = None
        self.paint_waypoints_json: Optional[str] = None
        self.paint_zone_raw: Optional[str]  = None
        self.exclusion_zones: list          = []
        self.paint_pending: bool            = False  # 이륙 완료됐는데 경로 아직 없을 때

        # 착륙 단계 추적
        self.landing_phase    = 0   # 0=XY정렬중, 1=Z하강중, 2=ArUco대기, 3=자동착륙
        self.aruco_wait_start = None  # ArUco 대기 시작 시각 (타임아웃용)

        self.bridge = CvBridge()

        # ══════════════════════════════════════
        #  구독
        # ══════════════════════════════════════

        # Web UI 명령
        self.create_subscription(
            String, '/ui/command',
            self._on_ui_command, qos_cmd,
            callback_group=self.cb_ui)

        # Web UI 도색 구역 JSON
        self.create_subscription(
            String, '/ui/paint_zone',
            self._on_paint_zone, qos_cmd,
            callback_group=self.cb_ui)

        # UAV SLAM 위치 (FAST-LIO → /Odometry)
        self.create_subscription(
            Odometry, uav_odom_topic,
            self._on_uav_odom, qos_sensor,
            callback_group=self.cb_sensor)

        # UGV SLAM 위치
        self.create_subscription(
            Odometry, ugv_odom_topic,
            self._on_ugv_odom, qos_sensor,
            callback_group=self.cb_sensor)

        # 카메라 이미지 (CAPTURE 캐시용)
        self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self._on_image, qos_sensor,
            callback_group=self.cb_sensor)

        # 비전 금지 구역 (bbox_detection_node 에서 발행)
        self.create_subscription(
            String, '/vision/exclusion_zones',
            self._on_exclusion_zones, 10,
            callback_group=self.cb_sensor)

        # Flight Control Node 상태 피드백
        # 값: "ARMED" | "TAKEOFF_OK" | "PAINT_DONE" | "LANDED_CONFIRM"
        self.create_subscription(
            String, '/flight_control/status',
            self._on_flight_status, qos_cmd,
            callback_group=self.cb_ui)

        # ArUco 착륙보조 노드 감지 신호
        # ArUco 마커 보이면 True 발행
        self.create_subscription(
            Bool, '/aruco_landing/marker_detected',
            self._on_aruco_detected, 10,
            callback_group=self.cb_ui)

        # 착륙 노드 상태 (디버그/모니터링)
        self.create_subscription(
            String, '/landing_status',
            self._on_landing_status, 10,
            callback_group=self.cb_ui)

        # ══════════════════════════════════════
        #  발행
        # ══════════════════════════════════════

        # 현재 FSM 상태 → Web UI (rosbridge로 브라우저 전달)
        self.state_pub = self.create_publisher(
            String, '/master/current_state', qos_cmd)

        # 비행 제어 노드 고수준 명령
        # 값: TAKEOFF | PAINT | ALIGN_FOR_LAND:<x>,<y> | EMERGENCY
        self.flight_cmd_pub = self.create_publisher(
            String, '/flight_control/mission_cmd', qos_cmd)

        # 도색 궤적 → 비행 제어 노드
        self.trajectory_pub = self.create_publisher(
            Path, '/flight_control/trajectory', qos_cmd)

        # 도색 궤적 + paint_on 메타데이터 → 비행 제어 노드 (밸브 제어용)
        self.paint_wp_pub = self.create_publisher(
            String, '/flight_control/paint_waypoints', qos_cmd)

        # 자동착륙 노드 활성화 트리거
        self.land_trigger_pub = self.create_publisher(
            Bool, '/landing/start_auto_land', qos_cmd)

        # UGV 드론 추종 활성화/비활성화
        self.follow_enable_pub = self.create_publisher(
            Bool, '/ugv/follow_enable', qos_cmd)

        # ══════════════════════════════════════
        #  서비스 클라이언트
        # ══════════════════════════════════════

        # 경로계획 서비스
        self.planner_cli = self.create_client(
            GeneratePath, '/planner/generate_path',
            callback_group=self.cb_service)

        # 경로계획 노드에 구역 정보 사전 발행 (TRANSIENT_LOCAL → 늦게 켜도 수신)
        self.planner_zone_pub = self.create_publisher(
            String, '/planner/paint_zone', qos_cmd)
        self.planner_excl_pub = self.create_publisher(
            String, '/planner/exclusion_zones', qos_cmd)

        # ── 1Hz 상태 브로드캐스트 타이머 ──
        self.create_timer(1.0, self._broadcast_state,
                          callback_group=self.cb_ui)

        self.get_logger().info(
            '\n══════════════════════════════════════\n'
            '  CO-PAINT 마스터 노드 시작\n'
            f'  UAV odom: {uav_odom_topic}\n'
            f'  UGV odom: {ugv_odom_topic}\n'
            '  ROS_DOMAIN_ID: 53\n'
            '══════════════════════════════════════\n'
            '/ui/command 명령 목록:\n'
            '  START_MAPPING : 정찰 모드 시작\n'
            '  CAPTURE       : 이미지+좌표 저장\n'
            '  STOP_MAPPING  : 정찰 종료 → 구역설정 대기\n'
            '  INIT          : 현재 위치를 홈 포인트로 저장\n'
            '  TAKEOFF       : 이륙\n'
            '  START_PAINT   : 도색 시작\n'
            '  LAND          : 착륙 시퀀스 시작\n'
            '  RTL           : UGV 초기 복귀\n'
            '  EMERGENCY     : 긴급 정지\n'
        )

    # ══════════════════════════════════════════════════════════
    #  콜백 - UI 명령
    # ══════════════════════════════════════════════════════════

    def _on_ui_command(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'[UI] {cmd}  (현재: {self.state.value})')

        allowed = VALID_TRANSITIONS.get(cmd, [])
        if cmd != 'EMERGENCY' and self.state not in allowed:
            self.get_logger().warn(
                f'명령 무시: {self.state.value} 상태에서 {cmd} 불가 '
                f'(허용: {[s.value for s in allowed]})')
            return

        if   cmd == 'START_MAPPING': self._transition(State.MAPPING)
        elif cmd == 'CAPTURE':       self._do_capture()
        elif cmd == 'STOP_MAPPING':  self._transition(State.ZONE_SETUP)
        elif cmd == 'INIT':          self._do_init()
        elif cmd == 'TAKEOFF':       self._do_takeoff()
        elif cmd == 'START_PAINT':   self._do_start_paint()
        elif cmd == 'LAND':          self._do_land()
        elif cmd == 'EMERGENCY':     self._do_emergency()

    def _on_paint_zone(self, msg: String):
        """UI → 도색 구역 JSON 수신 → 경로계획 서비스 비동기 요청"""
        if self.state not in (State.ZONE_SETUP, State.MAPPING, State.PLANNING):
            self.get_logger().warn(f'구역 수신 무시 (현재: {self.state.value})')
            return
        self.paint_zone_raw = msg.data
        self.get_logger().info(f'도색 구역 수신 → PLANNING')
        self._transition(State.PLANNING)
        self._request_path_async()

    # ══════════════════════════════════════════════════════════
    #  콜백 - 센서 / 외부 노드
    # ══════════════════════════════════════════════════════════

    def _on_uav_odom(self, msg: Odometry):
        """
        UAV 위치 수신
        출처: FAST-LIO → /Odometry (GCS PC에서 발행)
        이 값이 fast_lio_to_px4.py 통해 /fmu/in/vehicle_visual_odometry 로도 감
        """
        self.uav_odom = msg

        # LANDING 중: XY 정렬 상태 지속 체크
        if self.state == State.LANDING:
            self._check_landing_alignment()

    def _on_ugv_odom(self, msg: Odometry):
        self.ugv_odom = msg

    def _on_image(self, msg: Image):
        self.latest_img = msg

    def _on_exclusion_zones(self, msg: String):
        """비전 노드 → 도색 금지 구역 JSON 수신"""
        try:
            self.exclusion_zones = json.loads(msg.data)
            self.get_logger().info(
                f'금지 구역 {len(self.exclusion_zones)}개 수신')
        except Exception as e:
            self.get_logger().error(f'금지구역 파싱 오류: {e}')

    def _on_flight_status(self, msg: String):
        """
        Flight Control Node → 상태 피드백
        ARMED | TAKEOFF_OK | PAINT_DONE | LANDED_CONFIRM
        """
        status = msg.data.strip().upper()
        self.get_logger().info(f'[FlightCtrl] {status}')

        if status == 'TAKEOFF_OK' and self.state == State.TAKEOFF:
            if self.trajectory is not None:
                self.get_logger().info('✅ 이륙 완료 + 경로 준비됨 → 자동 도색 시작')
                self._do_start_paint()
            else:
                self.paint_pending = True
                self.get_logger().warn(
                    '✅ 이륙 완료 but 경로 미준비 → 경로 도착 시 자동 시작')

        elif status == 'PAINT_DONE' and self.state == State.PAINTING:
            self.get_logger().info('✅ 도색 완료 → 자동으로 착륙 시퀀스 시작')
            self._do_land()          # ⑥ 자동 연속

        elif status == 'LANDED_CONFIRM' and self.state == State.LANDING:
            self.get_logger().info('✅ 착륙 확인 → 임무 완료')
            self._transition(State.DONE)

    def _on_aruco_detected(self, msg: Bool):
        """
        ArUco 착륙보조 노드 → 마커 감지 신호
        phase 2 (ArUco 대기) 상태에서만 자동착륙 트리거
        """
        if not msg.data:
            return
        if self.state != State.LANDING:
            return
        if self.landing_phase < 2:
            self.get_logger().info(
                f'ArUco 감지됐지만 phase {self.landing_phase} → 대기'
                f' (XY/Z 정렬 미완료)')
            return
        if self.landing_phase >= 3:
            return   # 이미 착륙 진행 중

        self.get_logger().info('✅ ArUco 마커 감지 → 자동착륙 시작!')
        self._force_auto_land()

    def _on_landing_status(self, msg: String):
        """착륙 노드 상태 수신 (TRACKING / LOCKED ON / SEARCHING 등)"""
        if self.state == State.LANDING and self.landing_phase >= 3:
            self.get_logger().info(
                f'[착륙 추적] {msg.data}', throttle_duration_sec=1.0)

    def _force_auto_land(self):
        """
        자동착륙 최종 트리거 (ArUco 감지 or 타임아웃)
        follow 비활성화 + pid_align_landing_node 활성화 + Flight Control 느린 하강
        """
        self.landing_phase = 3

        # follow_node 비활성화 → pid_align_landing_node가 /cmd_vel 독점
        self.follow_enable_pub.publish(Bool(data=False))
        self.get_logger().info('→ UGV follow 비활성화')

        # pid_align_landing_node 활성화 → UGV가 ArUco PID로 드론 밑 정렬
        land_msg      = Bool()
        land_msg.data = True
        self.land_trigger_pub.publish(land_msg)
        self.get_logger().info('→ pid_align_landing_node 활성화')

        # Flight Control: 느린 하강 시작
        self._send_flight_cmd('START_AUTO_LAND')
        self.get_logger().info('→ Flight Control: START_AUTO_LAND')

    # ══════════════════════════════════════════════════════════
    #  미션 액션
    # ══════════════════════════════════════════════════════════

    def _do_capture(self):
        """
        CAPTURE: 현재 카메라 이미지 + UAV/UGV 좌표 저장
        MAPPING 단계에서 GCS 운용자가 키보드로 트리거
        파일: /tmp/copaint_capture/img_<ts>.jpg + pose_<ts>.json
        """
        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:20]

        # 이미지 저장
        img_ok = False
        if self.latest_img is not None:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(self.latest_img, 'bgr8')
                cv2.imwrite(
                    os.path.join(self.capture_dir, f'img_{ts}.jpg'), cv_img)
                img_ok = True
            except Exception as e:
                self.get_logger().error(f'이미지 저장 실패: {e}')

        # 좌표 저장
        def _p(odom):
            if odom is None:
                return None
            p = odom.pose.pose.position
            return {'x': round(p.x, 4), 'y': round(p.y, 4),
                    'z': round(p.z, 4)}

        pose = {'timestamp': ts,
                'uav': _p(self.uav_odom),
                'ugv': _p(self.ugv_odom)}
        with open(os.path.join(self.capture_dir, f'pose_{ts}.json'),
                  'w', encoding='utf-8') as f:
            json.dump(pose, f, indent=2, ensure_ascii=False)

        self.capture_cnt += 1
        self.get_logger().info(
            f'📸 캡처 #{self.capture_cnt} | 이미지:{"✅" if img_ok else "❌"}'
            f' | UAV:{_p(self.uav_odom)} | 저장:{self.capture_dir}')

    def _do_init(self):
        """
        INIT: 현재 UAV/UGV 위치를 홈 포인트로 저장
        ※ 서비스 호출 없음 — /Odometry 현재값을 그냥 저장

        이 홈 포인트를 착륙 시 UAV/UGV가 복귀할 좌표로 사용
        (드론이 UGV 위에 착륙해 있는 상태에서 실행해야 함)
        """
        if self.uav_odom is None or self.ugv_odom is None:
            self.get_logger().error(
                'INIT 실패: UAV 또는 UGV odometry 미수신\n'
                f'  UAV: {"✅" if self.uav_odom else "❌"}\n'
                f'  UGV: {"✅" if self.ugv_odom else "❌"}\n'
                '  SLAM 노드 실행 여부 확인 필요')
            return

        # 홈 포인트 저장
        self.uav_home.save_from_odom(self.uav_odom)
        self.ugv_home.save_from_odom(self.ugv_odom)

        self.get_logger().info(
            '✅ 홈 포인트 저장 완료\n'
            f'  UAV home: ({self.uav_home.x:.3f}, '
            f'{self.uav_home.y:.3f}, {self.uav_home.z:.3f})\n'
            f'  UGV home: ({self.ugv_home.x:.3f}, '
            f'{self.ugv_home.y:.3f}, {self.ugv_home.z:.3f})\n'
            '  → TAKEOFF 명령 대기')

        self._transition(State.INIT)
        self._broadcast_state()

    def _do_takeoff(self):
        """이륙 명령 → Flight Control Node"""
        if not self.uav_home.saved:
            self.get_logger().error('INIT 먼저 실행 필요')
            return
        self._transition(State.TAKEOFF)
        self._send_flight_cmd('TAKEOFF')

    def _do_start_paint(self):
        """도색 시작: 궤적 전달 + PAINT 명령 + UGV 추종 활성화"""
        if self.trajectory is None:
            self.get_logger().error(
                '경로 없음 → ZONE_SETUP 및 구역 지정 먼저 필요')
            return
        self._transition(State.PAINTING)
        self._send_trajectory()
        self._send_flight_cmd('PAINT')
        self.follow_enable_pub.publish(Bool(data=True))
        self.get_logger().info('→ UGV follow 활성화')

    def _do_land(self):
        """
        착륙 시퀀스 시작
        ① UAV → home XY 위치로 이동 (고도 유지)
        ② UGV는 follow_node가 드론을 따라 자동으로 이동
        ③ XY 정렬 완료 + ArUco 감지 → follow 끄기 → pid_align_landing_node → 자동착륙
        """
        if not self.uav_home.saved or not self.ugv_home.saved:
            self.get_logger().error('홈 포인트 없음 → INIT 먼저 실행 필요')
            return

        self._transition(State.LANDING)
        self.landing_phase = 0

        # follow_node 유지 → 드론이 홈으로 가면 UGV도 따라감
        # (phase 3 진입 시 비활성화)

        # UAV: 저장된 home XY로 이동 명령 (ENU → NED 변환)
        home_x_ned = self.uav_home.y
        home_y_ned = self.uav_home.x
        align_cmd = (f'ALIGN_FOR_LAND:'
                     f'{home_x_ned:.4f},'
                     f'{home_y_ned:.4f}')
        self._send_flight_cmd(align_cmd)

        self.get_logger().info(
            '착륙 시퀀스 시작\n'
            f'  UAV 목표 (NED): ({home_x_ned:.3f}, {home_y_ned:.3f})\n'
            '  UGV: follow_node로 드론 추종 유지\n'
            '  XY 정렬 완료 + ArUco 감지 대기 중...')

    def _check_landing_alignment(self):
        """
        LANDING 중 10Hz로 호출 (uav_odom 콜백에서)

        착륙 시퀀스:
          phase 0: UAV/UGV XY 정렬 대기
          phase 1: XY 정렬 완료 → UAV Z축 ArUco 감지 고도로 하강
          phase 2: Z 도달 → ArUco 감지 대기 (타임아웃 30초)
          phase 3: ArUco 감지 → 자동착륙 진행
        """
        if self.uav_odom is None or self.ugv_odom is None:
            return

        # ── phase 0: UAV XY 정렬 대기 (UGV는 follow_node로 자동 추종) ──
        if self.landing_phase == 0:
            uav_err = self.uav_home.dist_xy(self.uav_odom)

            self.get_logger().info(
                f'[ALIGN XY] UAV:{uav_err:.2f}m'
                f'  (허용:{self.land_align_tol:.2f}m)',
                throttle_duration_sec=2.0)

            if uav_err < self.land_align_tol:
                self.landing_phase = 1
                self.get_logger().info(
                    '✅ XY 정렬 완료 → Z축 하강 시작 (ArUco 감지 고도)')
                # UAV를 ArUco 감지 가능 고도로 하강 명령
                # uav_home은 ENU → NED 변환 필요
                home_x_ned = self.uav_home.y
                home_y_ned = self.uav_home.x
                hover_z_ned = -self.LAND_HOVER_ALT
                align_cmd = (
                    f'ALIGN_FOR_LAND:'
                    f'{home_x_ned:.4f},'
                    f'{home_y_ned:.4f},'
                    f'{hover_z_ned:.4f}'
                )
                self._send_flight_cmd(align_cmd)

        # ── phase 1: Z축 하강 대기 ──
        elif self.landing_phase == 1:
            uav_z   = self.uav_odom.pose.pose.position.z  # ENU z (양수=위)
            z_err   = abs(uav_z - self.LAND_HOVER_ALT)
            self.get_logger().info(
                f'[ALIGN Z] 현재(ENU):{uav_z:.2f}m  목표:{self.LAND_HOVER_ALT:.2f}m'
                f'  오차:{z_err:.2f}m',
                throttle_duration_sec=2.0)

            if z_err < 0.3:
                self.landing_phase    = 2
                self.aruco_wait_start = self.get_clock().now()
                self.get_logger().info(
                    f'✅ Z 정렬 완료 (고도:{uav_z:.2f}m) → ArUco 감지 대기'
                    f'  (타임아웃: {self.ARUCO_TIMEOUT}초)')

        # ── phase 2: ArUco 타임아웃 감시 ──
        elif self.landing_phase == 2:
            if self.aruco_wait_start is None:
                self.aruco_wait_start = self.get_clock().now()
                return

            elapsed = (self.get_clock().now() -
                       self.aruco_wait_start).nanoseconds / 1e9

            self.get_logger().info(
                f'[ArUco 대기] {elapsed:.1f}s / {self.ARUCO_TIMEOUT}s',
                throttle_duration_sec=3.0)

            if elapsed >= self.ARUCO_TIMEOUT:
                self.get_logger().warn(
                    f'⚠️  ArUco {self.ARUCO_TIMEOUT}초 타임아웃 → 강제 착륙')
                self.landing_phase = 3
                self._force_auto_land()

    def _do_emergency(self):
        """긴급 정지 → Flight Control에 위임"""
        self.get_logger().error('🚨 긴급 정지!')
        self._transition(State.EMERGENCY)
        self._send_flight_cmd('EMERGENCY')

    # ══════════════════════════════════════════════════════════
    #  경로계획 (비동기)
    # ══════════════════════════════════════════════════════════

    def _request_path_async(self):
        """
        경로계획 노드에 구역 정보 발행 후 GeneratePath 서비스 비동기 호출

        흐름:
          1. paint_zone_raw → /planner/paint_zone 발행
          2. exclusion_zones → /planner/exclusion_zones 발행
          3. GeneratePath 서비스 트리거 (step, wall_x만 전달)
        """
        if not self.paint_zone_raw:
            self.get_logger().error('도색 구역 없음 → ZONE_SETUP 먼저 필요')
            self._set_fallback_path()
            return

        self.get_logger().info(
            f'경로계획 요청 준비\n'
            f'  도색구역: ✅\n'
            f'  금지구역: {len(self.exclusion_zones)}개'
        )

        # ① 구역 정보 사전 발행 (TRANSIENT_LOCAL → 경로계획 노드가 늦게 켜져도 수신)
        zone_msg      = String()
        zone_msg.data = self.paint_zone_raw
        self.planner_zone_pub.publish(zone_msg)

        excl_msg      = String()
        excl_msg.data = json.dumps(self.exclusion_zones, ensure_ascii=False)
        self.planner_excl_pub.publish(excl_msg)

        # ② 서비스 가용 확인
        if not self.planner_cli.service_is_ready():
            self.get_logger().warn(
                '/planner/generate_path 미응답 → 폴백 경로로 진행')
            self._set_fallback_path()
            return

        # ③ 서비스 트리거 (step, wall_x는 paint_zone JSON에서 경로계획 노드가 읽음)
        req        = GeneratePath.Request()
        req.step   = 0.0   # 0이면 경로계획 노드 default_step 사용
        req.wall_x = 0.0   # 0이면 paint_zone JSON의 wall_x 사용

        future = self.planner_cli.call_async(req)
        future.add_done_callback(self._on_path_response)
        self.get_logger().info('경로계획 서비스 요청 발송')

    def _on_path_response(self, future):
        """GeneratePath 서비스 응답 콜백"""
        try:
            result = future.result()
            if result.success:
                self.trajectory = result.path
                self.paint_waypoints_json = json.dumps([
                    {'x': float(pw.x), 'y': float(pw.y), 'z': float(pw.z),
                     'paint_on': bool(pw.paint_on), 'speed': float(pw.speed)}
                    for pw in result.waypoints
                ])
                self.get_logger().info(
                    f'✅ 경로계획 완료: {result.waypoint_count}개 웨이포인트\n'
                    f'  {result.message}'
                )
            else:
                self.get_logger().error(
                    f'경로계획 실패: {result.message} → 폴백 경로로 진행')
                self._set_fallback_path()
        except Exception as e:
            self.get_logger().error(f'경로계획 서비스 오류: {e} → 폴백 경로로 진행')
            self._set_fallback_path()

        # 이륙 완료 후 경로를 기다리고 있었으면 지금 바로 시작
        if self.paint_pending and self.state == State.TAKEOFF:
            self.paint_pending = False
            self.get_logger().info('경로 도착 + paint_pending → 자동 도색 시작')
            self._do_start_paint()

        self._broadcast_state()

    def _set_fallback_path(self):
        """경로계획 실패 시 빈 Path (드론 공중 대기)"""
        self.trajectory = Path()
        self.trajectory.header.frame_id = 'map'
        self.paint_waypoints_json = None
        self.get_logger().warn('⚠️  폴백 경로 사용 중 (빈 궤적)')

    # ══════════════════════════════════════════════════════════
    #  발행 헬퍼
    # ══════════════════════════════════════════════════════════

    def _send_flight_cmd(self, cmd: str):
        """Flight Control Node에 고수준 명령 발송"""
        msg      = String()
        msg.data = cmd
        self.flight_cmd_pub.publish(msg)
        self.get_logger().info(f'✈️  비행 제어: {cmd}')

    def _send_trajectory(self):
        """도색 궤적을 Flight Control Node에 전달"""
        if self.trajectory is None:
            return
        self.trajectory.header.stamp = self.get_clock().now().to_msg()
        self.trajectory_pub.publish(self.trajectory)
        if self.paint_waypoints_json is not None:
            self.paint_wp_pub.publish(String(data=self.paint_waypoints_json))
        self.get_logger().info(
            f'궤적 전달: {len(self.trajectory.poses)}개 포인트')

    # ══════════════════════════════════════════════════════════
    #  FSM 전환 + 상태 브로드캐스트
    # ══════════════════════════════════════════════════════════

    def _transition(self, new: State):
        old          = self.state
        self.state   = new
        self.state_t = self.get_clock().now()
        self.get_logger().info(f'🔄 {old.value} → {new.value}')
        self._broadcast_state()

    def _broadcast_state(self):
        """
        /master/current_state 발행 (1Hz + 전환 시 즉시)
        Web UI가 이를 구독해 버튼 활성화/패널 업데이트
        """
        def _p(odom):
            if odom is None:
                return None
            p = odom.pose.pose.position
            return {'x': round(p.x, 3),
                    'y': round(p.y, 3),
                    'z': round(p.z, 3)}

        elapsed = (self.get_clock().now() - self.state_t
                   ).nanoseconds / 1e9

        payload = {
            'state':         self.state.value,
            'elapsed_s':     round(elapsed, 1),
            'uav':           _p(self.uav_odom),
            'ugv':           _p(self.ugv_odom),
            'uav_home':      self.uav_home.to_dict(),
            'ugv_home':      self.ugv_home.to_dict(),
            'captures':      self.capture_cnt,
            'path_ready':    self.trajectory is not None,
            'landing_phase': self.landing_phase,
        }
        msg      = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.state_pub.publish(msg)


# ══════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().warn('종료 → 긴급 정지')
        node._do_emergency()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()