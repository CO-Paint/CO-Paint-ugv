"""
path_planner_node.py  ─  CO-PAINT Path Planner Node
================================================================
[역할]
  마스터 노드의 /planner/generate_path 서비스 요청을 받아
  도색 구역 + 금지 구역 정보로 3D 지그재그 궤적을 생성.

[좌표계]
  입력 (paint_zone JSON): ENU
  내부 처리 / 출력 (nav_msgs/Path):    NED

  변환:  X_ned = Y_enu,  Y_ned = X_enu,  Z_ned = -Z_enu
  (fastlio_to_px4.py 와 동일 규칙)

  운용 규칙: 매핑 시작 시 드론 정면이 벽을 향함.
  → ENU 시점에서 벽은 X_enu 가 거의 같은 4점.
  → NED 변환 후 벽은 Y_ned 가 거의 같은 4점 ... 처럼 보이지만,
     fastlio_to_px4 변환식상 'X_ned = Y_enu' 이므로
     벽은 NED 좌표계에서 'X_ned 가 거의 같은 4점' 으로 나타난다.
  → 즉 NED 에서 벽은 X=상수 평면. 지그재그는 (Y_ned × Z_ned) 에서 생성.
  → 드론이 벽을 바라보는 yaw = 0 (NED +X 방향).

[입력 - 토픽 (TRANSIENT_LOCAL, 마스터/facade_area_node 가 미리 발행)]
  /planner/paint_zone       String (JSON)  도색 구역 4꼭짓점 (ENU)
  /planner/exclusion_zones  String (JSON)  금지 구역 리스트 (정규화)

[서비스 서버]
  /planner/generate_path    custom_msgs/srv/GeneratePath
    Request : step (float)         # 0 이면 default_step 사용
              wall_x (float)       # 무시 (4점에서 자동 추출, 호환성 위해 유지)
    Response: waypoints (PaintWaypoint[]), path (nav_msgs/Path),
              waypoint_count, success, message

[paint_zone JSON 형식 (ENU)]
  {
    "main_area": [
      {"x": 1.0, "y": 2.5, "z": 0.5},   # 4점 모두 y_enu 가 거의 같음 (= 벽)
      {"x": 3.0, "y": 2.5, "z": 0.5},
      {"x": 3.0, "y": 2.5, "z": 2.5},
      {"x": 1.0, "y": 2.5, "z": 2.5}
    ],
    "step": 0.4
  }
  - wall_y_enu 는 4점의 y 평균값에서 자동 추출.
  - step 은 JSON 또는 서비스 Request 에서 받음.

[exclusion_zones JSON 형식 - 비전 노드 출력 (정규화)]
  [
    {"class": "window",  "cx": 0.3,  "cy": -0.1, "w": 0.25, "h": 0.4},
    ...
  ]
  cx, cy : 이미지 중심 기준 정규화 [-1, 1]
  w, h   : 이미지 크기 대비 정규화 [0, 1]

[알고리즘]
  1. paint_zone 4점 ENU → NED 변환
  2. 변환된 4점에서 wall_x_ned (X 평균), Y/Z 범위 추출
  3. exclusion_zones 정규화 좌표 → NED Y/Z 실좌표 변환
  4. Z_min(높은 고도) → Z_max(낮은 고도) 방향으로 step 씩 행 생성 (위→아래 도색)
  5. 각 행에서 금지구역 Y 구간을 빼 → 페인트 ON 구간 / 스킵 구간 분리
  6. 지그재그 방향 교대 (좌→우 / 우→좌)
  7. yaw = 0 (벽을 바라봄, +X 방향) 으로 quaternion 채움
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import json
import math
from typing import List, Tuple, Optional
from dataclasses import dataclass

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from custom_msgs.msg import PaintWaypoint
from custom_msgs.srv import GeneratePath


# ══════════════════════════════════════════════════════════
#  좌표계 변환
# ══════════════════════════════════════════════════════════

def enu_to_ned(x_enu: float, y_enu: float, z_enu: float) -> Tuple[float, float, float]:
    """ENU -> NED 변환 (fastlio_to_px4.py 와 동일 규칙)"""
    return y_enu, x_enu, -z_enu


# ══════════════════════════════════════════════════════════
#  내부 데이터 구조
# ══════════════════════════════════════════════════════════

@dataclass
class ExclusionZone:
    """정규화 좌표 기반 금지 구역 (비전 노드 출력)"""
    cls: str
    cx:  float   # [-1, 1] 이미지 좌우
    cy:  float   # [-1, 1] 이미지 상하 (위가 -1, 아래가 +1)
    w:   float   # [0, 1] 폭
    h:   float   # [0, 1] 높이


@dataclass
class WorldExclusion:
    """NED 좌표로 변환된 금지구역 (Y/Z 평면)"""
    cls:      str
    y_center: float
    z_center: float
    y_half:   float
    z_half:   float

    @property
    def y_lo(self) -> float: return self.y_center - self.y_half
    @property
    def y_hi(self) -> float: return self.y_center + self.y_half
    @property
    def z_lo(self) -> float: return self.z_center - self.z_half  # 더 음수 = 더 높음
    @property
    def z_hi(self) -> float: return self.z_center + self.z_half

    def contains_z(self, z: float, margin: float = 0.0) -> bool:
        return (self.z_lo - margin) <= z <= (self.z_hi + margin)


@dataclass
class Waypoint3D:
    x:        float
    y:        float
    z:        float
    paint_on: bool  = True
    speed:    float = 0.0


@dataclass
class Segment:
    """행 내 단일 구간 (시작Y ~ 끝Y, paint_on 여부)"""
    y_start:  float
    y_end:    float
    paint_on: bool
    speed:    float


# ══════════════════════════════════════════════════════════
#  좌표 변환 헬퍼
# ══════════════════════════════════════════════════════════

def normalize_excl_to_ned(
    ez: ExclusionZone,
    y_min_ned: float, y_max_ned: float,
    z_min_ned: float, z_max_ned: float,
) -> WorldExclusion:
    """
    정규화 (cx, cy) → NED 실좌표 변환.

    매핑:
      cx = -1 → 이미지 왼쪽  → Y_ned 작은 쪽 (y_min)
      cx = +1 → 이미지 오른쪽 → Y_ned 큰 쪽 (y_max)
      cy = -1 → 이미지 위    → Z_ned 작은 쪽 (z_min, 더 음수, 높은 고도)
      cy = +1 → 이미지 아래  → Z_ned 큰 쪽 (z_max, 덜 음수, 낮은 고도)
    """
    y_range = y_max_ned - y_min_ned
    z_range = z_max_ned - z_min_ned

    y_center = y_min_ned + (ez.cx + 1.0) / 2.0 * y_range
    z_center = z_min_ned + (ez.cy + 1.0) / 2.0 * z_range
    y_half   = ez.w / 2.0 * y_range
    z_half   = ez.h / 2.0 * z_range

    return WorldExclusion(ez.cls, y_center, z_center, y_half, z_half)


# ══════════════════════════════════════════════════════════
#  핵심 알고리즘 (NED Y/Z 평면 기준)
# ══════════════════════════════════════════════════════════

def get_row_segments(
    y_min:         float,
    y_max:         float,
    current_z:     float,
    exclusions:    List[WorldExclusion],
    default_speed: float,
    slow_speed:    float,
    margin:        float,
) -> List[Segment]:
    """한 행에서 금지구역을 피한 페인트 구간 리스트 생성."""
    active = [ez for ez in exclusions if ez.contains_z(current_z, margin=0.0)]

    if not active:
        return [Segment(y_min, y_max, paint_on=True, speed=default_speed)]

    excl_intervals = [(ez.y_lo, ez.y_hi) for ez in active]
    slow_intervals = [(ez.y_lo - margin, ez.y_hi + margin) for ez in active]

    events = sorted(set(
        [y_min, y_max] +
        [max(y_min, lo) for lo, hi in excl_intervals] +
        [min(y_max, hi) for lo, hi in excl_intervals] +
        [max(y_min, lo) for lo, hi in slow_intervals] +
        [min(y_max, hi) for lo, hi in slow_intervals]
    ))

    segments = []
    for i in range(len(events) - 1):
        seg_lo = events[i]
        seg_hi = events[i + 1]
        if seg_hi <= seg_lo + 1e-6:
            continue

        seg_mid = (seg_lo + seg_hi) / 2.0
        in_excl = any(lo <= seg_mid <= hi for lo, hi in excl_intervals)
        in_slow = any(lo <= seg_mid <= hi for lo, hi in slow_intervals)

        if in_excl:
            paint_on, speed = False, 0.0
        elif in_slow:
            paint_on, speed = True,  slow_speed
        else:
            paint_on, speed = True,  default_speed

        segments.append(Segment(seg_lo, seg_hi, paint_on, speed))

    return segments if segments else [Segment(y_min, y_max, True, default_speed)]


def segments_to_waypoints(
    segments:  List[Segment],
    wall_x:    float,
    current_z: float,
    direction: int,
) -> List[Waypoint3D]:
    """Segment 리스트 → Waypoint3D 리스트."""
    if direction == -1:
        segments = [
            Segment(s.y_end, s.y_start, s.paint_on, s.speed)
            for s in reversed(segments)
        ]

    waypoints = []
    for i, seg in enumerate(segments):
        if i == 0:
            waypoints.append(Waypoint3D(
                x=wall_x, y=round(seg.y_start, 4), z=round(current_z, 4),
                paint_on=seg.paint_on, speed=seg.speed))
        waypoints.append(Waypoint3D(
            x=wall_x, y=round(seg.y_end, 4), z=round(current_z, 4),
            paint_on=seg.paint_on, speed=seg.speed))
    return waypoints


def generate_zigzag_waypoints_ned(
    points_ned:      List[Tuple[float, float, float]],
    step:            float,
    exclusion_zones: List[ExclusionZone],
    default_speed:   float,
    slow_speed:      float,
) -> List[Waypoint3D]:
    """
    NED 4꼭짓점 + 금지구역 → 3D 지그재그 웨이포인트.

    points_ned: [(x, y, z), ...] (NED 좌표)
      - x 가 거의 같은 4점 (= 벽 평면)
      - y/z 가 도색 면 확장
    """
    if len(points_ned) != 4:
        raise ValueError(f'꼭짓점 4개 필요 (받음: {len(points_ned)}개)')

    xs = [p[0] for p in points_ned]
    ys = [p[1] for p in points_ned]
    zs = [p[2] for p in points_ned]

    # 4점의 X 평균 → 벽 평면 위치
    wall_x = sum(xs) / len(xs)

    y_min, y_max = min(ys), max(ys)
    z_min, z_max = min(zs), max(zs)

    if (y_max - y_min) < 0.1 or (z_max - z_min) < 0.1:
        raise ValueError(
            f'도색 영역 너무 작음: Y={y_max-y_min:.2f}m  Z={z_max-z_min:.2f}m')

    # 벽 평면 일관성 체크 (4점의 X 가 너무 흩어져 있으면 경고)
    x_spread = max(xs) - min(xs)
    if x_spread > 0.2:
        # 운용 규칙 위반 (드론이 벽에 비스듬히 매핑함). 그래도 진행.
        pass  # 로깅은 호출부에서

    # 정규화 금지구역 → NED 실좌표
    world_exclusions = [
        normalize_excl_to_ned(ez, y_min, y_max, z_min, z_max)
        for ez in exclusion_zones
    ]

    margin = step * 0.5

    waypoints: List[Waypoint3D] = []
    current_z = z_min   # 높은 고도(더 음수)부터 시작 → 낮은 고도(덜 음수)
    direction = 1

    while current_z <= z_max + 1e-6:
        segments = get_row_segments(
            y_min, y_max, current_z,
            world_exclusions,
            default_speed, slow_speed, margin,
        )
        row_wps = segments_to_waypoints(segments, wall_x, current_z, direction)
        waypoints.extend(row_wps)

        current_z = round(current_z + step, 6)
        direction *= -1

    return waypoints


def waypoints_to_path(
    waypoints: List[Waypoint3D],
    frame_id:  str = 'map',
) -> Path:
    """
    Waypoint3D (NED) → nav_msgs/Path.
    yaw = 0 (벽을 바라봄, NED +X 방향) → quaternion (0,0,0,1).
    """
    path = Path()
    path.header.frame_id = frame_id

    for wp in waypoints:
        ps = PoseStamped()
        ps.header.frame_id    = frame_id
        ps.pose.position.x    = float(wp.x)
        ps.pose.position.y    = float(wp.y)
        ps.pose.position.z    = float(wp.z)
        # yaw = 0 (벽 바라봄)
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)

    return path


def waypoints_to_paint_waypoints(
    waypoints: List[Waypoint3D],
) -> List[PaintWaypoint]:
    result = []
    for wp in waypoints:
        pw          = PaintWaypoint()
        pw.x        = float(wp.x)
        pw.y        = float(wp.y)
        pw.z        = float(wp.z)
        pw.paint_on = wp.paint_on
        pw.speed    = float(wp.speed)
        result.append(pw)
    return result


# ══════════════════════════════════════════════════════════
#  Path Planner Node
# ══════════════════════════════════════════════════════════

class PathPlannerNode(Node):

    DEFAULT_STEP  = 0.4
    DEFAULT_SPEED = 0.5
    SLOW_SPEED    = 0.2

    def __init__(self):
        super().__init__('path_planner_node')

        # ── 파라미터 ──
        self.declare_parameter('default_step',  self.DEFAULT_STEP)
        self.declare_parameter('default_speed', self.DEFAULT_SPEED)
        self.declare_parameter('slow_speed',    self.SLOW_SPEED)
        self.declare_parameter('frame_id',      'map')

        self.default_step  = self.get_parameter('default_step').value
        self.default_speed = self.get_parameter('default_speed').value
        self.slow_speed    = self.get_parameter('slow_speed').value
        self.frame_id      = self.get_parameter('frame_id').value

        # ── 내부 상태 ──
        # 4점은 NED 변환 후 캐싱 (x, y, z)
        self.paint_zone_points_ned: Optional[List[Tuple[float, float, float]]] = None
        self.paint_zone_step:       Optional[float] = None
        self.exclusion_zones:       List[ExclusionZone] = []

        # ── QoS ──
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── 구독 ──
        self.create_subscription(
            String, '/planner/paint_zone',
            self._on_paint_zone, cmd_qos)
        self.create_subscription(
            String, '/planner/exclusion_zones',
            self._on_exclusion_zones, cmd_qos)

        # ── 서비스 서버 ──
        self.srv = self.create_service(
            GeneratePath,
            '/planner/generate_path',
            self._handle_generate_path,
        )

        self.get_logger().info(
            '\n══════════════════════════════════════\n'
            '  CO-PAINT Path Planner Node 시작\n'
            '  좌표계: 입력=ENU, 출력=NED\n'
            '  /planner/generate_path 서비스 대기 중\n'
            f'  기본 step={self.default_step}m  '
            f'speed={self.default_speed}m/s  '
            f'slow={self.slow_speed}m/s\n'
            '══════════════════════════════════════'
        )

    # ══════════════════════════════════════════════════════════
    #  구역 정보 수신
    # ══════════════════════════════════════════════════════════

    def _on_paint_zone(self, msg: String):
        """도색 구역 수신 (ENU JSON) → 내부에서 NED 변환 후 캐싱"""
        try:
            data   = json.loads(msg.data)
            points = data.get('main_area', data.get('points'))

            if not points or len(points) != 4:
                raise ValueError(
                    f'꼭짓점 4개 필요 (받음: {len(points or [])}개)')

            # ENU → NED 변환
            points_ned = [
                enu_to_ned(float(p['x']), float(p['y']), float(p['z']))
                for p in points
            ]

            self.paint_zone_points_ned = points_ned
            self.paint_zone_step       = data.get('step', None)

            xs_ned = [p[0] for p in points_ned]
            ys_ned = [p[1] for p in points_ned]
            zs_ned = [p[2] for p in points_ned]

            x_spread = max(xs_ned) - min(xs_ned)
            wall_x   = sum(xs_ned) / len(xs_ned)

            self.get_logger().info(
                f'✅ 도색 구역 수신 (ENU → NED 변환)\n'
                f'  wall_x (NED): {wall_x:.2f}m  (점 분산: {x_spread:.3f}m)\n'
                f'  Y_ned: {min(ys_ned):.2f} ~ {max(ys_ned):.2f}m\n'
                f'  Z_ned: {min(zs_ned):.2f} ~ {max(zs_ned):.2f}m'
            )

            if x_spread > 0.2:
                self.get_logger().warn(
                    f'⚠️  4점의 X_ned 분산이 큼 ({x_spread:.2f}m). '
                    f'드론이 벽과 평행하게 매핑됐는지 확인 필요. '
                    f'평균값을 wall 평면으로 사용함.'
                )

        except (json.JSONDecodeError, KeyError, ValueError, TypeError) as e:
            self.get_logger().error(f'paint_zone 파싱 오류: {e}')
            self.paint_zone_points_ned = None

    def _on_exclusion_zones(self, msg: String):
        """금지 구역 수신 (정규화 좌표)"""
        try:
            data = json.loads(msg.data)
            if not data:
                self.exclusion_zones = []
                return

            self.exclusion_zones = [
                ExclusionZone(
                    cls=z.get('class', 'unknown'),
                    cx=float(z['cx']),
                    cy=float(z['cy']),
                    w=float(z['w']),
                    h=float(z['h']),
                )
                for z in data
            ]
            self.get_logger().info(
                f'✅ 금지 구역 수신: {len(self.exclusion_zones)}개')
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f'exclusion_zones 파싱 오류: {e}')
            self.exclusion_zones = []

    # ══════════════════════════════════════════════════════════
    #  서비스 핸들러
    # ══════════════════════════════════════════════════════════

    def _handle_generate_path(
        self,
        request:  GeneratePath.Request,
        response: GeneratePath.Response,
    ) -> GeneratePath.Response:

        self.get_logger().info(
            f'경로계획 서비스 요청 수신\n'
            f'  금지구역: {len(self.exclusion_zones)}개'
        )

        if self.paint_zone_points_ned is None:
            response.success = False
            response.message = '도색 구역 없음 → /planner/paint_zone 먼저 수신 필요'
            self.get_logger().error(response.message)
            return response

        # step 우선순위: Request > JSON > default
        if request.step > 0.0:
            step = request.step
        elif self.paint_zone_step and self.paint_zone_step > 0.0:
            step = self.paint_zone_step
        else:
            step = self.default_step

        # request.wall_x 는 더 이상 사용 안 함 (호환성 위해 무시)
        self.get_logger().info(
            f'경로 생성 시작\n'
            f'  step={step}m  '
            f'speed={self.default_speed}m/s  slow={self.slow_speed}m/s'
        )

        try:
            waypoints = generate_zigzag_waypoints_ned(
                points_ned      = self.paint_zone_points_ned,
                step            = step,
                exclusion_zones = self.exclusion_zones,
                default_speed   = self.default_speed,
                slow_speed      = self.slow_speed,
            )
        except ValueError as e:
            response.success = False
            response.message = f'궤적 생성 실패: {e}'
            self.get_logger().error(response.message)
            return response

        now  = self.get_clock().now().to_msg()
        path = waypoints_to_path(waypoints, self.frame_id)
        path.header.stamp = now

        paint_wps = waypoints_to_paint_waypoints(waypoints)

        paint_cnt = sum(1 for w in waypoints if w.paint_on)
        skip_cnt  = sum(1 for w in waypoints if not w.paint_on)
        slow_cnt  = sum(1 for w in waypoints
                        if w.paint_on and w.speed == self.slow_speed)

        response.waypoints      = paint_wps
        response.path           = path
        response.waypoint_count = len(waypoints)
        response.success        = True
        response.message        = (
            f'궤적 생성 완료 (NED): 총 {len(waypoints)}개  '
            f'(페인트ON={paint_cnt}  스킵={skip_cnt}  감속={slow_cnt})'
        )

        self.get_logger().info(
            f'✅ {response.message}\n'
            f'  예상 거리: {self._estimate_dist(waypoints):.1f}m'
        )
        return response

    # ══════════════════════════════════════════════════════════
    #  유틸
    # ══════════════════════════════════════════════════════════

    def _estimate_dist(self, waypoints: List[Waypoint3D]) -> float:
        if len(waypoints) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(waypoints)):
            a, b = waypoints[i-1], waypoints[i]
            total += math.sqrt(
                (a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        return total


# ══════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()