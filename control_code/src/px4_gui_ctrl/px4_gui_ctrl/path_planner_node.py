"""
path_planner_node.py  ─  CO-PAINT Path Planner Node
================================================================
[역할]
  마스터 노드의 /planner/generate_path 서비스 요청을 받아
  도색 구역 + 금지 구역 정보로 3D 지그재그 궤적을 생성.

[입력 - 토픽 (TRANSIENT_LOCAL, 마스터/facade_area_node가 미리 발행)]
  /planner/paint_zone       String (JSON)  도색 구역 4꼭짓점
  /planner/exclusion_zones  String (JSON)  금지 구역 리스트

[서비스 서버]
  /planner/generate_path    custom_msgs/srv/GeneratePath
    Request : step (float), wall_x (float)
    Response: waypoints (PaintWaypoint[]), path (nav_msgs/Path),
              waypoint_count, success, message

[paint_zone JSON 형식]
  {
    "main_area": [
      {"x": 0.0, "y": -1.5, "z": -1.0},
      {"x": 0.0, "y":  1.5, "z": -1.0},
      {"x": 0.0, "y":  1.5, "z": -3.0},
      {"x": 0.0, "y": -1.5, "z": -3.0}
    ],
    "wall_x": 2.5,
    "step": 0.4
  }

[exclusion_zones JSON 형식 - bbox_detection_node 출력]
  [
    {"class": "window",  "cx": 0.3,  "cy": -0.1, "w": 0.25, "h": 0.4},
    {"class": "balcony", "cx": -0.2, "cy":  0.5, "w": 0.5,  "h": 0.2}
  ]
  cx, cy : 이미지 중심 기준 정규화 [-1, 1]
  w, h   : 이미지 크기 대비 정규화 [0, 1]

[NED 좌표계]
  X = North (벽 방향, 고정)
  Y = East  (좌우)
  Z = Down  (음수 = 위)

[알고리즘 - 고도화 버전]
  1. 4꼭짓점 → Y/Z 범위 추출
  2. exclusion_zones 정규화 좌표 → 실제 NED 좌표로 변환
  3. Z_max(낮은 고도) → Z_min(높은 고도) 방향으로 step씩 행 생성
  4. 각 행에서:
     a. 해당 Z에 걸치는 금지구역의 Y 범위 추출
     b. Y 전체 구간에서 금지구역 Y 범위를 빼 → 페인트 ON 구간 / OFF 구간 분리
     c. 각 구간마다 시작점 + 끝점 웨이포인트 생성
     d. 금지구역 경계 진입/진출 시 감속 웨이포인트 삽입
  5. 지그재그 방향에 맞게 구간 정렬 (좌→우 / 우→좌 교대)
  6. nav_msgs/Path + PaintWaypoint[] 동시 반환
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import json
import math
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass, field

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from custom_msgs.msg import PaintWaypoint
from custom_msgs.srv import GeneratePath


# ══════════════════════════════════════════════════════════
#  내부 데이터 구조
# ══════════════════════════════════════════════════════════

@dataclass
class ExclusionZone:
    """
    정규화 좌표 기반 금지 구역 (bbox_detection_node 출력 기준)

    cx, cy: 이미지 중심 기준 정규화 [-1, 1]
      cx > 0 → 이미지 오른쪽 → NED Y 양수 방향
      cy > 0 → 이미지 아래쪽 → NED Z 덜 음수 (낮은 고도)
    w, h: 이미지 크기 대비 정규화 [0, 1]
    """
    cls: str
    cx:  float
    cy:  float
    w:   float
    h:   float

    def to_world(
        self,
        y_min: float, y_max: float,
        z_min: float, z_max: float,
    ) -> Tuple[float, float, float, float]:
        """
        정규화 좌표 → 실제 NED 좌표 변환

        이미지 좌표계:
          cx=-1 → 이미지 왼쪽  → NED Y_min
          cx=+1 → 이미지 오른쪽 → NED Y_max
          cy=-1 → 이미지 위    → NED Z_min (더 높은 고도, 더 음수)
          cy=+1 → 이미지 아래  → NED Z_max (낮은 고도, 덜 음수)

        반환: (y_center, z_center, y_half, z_half) 단위: m
        """
        y_range = y_max - y_min          # 양수
        z_range = abs(z_max - z_min)     # 양수 (z_min이 더 음수)

        # cx [-1,1] → Y 실제 좌표
        y_center = y_min + (self.cx + 1.0) / 2.0 * y_range

        # cy [-1,1] → Z 실제 좌표
        # cy=-1 → z_min (높은 고도), cy=+1 → z_max (낮은 고도)
        z_center = z_min + (self.cy + 1.0) / 2.0 * z_range

        y_half = self.w / 2.0 * y_range
        z_half = self.h / 2.0 * z_range

        return y_center, z_center, y_half, z_half


@dataclass
class WorldExclusion:
    """실제 NED 좌표로 변환된 금지구역"""
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
        """해당 Z 고도가 이 금지구역에 걸치는지"""
        return (self.z_lo - margin) <= z <= (self.z_hi + margin)

    def y_interval(self, margin: float = 0.0) -> Tuple[float, float]:
        """금지구역의 Y 구간 (여유 포함)"""
        return self.y_lo - margin, self.y_hi + margin


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
#  핵심 알고리즘
# ══════════════════════════════════════════════════════════

def build_world_exclusions(
    exclusion_zones: List[ExclusionZone],
    y_min: float, y_max: float,
    z_min: float, z_max: float,
) -> List[WorldExclusion]:
    """ExclusionZone 리스트 → WorldExclusion 리스트 변환"""
    result = []
    for ez in exclusion_zones:
        yc, zc, yh, zh = ez.to_world(y_min, y_max, z_min, z_max)
        result.append(WorldExclusion(
            cls=ez.cls, y_center=yc, z_center=zc, y_half=yh, z_half=zh))
    return result


def get_row_segments(
    y_min:       float,
    y_max:       float,
    current_z:   float,
    exclusions:  List[WorldExclusion],
    default_speed: float,
    slow_speed:    float,
    margin:        float,
) -> List[Segment]:
    """
    한 행(Z 고도 고정)에서 금지구역을 피한 페인트 구간 리스트 생성

    알고리즘:
      1. 해당 Z에 걸치는 금지구역의 Y 구간 수집
      2. Y 전체 [y_min, y_max]에서 금지구역 Y 구간을 빼냄
      3. 남은 구간 = paint_on=True, 금지구역 구간 = paint_on=False
      4. 경계 진입/진출 margin 구간 = slow_speed

    반환: y_start ~ y_end 순서로 정렬된 Segment 리스트
    """
    # 해당 Z에 걸치는 금지구역만 필터
    active = [ez for ez in exclusions if ez.contains_z(current_z, margin=0.0)]

    if not active:
        # 금지구역 없음 → 전체 구간 페인트
        return [Segment(y_min, y_max, paint_on=True, speed=default_speed)]

    # 금지구역 Y 구간 수집 (여유 없이)
    excl_intervals = [(ez.y_lo, ez.y_hi) for ez in active]
    # 감속 구간 (margin 포함)
    slow_intervals = [(ez.y_lo - margin, ez.y_hi + margin) for ez in active]

    # 이벤트 포인트 수집 (y_min, y_max 포함)
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

        # 금지구역 내부인지
        in_excl = any(lo <= seg_mid <= hi for lo, hi in excl_intervals)
        # 감속 구간인지
        in_slow = any(lo <= seg_mid <= hi for lo, hi in slow_intervals)

        if in_excl:
            paint_on = False
            speed    = 0.0
        elif in_slow:
            paint_on = True
            speed    = slow_speed
        else:
            paint_on = True
            speed    = default_speed

        segments.append(Segment(seg_lo, seg_hi, paint_on, speed))

    return segments if segments else [Segment(y_min, y_max, True, default_speed)]


def segments_to_waypoints(
    segments:  List[Segment],
    wall_x:    float,
    current_z: float,
    direction: int,        # 1: 좌→우, -1: 우→좌
) -> List[Waypoint3D]:
    """
    Segment 리스트 → Waypoint3D 리스트 변환

    direction에 따라 구간 순서 반전.
    각 구간의 시작점 + 끝점 2개 웨이포인트 생성.
    연속된 같은 paint_on 구간은 중간 경계점만 추가.
    """
    if direction == -1:
        # 우→좌: 구간 반전 + 각 구간 내부도 반전
        segments = [
            Segment(s.y_end, s.y_start, s.paint_on, s.speed)
            for s in reversed(segments)
        ]

    waypoints = []
    for i, seg in enumerate(segments):
        # 첫 구간은 시작점 추가
        if i == 0:
            waypoints.append(Waypoint3D(
                x=wall_x, y=round(seg.y_start, 4), z=round(current_z, 4),
                paint_on=seg.paint_on, speed=seg.speed))

        # 끝점 추가
        waypoints.append(Waypoint3D(
            x=wall_x, y=round(seg.y_end, 4), z=round(current_z, 4),
            paint_on=seg.paint_on, speed=seg.speed))

    return waypoints


def generate_zigzag_waypoints(
    points:          List[dict],
    wall_x:          float,
    step:            float,
    exclusion_zones: List[ExclusionZone],
    default_speed:   float = 0.5,
    slow_speed:      float = 0.2,
) -> List[Waypoint3D]:
    """
    4꼭짓점 + 금지 구역 → 3D 지그재그 웨이포인트 생성 (고도화 버전)

    주요 개선점:
      - 행 내 구간 분할: 금지구역 Y 범위를 행에서 정확히 빼냄
      - 경계 감속: 금지구역 진입/진출 시 slow_speed 구간 자동 삽입
      - 좌표 변환 버그 수정: cy 이미지 좌표 → NED Z 정확한 매핑
    """
    all_y = [p['y'] for p in points]
    all_z = [p['z'] for p in points]

    y_min = min(all_y);  y_max = max(all_y)
    z_min = min(all_z);  z_max = max(all_z)
    # NED: z_min이 더 음수 = 더 높은 고도, z_max가 낮은 고도

    if (y_max - y_min) < 0.1 or abs(z_max - z_min) < 0.1:
        raise ValueError(
            f'도색 영역 너무 작음: Y={y_max-y_min:.2f}m  Z={abs(z_max-z_min):.2f}m')

    # 금지구역 → 실제 NED 좌표
    world_exclusions = build_world_exclusions(
        exclusion_zones, y_min, y_max, z_min, z_max)

    margin = step * 0.5   # 금지구역 경계 여유 (감속 구간 너비)

    waypoints: List[Waypoint3D] = []
    current_z = z_max   # 낮은 고도(덜 음수)부터 시작 → 높은 고도(더 음수)로
    direction = 1       # 1: y_min→y_max, -1: y_max→y_min

    while current_z >= z_min - 1e-6:
        segments = get_row_segments(
            y_min, y_max, current_z,
            world_exclusions,
            default_speed, slow_speed, margin,
        )
        row_wps = segments_to_waypoints(segments, wall_x, current_z, direction)
        waypoints.extend(row_wps)

        current_z = round(current_z - step, 6)
        direction *= -1

    return waypoints


def waypoints_to_path(
    waypoints: List[Waypoint3D],
    frame_id:  str = 'map',
) -> Path:
    """Waypoint3D 리스트 → nav_msgs/Path 변환"""
    path = Path()
    path.header.frame_id = frame_id

    for wp in waypoints:
        ps = PoseStamped()
        ps.header.frame_id    = frame_id
        ps.pose.position.x    = float(wp.x)
        ps.pose.position.y    = float(wp.y)
        ps.pose.position.z    = float(wp.z)
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)

    return path


def waypoints_to_paint_waypoints(
    waypoints: List[Waypoint3D],
) -> List[PaintWaypoint]:
    """Waypoint3D 리스트 → PaintWaypoint[] 변환"""
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

    DEFAULT_STEP    = 0.4
    DEFAULT_WALL_X  = 2.5
    DEFAULT_SPEED   = 0.5   # m/s 기본 도색 속도
    SLOW_SPEED      = 0.2   # m/s 금지구역 경계 감속

    def __init__(self):
        super().__init__('path_planner_node')

        # ── 파라미터 ──
        self.declare_parameter('default_step',   self.DEFAULT_STEP)
        self.declare_parameter('default_wall_x', self.DEFAULT_WALL_X)
        self.declare_parameter('default_speed',  self.DEFAULT_SPEED)
        self.declare_parameter('slow_speed',     self.SLOW_SPEED)
        self.declare_parameter('frame_id',       'map')

        self.default_step   = self.get_parameter('default_step').value
        self.default_wall_x = self.get_parameter('default_wall_x').value
        self.default_speed  = self.get_parameter('default_speed').value
        self.slow_speed     = self.get_parameter('slow_speed').value
        self.frame_id       = self.get_parameter('frame_id').value

        # ── 내부 상태 ──
        self.paint_zone_raw:    Optional[str]       = None
        self.paint_zone_points: Optional[List]      = None
        self.exclusion_zones:   List[ExclusionZone] = []

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
            '  /planner/generate_path 서비스 대기 중\n'
            f'  기본 step={self.default_step}m  '
            f'wall_x={self.default_wall_x}m  '
            f'speed={self.default_speed}m/s\n'
            '══════════════════════════════════════'
        )

    # ══════════════════════════════════════════════════════════
    #  구역 정보 수신
    # ══════════════════════════════════════════════════════════

    def _on_paint_zone(self, msg: String):
        """마스터/facade_area_node → 도색 구역 수신"""
        try:
            data   = json.loads(msg.data)
            points = data.get('main_area', data.get('points'))

            if not points or len(points) != 4:
                raise ValueError(
                    f'꼭짓점 4개 필요 (받음: {len(points or [])}개)')

            self.paint_zone_points = points
            self.paint_zone_raw    = msg.data

            ys = [p['y'] for p in points]
            zs = [p['z'] for p in points]
            self.get_logger().info(
                f'✅ 도색 구역 수신\n'
                f'  Y: {min(ys):.2f} ~ {max(ys):.2f}m\n'
                f'  Z: {min(zs):.2f} ~ {max(zs):.2f}m\n'
                f'  wall_x: {data.get("wall_x", self.default_wall_x):.2f}m'
            )
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f'paint_zone 파싱 오류: {e}')
            self.paint_zone_points = None

    def _on_exclusion_zones(self, msg: String):
        """마스터 → 금지 구역 수신"""
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
                f'✅ 금지 구역 수신: {len(self.exclusion_zones)}개\n' +
                '\n'.join(
                    f'  [{ez.cls}] cx={ez.cx:+.2f} cy={ez.cy:+.2f} '
                    f'w={ez.w:.2f} h={ez.h:.2f}'
                    for ez in self.exclusion_zones
                )
            )
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

        if self.paint_zone_points is None:
            response.success = False
            response.message = '도색 구역 없음 → /planner/paint_zone 먼저 수신 필요'
            self.get_logger().error(response.message)
            return response

        step   = request.step   if request.step   > 0.0 else self.default_step
        wall_x = request.wall_x if request.wall_x > 0.0 else self._get_wall_x()

        self.get_logger().info(
            f'경로 생성 시작\n'
            f'  step={step}m  wall_x={wall_x}m  '
            f'speed={self.default_speed}m/s  slow={self.slow_speed}m/s'
        )

        try:
            waypoints = generate_zigzag_waypoints(
                points          = self.paint_zone_points,
                wall_x          = wall_x,
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
            f'궤적 생성 완료: 총 {len(waypoints)}개  '
            f'(페인트ON={paint_cnt}  스킵={skip_cnt}  감속={slow_cnt})'
        )

        self.get_logger().info(
            f'✅ {response.message}\n'
            f'  예상 거리: {self._estimate_dist(waypoints):.1f}m\n'
            f'  금지구역 스킵 구간: {skip_cnt}개'
        )
        return response

    # ══════════════════════════════════════════════════════════
    #  유틸
    # ══════════════════════════════════════════════════════════

    def _get_wall_x(self) -> float:
        if self.paint_zone_raw:
            try:
                return float(json.loads(self.paint_zone_raw)
                             .get('wall_x', self.default_wall_x))
            except Exception:
                pass
        return self.default_wall_x

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
