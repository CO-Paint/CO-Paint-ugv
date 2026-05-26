"""
path_planner_node.py  ─  CO-PAINT Path Planner Node
================================================================
[역할]
  마스터 노드의 /planner/generate_path 서비스 요청을 받아
  도색 구역 + 금지 구역 정보로 3D 지그재그 궤적을 생성.

[입력 - 토픽 (TRANSIENT_LOCAL, 마스터가 미리 발행)]
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

[exclusion_zones JSON 형식]
  [
    {"class": "window",  "cx": 0.3, "cy": -0.1, "w": 0.25, "h": 0.4},
    {"class": "balcony", "cx": 0.3, "cy":  0.5, "w": 0.5,  "h": 0.2}
  ]
  cx, cy : 정규화 중심 [-1, 1]  (bbox_detection_node 출력 기준)
  w, h   : 정규화 크기  [0, 1]

[NED 좌표계]
  X = North (벽 방향)
  Y = East  (좌우)
  Z = Down  (음수 = 위)

[알고리즘]
  1. 4꼭짓점 → Y/Z 범위 추출
  2. Z_max(낮은 고도) → Z_min(높은 고도) 방향으로 step씩 행 생성
  3. 각 행: 좌→우 / 우→좌 교대 (지그재그)
  4. 각 웨이포인트 → 금지구역과 겹치는지 판단 → paint_on 설정
  5. nav_msgs/Path + PaintWaypoint[] 동시 반환
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import json
import math
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

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
    정규화 좌표 기반 금지 구역
    cx, cy: 이미지 중심 기준 정규화 [-1, 1]
    w, h:   정규화 크기 [0, 1]
    """
    cls:  str
    cx:   float   # 정규화 Y 중심
    cy:   float   # 정규화 Z 중심
    w:    float   # 정규화 Y 폭
    h:    float   # 정규화 Z 높이

    def to_world(self, y_min: float, y_max: float,
                 z_min: float, z_max: float) -> Tuple[float, float, float, float]:
        """
        정규화 좌표 → 실제 NED 좌표 (m) 변환
        반환: (yz_center, z_center, y_half, z_half)
        """
        y_range = y_max - y_min
        z_range = z_max - z_min   # 음수 범위

        # cx [-1,1] → Y 실제 좌표
        y_center = y_min + (self.cx + 1.0) / 2.0 * y_range
        # cy [-1,1] → Z 실제 좌표 (이미지 위 = NED 위 = 더 음수)
        z_center = z_max + (-self.cy + 1.0) / 2.0 * z_range

        y_half = self.w / 2.0 * y_range
        z_half = self.h / 2.0 * abs(z_range)

        return y_center, z_center, y_half, z_half


@dataclass
class Waypoint3D:
    x:        float
    y:        float
    z:        float
    paint_on: bool  = True
    speed:    float = 0.0


# ══════════════════════════════════════════════════════════
#  핵심 알고리즘
# ══════════════════════════════════════════════════════════

def generate_zigzag_waypoints(
    points:          List[dict],
    wall_x:          float,
    step:            float,
    exclusion_zones: List[ExclusionZone],
    default_speed:   float = 0.5,
    slow_speed:      float = 0.2,
) -> List[Waypoint3D]:
    """
    4꼭짓점 + 금지 구역 → 3D 지그재그 웨이포인트 생성

    paint_on 결정 기준:
      해당 웨이포인트의 Y, Z 좌표가 금지구역 범위 안에 있으면 False
      금지구역 경계 ±margin 이내면 slow_speed 적용
    """
    all_y = [p['y'] for p in points]
    all_z = [p['z'] for p in points]

    y_min = min(all_y);  y_max = max(all_y)
    z_min = min(all_z);  z_max = max(all_z)

    if (y_max - y_min) < 0.1 or abs(z_max - z_min) < 0.1:
        raise ValueError(
            f'도색 영역 너무 작음: Y={y_max-y_min:.2f}m  Z={abs(z_max-z_min):.2f}m')

    # 금지구역 → 실제 NED 좌표 변환
    exclusions_world = [
        (ez, *ez.to_world(y_min, y_max, z_min, z_max))
        for ez in exclusion_zones
    ]

    waypoints: List[Waypoint3D] = []
    current_z = z_max          # 낮은 고도부터 시작
    direction = 1              # 1: y_min→y_max, -1: y_max→y_min
    margin    = step * 0.5     # 금지구역 경계 여유

    while current_z >= z_min - 1e-6:
        row_y = [y_min, y_max] if direction == 1 else [y_max, y_min]

        for y in row_y:
            paint_on = True
            speed    = default_speed

            for ez, y_center, z_center, y_half, z_half in exclusions_world:
                # 금지구역 내부 판정
                in_y = abs(y - y_center) <= y_half
                in_z = abs(current_z - z_center) <= z_half

                if in_y and in_z:
                    paint_on = False
                    speed    = 0.0
                    break

                # 경계 근처 → 감속
                near_y = abs(y - y_center) <= y_half + margin
                near_z = abs(current_z - z_center) <= z_half + margin
                if near_y and near_z:
                    speed = slow_speed

            waypoints.append(Waypoint3D(
                x        = wall_x,
                y        = round(y, 4),
                z        = round(current_z, 4),
                paint_on = paint_on,
                speed    = speed,
            ))

        current_z -= step
        direction *= -1

    return waypoints


def waypoints_to_path(waypoints: List[Waypoint3D],
                      frame_id: str = 'map') -> Path:
    """Waypoint3D 리스트 → nav_msgs/Path 변환"""
    path        = Path()
    path.header.frame_id = frame_id

    for wp in waypoints:
        ps = PoseStamped()
        ps.header.frame_id          = frame_id
        ps.pose.position.x          = float(wp.x)
        ps.pose.position.y          = float(wp.y)
        ps.pose.position.z          = float(wp.z)
        ps.pose.orientation.w       = 1.0   # 기본 자세
        path.poses.append(ps)

    return path


def waypoints_to_paint_waypoints(
        waypoints: List[Waypoint3D]) -> List[PaintWaypoint]:
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
    SLOW_SPEED      = 0.2   # m/s 금지구역 근처 감속

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
        self.paint_zone_raw:    Optional[str]  = None   # JSON String
        self.exclusion_raw:     Optional[str]  = None   # JSON String
        self.paint_zone_points: Optional[List] = None   # 파싱된 4꼭짓점
        self.exclusion_zones:   List[ExclusionZone] = []

        # ── QoS ──
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── 구독: 마스터가 미리 발행한 구역 정보 ──
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
            f'  기본 step: {self.default_step}m  '
            f'wall_x: {self.default_wall_x}m\n'
            '══════════════════════════════════════'
        )

    # ══════════════════════════════════════════════════════════
    #  구역 정보 수신
    # ══════════════════════════════════════════════════════════

    def _on_paint_zone(self, msg: String):
        """마스터 → 도색 구역 4꼭짓점 수신"""
        try:
            data = json.loads(msg.data)
            self.paint_zone_points = data.get('main_area', data.get('points'))
            self.paint_zone_raw    = msg.data

            if not self.paint_zone_points or len(self.paint_zone_points) != 4:
                raise ValueError(
                    f'꼭짓점 4개 필요 (받음: {len(self.paint_zone_points or [])}개)')

            ys = [p['y'] for p in self.paint_zone_points]
            zs = [p['z'] for p in self.paint_zone_points]
            self.get_logger().info(
                f'✅ 도색 구역 수신\n'
                f'  Y: {min(ys):.2f} ~ {max(ys):.2f}m\n'
                f'  Z: {min(zs):.2f} ~ {max(zs):.2f}m'
            )
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f'paint_zone 파싱 오류: {e}')
            self.paint_zone_points = None

    def _on_exclusion_zones(self, msg: String):
        """마스터 → 금지 구역 리스트 수신"""
        try:
            data = json.loads(msg.data)
            self.exclusion_zones = [
                ExclusionZone(
                    cls = z.get('class', 'unknown'),
                    cx  = float(z['cx']),
                    cy  = float(z['cy']),
                    w   = float(z['w']),
                    h   = float(z['h']),
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
        """
        /planner/generate_path 서비스 핸들러

        Request.step    : 0이면 default_step 사용
        Request.wall_x  : 0이면 paint_zone JSON의 wall_x or default_wall_x 사용
        """
        self.get_logger().info('경로계획 서비스 요청 수신')

        # ── 입력 검증 ──
        if self.paint_zone_points is None:
            response.success = False
            response.message = '도색 구역 없음 → /planner/paint_zone 먼저 수신 필요'
            self.get_logger().error(response.message)
            return response

        # ── 파라미터 결정 ──
        step   = request.step   if request.step   > 0.0 else self.default_step
        wall_x = request.wall_x if request.wall_x > 0.0 else self._get_wall_x()

        self.get_logger().info(
            f'경로 생성 시작\n'
            f'  step: {step}m  wall_x: {wall_x}m\n'
            f'  금지구역: {len(self.exclusion_zones)}개'
        )

        # ── 궤적 생성 ──
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

        # ── 응답 구성 ──
        now = self.get_clock().now().to_msg()

        path             = waypoints_to_path(waypoints, self.frame_id)
        path.header.stamp = now

        paint_waypoints  = waypoints_to_paint_waypoints(waypoints)

        response.waypoints      = paint_waypoints
        response.path           = path
        response.waypoint_count = len(waypoints)
        response.success        = True
        response.message        = (
            f'궤적 생성 완료: {len(waypoints)}개 웨이포인트  '
            f'(paint_on={sum(w.paint_on for w in waypoints)}  '
            f'skip={sum(not w.paint_on for w in waypoints)})'
        )

        self.get_logger().info(
            f'✅ {response.message}\n'
            f'  예상 거리: {self._estimate_dist(waypoints):.1f}m'
        )
        return response

    # ══════════════════════════════════════════════════════════
    #  유틸
    # ══════════════════════════════════════════════════════════

    def _get_wall_x(self) -> float:
        """paint_zone JSON에서 wall_x 추출, 없으면 default 사용"""
        if self.paint_zone_raw:
            try:
                data = json.loads(self.paint_zone_raw)
                return float(data.get('wall_x', self.default_wall_x))
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
