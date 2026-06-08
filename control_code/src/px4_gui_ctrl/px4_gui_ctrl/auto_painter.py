import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleOdometry
import math
import json


class AutoPainterNode(Node):
    def __init__(self):
        super().__init__('auto_painter_node')

        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        trajectory_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # PX4 setpoint는 직접 publish하지 않고 flight_control_node 입력만 publish
        self.paint_waypoints_pub = self.create_publisher(
            String, '/flight_control/paint_waypoints', trajectory_qos)
        self.mission_cmd_pub = self.create_publisher(
            String, '/flight_control/mission_cmd', command_qos)
        self.create_subscription(
            String, '/auto_painter/command', self.command_callback, command_qos)

        # 서브스크라이버 (현재 위치 파악)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, 10)

        # 현재 드론 위치
        self.current_pos = [0.0, 0.0, 0.0]

        # 도색 궤적(웨이포인트) 생성!
        self.waypoints = self.generate_paint_path(
            wall_distance=2.5,   # 드론과 벽 사이의 거리 (X축, 2.5m 앞)
            y_start=-1.5,        # 도색 시작점 (왼쪽 1.5m)
            y_end=1.5,           # 도색 끝점 (오른쪽 1.5m)
            z_bottom=-1.0,       # 도색 하단 높이 (1m 고도, NED 좌표계라 -)
            z_top=-3.0,          # 도색 상단 높이 (3m 고도, NED 좌표계라 -)
            z_step=-0.5          # 한 번 왕복 후 올라갈 높이 (0.5m씩 상승)
        )
        self.current_wp_index = 0
        self.is_painting = False
        self.waypoints_published = False
        self.create_timer(1.0, self.publish_waypoints_once)

        self.get_logger().info("🎨 자동 도색 궤적 준비 완료! 이륙 명령을 대기합니다.")

    def generate_paint_path(self, wall_distance, y_start, y_end, z_bottom, z_top, z_step):
        """ ㄹ자(지그재그) 비행 궤적을 계산하여 리스트로 반환 """
        waypoints = []
        current_z = z_bottom
        direction = 1 # 1: 왼쪽->오른쪽, -1: 오른쪽->왼쪽

        # 이륙 후 벽 앞의 첫 시작점 대기 위치
        waypoints.append([wall_distance, y_start, current_z])

        while current_z >= z_top: # Z는 위로 갈수록 음수이므로 >= 사용
            if direction == 1:
                waypoints.append([wall_distance, y_start, current_z])
                waypoints.append([wall_distance, y_end, current_z])
            else:
                waypoints.append([wall_distance, y_end, current_z])
                waypoints.append([wall_distance, y_start, current_z])
            
            current_z += z_step # Z축 위로 상승 (z_step이 음수임)
            direction *= -1     # 방향 전환
        
        # 도색이 끝나면 최초 위치로 복귀
        waypoints.append([0.0, 0.0, -1.0]) 
        return waypoints

    def publish_waypoints_once(self):
        if self.waypoints_published:
            return
        payload = [
            {'x': float(x), 'y': float(y), 'z': float(z), 'paint_on': True}
            for x, y, z in self.waypoints
        ]
        self.paint_waypoints_pub.publish(String(data=json.dumps(payload)))
        self.waypoints_published = True
        self.get_logger().info(
            f"도색 waypoint {len(payload)}개를 flight_control_node로 전달했습니다.")

    def command_callback(self, msg):
        command = msg.data.strip().upper()
        if command in ('START', 'PAINT'):
            self.publish_waypoints_once()
            self.is_painting = True
            self.current_wp_index = 0
            self.mission_cmd_pub.publish(String(data='PAINT'))
            self.get_logger().info("PAINT 명령을 flight_control_node로 전달했습니다.")
        elif command == 'STOP':
            self.is_painting = False
            self.mission_cmd_pub.publish(String(data='START_AUTO_LAND'))
            self.get_logger().info("START_AUTO_LAND 명령을 flight_control_node로 전달했습니다.")
        else:
            self.get_logger().warn(f"Unknown auto_painter command ignored: {command}")

    def odom_callback(self, msg):
        """ 드론의 현재 위치를 지속적으로 업데이트 """
        self.current_pos = [msg.position[0], msg.position[1], msg.position[2]]
        
        # 도색 중이고, 아직 갈 길이 남았다면
        if self.is_painting and self.current_wp_index < len(self.waypoints):
            target = self.waypoints[self.current_wp_index]
            
            # 현재 위치와 목표 웨이포인트 사이의 거리 계산 (유클리디안 거리)
            dist = math.sqrt(
                (self.current_pos[0] - target[0])**2 +
                (self.current_pos[1] - target[1])**2 +
                (self.current_pos[2] - target[2])**2
            )
            
            # 오차 0.2m 이내로 도달했으면 다음 목표로 인덱스 이동!
            if dist < 0.2:
                self.get_logger().info(f"✅ 웨이포인트 {self.current_wp_index} 도달! 다음 구역으로 이동합니다.")
                self.current_wp_index += 1

                if self.current_wp_index >= len(self.waypoints):
                    self.get_logger().info("🎉 도색 작업이 모두 완료되었습니다!")
                    self.is_painting = False

def main(args=None):
    rclpy.init(args=args)
    node = AutoPainterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
