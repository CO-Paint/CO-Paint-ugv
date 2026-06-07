import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import math

# ROS 2 표준 메시지 및 PX4 메시지 임포트
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

class FastLioToPx4Bridge(Node):
    def __init__(self):
        super().__init__('fastlio_to_px4_bridge')

        # QoS 프로파일 설정 (PX4 uXRCE-DDS는 보통 SensorDataQoS를 선호함)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publisher: PX4로 VehicleOdometry 전송
        self.odom_pub = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            qos_profile
        )

        # Subscriber: FAST-LIO에서 Odometry 수신 (FAST-LIO 설정에 따라 토픽명이 다를 수 있음)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',  # FAST-LIO 출력 토픽 확인 
            self.odom_callback,
            qos_profile
        )

        self.get_logger().info('FAST-LIO to PX4 Bridge Node Started. (ENU -> NED mapping active)')

    def odom_callback(self, msg: Odometry):
        px4_msg = VehicleOdometry()

        # 1. 타임스탬프 설정 (마이크로초 단위)
        # PX4는 timestamp와 timestamp_sample 두 개를 사용합니다.
        #now = self.get_clock().now().nanoseconds
        #px4_msg.timestamp = int(now / 1000)
        #px4_msg.timestamp_sample = int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1000)
        px4_msg.timestamp = 0
        px4_msg.timestamp_sample = 0

        # 2. 기준 프레임 설정 (PX4 1.16 기준 NED 프레임 = 1)
        px4_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        px4_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        # 3. 위치 (Position) 변환: ENU -> NED
        # X_ned = Y_enu, Y_ned = X_enu, Z_ned = -Z_enu
        px4_msg.position[0] = msg.pose.pose.position.y
        px4_msg.position[1] = msg.pose.pose.position.x
        px4_msg.position[2] = -msg.pose.pose.position.z

        # 4. 자세 (Quaternion) 변환: ENU -> NED
        # q_ned = [q_enu.w, q_enu.y, q_enu.x, -q_enu.z]
        px4_msg.q[0] = msg.pose.pose.orientation.w
        px4_msg.q[1] = msg.pose.pose.orientation.y
        px4_msg.q[2] = msg.pose.pose.orientation.x
        px4_msg.q[3] = -msg.pose.pose.orientation.z

        # 5. 선속도 (Linear Velocity) 변환: ENU -> NED
        px4_msg.velocity[0] = msg.twist.twist.linear.y
        px4_msg.velocity[1] = msg.twist.twist.linear.x
        px4_msg.velocity[2] = -msg.twist.twist.linear.z

        # 6. 각속도 (Angular Velocity) 변환: FLU -> FRD (드론 Body 프레임 기준)
        # Roll_frd = Roll_flu, Pitch_frd = -Pitch_flu, Yaw_frd = -Yaw_flu
        px4_msg.angular_velocity[0] = msg.twist.twist.angular.x
        px4_msg.angular_velocity[1] = -msg.twist.twist.angular.y
        px4_msg.angular_velocity[2] = -msg.twist.twist.angular.z

        # 7. 분산 (Variance) 설정
        # NaN으로 설정하면 PX4의 EKF2 파라미터(EKF2_EVP_NOISE 등) 값을 우선적으로 사용합니다.
        # 만약 FAST-LIO의 Covariance를 활용하고 싶다면 매핑해주어야 합니다.
        px4_msg.position_variance = [math.nan, math.nan, math.nan]
        px4_msg.orientation_variance = [math.nan, math.nan, math.nan]
        px4_msg.velocity_variance = [math.nan, math.nan, math.nan]

        # 메시지 발행
        self.odom_pub.publish(px4_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FastLioToPx4Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()