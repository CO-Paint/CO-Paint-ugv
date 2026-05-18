import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np

# ROS 2 표준 메시지
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class PidLandingControllerNode(Node):
    def __init__(self):
        super().__init__('pid_landing_controller_node')

        # 1. ROS 2 발행자(Publisher) 설정
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/landing_status', 10)

        # 2. 물리적 카메라 직접 연결 (이전 성공하셨던 V4L2 백엔드 및 1번 카메라 기준)
        self.cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다! 포트 번호(0 또는 1)를 확인하세요.")
            # 실패 시 0번으로 재시도
            self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # 3. ArUco 마커 설정 (구버전 호환 API)
        try:
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        except AttributeError:
            self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        self.parameters = aruco.DetectorParameters_create()
        self.marker_length = 0.055  # 마커 크기 5.5cm

        # 4. 카메라 캘리브레이션 데이터 (임시 보정값)
        self.camera_matrix = np.array([
            [657.8, 0.0, 320.0], 
            [0.0, 657.8, 240.0], 
            [0.0, 0.0, 1.0]
        ], dtype=float)
        self.dist_coeffs = np.zeros((4,1))

        # 5. PID 제어기 게인 및 변수 (X: 전후, Y: 좌우)
        self.kp = 1.2
        self.ki = 0.01
        self.kd = 0.5

        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

        # 6. 메인 처리 루프 생성 (30Hz 주기로 카메라 읽기 및 제어 수행)
        self.timer_period = 1.0 / 30.0  # 약 0.033초
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info("비전 + PID 제어 노드 실행 중... GUI 디버그 창을 확인하세요.")

    def control_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("카메라 프레임을 수신할 수 없습니다.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        cmd = Twist()
        status_msg = String()

        # 디버깅 패널을 위한 텍스트 정보 변수
        debug_info = []

        if ids is not None and len(ids) > 0:
            # PnP 3D 자세 추정
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            
            cam_x = tvecs[0][0][0]
            cam_y = tvecs[0][0][1]
            cam_z = tvecs[0][0][2]

            # --- 시점 역전 좌표 매핑 (카메라 -> UGV Base) ---
            error_x = cam_y  # 전후 오차
            error_y = -cam_x  # 좌우 오차
            drone_alt = cam_z # 고도

            # --- PID 계산 ---
            self.integral_x += error_x * self.timer_period
            deriv_x = (error_x - self.prev_error_x) / self.timer_period
            output_x = (self.kp * error_x) + (self.ki * self.integral_x) + (self.kd * deriv_x)
            self.prev_error_x = error_x

            self.integral_y += error_y * self.timer_period
            deriv_y = (error_y - self.prev_error_y) / self.timer_period
            output_y = (self.kp * error_y) + (self.ki * self.integral_y) + (self.kd * deriv_y)
            self.prev_error_y = error_y

            # --- 제어 명령 판별 ---
            if abs(error_x) < 0.03 and abs(error_y) < 0.03:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                state_str = "STATE: [ LOCKED ON ] - Ready to Land"
                color = (0, 255, 255) # 노란색
            else:
                cmd.linear.x = float(np.clip(output_x, -0.5, 0.5))
                cmd.linear.y = float(np.clip(output_y, -0.5, 0.5))
                state_str = "STATE: [ TRACKING ]"
                color = (0, 255, 0) # 초록색

            status_msg.data = f"{state_str} | alt: {drone_alt:.2f}m"
            
            # 디버깅 텍스트 준비
            debug_info.append(state_str)
            debug_info.append(f"Drone Alt (Z) : {drone_alt:.3f} m")
            debug_info.append(f"Error X (Fwd) : {error_x*100:+.1f} cm  ->  cmd_vel.x : {cmd.linear.x:+.2f} m/s")
            debug_info.append(f"Error Y (Lat) : {error_y*100:+.1f} cm  ->  cmd_vel.y : {cmd.linear.y:+.2f} m/s")

            # 영상에 마커 및 3D 축 그리기
            aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[0], tvecs[0], 0.1)

        else:
            # 마커를 놓쳤을 때 정지
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            status_msg.data = "STATE: [ SEARCHING ]"
            color = (0, 0, 255) # 빨간색
            
            self.integral_x = 0.0
            self.integral_y = 0.0

            debug_info.append("STATE: [ SEARCHING MARKER ]")
            debug_info.append("UGV STOPPED (0.0 m/s)")

        # ROS 2 토픽 발행
        self.cmd_pub.publish(cmd)
        self.status_pub.publish(status_msg)

        # =========================================================
        # 📺 직관적인 GUI 디버그 오버레이 출력
        # =========================================================
        # 배경 반투명 박스 그리기 (텍스트 가독성 향상)
        overlay = frame.copy()
        cv2.rectangle(overlay, (5, 5), (635, 130), (0, 0, 0), -1)
        frame = cv2.addWeighted(overlay, 0.6, frame, 0.4, 0)

        # 텍스트 렌더링
        y_offset = 30
        for text in debug_info:
            cv2.putText(frame, text, (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color if "STATE" in text else (255, 255, 255), 2)
            y_offset += 25

        cv2.imshow("ROS 2 Auto Landing Debug GUI", frame)
        
        # 'q' 키를 누르면 안전하게 종료되도록 훅(Hook) 삽입
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("'q' 입력 감지. 노드를 종료합니다.")
            rclpy.shutdown()

    def destroy_node(self):
        # 노드 종료 시 카메라 자원 및 창 해제
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PidLandingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C 입력 감지. 안전하게 종료합니다.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()