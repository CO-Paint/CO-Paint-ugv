import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
import tkinter as tk
from tkinter import ttk
import threading
import math

# 쿼터니언(w, x, y, z)을 오일러 각도(Roll, Pitch, Yaw)로 변환하는 헬퍼 함수
def quat_to_euler(w, x, y, z):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z)

class OdomMonitorNode(Node):
    def __init__(self):
        super().__init__('odom_monitor_gui_node')
        
        # 데이터 저장을 위한 딕셔너리
        self.data = {
            'fastlio': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'bridge': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'ekf2': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        }

        # 1. FAST-LIO 원본 (nav_msgs/Odometry)
        self.sub_fastlio = self.create_subscription(Odometry, '/Odometry', self.cb_fastlio, 10)
        
        # 2. 브릿지 입력 (px4_msgs/VehicleOdometry)
        self.sub_bridge = self.create_subscription(
            VehicleOdometry, 
            '/fmu/in/vehicle_visual_odometry', 
            self.cb_bridge, 
            qos_profile_sensor_data
        )
        
        # 3. 픽스호크 EKF2 최종 출력 (px4_msgs/VehicleOdometry)
        self.sub_ekf2 = self.create_subscription(
            VehicleOdometry, 
            '/fmu/out/vehicle_odometry', 
            self.cb_ekf2, 
            qos_profile_sensor_data
        )

    def cb_fastlio(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        r, p, y = quat_to_euler(q.w, q.x, q.y, q.z) # ROS 2 quaternion
        self.data['fastlio'] = {'x': pos.x, 'y': pos.y, 'z': pos.z, 'roll': r, 'pitch': p, 'yaw': y}

    def cb_bridge(self, msg):
        pos = msg.position
        q = msg.q
        r, p, y = quat_to_euler(q[0], q[1], q[2], q[3]) # PX4 quaternion (w, x, y, z)
        self.data['bridge'] = {'x': pos[0], 'y': pos[1], 'z': pos[2], 'roll': r, 'pitch': p, 'yaw': y}

    def cb_ekf2(self, msg):
        pos = msg.position
        q = msg.q
        r, p, y = quat_to_euler(q[0], q[1], q[2], q[3])
        self.data['ekf2'] = {'x': pos[0], 'y': pos[1], 'z': pos[2], 'roll': r, 'pitch': p, 'yaw': y}

class DashboardGUI:
    def __init__(self, ros_node):
        self.node = ros_node
        self.root = tk.Tk()
        self.root.title("PX4 Data Pipeline Monitor")
        self.root.geometry("850x250")
        
        # 스타일 설정
        style = ttk.Style()
        style.configure("Title.TLabel", font=("Helvetica", 12, "bold"))
        style.configure("Data.TLabel", font=("Consolas", 11))

        self.labels = {}
        
        # 3개의 섹터 생성 (1: FAST-LIO, 2: Bridge IN, 3: EKF2 OUT)
        sectors = [
            ("1. FAST-LIO 원본\n(/Odometry) [ENU]", 'fastlio'),
            ("2. Bridge 변환\n(/fmu/in/visual_odom) [NED]", 'bridge'),
            ("3. EKF2 융합 완료\n(/fmu/out/odom) [NED]", 'ekf2')
        ]

        for i, (title, key) in enumerate(sectors):
            frame = ttk.Frame(self.root, padding=10, borderwidth=2, relief="groove")
            frame.grid(row=0, column=i, padx=5, pady=5, sticky="nsew")
            
            ttk.Label(frame, text=title, style="Title.TLabel", justify="center").pack(pady=(0, 10))
            
            # 데이터를 표시할 Label 딕셔너리 생성
            self.labels[key] = ttk.Label(frame, text="Waiting for data...", style="Data.TLabel", justify="left")
            self.labels[key].pack()

        # 화면 갱신 루프 시작 (100ms 마다)
        self.update_gui()

    def update_gui(self):
        for key in self.labels.keys():
            d = self.node.data[key]
            text = (
                f"Position (m)\n"
                f" X: {d['x']: 7.3f}\n"
                f" Y: {d['y']: 7.3f}\n"
                f" Z: {d['z']: 7.3f}\n\n"
                f"Attitude (deg)\n"
                f" Roll : {d['roll']: 7.2f}\n"
                f" Pitch: {d['pitch']: 7.2f}\n"
                f" Yaw  : {d['yaw']: 7.2f}"
            )
            self.labels[key].config(text=text)
            
        self.root.after(100, self.update_gui)

def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = OdomMonitorNode()
    
    # ROS 2 노드를 백그라운드 스레드에서 실행
    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()
    
    # 메인 스레드에서는 GUI 실행
    app = DashboardGUI(node)
    app.root.mainloop()
    
    # GUI 창이 닫히면 종료
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()