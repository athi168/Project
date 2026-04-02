# import sys
# import time
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# from PyQt5.QtWidgets import QApplication, QMainWindow
# from home import Ui_MainWindow  # giao diện đã tạo bằng pyuic5

# class AGVController(Node):
#     def __init__(self):
#         super().__init__('agv_controller')
#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

#     def send_velocity(self, linear=0.0, angular=0.0, duration=0):
#         twist = Twist()
#         twist.linear.x = linear
#         twist.angular.z = angular
#         self.publisher_.publish(twist)
#         self.get_logger().info(f'Đã gửi vận tốc: linear={linear}, angular={angular}')
#         time.sleep(duration)

#         # Dừng robot sau khi hoàn tất hành động
#         stop = Twist()
#         self.publisher_.publish(stop)
#         self.get_logger().info('Đã dừng robot.')

# class MainWindow(QMainWindow, Ui_MainWindow):
#     def __init__(self, ros_node):
#         super().__init__()
#         self.setupUi(self)
#         self.ros_node = ros_node
#         self.counter = 0

#         # Gắn sự kiện cho nút push_mode
#         self.push_mode.clicked.connect(self.handle_push_mode)

#     def handle_push_mode(self):
#         self.counter += 1
#         self.ros_node.get_logger().info(f'Nhấn push_mode lần thứ {self.counter}')

#         if self.counter == 1:
#             # Đi thẳng trong 3 giây
#             self.ros_node.send_velocity(linear=0.3, angular=0.0, duration=3)

#         elif self.counter == 2:
#             # Quay trái tại chỗ 360 độ
#                 self.ros_node.send_velocity(linear=0.0, angular=1.0, duration=2)
#         elif self.counter == 3:
#             # Quay phải tại chỗ 360 độ
#             self.ros_node.send_velocity(linear=0.0, angular=-1.0, duration=2)

#         elif self.counter == 4:
#             # Đi lùi trong 3 giây và reset đếm
#             self.ros_node.send_velocity(linear=-0.2, angular=0.0, duration=3)
#             self.counter = 0  # Reset đếm

# def main():
#     rclpy.init()
#     ros_node = AGVController()

#     app = QApplication(sys.argv)
#     window = MainWindow(ros_node)
#     window.show()

#     # # Giữ ROS chạy bằng timer (để ROS không bị block trong Qt)
#     # ros_node.create_timer(0.1, lambda: None)

#     try:
#         sys.exit(app.exec_())
#     finally:
#         ros_node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from PyQt5.QtWidgets import QApplication, QMainWindow
from home import Ui_MainWindow  # giao diện đã tạo bằng pyuic5

class AGVController(Node):
    def __init__(self):
        super().__init__('agv_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def send_velocity(self, linear=0.0, angular=0.0, duration=0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        # Lặp lại việc publish trong suốt thời gian duration
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(twist)
            self.get_logger().info(f'Đã gửi vận tốc: linear={linear}, angular={angular}')
            time.sleep(0.1)  # Đảm bảo publish mỗi 100ms để giữ lệnh vận tốc ổn định

        # Dừng robot sau khi hoàn tất hành động
        stop = Twist()
        self.publisher_.publish(stop)
        self.get_logger().info('Đã dừng robot.')

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.setupUi(self)
        self.ros_node = ros_node
        self.counter = 0

        # Gắn sự kiện cho nút push_mode
        self.push_mode.clicked.connect(self.handle_push_mode)

    def handle_push_mode(self):
        self.counter += 1
        self.ros_node.get_logger().info(f'Nhấn push_mode lần thứ {self.counter}')

        if self.counter == 1:
            # Đi thẳng trong 3 giây
            self.ros_node.send_velocity(linear=0.3, angular=0.0, duration=2)

        elif self.counter == 2:
            # Quay trái tại chỗ 360 độ
            self.ros_node.send_velocity(linear=0.0, angular=1.0, duration=2.8)
        elif self.counter == 3:
            # Quay phải tại chỗ 360 độ
            self.ros_node.send_velocity(linear=0.0, angular=-1.0, duration=2.8)

        elif self.counter == 4:
            # Đi lùi trong 3 giây và reset đếm
            self.ros_node.send_velocity(linear=-0.2, angular=0.0, duration=2)
            self.counter = 0  # Reset đếm

def main():
    rclpy.init()
    ros_node = AGVController()

    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()

    # # Giữ ROS chạy bằng timer (để ROS không bị block trong Qt)
    # ros_node.create_timer(0.1, lambda: None)

    try:
        sys.exit(app.exec_())
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
