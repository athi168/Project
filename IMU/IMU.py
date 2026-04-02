#!/usr/bin/env python3
import rclpy.time
import smbus
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Địa chỉ thanh ghi của MPU9150
PWR_MGMT_1   = 0x6B     #bật/khởi động cảm biến
SMPLRT_DIV   = 0x19     #Thiết lập tốc độ lấy mẫu
CONFIG       = 0x1A     #Thiết lập băng thông tần số thấp
GYRO_CONFIG  = 0x1B     #Thiết lập dải đo cảm biến gyro (250 - 2000 độ/s)
ACCEL_CONFIG = 0x1C     #Thiết lập dải đo gia tốc (+-2g đến +-16g)
INT_ENABLE   = 0x38     # Cho phép ngắt
ACCEL_XOUT_H = 0x3B     #Đọc giá trị gia tốc  trục x
ACCEL_YOUT_H = 0x3D     #Đọc giá trị gia tốc  trục y
ACCEL_ZOUT_H = 0x3F     #Đọc giá trị gia tốc  trục z
GYRO_XOUT_H  = 0x43     #Đọc góc quay trục x
GYRO_YOUT_H  = 0x45     #Đọc góc quay trục y
GYRO_ZOUT_H  = 0x47     #Đọc góc quay trục z
USER_CTRL    = 0x6A     #Bật I2C
INT_PIN_CFG  = 0x37     #Cho phép cấu hình pin ngắt và chế độ bypass I2C (để giao tiếp với từ kế).

# Địa chỉ và thanh ghi của AK8975 (từ kế)
MAG_ADDRESS  = 0x0C     # Địa chỉ I2C của AK8975
MAG_XOUT_L   = 0x03     # Đọc giá trị từ kế  X (Byte thấp)
MAG_XOUT_H   = 0x04     # Đọc giá trị từ kế  X (Byte cao)
MAG_YOUT_L   = 0x05     # Đọc giá trị từ kế  Y (Byte thấp)
MAG_YOUT_H   = 0x06     # Đọc giá trị từ kế  Y (Byte cao)
MAG_ZOUT_L   = 0x07     # Đọc giá trị từ kế  Z (Byte thấp)
MAG_ZOUT_H   = 0x08     # Đọc giá trị từ kế  Z (Byte cao)
MAG_CNTL     = 0x0A     # Địa chỉ thanh ghi điều khiển từ kế
MAG_ST1      = 0x02     # Địa chỉ thanh ghi trạng thái từ kế
DEVICE_ADDRESS = 0x68   # I2C address of MPU9150

class MPU9150_Driver(Node):
    def __init__(self):
        super().__init__("mpu9150_driver")

        # I2C Interface
        self.is_connected_ = False
        self.init_i2c()

        # ROS 2 Interface
        qos_profile_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_best_effort)
        self.mag_pub_ = self.create_publisher(MagneticField, "/imu/mag", qos_profile=qos_profile_best_effort)
        self.imu_msg_ = Imu()
        self.mag_msg_ = MagneticField()
        self.imu_msg_.header.frame_id = "imu_link"
        self.mag_msg_.header.frame_id = "imu_link"
        self.frequency_ = 0.01  # 100 Hz
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Orientation variables
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.last_time = self.get_clock().now()
        self.alpha = 0.98

        # Conversion factors (based on datasheet)
        self.accel_scale = 16384.0
        self.gyro_scale = 131.0
        self.mag_scale = 0.15

    def init_i2c(self):
        try:
            self.bus_ = smbus.SMBus(1)  # Khởi tạo giao tiếp I2C
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)  # Thiết lập tốc độ lấy mẫu
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 0)  # Bật cảm biến
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG, 0)  # Thiết lập băng thông tần số thấp
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 0)  # Thiết lập dải đo cảm biến gyro
            self.bus_.write_byte_data(DEVICE_ADDRESS, ACCEL_CONFIG, 0)  # Thiết lập dải đo gia tốc
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)  # Cho phép ngắt
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_PIN_CFG, 0x02)  # Cấu hình chế độ bypass I2C
            self.bus_.write_byte_data(DEVICE_ADDRESS, USER_CTRL, 0x00)  # Tắt chế độ master I2C
            self.bus_.write_byte_data(MAG_ADDRESS, MAG_CNTL, 0x01)  # Bật chế độ đo từ kế
            self.is_connected_ = True  # Đánh dấu kết nối thành công
        except OSError:
            self.is_connected_ = False  # Đánh dấu kết nối thất bại

    def read_raw_data(self, addr):
        high = self.bus_.read_byte_data(DEVICE_ADDRESS, addr)  # Đọc byte cao
        low = self.bus_.read_byte_data(DEVICE_ADDRESS, addr + 1)  # Đọc byte thấp
        value = (high << 8) | low  # Kết hợp byte cao và thấp thành giá trị 16-bit
        if value > 32768:
            value -= 65536  # Chuyển đổi giá trị âm nếu cần
        return value

    def read_mag_raw_data(self, low_addr, high_addr):
        high = self.bus_.read_byte_data(MAG_ADDRESS, high_addr)  # Đọc byte cao từ kế
        low = self.bus_.read_byte_data(MAG_ADDRESS, low_addr)  # Đọc byte thấp từ kế
        value = (high << 8) | low  # Kết hợp byte cao và thấp thành giá trị 16-bit
        if value > 32768:
            value -= 65536  # Chuyển đổi giá trị âm nếu cần
        return value

    def read_magnetometer(self):
        while not (self.bus_.read_byte_data(MAG_ADDRESS, MAG_ST1) & 0x01):  # Chờ dữ liệu sẵn sàng
            pass
        mag_x = self.read_mag_raw_data(MAG_XOUT_L, MAG_XOUT_H)  # Đọc giá trị từ kế trục X
        mag_y = self.read_mag_raw_data(MAG_YOUT_L, MAG_YOUT_H)  # Đọc giá trị từ kế trục Y
        mag_z = self.read_mag_raw_data(MAG_ZOUT_L, MAG_ZOUT_H)  # Đọc giá trị từ kế trục Z
        self.bus_.write_byte_data(MAG_ADDRESS, MAG_CNTL, 0x01)  # Bật chế độ đo từ kế
        return mag_x, mag_y, mag_z

    def timerCallback(self):
        try:
            if not self.is_connected_:
                self.init_i2c()  # Khởi tạo lại giao tiếp I2C nếu chưa kết nối

            # Đọc giá trị gia tốc từ cảm biến
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)

            # Đọc giá trị góc quay từ cảm biến
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)

            # Chuyển đổi giá trị gia tốc từ raw sang m/s^2
            self.imu_msg_.linear_acceleration.x = acc_x / self.accel_scale * 9.81
            self.imu_msg_.linear_acceleration.y = acc_y / self.accel_scale * 9.81
            self.imu_msg_.linear_acceleration.z = acc_z / self.accel_scale * 9.81

            # Chuyển đổi giá trị góc quay từ raw sang rad/s
            self.imu_msg_.angular_velocity.x = np.deg2rad(gyro_x / self.gyro_scale)
            self.imu_msg_.angular_velocity.y = np.deg2rad(gyro_y / self.gyro_scale)
            self.imu_msg_.angular_velocity.z = np.deg2rad(gyro_z / self.gyro_scale)

            # Đọc giá trị từ kế và chuyển đổi sang Tesla
            mag_x, mag_y, mag_z = self.read_magnetometer()
            self.mag_msg_.magnetic_field.x = mag_x * self.mag_scale * 1e-6
            self.mag_msg_.magnetic_field.y = mag_y * self.mag_scale * 1e-6
            self.mag_msg_.magnetic_field.z = mag_z * self.mag_scale * 1e-6

            # Tính toán thời gian delta (dt) giữa các lần callback
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds * 1e-9
            self.last_time = current_time

            # Tính toán quaternion từ gyro và acc
            gyro = np.array([
                self.imu_msg_.angular_velocity.x,
                self.imu_msg_.angular_velocity.y,
                self.imu_msg_.angular_velocity.z
            ])
            q_gyro = self.integrate_gyro(self.quaternion, gyro, dt)

            acc = np.array([
                self.imu_msg_.linear_acceleration.x,
                self.imu_msg_.linear_acceleration.y,
                self.imu_msg_.linear_acceleration.z
            ])
            q_acc = self.orientation_from_acc(acc)

            # Kết hợp giá trị từ gyro và acc bằng bộ lọc complementary filter
            self.quaternion = self.alpha * q_gyro + (1 - self.alpha) * q_acc
            norm = np.linalg.norm(self.quaternion)
            if norm > 1e-6:
                self.quaternion /= norm

            # Gán giá trị quaternion vào message IMU
            self.imu_msg_.orientation.w = self.quaternion[0]
            self.imu_msg_.orientation.x = -self.quaternion[1]
            self.imu_msg_.orientation.y = self.quaternion[2]
            self.imu_msg_.orientation.z = self.quaternion[3]

            # Tạo và gửi transform từ base_footprint đến imu_link
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = "base_footprint"
            t.child_frame_id = "imu_link"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = self.quaternion[0]
            t.transform.rotation.x = -self.quaternion[1]
            t.transform.rotation.y = -self.quaternion[2]
            t.transform.rotation.z = -self.quaternion[3]
            self.tf_broadcaster.sendTransform(t)

            # Cập nhật timestamp và publish các message IMU và từ kế
            self.imu_msg_.header.stamp = current_time.to_msg()
            self.mag_msg_.header.stamp = current_time.to_msg()
            self.imu_pub_.publish(self.imu_msg_)
            self.mag_pub_.publish(self.mag_msg_)
        except OSError:
            self.is_connected_ = False 

    # Tính đạo hàm quaternion từ giá trị gyro
    def integrate_gyro(self, q, gyro, dt): 
        angle = np.linalg.norm(gyro) * dt
        if angle > 1e-6:
            axis = gyro / np.linalg.norm(gyro)
            half_angle = angle / 2.0
            sin_half_angle = np.sin(half_angle)
            q_rot = np.array([
                np.cos(half_angle),
                axis[0] * sin_half_angle,
                axis[1] * sin_half_angle,
                axis[2] * sin_half_angle
            ])
            q = self.quaternion_multiply(q, q_rot)
        return q
    
    # Tính toán quaternion từ giá trị gia tốc
    def orientation_from_acc(self, acc):
        acc_norm = np.linalg.norm(acc)
        if acc_norm < 1e-6:
            return np.array([1.0, 0.0, 0.0, 0.0])
        acc = acc / acc_norm
        roll = np.arctan2(acc[1], acc[2])
        pitch = np.arctan2(-acc[0], np.sqrt(acc[1]**2 + acc[2]**2))
        cy = np.cos(roll * 0.5)
        sy = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        w = cy * cp
        x = sy * cp
        y = cy * sp
        z = sy * sp
        return np.array([w, x, y, z])

    # Kết hợp hai quaternion
    def quaternion_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        # Tính tích Hamilton của hai quaternion
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return np.array([w, x, y, z])

def main():
    rclpy.init()
    mpu9150_driver = MPU9150_Driver()
    rclpy.spin(mpu9150_driver)
    mpu9150_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
