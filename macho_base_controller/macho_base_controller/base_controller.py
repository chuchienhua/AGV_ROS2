import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped,Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
import serial
import math
from transformations import quaternion_from_euler
import sys
sys.path.append('/home/eddie/ros2_ws/src/macho_base_controller/macho_base_controller/cube_x3_messages.py')
from .cube_x3_messages import CubeX3FrameHeader, CubeX3ExtAckermannCtrlMsg


class TDKBaseController(Node):
    def __init__(self):
        super().__init__('macho_base_controller')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.emergency_stop_sub = self.create_subscription(Bool, 'macho_base_controller/emergency_stop', self.emergency_stop_callback, 10) # emergency_stop
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.port_name = "/dev/teensy4"
        self.serial_port = serial.Serial(port=self.port_name, baudrate=115200, timeout=0.1)
        self.emergency_stop_flag = False

        self.v = 0.0
        self.theta = 0.0
        self.turning_radius = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.mode_setting_msg = CubeX3FrameHeader(opcode=0x20, length=0x01)
        self.ackermann_ctrl_msg = CubeX3ExtAckermannCtrlMsg(0, 0, 0)

        self.connection_timer = self.create_timer(1.0, self.connection_check_callback)
        self.communication_timer = self.create_timer(0.010, self.communication_callback)
        self.odom_timer = self.create_timer(0.010, self.odom_callback)

    def emergency_stop_callback(self, msg):
        self.emergency_stop_flag = msg.data

    def connection_check_callback(self):
        if not self.serial_port.is_open:
            try:
                self.serial_port.open()
                self.get_logger().info("Connected to Teensy successfully!")
            except serial.SerialException as e:
                self.get_logger().error(f"Error when communicating to Teensy: {str(e)}")
                self.serial_port.close()

    def communication_callback(self):
        if not self.serial_port.is_open:
            return

        self.ackermann_ctrl_msg.velocity = int(self.v * 1000)  # Convert m/s to mm/s
        radius_mm = self.turning_radius * 1000

        # Adjust the radius values according to constraints
        if 240 <= abs(radius_mm) <= 10000:
            self.ackermann_ctrl_msg.radius = int(radius_mm)
        elif radius_mm > 10000 or radius_mm < -10000:
            self.ackermann_ctrl_msg.radius = 0
        elif radius_mm > 0 and radius_mm < 240:
            self.ackermann_ctrl_msg.radius = 240
        elif radius_mm < 0 and radius_mm > -240:
            self.ackermann_ctrl_msg.radius = -240
        else:
            self.ackermann_ctrl_msg.radius = 0

        self.ackermann_ctrl_msg.heartbeat = (self.ackermann_ctrl_msg.heartbeat + 1) % 256

        data_bytes = self.ackermann_ctrl_msg.to_bytes()

        try:
            self.serial_port.write(data_bytes)
            # self.get_logger().info('Data sent to Teensy.')
        except serial.SerialException as e:
            self.get_logger().error(f"Error when communicating to Teensy: {str(e)}")
            self.serial_port.close()

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        if self.v != 0:
            self.theta = msg.angular.z
        else:
            self.theta = 0
        if self.theta  != 0:
            half_wheelbase = 0.210 / 2
            self.turning_radius = half_wheelbase / math.tan(self.theta )
        else:
            self.turning_radius = 0

    def odom_callback(self):
        dt = 0.01
        if self.theta != 0 and self.turning_radius != 0:  # 檢查 self.turning_radius 不為零
            self.odom_yaw += (self.v / self.turning_radius) * dt
        else:
            self.odom_yaw += 0 
        dx = self.v * math.cos(self.odom_yaw) * dt
        dy = self.v * math.sin(self.odom_yaw) * dt
        self.odom_x += dx
        self.odom_y += dy

        now = self.get_clock().now()
        quaternion = quaternion_from_euler(0, 0, self.odom_yaw)
        odom_tf = TransformStamped()
        odom_tf.header.stamp = now.to_msg()
        odom_tf.header.frame_id = 'odom'
        odom_tf.child_frame_id = 'base_link'
        odom_tf.transform.translation.x = self.odom_x
        odom_tf.transform.translation.y = self.odom_y
        odom_tf.transform.translation.z = 0.0
        odom_tf.transform.rotation.x = quaternion[0]
        odom_tf.transform.rotation.y = quaternion[1]
        odom_tf.transform.rotation.z = quaternion[2]
        odom_tf.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(odom_tf)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = odom_tf.transform.rotation
        odom_msg.twist.twist.linear.x = self.v
        if self.theta != 0 and self.turning_radius != 0:  # 同樣的檢查應用於這裡
            odom_msg.twist.twist.angular.z = self.v / self.turning_radius
        else:
            odom_msg.twist.twist.angular.z = 0.0
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = TDKBaseController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

