#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32
from .rosmaster_lib import Rosmaster
import math
import time

class RosmasterNode(Node):
    def __init__(self):
        super().__init__('rosmaster_node')
        
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('car_type', 1)
        
        port = self.get_parameter('port').value
        car_type = self.get_parameter('car_type').value
        
        self.bot = Rosmaster(port=port, car_type=car_type)
        self.bot.create_receive_threading()
        
        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.bat_pub = self.create_publisher(Float32, '/battery', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer publication (20Hz)
        self.create_timer(0.05, self.timer_callback)
        
        self.last_cmd_time = time.time()
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()
        self.bot.set_car_motion(msg.linear.x, msg.linear.y, msg.angular.z)

    def watchdog_callback(self):
        # Sécurité: si pas de commande depuis 0.5s, arrêt
        if time.time() - self.last_cmd_time > 0.5:
            self.bot.set_car_motion(0, 0, 0)

    def timer_callback(self):
        # IMU
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "imu_link"
        imu.linear_acceleration.x = float(self.bot.imu_data['ax'])
        # ... remplir autres
        self.imu_pub.publish(imu)
        
        # Battery
        bat = Float32()
        bat.data = float(self.bot.battery)
        self.bat_pub.publish(bat)
        
        # Joints DOFBOT
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        # Conversion angle (deg?) -> rad if needed, assuming rad in list
        js.position = [float(x) for x in self.bot.joints]
        self.joint_pub.publish(js)

    def destroy_node(self):
        self.bot.set_car_motion(0, 0, 0)
        del self.bot
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RosmasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
