import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import numpy as np
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
rclpy.qos.qos_profile_sensor_data

class Odometry(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.w_r = 0.0
        self.w_l = 0.0

        self.x_dot = 0.0
        self.theta_dot = 0.0
        self.theta = 0.0

        #Parámetros físicos del robot
        self.r = 0.05
        self.l = 0.19

        self.x = 0.0
        self.y = 0.0

        self.x1 = 0.0
        self.y1 = 0.0
        self.theta1 = 0.0

        self.msg=Pose2D()   
        self.msg1=Pose2D()       

       
       
        self.w_l_sub = self.create_subscription(Float32, '/VelocityEncL', self.w_l_callback, rclpy.qos.qos_profile_sensor_data)
        self.w_r_sub = self.create_subscription(Float32, '/VelocityEncR', self.w_r_callback, rclpy.qos.qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Pose2D, 'Odometry', 10)
        self.odom1_pub = self.create_publisher(Pose2D, 'Odometry1', 10)

        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info('Odometry node initialized')

       

    def timer_callback(self):
        self.x_dot = self.r * (self.w_l + self.w_r) / 2.0
        self.theta_dot = self.r * (self.w_r - self.w_l) / self.l

        self.theta += self.theta_dot * self.dt

             # Actualizar las posiciones

        self.x += self.x_dot * np.cos(self.theta)
        self.y += self.x_dot * np.sin(self.theta)

        self.theta1  = self.theta + self.dt * self.r  * ((self.w_r - self.w_l) / self.l)
        self.x1 += self.dt * self.r * ((self.w_r + self.w_l) / 2) * np.cos(self.theta)
        self.y1 += self.dt * self.r  * ((self.w_r + self.w_l) / 2) * np.sin(self.theta)

        self.msg.x = self.x
        self.msg.y = self.y
        self.msg.theta = self.theta
        self.odom_pub.publish(self.msg)

        self.msg1.x = self.x1
        self.msg1.y = self.y1
        self.msg1.theta = self.theta1
        self.odom1_pub.publish(self.msg1)
            
    def w_l_callback(self, msg):
        self.w_l = msg.data

    def w_r_callback(self, msg):
        self.w_r = msg.data

def main(args=None):
    rclpy.init(args=args)
    o = Odometry()
    rclpy.spin(o)
    o.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

