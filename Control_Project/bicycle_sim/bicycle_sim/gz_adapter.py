import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class GazeboAdapter(Node):
    def __init__(self):
        super().__init__('gz_adapter')
        
        # Subscribing to your controller's output topics
        self.throttle_sub = self.create_subscription(Float32, '/throttle', self.throttle_cb, 10)
        self.steer_sub = self.create_subscription(Float32, '/steer', self.steer_cb, 10)
        
        # Publishing to Gazebo's Twist topic
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.current_throttle = 0.0
        self.current_steer = 0.0
        self.v_cmd = 0.0                       # commanded speed [m/s], integrated like bicycle_model.py
        self.dt = 0.05
        self.create_timer(self.dt, self.publish_cmd)

    def throttle_cb(self, msg):
        self.current_throttle = max(-1.0, min(1.0, msg.data))

    def steer_cb(self, msg):
        self.current_steer = max(-0.610865, min(0.610865, msg.data))

    def publish_cmd(self):
        # same longitudinal plant as bicycle_model.py: v_dot = k_a*u - (c_drag*v^2 + c_roll*v)
        v = self.v_cmd
        a = 4.0 * self.current_throttle - (0.005 * v * v + 0.05 * v)
        self.v_cmd = min(max(v + a * self.dt, 0.0), 25.0)

        twist_msg = Twist()
        twist_msg.linear.x = self.v_cmd                                          # speed [m/s]
        twist_msg.angular.z = self.v_cmd / 1.25 * math.tan(self.current_steer)   # YAW RATE [rad/s]
        self.cmd_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()