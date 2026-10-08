"""
Teleoperation bridge node.

Subscribes to geometry_msgs/Twist on /cmd_vel (teleop_twist_keyboard or joystick) and
translates it into /throttle (Float32 in [-1, 1]) and /steer (Float32, radians, +left).

Phase 1 (Milestone 3): open-loop mapping
    throttle = clip(linear.x / max_linear_vel, -1, 1)
    steer    = clip(angular.z / max_angular_vel * max_steer_rad, -max_steer, +max_steer)
    A watchdog zeroes both commands if no Twist arrives for `auto_zero_timeout` (0.5 s).
Phase 2 (Milestone 4): with use_cruise_control:=true, linear.x is a TARGET SPEED (m/s) and
    PIDLongitudinalController closes the loop on the speed measured on /state.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from bicycle_control.longitudinal_pid import PIDLongitudinalController


class TeleopBridge(Node):
    def __init__(self):
        super().__init__('teleop_bridge')

        self.declare_parameter('max_linear_vel', 5.0)      # m/s <-> full throttle
        self.declare_parameter('max_angular_vel', 1.0)     # rad/s <-> full steering lock
        self.declare_parameter('max_steer_rad', 0.610865)  # 35 deg
        self.declare_parameter('auto_zero_timeout', 0.5)   # watchdog [s]
        self.declare_parameter('use_cruise_control', False)

        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.max_steer_rad = float(self.get_parameter('max_steer_rad').value)
        self.auto_zero_timeout = float(self.get_parameter('auto_zero_timeout').value)
        self.use_cruise_control = bool(self.get_parameter('use_cruise_control').value)

        self.throttle_pub = self.create_publisher(Float32, '/throttle', 10)
        self.steer_pub = self.create_publisher(Float32, '/steer', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.current_throttle = 0.0
        self.current_steer = 0.0
        self.target_vel = 0.0
        self.current_speed = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.pid = PIDLongitudinalController(dt=0.1)
        if self.use_cruise_control:
            self.odom_sub = self.create_subscription(Odometry, '/state', self.odom_callback, 10)
            self.get_logger().info('Cruise control enabled: linear.x = target speed (m/s)')

        self.timer = self.create_timer(0.1, self.publish_commands)   # 10 Hz
        self.get_logger().info('Teleoperation Bridge Node Initialized')

    def odom_callback(self, msg: Odometry):
        """Milestone 4: forward speed from /state."""
        self.current_speed = msg.twist.twist.linear.x

    def cmd_callback(self, msg: Twist):
        """Open-loop Twist -> (throttle, steer) mapping; refreshes the watchdog."""
        self.current_throttle = float(np.clip(msg.linear.x / self.max_linear_vel, -1.0, 1.0))
        self.current_steer = float(np.clip(
            (msg.angular.z / self.max_angular_vel) * self.max_steer_rad,
            -self.max_steer_rad, self.max_steer_rad))
        self.target_vel = msg.linear.x
        self.last_cmd_time = self.get_clock().now()

    def publish_commands(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.auto_zero_timeout:                     # watchdog
            self.current_throttle = 0.0
            self.current_steer = 0.0
            self.target_vel = 0.0

        if self.use_cruise_control:
            if self.target_vel < 0.05 and self.current_speed < 0.05:
                self.pid.reset()                             # stopped: clear the integrator
                self.current_throttle = 0.0
            else:
                self.current_throttle = self.pid.compute(max(self.target_vel, 0.0),
                                                         self.current_speed)

        throttle_msg = Float32()
        throttle_msg.data = float(self.current_throttle)
        self.throttle_pub.publish(throttle_msg)

        steer_msg = Float32()                                # was missing: car could not steer
        steer_msg.data = float(self.current_steer)
        self.steer_pub.publish(steer_msg)


def main(args=None):
    rclpy.init(args=args)
    bridge = TeleopBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
