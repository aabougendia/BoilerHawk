#!/usr/bin/env python3
"""
Keyboard teleop node for controlling the drone in Gazebo.
Controls:
  W/S - Forward/Backward
  A/D - Left/Right  
  T/G - Up/Down
  Q/E - Rotate Left/Right
  Space - Stop all movement
"""

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DroneKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('drone_keyboard_teleop')
        
        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/model/iris_drone/cmd_vel', 10)
        
        # Movement speeds
        self.linear_speed = 2.0  # m/s
        self.angular_speed = 1.0  # rad/s
        
        self.get_logger().info('Drone Keyboard Teleop Started')
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "="*50)
        print("Drone Keyboard Control")
        print("="*50)
        print("Movement:")
        print("  W - Forward")
        print("  S - Backward")
        print("  A - Left")
        print("  D - Right")
        print("  T - Up")
        print("  G - Down")
        print("\nRotation:")
        print("  Q - Rotate Left")
        print("  E - Rotate Right")
        print("\nOther:")
        print("  Space - Stop all movement")
        print("  Ctrl+C - Quit")
        print("="*50 + "\n")
        
    def get_key(self):
        """Get a single keypress from terminal."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
        
    def publish_velocity(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0):
        """Publish velocity command."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.z = angular_z
        self.publisher.publish(msg)
        
    def run(self):
        """Main loop to read keyboard and publish commands."""
        try:
            while rclpy.ok():
                key = self.get_key().lower()
                
                # Handle key commands
                if key == 'w':
                    self.publish_velocity(linear_x=self.linear_speed)
                    self.get_logger().info('Moving Forward')
                elif key == 's':
                    self.publish_velocity(linear_x=-self.linear_speed)
                    self.get_logger().info('Moving Backward')
                elif key == 'a':
                    self.publish_velocity(linear_y=self.linear_speed)
                    self.get_logger().info('Moving Left')
                elif key == 'd':
                    self.publish_velocity(linear_y=-self.linear_speed)
                    self.get_logger().info('Moving Right')
                elif key == 't':
                    self.publish_velocity(linear_z=self.linear_speed)
                    self.get_logger().info('Moving Up')
                elif key == 'g':
                    self.publish_velocity(linear_z=-self.linear_speed)
                    self.get_logger().info('Moving Down')
                elif key == 'q':
                    self.publish_velocity(angular_z=self.angular_speed)
                    self.get_logger().info('Rotating Left')
                elif key == 'e':
                    self.publish_velocity(angular_z=-self.angular_speed)
                    self.get_logger().info('Rotating Right')
                elif key == ' ':
                    self.publish_velocity()
                    self.get_logger().info('Stop')
                elif key == '\x03':  # Ctrl+C
                    break
                    
        except KeyboardInterrupt:
            pass
        finally:
            # Stop the drone before exiting
            self.publish_velocity()
            self.get_logger().info('Teleop stopped')


def main(args=None):
    rclpy.init(args=args)
    
    teleop_node = DroneKeyboardTeleop()
    
    try:
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
