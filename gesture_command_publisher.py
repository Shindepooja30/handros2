_#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GestureCommandPublisher(Node):
    def __init__(self):
        super().__init__('gesture_command_publisher')
        self.publisher_ = self.create_publisher(String, '/gesture', 10)
        self.timer = self.create_timer(2.0, self.publish_gesture)  # every 2 seconds
        self.gestures = ["open", "close"]
        self.current_index = 0

    def publish_gesture(self):
        msg = String()
        msg.data = self.gestures[self.current_index]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published gesture: {msg.data}')
        self.current_index = (self.current_index + 1) % len(self.gestures)

def main(args=None):
    rclpy.init(args=args)
    node = GestureCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

