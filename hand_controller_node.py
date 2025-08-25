#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class HandControllerNode(Node):
    def __init__(self):
        super().__init__('hand_controller_node')

        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            '/hand_joint_position_controller/commands',
            10
        )

        # Also publish joint states directly for visualization
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.gesture_sub = self.create_subscription(
            String,
            '/gesture',
            self.gesture_callback,
            10
        )

        # Timeout timer - return to open state if no command for 5 seconds
        self.timeout_duration = 5.0  # seconds
        self.timeout_timer = self.create_timer(self.timeout_duration, self.timeout_callback)
        self.last_command_time = self.get_clock().now()

        # Smooth transition variables
        self.current_positions = [0.0] * 15  # Start with open position
        self.target_positions = [0.0] * 15
        self.transition_duration = 1.0  # seconds for smooth transition
        self.transition_steps = 50  # number of interpolation steps
        self.transition_timer = self.create_timer(0.02, self.smooth_transition_callback)  # 50Hz
        self.is_transitioning = False
        self.transition_step = 0

        # 15 joint angles for different gestures (including all joints)
        # Joint order: base_joint, joint1-bd, joint1-du, joint2-bd, joint2-dm, joint2-mu,
        #              joint3-bd, joint3-dm, joint3-mu, joint4-bd, joint4-dm, joint4-mu,
        #              joint5-bd, joint5-dm, joint5-mu
        self.gesture_map = {
            'open': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'close': [0.0, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7],
            'thumbs_up': [0.0, 0.0, 0.0, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7],  # Only thumb extended
            'peace': [0.0, 0.7, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7],  # Index and middle finger extended
            'stone': [0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]   # Fully closed fist
        }

        # All joints including base_joint (fixed joint needs position 0.0)
        self.joint_names = [
            'base_joint',
            'joint1-bd', 'joint1-du',
            'joint2-bd', 'joint2-dm', 'joint2-mu',
            'joint3-bd', 'joint3-dm', 'joint3-mu',
            'joint4-bd', 'joint4-dm', 'joint4-mu',
            'joint5-bd', 'joint5-dm', 'joint5-mu'
        ]

        self.get_logger().info("Hand controller node started.")

    def gesture_callback(self, msg):
        gesture = msg.data.lower()

        if gesture in self.gesture_map:
            self.get_logger().info(f"Received gesture: {gesture} - starting smooth transition")

            # Update last command time
            self.last_command_time = self.get_clock().now()

            # Start smooth transition to new gesture
            self.target_positions = self.gesture_map[gesture].copy()
            self.is_transitioning = True
            self.transition_step = 0
        else:
            self.get_logger().warn(f"Unknown gesture: {gesture}")

    def smooth_transition_callback(self):
        """Handle smooth interpolation between gestures"""
        if not self.is_transitioning:
            return

        # Calculate interpolation factor (0.0 to 1.0)
        progress = self.transition_step / self.transition_steps

        if progress >= 1.0:
            # Transition complete
            self.current_positions = self.target_positions.copy()
            self.is_transitioning = False
            self.get_logger().info("Smooth transition completed")
        else:
            # Interpolate between current and target positions
            for i in range(len(self.current_positions)):
                start_pos = self.current_positions[i]
                target_pos = self.target_positions[i]
                self.current_positions[i] = start_pos + (target_pos - start_pos) * progress

            self.transition_step += 1

        # Publish current interpolated positions
        command_msg = Float64MultiArray()
        command_msg.data = self.current_positions.copy()
        self.joint_pub.publish(command_msg)

        # Also publish joint states for visualization
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = ""
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.current_positions.copy()
        joint_state_msg.velocity = [0.0] * len(self.joint_names)
        joint_state_msg.effort = [0.0] * len(self.joint_names)

        self.joint_state_pub.publish(joint_state_msg)

    def timeout_callback(self):
        """Return to open state if no command received for timeout duration"""
        current_time = self.get_clock().now()
        time_since_last_command = (current_time - self.last_command_time).nanoseconds / 1e9

        if time_since_last_command >= self.timeout_duration:
            self.get_logger().info("No gesture command for 5 seconds - smoothly returning to open state")

            # Start smooth transition to open gesture
            self.target_positions = self.gesture_map['open'].copy()
            self.is_transitioning = True
            self.transition_step = 0

            # Reset timer to avoid repeated open commands
            self.last_command_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = HandControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
