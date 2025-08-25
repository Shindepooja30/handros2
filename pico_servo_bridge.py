#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class PicoServoBridge(Node):
    def __init__(self):
        super().__init__('pico_servo_bridge')
        
        # Connect to Raspberry Pi Pico 2W
        try:
            # Try common Pico serial ports (ACM1 found!)
            possible_ports = ['/dev/ttyACM1', '/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyUSB1']
            self.pico = None
            
            for port in possible_ports:
                try:
                    self.pico = serial.Serial(port, 115200, timeout=1)
                    time.sleep(2)
                    self.get_logger().info(f"Connected to Pico 2W on {port}!")
                    break
                except:
                    continue
                    
            if not self.pico:
                self.get_logger().error("Can't find Pico 2W. Check USB connection.")
                
        except Exception as e:
            self.get_logger().error(f"Pico connection error: {e}")
            self.pico = None
        
        # Subscribe to gestures (same topic as your existing system)
        self.subscription = self.create_subscription(
            String,
            '/gesture',
            self.gesture_callback,
            10
        )
        
        # Gesture to servo angles mapping for 5 MG995 servos
        # Format: [thumb, index, middle, ring, pinky]
        self.gesture_angles = {
            'open': [0, 0, 0, 0, 0],                    # All fingers open
            'close': [180, 180, 180, 180, 180],         # All fingers closed (fist)
            'thumbs_up': [0, 180, 180, 180, 180],       # Only thumb open 👍
            'peace': [180, 0, 0, 180, 180],             # Index & middle open ✌️
            'stone': [180, 180, 180, 180, 180]          # Tight fist ✊
        }
        
        self.get_logger().info("Pico Servo Bridge ready - Physical servo will follow gestures!")
    
    def gesture_callback(self, msg):
        """Send gesture to 5 physical servos (doesn't interfere with RViz)"""
        gesture = msg.data.strip()

        if gesture in self.gesture_angles and self.pico:
            angles = self.gesture_angles[gesture]  # List of 5 angles
            try:
                # Send all 5 angles to Pico: "thumb,index,middle,ring,pinky"
                angle_string = ",".join(map(str, angles))
                self.pico.write(f"{angle_string}\n".encode())
                self.get_logger().info(f"🤖 5 Servos: '{gesture}' → {angles}")
            except Exception as e:
                self.get_logger().error(f"Error sending to Pico: {e}")
        else:
            if not self.pico:
                self.get_logger().warn("Pico not connected - only RViz will move")
    
    def destroy_node(self):
        """Clean up"""
        if self.pico:
            self.pico.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    bridge = PicoServoBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
