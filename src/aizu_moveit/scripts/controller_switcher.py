#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

# Button 8 is the central Logitech/Home button on the F310
BUTTON_SWITCH = 8

class ControllerSwitcherNode(Node):
    def __init__(self):
        super().__init__('controller_switcher')
        
        # Use reliable QoS for mode switching
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.mode_pub = self.create_publisher(String, '/active_controller', qos_profile)
        
        self.modes = ['choreonoid', 'teleop']
        self.current_mode_index = 0
        self.last_button_state = 0

        # initialize by publishing the first mode
        self.initial_mode_timer = self.create_timer(1.0, self.publish_initial_mode)
        
        self.get_logger().info("Controller Switcher started. Press 'Logitech Button' to toggle.")
        self.get_logger().info(f"Available modes: {', '.join(self.modes)}")

    def publish_initial_mode(self):
        """Publishes the initial mode and then destroys this timer."""
        self.get_logger().info("Publishing initial controller mode...")
        self.publish_mode(log_change=True)
        # Destroy the timer so this function doesn't run again
        self.initial_mode_timer.destroy()

    def publish_mode(self, log_change=False):
        msg = String()
        current_mode = self.modes[self.current_mode_index]
        msg.data = current_mode
        self.mode_pub.publish(msg)
        if log_change:
            self.get_logger().info(f"==> Switching to '{current_mode}' mode <==")

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) <= BUTTON_SWITCH:
            return

        current_button = msg.buttons[BUTTON_SWITCH]
        button_pressed = current_button == 1 and self.last_button_state == 0
        self.last_button_state = current_button

        if button_pressed:
            # Store old mode for debugging
            old_mode = self.modes[self.current_mode_index]
            
            # Switch to next mode
            self.current_mode_index = (self.current_mode_index + 1) % len(self.modes)
            new_mode = self.modes[self.current_mode_index]
            
            self.get_logger().info(f"Button pressed! Switching: '{old_mode}' -> '{new_mode}'")
            self.publish_mode(log_change=True)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerSwitcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()