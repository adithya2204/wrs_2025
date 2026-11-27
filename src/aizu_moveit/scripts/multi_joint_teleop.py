#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration
import time
from threading import Event

# Controller Mapping for Logitech F310 (in XInput mode)
# Buttons
BUTTON_DEADMAN = 0      # 'A' button
BUTTON_GRIPPER = 3      # 'Y' button
BUTTON_SELECT_ARM1 = 4  # Left Bumper (LB)
BUTTON_SELECT_ARM2 = 5  # Right Bumper (RB)
BUTTON_SPEED_UP = 1     # 'B' button
BUTTON_SPEED_DOWN = 2   # 'X' button
BUTTON_MODE_SWITCH = 8  # Logitech/Home button to toggle teleop activation

# Axes
AXIS_JOINT_0 = 1  # Left Stick Up/Down
AXIS_JOINT_1 = 0  # Left Stick Left/Right
AXIS_JOINT_2 = 4  # Right Stick Up/Down
AXIS_JOINT_3 = 3  # Right Stick Left/Right
AXIS_JOINT_4 = 7  # D-pad Up/Down
AXIS_JOINT_5 = 6  # D-pad Left/Right

class MultiJointTeleopNode(Node):
    def __init__(self):
        super().__init__('multi_joint_teleop')

        # Arm configurations
        self.arm1_joint_names = ["ARM1_SHOULDER", "ARM1_ARM", "ARM1_FOREARM", "ARM1_WRIST1", "ARM1_WRIST2", "ARM1_HAND"]
        self.arm1_positions = [0.0] * len(self.arm1_joint_names)
        self.arm1_pub = self.create_publisher(JointTrajectory, '/arm1_manipulator_controller/joint_trajectory', 10)

        self.arm2_joint_names = ["ARM2_SHOULDER", "ARM2_ARM", "ARM2_FOREARM", "ARM2_WRIST1", "ARM2_WRIST2", "ARM2_HAND"]
        self.arm2_positions = [0.0] * len(self.arm2_joint_names)
        self.arm2_pub = self.create_publisher(JointTrajectory, '/arm2_manipulator_controller/joint_trajectory', 10)

        # Gripper configurations
        self.arm1_fingers_joint_names = ["ARM1_FINGER1", "ARM1_FINGER2", "ARM1_FINGER3"]
        self.arm1_fingers_pub = self.create_publisher(JointTrajectory, '/arm1_fingers_controller/joint_trajectory', 10)
        self.arm1_gripper_open_pos = [0.0, 0.0, 0.0]
        self.arm1_gripper_close_pos = [1.35, 1.46, 1.46]
        self.arm1_gripper_is_open = True

        self.arm2_fingers_joint_names = ["ARM2_FINGER1", "ARM2_FINGER2", "ARM2_FINGER3"]
        self.arm2_fingers_pub = self.create_publisher(JointTrajectory, '/arm2_fingers_controller/joint_trajectory', 10)
        self.arm2_gripper_open_pos = [0.0, 0.0, 0.0]
        self.arm2_gripper_close_pos = [1.35, 1.46, 1.46]
        self.arm2_gripper_is_open = True

        # Node State
        self.is_initialized = Event()
        self.last_joy_msg = Joy()
        self.last_joy_msg_time = 0.0
        self.joy_timeout_s = 0.5
        self.active_arm = None
        self.prev_joy_msg = None
        self.deadman_enabled = False
        self.joint_velocity_scale = 0.5
        self.speed_increment = 0.1
        self.is_active = False

        # Publisher for changing the active controller
        self.controller_select_pub = self.create_publisher(String, '/active_controller', 10)
        
        # Publisher for passing through joy messages to spider controller
        self.joy_passthrough_pub = self.create_publisher(Joy, '/aizu_spider_controller', 10)

        # Subscribers and Timer
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.create_subscription(String, '/active_controller', self.controller_select_callback, 10)
        self.create_timer(0.05, self.publish_command)

        self.get_logger().info("--- Multi-Joint Teleop Node (Updated) Started ---")
        self.get_logger().info("Waiting for initial joint states and controller selection...")
        self.get_logger().info("Press the 'Logitech/Home' button to toggle arm teleop mode.")

    def controller_select_callback(self, msg: String):
        if msg.data == 'teleop':
            if not self.is_active:
                self.get_logger().info("==> Arm Teleop Activated <==")
            self.is_active = True
        else:
            if self.is_active:
                self.get_logger().info("==> Arm Teleop Deactivated <==")
            self.is_active = False
            self.deadman_enabled = False # Safety: disable movement when switched off

    def joint_state_callback(self, msg: JointState):
        joint_state_map = dict(zip(msg.name, msg.position))
        try:
            for i, name in enumerate(self.arm1_joint_names):
                self.arm1_positions[i] = joint_state_map[name]
            for i, name in enumerate(self.arm2_joint_names):
                self.arm2_positions[i] = joint_state_map[name]
            if not self.is_initialized.is_set():
                self.is_initialized.set()
                self.get_logger().info("Initial joint states received! Ready for teleop.")
                self.get_logger().info("Press 'Logitech/Home' button to switch to Arm control mode.")
        except KeyError:
            if not self.is_initialized.is_set():
                self.get_logger().warn("Waiting for all arm joints in /joint_states...", throttle_duration_sec=5.0)

    def joy_callback(self, msg: Joy):
        self.last_joy_msg = msg
        self.last_joy_msg_time = time.time()

        # Handle mode switching first, this should work even if teleop is not active.
        if self.prev_joy_msg is not None and msg.buttons[BUTTON_MODE_SWITCH] and not self.prev_joy_msg.buttons[BUTTON_MODE_SWITCH]:
            mode_msg = String()
            if self.is_active:
                # If active, publish 'drive' (or other mode name) to deactivate self
                mode_msg.data = 'drive' 
                self.get_logger().info("Logitech Button: Deactivating arm teleop.")
            else:
                # If not active, publish 'teleop' to activate self
                mode_msg.data = 'teleop'
                self.get_logger().info("Logitech Button: Activating arm teleop.")
            self.controller_select_pub.publish(mode_msg)
            # The controller_select_callback will handle the self.is_active flag change.

        # If not active for arm control, pass joy commands to spider controller
        if not self.is_active:
            self.joy_passthrough_pub.publish(msg)
            self.prev_joy_msg = msg
            return
        
        # Guard against first message after activation
        if self.prev_joy_msg is None:
            self.prev_joy_msg = msg
            return
        
        # Deadman switch logic
        if msg.buttons[BUTTON_DEADMAN] and not self.prev_joy_msg.buttons[BUTTON_DEADMAN]:
            self.deadman_enabled = not self.deadman_enabled
            self.get_logger().info(f"==> MOVEMENT {'ENABLED' if self.deadman_enabled else 'DISABLED'} <==")

        # Arm selection logic
        if msg.buttons[BUTTON_SELECT_ARM1] and not self.prev_joy_msg.buttons[BUTTON_SELECT_ARM1]:
            self.active_arm = 'arm1'
            self.get_logger().info("==> ARM 1 SELECTED <==")
        elif msg.buttons[BUTTON_SELECT_ARM2] and not self.prev_joy_msg.buttons[BUTTON_SELECT_ARM2]:
            self.active_arm = 'arm2'
            self.get_logger().info("==> ARM 2 SELECTED <==")
        
        # Gripper logic
        if msg.buttons[BUTTON_GRIPPER] and not self.prev_joy_msg.buttons[BUTTON_GRIPPER]:
            if self.active_arm == 'arm1':
                self.arm1_gripper_is_open = not self.arm1_gripper_is_open
                pos = self.arm1_gripper_open_pos if self.arm1_gripper_is_open else self.arm1_gripper_close_pos
                self.publish_gripper_command(self.arm1_fingers_pub, self.arm1_fingers_joint_names, pos)
                self.get_logger().info(f"Arm 1 Gripper: {'OPEN' if self.arm1_gripper_is_open else 'CLOSED'}")
            elif self.active_arm == 'arm2':
                self.arm2_gripper_is_open = not self.arm2_gripper_is_open
                pos = self.arm2_gripper_open_pos if self.arm2_gripper_is_open else self.arm2_gripper_close_pos
                self.publish_gripper_command(self.arm2_fingers_pub, self.arm2_fingers_joint_names, pos)
                self.get_logger().info(f"Arm 2 Gripper: {'OPEN' if self.arm2_gripper_is_open else 'CLOSED'}")
        
        # Speed control logic
        if msg.buttons[BUTTON_SPEED_UP] and not self.prev_joy_msg.buttons[BUTTON_SPEED_UP]:
            self.joint_velocity_scale = min(5.0, self.joint_velocity_scale + self.speed_increment)
            self.get_logger().info(f"Speed increased to: {self.joint_velocity_scale:.1f}")
        if msg.buttons[BUTTON_SPEED_DOWN] and not self.prev_joy_msg.buttons[BUTTON_SPEED_DOWN]:
            self.joint_velocity_scale = max(0.1, self.joint_velocity_scale - self.speed_increment)
            self.get_logger().info(f"Speed decreased to: {self.joint_velocity_scale:.1f}")
        
        self.prev_joy_msg = msg

    def publish_gripper_command(self, publisher, joint_names, positions):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(seconds=1).to_msg()
        traj_msg.points.append(point)
        publisher.publish(traj_msg)

    def publish_command(self):
        if not self.is_active:
            return

        if not self.is_initialized.is_set() or time.time() - self.last_joy_msg_time > self.joy_timeout_s:
            return

        if not self.deadman_enabled:
            return
        
        joy_msg = self.last_joy_msg
        if self.active_arm == 'arm1':
            joint_positions = self.arm1_positions
            joint_names = self.arm1_joint_names
            publisher = self.arm1_pub
        elif self.active_arm == 'arm2':
            joint_positions = self.arm2_positions
            joint_names = self.arm2_joint_names
            publisher = self.arm2_pub
        else:
            return
            
        deltas = [0.0] * 6
        deltas[0] = -joy_msg.axes[AXIS_JOINT_0]
        deltas[1] =  joy_msg.axes[AXIS_JOINT_1]
        deltas[2] = -joy_msg.axes[AXIS_JOINT_2]
        deltas[3] =  joy_msg.axes[AXIS_JOINT_3]
        deltas[4] = -joy_msg.axes[AXIS_JOINT_4]
        deltas[5] =  joy_msg.axes[AXIS_JOINT_5]
        
        for i in range(6):
            joint_positions[i] += deltas[i] * self.joint_velocity_scale * 0.05

        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in joint_positions]
        point.time_from_start = Duration(seconds=0, nanoseconds=100_000_000).to_msg()
        traj_msg.points.append(point)
        publisher.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MultiJointTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()