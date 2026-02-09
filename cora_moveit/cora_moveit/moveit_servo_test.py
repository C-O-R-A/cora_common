#!/usr/bin/env python3
"""
Keyboard teleop node for MoveIt Servo (Python version)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
import sys
import termios
import tty
import signal

# Key mappings
KEYCODE_UP = "\x41"
KEYCODE_DOWN = "\x42"
KEYCODE_RIGHT = "\x43"
KEYCODE_LEFT = "\x44"
KEYCODE_PERIOD = "."
KEYCODE_SEMICOLON = ";"
KEYCODE_1 = "1"
KEYCODE_2 = "2"
KEYCODE_3 = "3"
KEYCODE_4 = "4"
KEYCODE_5 = "5"
KEYCODE_6 = "6"
KEYCODE_7 = "7"
KEYCODE_Q = "Q"
KEYCODE_R = "r"
KEYCODE_J = "j"
KEYCODE_T = "t"
KEYCODE_W = "w"
KEYCODE_E = "e"

# Frames
PLANNING_FRAME = "base_link"
EE_FRAME = "endeffector"  # replace with your EE link


class KeyboardServo(Node):
    def __init__(self):
        super().__init__("servo_keyboard_input")
        self.twist_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10
        )
        self.joint_pub = self.create_publisher(
            JointJog, "/servo_node/delta_joint_cmds", 10
        )
        self.switch_input_client = self.create_client(
            ServoCommandType, "servo_node/switch_command_type"
        )

        self.joint_vel_cmd = 1.0
        self.command_frame = PLANNING_FRAME

        self.joint_names = [
            "J1",
            "J2",
            "J3",
            "J4",
            "J5",
            "J6",
        ]  # replace with your robot joint names

        self.get_logger().info("Keyboard Servo Node started. Press 'Q' to quit.")

    def run(self):
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while True:
                c = sys.stdin.read(1)
                if not c:
                    continue
                if self.handle_key(c):
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def handle_key(self, c):
        publish_twist = False
        publish_joint = False

        twist_msg = TwistStamped()
        joint_msg = JointJog()
        joint_msg.joint_names = self.joint_names
        joint_msg.velocities = [0.0] * len(self.joint_names)

        # Cartesian commands
        if c == KEYCODE_UP:
            twist_msg.twist.linear.x = 0.5
            publish_twist = True
        elif c == KEYCODE_DOWN:
            twist_msg.twist.linear.x = -0.5
            publish_twist = True
        elif c == KEYCODE_LEFT:
            twist_msg.twist.linear.y = -0.5
            publish_twist = True
        elif c == KEYCODE_RIGHT:
            twist_msg.twist.linear.y = 0.5
            publish_twist = True
        elif c == KEYCODE_PERIOD:
            twist_msg.twist.linear.z = -0.5
            publish_twist = True
        elif c == KEYCODE_SEMICOLON:
            twist_msg.twist.linear.z = 0.5
            publish_twist = True

        # Joint commands
        elif c == KEYCODE_1:
            joint_msg.velocities[0] = self.joint_vel_cmd
            publish_joint = True
        elif c == KEYCODE_2:
            joint_msg.velocities[1] = self.joint_vel_cmd
            publish_joint = True
        elif c == KEYCODE_3:
            joint_msg.velocities[2] = self.joint_vel_cmd
            publish_joint = True
        elif c == KEYCODE_4:
            joint_msg.velocities[3] = self.joint_vel_cmd
            publish_joint = True
        elif c == KEYCODE_5:
            joint_msg.velocities[4] = self.joint_vel_cmd
            publish_joint = True
        elif c == KEYCODE_6:
            joint_msg.velocities[5] = self.joint_vel_cmd
            publish_joint = True
        elif c == KEYCODE_7:
            joint_msg.velocities[6] = self.joint_vel_cmd
            publish_joint = True
        elif c == KEYCODE_R:
            self.joint_vel_cmd *= -1

        # Switch input type
        elif c == KEYCODE_J:
            self.switch_command_type("JOINT_JOG")
        elif c == KEYCODE_T:
            self.switch_command_type("TWIST")

        # Switch command frame
        elif c == KEYCODE_W:
            self.command_frame = PLANNING_FRAME
            self.get_logger().info(f"Command frame set to {PLANNING_FRAME}")
        elif c == KEYCODE_E:
            self.command_frame = EE_FRAME
            self.get_logger().info(f"Command frame set to {EE_FRAME}")

        # Quit
        elif c.upper() == KEYCODE_Q:
            return True

        # Publish messages
        if publish_twist:
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = self.command_frame
            self.twist_pub.publish(twist_msg)
        if publish_joint:
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.header.frame_id = PLANNING_FRAME
            self.joint_pub.publish(joint_msg)

        return False

    def switch_command_type(self, cmd_type: str):
        if not self.switch_input_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("ServoCommandType service not available")
            return

        req = ServoCommandType.Request()
        try:
            if cmd_type == "JOINT_JOG":
                req.command_type = ServoCommandType.Request.JOINT_JOG
            elif cmd_type == "TWIST":
                req.command_type = ServoCommandType.Request.TWIST
            elif cmd_type == "POSE":
                req.command_type = ServoCommandType.Request.POSE
            else:
                raise ValueError(f"Failed to switch input type: {cmd_type}")
        except Exception as e:
            self.get_logger().warn(f'Warning: {e}')

        future = self.switch_input_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info(f"Switched to input type: {cmd_type}")
        else:
            self.get_logger().warn(f"Failed to switch input type: {cmd_type}")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardServo()

    def signal_handler(sig, frame):
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    node.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# ros2 run cora_moveit keyboard_servo_node
