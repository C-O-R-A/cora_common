import os

from ament_index_python.packages import get_package_share_directory

from lifecycle_msgs.srv import ChangeState

from rclpy.action import ActionClient
from rclpy.node import Node
from lifecycle_msgs.msg import Transition
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType

from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, TwistStamped
from cora_msgs.action import PoseGoal
from pathlib import Path
from codi import CoraServer

import numpy as np

import rclpy

HERE = Path(__file__).resolve().parent
CONFIG = HERE.parent / "config" / "server_params.yaml"


class CodiNode(Node):
    # TODO: Add new predefined pose fucntionality using MoveitPy and srdf
    # TODO: ADD Gripper command
    def __init__(self):
        super().__init__("codi_node")

        # Status
        self.status = "idle"
        self.timer_period = 0.01

        # Declare YAML config file path param
        self.declare_parameter(
            "config_file",  # name of parameter
            str(HERE.parent / "config" / "server_params.yaml"),  # default value
        )

        # Start CoDI server
        codi_config_file = self.get_parameter("config_file").value
        self.codi_server = CoraServer(codi_config_file)
        self.codi_server.start()
        self.last_command = self.codi_server.get_command()

        # Transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(self.timer_period, self.transforms_callback)

        # Pose goal action for preplanned goal
        self.pose_timer = self.create_timer(self.timer_period, self.pose_timer_callback)
        self.pose_action_client = ActionClient(self, PoseGoal, "posegoal")

        # Servo nodes for real time goals
        self.twist_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10
        )
        self.joint_pub = self.create_publisher(
            JointJog, "/servo_node/delta_joint_cmds", 10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, "/servo_node/delta_pose_cmds", 10
        )
        self.switch_input_client = self.create_client(
            ServoCommandType, "servo_node/switch_command_type"
        )
        self.rt_vel = np.zeros((2, 3))
        self.current_servo_mode = None

        # Lifecycle Node clients and Config
        self.vision_client = self.create_client(
            ChangeState, "/vision_node/change_state"
        )
        self.camera_client = self.create_client(
            ChangeState, "/camera_node/change_state"
        )
        self.controller_client = self.create_client(
            ChangeState, "/controller_node/change_state"
        )

        self.config = {
            "vision": {"value": False, "node": self.vision_client},
            "camera": {"value": False, "node": self.camera_client},
            "controller": {"value": False, "node": self.controller_client},
        }

        self.apply_config()

        # for c in [self.vision_client, self.camera_client, self.controller_client]:
        #     c.wait_for_service()
        self.create_timer(0.5, self.config_callback)

    def activate_node(self, client):
        if client.service_is_ready():
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_ACTIVATE
            client.call_async(req)
        else:
            self.get_logger().warn("Lifecycle service unavailable. skipping activation")

    def deactivate_node(self, client):
        if client.service_is_ready():
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_DEACTIVATE
            client.call_async(req)
        else:
            self.get_logger().warn(
                "Lifecycle service unavailable. skipping deactivation"
            )

    def apply_config(self):
        for name, cfg in self.config.items():
            use_node = cfg["value"]
            node = cfg["node"]

            if use_node:
                self.get_logger().info(f"Activating {name} node")
                self.activate_node(node)
            else:
                self.get_logger().info(f"Deactivating {name} node")
                self.deactivate_node(node)

    def transform_to_array(self, tf: TransformStamped):
        t = tf.transform.translation
        q = tf.transform.rotation
        return np.array(
            [
                [t.x, t.y, t.z, 1.0],
                [q.x, q.y, q.z, q.w],
            ],
            dtype=float,
        )

    def transforms_callback(self):
        try:
            gripper_transform_stamped = self.tf_buffer.lookup_transform(
                "base_link", "Gripper", rclpy.time.Time()
            )
            camera_transform_stamped = self.tf_buffer.lookup_transform(
                "base_link", "Camera", rclpy.time.Time()
            )
            end_effector_transform_stamped = self.tf_buffer.lookup_transform(
                "base_link", "endeffector", rclpy.time.Time()
            )

            self.gripper_transforms = self.transform_to_array(gripper_transform_stamped)
            self.camera_transforms = self.transform_to_array(camera_transform_stamped)
            self.end_effector_transforms = self.transform_to_array(
                end_effector_transform_stamped
            )

            # Send to Client
            self.codi_server.send_state(
                self.status,
                "TS",
                self.end_effector_transforms,
                self.camera_transforms,
                self.gripper_transforms,
            )

        except Exception as e:
            self.get_logger().warn(
                f"Could not get transform: {e}", throttle_duration_sec=2.0
            )

    def pose_timer_callback(self):
        try:
            command = self.codi_server.get_command()
            (
                rt,
                space,
                interface_type,
                target,
                gripper_command,
                pose_command,
                predef_pose,
            ) = command

            if rt:  # Moveit Servo pipeline
                # Moveit Servo is kept separate
                # due to its real-time requirements
                # Ros2 actions are not real-time safe

                twist_msg = TwistStamped()
                joint_msg = JointJog()
                pose_msg = PoseStamped()
                gripper_msg = JointJog()

                joint_msg.joint_names = [f"J{i}" for i in range(1, 7)]

                # joint_msg.displacements = [0.0] * len(joint_msg.joint_names) # not yet supported
                joint_msg.velocities = [0.0] * len(joint_msg.joint_names)

                self.publish_pose = False
                self.publish_twist = False
                self.publish_joint = False

                try:
                    # ------------------------- #
                    # Construct gripper message #
                    # ------------------------- #

                    # ----------------------- #
                    # Construct Robot message #
                    # ----------------------- #
                    match space:
                        case "JS":
                            if self.current_servo_mode != "JOINT_JOG":
                                self.switch_command_type("JOINT_JOG")
                                self.current_servo_mode = "JOINT_JOG"
                            self.publish_joint = True
                            interface_methods = {
                                "position": joint_msg.displacements,
                                "velocity": joint_msg.velocities,
                            }

                            for i, _ in enumerate(joint_msg.joint_names, 0):
                                if interface_type == "acceleration":
                                    interface_methods["velocity"][i] += (
                                        pose_command[i] * self.timer_period
                                    )
                                else:
                                    interface_methods[interface_type][i] = (
                                        pose_command[i]
                                    )
                            pass

                        case "TS":
                            interface_methods = {
                                "position": [self.construct_pose_msg, pose_msg],
                                "velocity": [self.construct_twist_msg, twist_msg],
                                "acceleration": [
                                    self.construct_twist_from_accel,
                                    twist_msg,
                                ],
                            }

                            if interface_type in (
                                "position",
                                "velocity",
                                "acceleration",
                            ):
                                interface_methods[interface_type][0](
                                    pose_command, interface_methods[interface_type][1]
                                )
                            pass
                        
                except Exception as e:
                    self.get_logger().info(f"Error: {e}")

                timestamp = self.get_clock().now().to_msg()

                if gripper_command is not None:
                    gripper_msg.header.stamp = timestamp
                    gripper_msg.header.frame_id = 'Gripper'
                    gripper_msg.joint_names = ['Finger1']
                    gripper_msg.velocities = [gripper_command]

                if self.publish_pose:
                    pose_msg.header.stamp = timestamp
                    pose_msg.header.frame_id = "base_link"
                    self.pose_pub.publish(pose_msg)

                elif self.publish_twist:
                    twist_msg.header.stamp = timestamp
                    twist_msg.header.frame_id = "base_link"
                    self.twist_pub.publish(twist_msg)

                elif self.publish_joint:
                    joint_msg.header.stamp = timestamp
                    joint_msg.header.frame_id = "base_link"
                    self.joint_pub.publish(joint_msg)

                self.last_command = None

                return

            else:  # MoveitPy pipeline
                if command != self.last_command:
                    self.get_logger().info(
                        f"New command received from client: <<<{command}>>>"
                    )

                    goal = PoseGoal.Goal()
                    goal.interface_type = interface_type

                    if gripper_command is not None:
                        goal.gripper_goal = gripper_command

                    # Fill predefined pose field
                    if predef_pose:
                        goal.predefined_pose = predef_pose

                    # If no predefined pose is specified, fill the rest of the message
                    else:
                        goal.space = space
                        match space:
                            case "TS":
                                # Fill TargetedPoseStamped
                                #  NOTE (if the space is ts, pose_command was sent as [[x, y, z, 1],[rx, ry, rz, w]])
                                goal.pose_goal.target_frame = target

                                goal.pose_goal.pose.header.frame_id = "base_link"
                                goal.pose_goal.pose.header.stamp = (
                                    self.get_clock().now().to_msg()
                                )

                                goal.pose_goal.pose.pose.position.x = pose_command[0, 0]
                                goal.pose_goal.pose.pose.position.y = pose_command[0, 1]
                                goal.pose_goal.pose.pose.position.z = pose_command[0, 2]

                                goal.pose_goal.pose.pose.orientation.x = pose_command[1, 0]
                                goal.pose_goal.pose.pose.orientation.y = pose_command[1, 1]
                                goal.pose_goal.pose.pose.orientation.z = pose_command[1, 2]
                                goal.pose_goal.pose.pose.orientation.w = pose_command[1, 3]
                                pass

                            case "JS":
                                # Fill Joint Goal
                                # NOTE ( if the space is js, pose_command was sent as [J1, J2, J3, ...] )
                                goal.joint_goal = pose_command.tolist()
                                pass

                            case _:
                                raise ValueError(
                                    f'Unsupported planning space requested,\
                                    expected "JS" or "TS" but got: {space}'
                                )

                    self.get_logger().info("Waiting for action server...")
                    self.pose_action_client.wait_for_server()

                    # Send Goal
                    send_goal_future = self.pose_action_client.send_goal_async(
                        goal, feedback_callback=self.pose_feedback_callback
                    )
                    send_goal_future.add_done_callback(self.pose_response_callback)

                    self.last_command = command

                else:
                    # skip sending a command if the command is the same
                    pass

        except Exception:
            pass

    def construct_twist_from_accel(self, pose_command, msg):
        if self.current_servo_mode != "TWIST":
            self.switch_command_type("TWIST")
            self.current_servo_mode = "TWIST"
        self.publish_twist = True
        self.rt_vel[0, 0] += pose_command[0, 0] * self.timer_period
        msg.twist.linear.x = self.rt_vel[0, 0]

        self.rt_vel[0, 1] += pose_command[0, 1] * self.timer_period
        msg.twist.linear.y = self.rt_vel[0, 1]

        self.rt_vel[0, 2] += pose_command[0, 2] * self.timer_period
        msg.twist.linear.z = self.rt_vel[0, 2]

        self.rt_vel[1, 0] += pose_command[1, 0] * self.timer_period
        msg.twist.angular.x = self.rt_vel[1, 0]

        self.rt_vel[1, 1] += pose_command[1, 1] * self.timer_period
        msg.twist.angular.y = self.rt_vel[1, 1]

        self.rt_vel[1, 2] += pose_command[1, 2] * self.timer_period
        msg.twist.angular.z = self.rt_vel[1, 2]

    def construct_twist_msg(self, pose_command, msg):
        if self.current_servo_mode != "TWIST":
            self.switch_command_type("TWIST")
            self.current_servo_mode = "TWIST"
        self.publish_twist = True
        msg.twist.linear.x = pose_command[0, 0]
        msg.twist.linear.y = pose_command[0, 1]
        msg.twist.linear.z = pose_command[0, 2]
        msg.twist.angular.x = pose_command[1, 0]
        msg.twist.angular.y = pose_command[1, 1]
        msg.twist.angular.z = pose_command[1, 2]
        self.rt_vel = pose_command

    def construct_pose_msg(self, pose_command, msg):
        if self.current_servo_mode != "POSE":
            self.switch_command_type("POSE")
            self.current_servo_mode = "POSE"
        self.publish_pose = True
        msg.pose.position.x = pose_command[0, 0]
        msg.pose.position.y = pose_command[0, 1]
        msg.pose.position.z = pose_command[0, 2]
        msg.pose.orientation.x = pose_command[1, 0]
        msg.pose.orientation.y = pose_command[1, 1]
        msg.pose.orientation.z = pose_command[1, 2]
        msg.pose.orientation.w = pose_command[1, 3]

    def pose_feedback_callback(self, feedback_msg):
        # feedback = feedback_msg.feedback()
        self.status = "done"

    def pose_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.pose_result_callback)

    def pose_result_callback(self, future):
        result = future.result().result

        self.status = result.status_result

        if result.success:
            self.get_logger().info("Motion completed successfully")
        else:
            self.get_logger().warn("Motion failed")

    def config_callback(self):
        config = self.codi_server.get_config()

        if config != (
            self.config["controller"]["value"],
            self.config["camera"]["value"],
            self.config["vision"]["value"],
        ):
            self.get_logger().info(f"Configuration changed to{config}")
            (
                self.config["controller"]["value"],
                self.config["camera"]["value"],
                self.config["vision"]["value"],
            ) = config

            self.apply_config()

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
                raise ValueError(f"Unknown input type: {cmd_type}")
        except Exception as e:
            self.get_logger().warn(f"Warning: {e}")

        future = self.switch_input_client.call_async(req)
        if future.result() and future.result().success:
            self.get_logger().info(f"Switched to input type: {cmd_type}")
        else:
            self.get_logger().warn(f"Failed to switch input type: {cmd_type}")


def main(args=None):
    rclpy.init(args=args)
    node = CodiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# ├── moveitpy_node          (Action Client Node)
# ├── vision_node            (LifecycleNode)
# ├── camera_node            (LifecycleNode)
# ├── teleop_node            (LifecycleNode)
# └── lifecycle_manager_node (or logic inside cora_comms)
