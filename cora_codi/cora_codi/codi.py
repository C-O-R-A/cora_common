"""
codi_node
=========

ROS 2 bridge node that connects the codi Python SDK to the CORA hardware
stack. Receives motion commands from a connected codi client over ZMQ,
translates them into ROS 2 action goals or MoveIt Servo messages, and
publishes robot state (transforms) back to the client.

Two motion pipelines are supported:

- **MoveIt Servo** (real-time): commands are published directly as
  ``TwistStamped``, ``JointJog``, or ``PoseStamped`` topics at 100 Hz.
- **MoveIt action** (planned): commands are sent as ``PoseGoal`` action
  goals to the ``posegoal`` action server and executed via MoveIt 2.

Node name: ``codi_node``
"""

import os

from ament_index_python.packages import get_package_share_directory

from lifecycle_msgs.srv import ChangeState

from rclpy.action import ActionClient
from rclpy.node import Node
from lifecycle_msgs.msg import Transition
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType

from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from tf2_msgs.msg import TFMessage
from cora_msgs.action import PoseGoal
from pathlib import Path
from codi import CoraServer
from codi.codi_enums import MoveStatus, GoalSpace, InterfaceType
from codi.messages import ConfigMessage, CommandMessage

from sensor_msgs.msg import JointState

import numpy as np

import rclpy
from rosidl_runtime_py.convert import message_to_ordereddict

HERE = Path(__file__).resolve().parent
CONFIG = HERE.parent / "config" / "server_params.yaml"


class CodiNode(Node):
    """ROS 2 node that bridges the codi SDK to the CORA hardware stack.

    Starts a :class:`~codi.CoraServer` instance, polls it for incoming
    commands, and dispatches them to either the MoveIt Servo pipeline
    (real-time) or the MoveIt action pipeline (planned motion).

    Concurrently, TF2 transforms for the end-effector, camera, and gripper
    frames are looked up at 100 Hz and forwarded back to connected codi
    clients via :meth:`~codi.CoraServer.send_state`.

    Publishers:
        - ``/servo_node/delta_twist_cmds`` (:class:`TwistStamped`)
        - ``/servo_node/delta_joint_cmds`` (:class:`JointJog`)
        - ``/servo_node/delta_pose_cmds`` (:class:`PoseStamped`)

    Action clients:
        - ``posegoal`` (:class:`~cora_msgs.action.PoseGoal`)

    Service clients:
        - ``servo_node/switch_command_type`` (:class:`~moveit_msgs.srv.ServoCommandType`)

        - ``/camera_node/change_state`` (:class:`~lifecycle_msgs.srv.ChangeState`)
        - ``/controller_node/change_state`` (:class:`~lifecycle_msgs.srv.ChangeState`)

    Parameters:
        config_file (str): Path to the codi server YAML config file.
            Defaults to ``<package_root>/config/server_params.yaml``.
    """

    def __init__(self):
        """Initialise the node, start the codi server, and create all
        publishers, subscribers, action clients, and timers."""
        super().__init__("codi_node")

        # Status
        self.status = MoveStatus.IDLE
        self.timer_period = 0.01

        # Declare YAML config file path param
        self.declare_parameter(
            "config_file",
            str(HERE.parent / "config" / "server_params.yaml"),
        )

        # Start CoDI server
        codi_config_file = self.get_parameter("config_file").value
        self.codi_server = CoraServer(codi_config_file)
        self.codi_server.start()
        self.last_command = self.codi_server.get_command()

        # Transform listener
        # TODO: #1 read frames from the generated robot_layout.yaml instead of
        # hardcoding. "endeffector" only exists in the hand-written cora URDF;
        # configurator-generated robots name their tip link
        # "<lastJoint>_joint_out". Needs ee_frame + extra_frames from the
        # contract (C-O-R-A/configurator#2).
        self.reference_frames = ["Gripper", "Camera", "endeffector"]
        self.transforms = None
        self.create_subscription(TFMessage, "/tf", self.tf_callback, 10)

        # JointState subscriber
        self.joint_states = None
        self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Pose goal action for preplanned goal
        self.pose_timer = self.create_timer(self.timer_period, self.pose_timer_callback)
        self.pose_action_client = ActionClient(self, PoseGoal, "posegoal")

        # Servo publishers
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

        # Lifecycle node clients
        self.camera_client = self.create_client(
            ChangeState, "/camera_node/change_state"
        )
        self.controller_client = self.create_client(
            ChangeState, "/controller_node/change_state"
        )

        self.config = {
            "camera": {"value": False, "node": self.camera_client},
            "controller": {"value": False, "node": self.controller_client},
        }

        self.apply_config()
        self.create_timer(0.5, self.config_callback)

    def activate_node(self, client):
        """Send an ``ACTIVATE`` lifecycle transition to a managed node.

        Args:
            client: The :class:`~lifecycle_msgs.srv.ChangeState` service
                client for the target node.
        """
        if client.service_is_ready():
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_ACTIVATE
            client.call_async(req)
        else:
            self.get_logger().warn("Lifecycle service unavailable. skipping activation")

    def deactivate_node(self, client):
        """Send a ``DEACTIVATE`` lifecycle transition to a managed node.

        Args:
            client: The :class:`~lifecycle_msgs.srv.ChangeState` service
                client for the target node.
        """
        if client.service_is_ready():
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_DEACTIVATE
            client.call_async(req)
        else:
            self.get_logger().warn(
                "Lifecycle service unavailable. skipping deactivation"
            )

    def apply_config(self):
        """Apply the current ``self.config`` state by activating or
        deactivating the camera and controller lifecycle nodes."""
        for name, cfg in self.config.items():
            use_node = cfg["value"]
            node = cfg["node"]

            if use_node:
                self.get_logger().info(f"Activating {name} node")
                self.activate_node(node)
            else:
                self.get_logger().info(f"Deactivating {name} node")
                self.deactivate_node(node)

    def joint_state_callback(self, msg: JointState):
        """Callback for the ``/joint_states`` topic.

        Stores the latest joint state as a plain dict on ``self.joint_states``
        so downstream code can serialize or map it into the CoDI format.

        Args:
            msg: The :class:`JointState` message received.
        """
        try:
            self.joint_states = dict(message_to_ordereddict(msg))
        except Exception as e:
            self.get_logger().warn(f"Failed to convert JointState: {e}")
            self.joint_states = None

    def tf_callback(self, msg: TFMessage):
        """Callback for the ``/tf`` topic. Stores the latest transform
        messages in ``self.transforms`` for later use.

        Args:
            msg: The :class:`tf2_msgs.msg.TFMessage` message received.
        """
        try:
            self.transforms = [dict(message_to_ordereddict(t)) for t in msg.transforms]
        except Exception as e:
            self.get_logger().warn(f"Failed to convert TF message: {e}")
            self.transforms = None

    def feedback_callback(self):
        """
        Timer callback (100 Hz) that looks up TF2 transforms for all reference frames and forwards the robot state to the codi server.

        Looks up ``base_link`` → ``Gripper``, ``Camera``, and
        ``endeffector`` transforms. On failure the transform is set to a
        zero array and a throttled warning is logged.
        """
        try:
            if not (self.joint_states and self.transforms):
                return

            self.codi_server.send_state(
                self.transforms,
                self.joint_states,
                self.status,
            )

        except Exception as e:
            self.get_logger().warn(
                f"Could not get transform: {e}", throttle_duration_sec=2.0
            )

    def pose_timer_callback(self):
        """Timer callback (100 Hz) that polls the codi server for new
        commands and dispatches them to the appropriate motion pipeline.

        If the ``rt`` flag is set in the command, the Servo pipeline is
        used and messages are published directly. Otherwise the command is
        sent as a ``PoseGoal`` action goal to the MoveIt action server.
        """
        try:
            command = self.codi_server.get_command()
            if command is None:
                return

            if command.rt:  # MoveIt Servo pipeline
                twist_msg = TwistStamped()
                joint_msg = JointJog()
                pose_msg = PoseStamped()
                gripper_msg = JointJog()

                # TODO: #1 THE sharpest hardcode in the repo — range(1, 7) pins
                # both the DOF count and the "J<n>" naming convention in one
                # expression. Replace with self.arm_joints loaded from the
                # generated robot_layout.yaml (C-O-R-A/configurator#2).
                joint_msg.joint_names = [f"J{i}" for i in range(1, 7)]
                joint_msg.velocities = [0.0] * len(joint_msg.joint_names)

                self.publish_pose = False
                self.publish_twist = False
                self.publish_joint = False

                try:

                    # --------------------------- #
                    # Deconstruct Command message #
                    # --------------------------- #
                    if command.joint_command is not None:
                        if self.current_servo_mode != "JOINT_JOG":
                            self.switch_command_type("JOINT_JOG")
                            self.current_servo_mode = "JOINT_JOG"
                        self.publish_joint = True
                        interface_methods = {
                            InterfaceType.POSITION: joint_msg.displacements,
                            InterfaceType.VELOCITY: joint_msg.velocities,
                        }

                        for i, _ in enumerate(joint_msg.joint_names, 0):
                            if command.interface_type == InterfaceType.EFFORT:
                                joint_msg.velocities[i] += (
                                    pose_command[i] * self.timer_period
                                )
                            else:
                                interface_methods[command.interface_type][i] = (
                                    command.joint_command[i]
                                )
                    elif command.pose_command is not None:
                        interface_methods = {
                            InterfaceType.POSITION: [self.construct_pose_msg, pose_msg],
                            InterfaceType.VELOCITY: [
                                self.construct_twist_msg,
                                twist_msg,
                            ],
                            InterfaceType.EFFORT: [
                                self.construct_twist_from_accel,
                                twist_msg,
                            ],
                        }
                        if command.interface_type in interface_methods:
                            interface_methods[command.interface_type][0](
                                command.pose_command,
                                interface_methods[command.interface_type][1],
                            )

                except Exception as e:
                    self.get_logger().info(f"Error: {e}")

                timestamp = self.get_clock().now().to_msg()

                if command.gripper_command is not None:
                    gripper_msg.header.stamp = timestamp
                    gripper_msg.header.frame_id = "Gripper"
                    # TODO: #1 use self.gripper_joints from robot_layout.yaml;
                    # a robot may have no gripper, or one not named "Finger1".
                    gripper_msg.joint_names = ["Finger1"]
                    gripper_msg.velocities = [command.gripper_command]

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

            else:  # MoveIt action pipeline
                if command != self.last_command:
                    self.get_logger().info(
                        f"New command received from client: <<<{command}>>>"
                    )

                    goal = PoseGoal.Goal()
                    goal.interface_type = command.interface_type

                    if command.gripper_command is not None:
                        goal.gripper_goal = command.gripper_command

                    if command.predef_pose:
                        goal.predefined_pose = command.predef_pose
                    else:
                        if command.pose_command is not None:
                            goal.space = PoseGoal.Goal().TS
                            goal.pose_goal.target_frame = command.target
                            goal.pose_goal.pose.header.frame_id = "base_link"
                            goal.pose_goal.pose.header.stamp = (
                                self.get_clock().now().to_msg()
                            )
                            pose_array = np.array(command.pose_command)
                            goal.pose_goal.pose.pose.position.x = pose_array[0, 0]
                            goal.pose_goal.pose.pose.position.y = pose_array[0, 1]
                            goal.pose_goal.pose.pose.position.z = pose_array[0, 2]
                            goal.pose_goal.pose.pose.orientation.x = pose_array[1, 0]
                            goal.pose_goal.pose.pose.orientation.y = pose_array[1, 1]
                            goal.pose_goal.pose.pose.orientation.z = pose_array[1, 2]
                            goal.pose_goal.pose.pose.orientation.w = pose_array[1, 3]

                        elif command.joint_command is not None:
                            goal.space = PoseGoal.Goal().JS
                            goal.joint_goal = command.joint_command

                    self.get_logger().info("Waiting for action server...")
                    self.pose_action_client.wait_for_server()

                    send_goal_future = self.pose_action_client.send_goal_async(
                        goal, feedback_callback=self.pose_feedback_callback
                    )
                    send_goal_future.add_done_callback(self.pose_response_callback)
                    self.last_command = command

        except Exception:
            pass

    def construct_twist_from_accel(self, pose_command: np.ndarray, msg: TwistStamped):
        """Integrate an acceleration command into a :class:`TwistStamped` message.

        Accumulates linear and angular velocities by integrating the
        acceleration values over ``self.timer_period`` and writes the
        result into ``msg``. Switches the Servo input mode to ``TWIST``
        if not already active.

        Args:
            pose_command: Shape ``(2, 3)`` array. Row 0 is linear
                acceleration ``[ax, ay, az]``; row 1 is angular
                acceleration ``[αx, αy, αz]``.
            msg: The :class:`TwistStamped` message to populate.
        """
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

    def construct_twist_msg(self, pose_command: np.ndarray, msg: TwistStamped):
        """Populate a :class:`TwistStamped` from a velocity command.

        Switches the Servo input mode to ``TWIST`` if not already active
        and writes the velocity values directly into ``msg``.

        Args:
            pose_command: Shape ``(2, 3)`` array. Row 0 is linear velocity
                ``[vx, vy, vz]``; row 1 is angular velocity
                ``[ωx, ωy, ωz]``.
            msg: The :class:`TwistStamped` message to populate.
        """
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

    def construct_pose_msg(self, pose_command: np.ndarray, msg: PoseStamped):
        """Populate a :class:`PoseStamped` from a position command.

        Switches the Servo input mode to ``POSE`` if not already active
        and writes the pose values into ``msg``.

        Args:
            pose_command: Shape ``(2, 4)`` array. Row 0 is
                ``[x, y, z, 1.0]``; row 1 is ``[qx, qy, qz, qw]``.
            msg: The :class:`PoseStamped` message to populate.
        """
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
        """Action feedback callback — sets status to ``"done"`` on receipt.

        Args:
            feedback_msg: The feedback message from the ``posegoal`` action
                server (currently unused beyond updating status).
        """
        self.status = "done"

    def pose_response_callback(self, future):
        """Action goal response callback.

        Logs whether the goal was accepted and, if so, registers
        :meth:`pose_result_callback` on the result future.

        Args:
            future: The future returned by
                :meth:`~rclpy.action.ActionClient.send_goal_async`.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.pose_result_callback)

    def pose_result_callback(self, future):
        """Action result callback.

        Updates ``self.status`` from the action result and logs success
        or failure.

        Args:
            future: The future returned by
                :meth:`~rclpy.action.GoalHandle.get_result_async`.
        """
        result = future.result().result
        self.status = result.status_result
        if result.success:
            self.get_logger().info("Motion completed successfully")
        else:
            self.get_logger().warn("Motion failed")

    def config_callback(self):
        """Timer callback (2 Hz) that polls the codi server for config
        changes and activates or deactivates lifecycle nodes accordingly."""
        config = self.codi_server.get_config()
        if config != (
            self.config["controller"]["value"],
            self.config["camera"]["value"],
        ):
            self.get_logger().info(f"Configuration changed to{config}")
            (
                self.config["controller"]["value"],
                self.config["camera"]["value"],
            ) = config
            self.apply_config()

    def switch_command_type(self, cmd_type: str):
        """Switch the MoveIt Servo input command type.

        Calls the ``servo_node/switch_command_type`` service to change
        between ``JOINT_JOG``, ``TWIST``, and ``POSE`` modes.

        Args:
            cmd_type: One of ``"JOINT_JOG"``, ``"TWIST"``, or ``"POSE"``.

        Raises:
            ValueError: If ``cmd_type`` is not one of the accepted values.
        """
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
    """Entry point for the ``codi_node`` executable.

    Initialises rclpy, spins the :class:`CodiNode`, then shuts down
    cleanly on exit. gripper_command

    Args:
        args: Optional command-line arguments passed to
            :func:`rclpy.init`.
    """
    rclpy.init(args=args)
    node = CodiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
