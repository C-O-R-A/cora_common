import collections
import threading
import time
import numpy as np


# rclpy library
import rclpy
from rclpy.node import Node
from cora_msgs.action import PoseGoal
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit_msgs import moveit_msgs_s__rosidl_typesupport_c
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.core.kinematic_constraints import construct_joint_constraint, construct_constraints_from_node, construct_link_constraint


class MoverNodeServer(Node):

    def __init__(self):
        super().__init__("mover_node")
        self.move = ActionServer(
            self,
            PoseGoal,
            "posegoal",
            # callback_group=ReentrantCallbackGroup(),
            handle_accepted_callback=self.handle_accepted_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
        )
        self.goal_handle = None
        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()

        # Instantiate a MoveitPy instance
        self.cora = MoveItPy(node_name="mover_node_server")
        self.get_logger().info("MoveitPy instance created!")
        # TODO: #1 planning group name is hardcoded. Read planning_groups from the
        # generated robot_layout.yaml (C-O-R-A/configurator#2) instead of "arm".
        self.arm = self.cora.get_planning_component("arm")

        # Instantiate a RobotState instance using the current robot model
        self.robot_model = self.cora.get_robot_model()

    def handle_accepted_callback(self, goal_handle):
        with self._goal_queue_lock:
            if self.goal_handle is not None:
                self.get_logger().info(f"Adding goal <<<{goal_handle}>>> to queue")
                self._goal_queue.append(goal_handle)
            else:
                self.goal_handle = goal_handle
                self.goal_handle.execute()

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        try:
            self.get_logger().info(
                f"Execution callback started on goal <<<{goal_handle}>>>"
            )
            space = goal_handle.request.space.strip().upper()
            interface_type = goal_handle.request.interface_type.strip().lower()
            predefined_pose = None
            gripper_goal = goal_handle.request.gripper_goal
            arm_planning_group = "arm"
            gripper_planning_group = "gripper_fingers"

            self.arm_planning_component = self.cora.get_planning_component(arm_planning_group)
            self.arm_joint_model_group = self.robot_model.get_joint_model_group(arm_planning_group)            
            

            # Set start state to the current state
            goal_state = RobotState(self.robot_model)
            self.arm_planning_component.set_start_state_to_current_state()

            # Selecting a predefined pose takes precidence over any other goals
            if goal_handle.request.predefined_pose:
                predefined_pose = goal_handle.request.predefined_pose.strip().lower()
                self.get_logger().info(
                    f"Using predefined pose: {predefined_pose}"
                )
                self.arm_planning_component.set_goal_state(configuration_name=predefined_pose)
                self.plan_and_execute(self.arm_planning_component)

            # If no predefined pose is specified, proceed to set goal based on space
            else:

                # Gripper Goal
                # TODO: #4 this condition is ALWAYS TRUE — gripper_goal is a float action
                # field defaulting to 0.0, so the node always selects "arm_with_gripper"
                # and fails outright on a gripper-less robot. Check whether the group
                # exists in robot_layout.yaml instead of `is not None`.
                # TODO: #1 both group names below are hardcoded.
                if gripper_goal is not None:
                    gripper_state = RobotState(self.robot_model)
                    gripper_state.joint_positions = {'Finger1': gripper_goal}
                    gripper_constraint = construct_joint_constraint(
                        robot_state=gripper_state,
                        joint_model_group=self.joint_model_group,
                    )

                # Task Space Goal
                if space == "TS":
                    self.get_logger().info("Setting Task Space goal...")

                    # Extract task space goal and target from action request
                    # pose goal should be posestamped msg sent from the client
                    try:
                        if interface_type == "position":
                            pose_goal = goal_handle.request.pose_goal.pose
                            target = goal_handle.request.pose_goal.target_frame

                            # Set goal state
                            self.arm.set_goal_state(
                            # self.cora_planning_component.set_goal_state(
                                pose_stamped_msg=pose_goal, 
                                pose_link=target,
                                )
                            self.plan_and_execute(self.arm)

                    except ValueError as e:
                        self.get_logger().error(e + "Cancelling motion plan request...")
                        goal_handle.abort()
                        return PoseGoal.Result()

                # Joint Space Goal
                elif space == "JS":
                    joint_interface_methods = {
                        "position": goal_state.set_joint_group_positions,
                        "velocity": goal_state.set_joint_group_velocities,
                        "effort": goal_state.joint_efforts,
                        "acceleration": goal_state.set_joint_group_accelerations,
                    }
                    self.get_logger().info("Setting Joint Space goal...")

                    # Extract joint space goal array from action request
                    joint_goal = goal_handle.request.joint_goal

                    # Assign values from array to dict elements
                    joint_values = np.array(joint_goal, dtype=np.float64)

                    # Set joint values to the correct interface
                    joint_interface_methods[interface_type]('arm', joint_values)

                    self.arm_planning_component.set_goal_state(
                            robot_state=goal_state,
                            )
                    
                    self.plan_and_execute(self.arm_planning_component)

                self.arm_planning_component.set_start_state_to_current_state()

                # Gripper Goal
                if gripper_goal is not None:
                    self.gripper_planning_component = self.cora.get_planning_component(gripper_planning_group)
                    self.gripper_joint_model_group = self.robot_model.get_joint_model_group(gripper_planning_group)

                    gripper_state = RobotState(self.robot_model)
                    gripper_state.joint_positions = {'Finger1': gripper_goal}
                    gripper_constraint = construct_joint_constraint(
                        robot_state=gripper_state,
                        joint_model_group=self.gripper_joint_model_group,
                    )

                    self.gripper_planning_component.set_goal_state(
                        motion_plan_constraints=[gripper_constraint]
                    )
                    self.plan_and_execute(self.gripper_planning_component)
    

            # Create a PoseStamped message
            pose_stamped_result = PoseStamped()
            pose_stamped_result.header.stamp = (
                self.get_clock().now().to_msg()
            )  # current ROS time

            # TODO: #1 "endeffector" does not exist on configurator-generated robots
            # (their tip link is "<lastJoint>_joint_out"). Use ee_frame from
            # robot_layout.yaml here and in get_pose() below.
            pose_stamped_result.header.frame_id = "endeffector"  # or the frame you used
            pose_stamped_result.pose = goal_state.get_pose(
                "endeffector"
            )  # the Pose object

            result = PoseGoal.Result()
            result.pose_result = pose_stamped_result
            result.status_result = "Complete"
            result.success = True
            goal_handle.succeed()

            return result

        finally:
            with self._goal_queue_lock:
                try:
                    self.get_logger().info("Retrieving goal from queue")
                    self.goal_handle = self._goal_queue.popleft()
                    self.goal_handle.execute()
                except IndexError:
                    self.get_logger().info("No goals left in queue")
                    self.goal_handle = None

    def plan_and_execute(self, planning_component):
            self.get_logger().info("Planning Trajectory")
            plan_result = planning_component.plan()

            # execute the plan
            if plan_result:
                self.get_logger().info("Executing plan")
                robot_trajectory = plan_result.trajectory
                # TODO: #1 controller names hardcoded. Read the `controllers` map from
                # robot_layout.yaml — a robot may have no gripper controller at all.
                self.cora.execute(robot_trajectory, controllers=["arm_controller", 'gripper_fingers_controller'])
            else:
                self.get_logger().error("Planning failed")
                return PoseGoal.Result()

    def destroy(self):
        self.move.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    action_server = MoverNodeServer()

    executor = SingleThreadedExecutor()
    try:
        executor.add_node(action_server)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()
