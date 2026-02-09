#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit_msgs import moveit_msgs_s__rosidl_typesupport_c
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # Instantiate MoveitPy with the chosen node name, 
    # moveitpy picks up the moveit config passed 
    # from the launch to the node with this name
    cora = MoveItPy(node_name="cora_moveit_test")

    # Get planning component
    cora_arm = cora.get_planning_component("arm")
    logger.info("MoveItPy instance created")

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################
    logger.info("---------------    PLAN 1 SET STATES WITH PREDEFINED STRING    ---------------")
    # set plan start state using predefined state

    cora_arm.set_start_state_to_current_state()

    cora_arm.set_goal_state(configuration_name="standby")

    plan_and_execute(cora, cora_arm, logger, sleep_time=3.0)

    cora_arm.set_start_state_to_current_state()

    # set pose goal using predefined state
    cora_arm.set_goal_state(configuration_name="straight")

    # plan to goal
    plan_and_execute(cora, cora_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 2 - set goal state with RobotState object
    ###########################################################################

    logger.info("---------------    PLAN 2 SET GOAL STATE WITH ROBOTSTATE OBJECT    ---------------")

    # instantiate a RobotState instance using the current robot model
    robot_model = cora.get_robot_model()
    robot_state = RobotState(robot_model)

    # randomize the robot state
    robot_state.set_to_random_positions()
    robot_state.joint_positions["Finger1"] = 0.0

    # set plan start state to current state
    cora_arm.set_start_state_to_current_state()

    # set goal state to the initialized robot state
    logger.info("Set goal state to the initialized robot state")
    cora_arm.set_goal_state(robot_state=robot_state)

    # plan to goal
    plan_and_execute(cora, cora_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 3 - set goal state with PoseStamped message
    ###########################################################################

    logger.info("---------------    PLAN 3 SET GOAL STATE WITH POSESTAMPED MESSAGE    ---------------")

    # set plan start state to current state
    cora_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    from geometry_msgs.msg import PoseStamped

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "baselink"
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.28
    pose_goal.pose.position.y = -0.2
    pose_goal.pose.position.z = 0.5
    cora_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="Gripper")

    # plan to goal
    plan_and_execute(cora, cora_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 4 - set goal state with constraints
    ###########################################################################

    logger.info("---------------    PLAN 4 SET GOAL STATE WITH CONSTRAINTS    ---------------")

    # set plan start state to current state
    cora_arm.set_start_state_to_current_state()

    # set constraints message
    from moveit.core.kinematic_constraints import construct_joint_constraint

    joint_values = {
        "J1": -1.0,
        "J2": 0.7,
        "J3": 0.7,
        "J4": -1.5,
        "J5": -0.7,
        "J6": 2.0,
        "Finger1": 0.0,
    }
    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=cora.get_robot_model().get_joint_model_group("arm"),
    )
    cora_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

    # plan to goal
    plan_and_execute(cora, cora_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 5 - Planning with Multiple Pipelines simultaneously
    ###########################################################################

    logger.info("---------------    PLAN 5 PLANNING WITH MULTIPLE PIPELINES SIMULTANEOUSLY    ---------------")

    # set plan start state to current state
    cora_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    cora_arm.set_goal_state(configuration_name="standby")

    # initialise multi-pipeline plan request parameters
    multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
        cora, ["ompl_rrtc", "pilz_lin", "chomp_planner"]
    )

    # plan to goal
    plan_and_execute(
        cora,
        cora_arm,
        logger,
        multi_plan_parameters=multi_pipeline_plan_request_params,
        sleep_time=3.0,
    )


if __name__ == "__main__":
    main()