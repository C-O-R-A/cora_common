
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "cora_msgs/action/pose_goal.hpp"

#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

class MoverNodeServer : public rclcpp::Node
{
    public:
        using PoseGoal = cora_msgs::action::PoseGoal;
        using GoalHandlePoseGoal = rclcpp_action::ServerGoalHandle<PoseGoal>;

        explicit MoverNodeServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("mover_node", options)
        {
            using namespace std::placeholders;

            this->actionserver = rclcpp_action::create_server<PoseGoal>(
                this->get_node_base_interface(),
                this->get_node_clock_interface(),
                this->get_node_logging_interface(),
                this->get_node_waitables_interface(),
                "pose_goal",
                std::bind(&MoverNodeServer::handle_goal, this, _1, _2),
                std::bind(&MoverNodeServer::handle_cancel, this, _1),
                std::bind(&MoverNodeServer::handle_accepted, this, _1));
        }
    
    private:
        rclcpp_action::Server<PoseGoal>::SharedPtr action_server_;

        rclcpp_action::GoalResponse handle_goal(
            rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const PoseGoal::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger, "Received goal request");
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandlePoseGoal> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void execute(){

        }

        void handle_accepted(){

        }
};

int main(int argc, char ** argv){
    rclcpp::init
}