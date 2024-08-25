#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <learning_arm_msgs/action/learning_arm_task.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <thread>

using namespace std::placeholders;

namespace learning_arm_remote
{
    class TaskServer : public rclcpp::Node
    {
    public:
        explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
            : Node("task_server", options)
        {
            action_server_ = rclcpp_action::create_server<learning_arm_msgs::action::LearningArmTask>(
                this, "task_server", 
                std::bind(&TaskServer::goalCallback, this, _1, _2),
                std::bind(&TaskServer::cancelCallback, this, _1),
                std::bind(&TaskServer::acceptedCallback, this, _1)
            );

            RCLCPP_INFO(this->get_logger(), "Starting the Action Server");  // Fixed logger usage
        }

    private:
        rclcpp_action::Server<learning_arm_msgs::action::LearningArmTask>::SharedPtr action_server_;

        rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const learning_arm_msgs::action::LearningArmTask::Goal> goal)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request with id: " << goal->task_number);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<learning_arm_msgs::action::LearningArmTask>> goal_handle)
        {
            std::thread{std::bind(&TaskServer::execute, this, goal_handle)}.detach();  // Fixed thread creation
        }

        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<learning_arm_msgs::action::LearningArmTask>> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
            auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");  // Fixed typo

            std::vector<double> arm_joint_goal;
            std::vector<double> gripper_joint_goal;

            if (goal_handle->get_goal()->task_number == 0)
            {
                arm_joint_goal = {0.0, 0.0, 0.0};  // Fixed missing commas
                gripper_joint_goal = {-0.7, 0.7};  // Fixed missing commas
            }
            else if (goal_handle->get_goal()->task_number == 1)
            {
                arm_joint_goal = {-1.14, -0.6, -0.07};  // Fixed missing commas
                gripper_joint_goal = {0.0, 0.0};  // Fixed missing commas
            }
            else if (goal_handle->get_goal()->task_number == 2)  // Fixed task number comparison
            {
                arm_joint_goal = {-1.57, 0.0, -1.0};  // Fixed missing commas
                gripper_joint_goal = {0.0, 0.0};  // Fixed missing commas
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid Task Number");  // Fixed logger usage
                return;
            }

            bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
            bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);
            
            if (!arm_within_bounds || !gripper_within_bounds)  // Fixed bitwise OR to logical OR
            {
                RCLCPP_WARN(this->get_logger(), "Target joint positions were outside limits");  // Fixed logger usage
            }

            moveit::planning_interface::MoveGroupInterface::Plan arm_plan; 
            moveit::planning_interface::MoveGroupInterface::Plan gripper_plan; 

            bool arm_plan_success = arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS;
            bool gripper_plan_success = gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS;

            if (arm_plan_success && gripper_plan_success)
            {
                RCLCPP_INFO(this->get_logger(), "Planner Succeeded, moving the arm and the gripper");  // Fixed logger usage and capitalization
                arm_move_group.move();
                gripper_move_group.move();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "One or more planners failed");  // Fixed logger usage
            }

            auto result = std::make_shared<learning_arm_msgs::action::LearningArmTask::Result>();  // Fixed template usage
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal Succeeded");  // Fixed logger usage and capitalization
        }

        rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<learning_arm_msgs::action::LearningArmTask>> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel the goal");  // Fixed logger usage
            auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
            auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");  // Fixed typo

            arm_move_group.stop();
            gripper_move_group.stop();
            return rclcpp_action::CancelResponse::ACCEPT;
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(learning_arm_remote::TaskServer)
