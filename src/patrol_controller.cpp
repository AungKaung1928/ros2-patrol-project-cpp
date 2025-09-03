#include "patrol_navigation_project_cpp/patrol_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <thread>
#include <cmath>

namespace patrol_navigation_project_cpp
{

using namespace std::chrono_literals;

PatrolController::PatrolController()
    : Node("patrol_controller"),
      current_point_index_(0),
      is_patrolling_(false)
{
    // Initialize action client for navigation
    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, "navigate_to_pose");
    
    // Initialize initial pose publisher
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10);
    
    // Initialize waypoint manager
    waypoint_manager_ = std::make_shared<WaypointManager>();
    patrol_points_ = waypoint_manager_->getPatrolPoints();
    
    // Wait for Nav2 to be ready
    RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 to be ready...");
    waitForNav2();
    RCLCPP_INFO(this->get_logger(), "Nav2 is ready!");
    
    // Start patrol
    startPatrol();
}

void PatrolController::waitForNav2()
{
    // Wait for the action server to be available
    while (!navigate_to_pose_client_->wait_for_action_server(5s)) {
        RCLCPP_INFO(this->get_logger(), 
            "Waiting for navigate_to_pose action server to be available...");
    }
    
    // Additional wait to ensure Nav2 is fully ready
    RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 to fully initialize...");
    std::this_thread::sleep_for(10s);  // Give Nav2 time to fully start
    
    // Set initial pose (at origin)
    setInitialPose();
    
    // Wait more for navigation stack to be fully ready after initial pose
    RCLCPP_INFO(this->get_logger(), "Waiting for navigation stack to activate...");
    std::this_thread::sleep_for(5s);  // Additional wait after setting initial pose
}

geometry_msgs::msg::PoseStamped PatrolController::createPoseStamped(
    double x, double y, double z, double yaw)
{
    geometry_msgs::msg::PoseStamped pose;
    
    // Set reference frame and timestamp
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    
    // Set 3D position coordinates
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    
    // Convert yaw to quaternion (FIXED VERSION)
    // For 2D rotation around Z-axis: q = [0, 0, sin(yaw/2), cos(yaw/2)]
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    
    return pose;
}

void PatrolController::setInitialPose()
{
    // Publish initial pose to set robot location at origin
    auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = this->now();
    
    // Set position at origin (0, 0, 0)
    initial_pose.pose.pose.position.x = 0.0;
    initial_pose.pose.pose.position.y = 0.0;
    initial_pose.pose.pose.position.z = 0.0;
    
    // Set orientation (facing forward)
    initial_pose.pose.pose.orientation.x = 0.0;
    initial_pose.pose.pose.orientation.y = 0.0;
    initial_pose.pose.pose.orientation.z = 0.0;
    initial_pose.pose.pose.orientation.w = 1.0;
    
    // Set covariance (small values for high confidence)
    for (int i = 0; i < 36; i++) {
        initial_pose.pose.covariance[i] = 0.0;
    }
    initial_pose.pose.covariance[0] = 0.08;  // x variance
    initial_pose.pose.covariance[7] = 0.08;  // y variance
    initial_pose.pose.covariance[35] = 0.05; // yaw variance
    
    // Publish initial pose
    initial_pose_pub_->publish(initial_pose);
    RCLCPP_INFO(this->get_logger(), "Published initial pose at origin");
    
    // Wait a bit for AMCL to process the initial pose
    std::this_thread::sleep_for(2s);
}

void PatrolController::startPatrol()
{
    RCLCPP_INFO(this->get_logger(), "Starting patrol...");
    is_patrolling_ = true;
    goToNextPoint();
}

void PatrolController::goToNextPoint()
{
    if (patrol_points_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No patrol points available!");
        return;
    }
    
    const auto& current_point = patrol_points_[current_point_index_];
    RCLCPP_INFO(this->get_logger(), 
        "Going to %s at (%.2f, %.2f)",
        current_point.name.c_str(), current_point.x, current_point.y);
    
    // Create goal pose
    auto goal_pose = createPoseStamped(
        current_point.x,
        current_point.y,
        current_point.z
    );
    
    // Create goal message
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose;
    
    // Send goal options
    auto send_goal_options = 
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
        std::bind(&PatrolController::goalResponseCallback, this, std::placeholders::_1);
    
    send_goal_options.feedback_callback =
        std::bind(&PatrolController::feedbackCallback, this, 
            std::placeholders::_1, std::placeholders::_2);
    
    send_goal_options.result_callback =
        std::bind(&PatrolController::resultCallback, this, std::placeholders::_1);
    
    // Send goal to navigator
    goal_handle_future_ = navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
}

void PatrolController::goalResponseCallback(
    const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        goal_handle_ = goal_handle;
    }
}

void PatrolController::feedbackCallback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    // Optional: Log progress
    (void)feedback;  // Suppress unused parameter warning
}

void PatrolController::resultCallback(
    const GoalHandleNavigateToPose::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            {
                const auto& current_point = patrol_points_[current_point_index_];
                RCLCPP_INFO(this->get_logger(), "Reached %s!", current_point.name.c_str());
                
                // Wait a bit at the patrol point
                std::this_thread::sleep_for(2s);
                
                // Move to next point
                current_point_index_ = (current_point_index_ + 1) % patrol_points_.size();
                
                // Continue patrol if still active
                if (is_patrolling_) {
                    goToNextPoint();
                }
            }
            break;
            
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Navigation was canceled");
            break;
            
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Navigation failed! Retrying...");
            std::this_thread::sleep_for(2s);
            if (is_patrolling_) {
                goToNextPoint();
            }
            break;
            
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
    }
}

void PatrolController::stopPatrol()
{
    RCLCPP_INFO(this->get_logger(), "Stopping patrol...");
    is_patrolling_ = false;
    
    if (goal_handle_) {
        auto cancel_future = navigate_to_pose_client_->async_cancel_goal(goal_handle_);
    }
}

}  // namespace patrol_navigation_project_cpp

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto patrol_controller = 
        std::make_shared<patrol_navigation_project_cpp::PatrolController>();
    
    try {
        rclcpp::spin(patrol_controller);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(patrol_controller->get_logger(), 
            "Exception in patrol_controller: %s", e.what());
        patrol_controller->stopPatrol();
    }
    
    rclcpp::shutdown();
    return 0;
}