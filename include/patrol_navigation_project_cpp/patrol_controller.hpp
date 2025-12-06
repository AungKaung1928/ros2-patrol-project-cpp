#ifndef PATROL_NAVIGATION_PROJECT_CPP__PATROL_CONTROLLER_HPP_
#define PATROL_NAVIGATION_PROJECT_CPP__PATROL_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include "patrol_navigation_project_cpp/waypoint_manager.hpp"
#include "patrol_navigation_project_cpp/patrol_visualizer.hpp"

namespace patrol_navigation_project_cpp
{

class PatrolController : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    PatrolController();
    ~PatrolController() = default;

    void startPatrol();
    void stopPatrol();

private:
    // Navigation action client
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
    
    // Initial pose publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    
    // Waypoint manager
    std::shared_ptr<WaypointManager> waypoint_manager_;
    std::vector<PatrolPoint> patrol_points_;
    
    // Visualizer
    std::shared_ptr<PatrolVisualizer> visualizer_;
    
    // Patrol state
    size_t current_point_index_;
    bool is_patrolling_;
    
    // Timer for checking navigation status
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Current goal handle
    std::shared_future<GoalHandleNavigateToPose::SharedPtr> goal_handle_future_;
    GoalHandleNavigateToPose::SharedPtr goal_handle_;
    
    // Methods
    geometry_msgs::msg::PoseStamped createPoseStamped(double x, double y, double z = 0.0, double yaw = 0.0);
    void goToNextPoint();
    void checkNavigationResult();
    void waitForNav2();
    void setInitialPose();
    
    // Action callbacks
    void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
    void feedbackCallback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result);
};

}  // namespace patrol_navigation_project_cpp

#endif  // PATROL_NAVIGATION_PROJECT_CPP__PATROL_CONTROLLER_HPP_