#ifndef PATROL_NAVIGATION_PROJECT_CPP__PATROL_VISUALIZER_HPP_
#define PATROL_NAVIGATION_PROJECT_CPP__PATROL_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "patrol_navigation_project_cpp/waypoint_manager.hpp"

namespace patrol_navigation_project_cpp
{

class PatrolVisualizer : public rclcpp::Node
{
public:
    PatrolVisualizer();
    ~PatrolVisualizer() = default;

    void visualizeWaypoints(const std::vector<PatrolPoint>& waypoints);
    void visualizeCurrentTarget(const PatrolPoint& current, size_t index);
    void visualizePatrolPath(const std::vector<PatrolPoint>& waypoints);

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr current_target_pub_;
    
    visualization_msgs::msg::Marker createWaypointMarker(
        const PatrolPoint& point, 
        size_t index);
    
    visualization_msgs::msg::Marker createPathMarker(
        const std::vector<PatrolPoint>& waypoints);
};

}  // namespace patrol_navigation_project_cpp

#endif  // PATROL_NAVIGATION_PROJECT_CPP__PATROL_VISUALIZER_HPP_