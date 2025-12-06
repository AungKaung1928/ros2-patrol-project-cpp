#include "patrol_navigation_project_cpp/patrol_visualizer.hpp"

namespace patrol_navigation_project_cpp
{

PatrolVisualizer::PatrolVisualizer()
    : Node("patrol_visualizer")
{
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "patrol_waypoints", 10);
    
    current_target_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "current_target", 10);
    
    RCLCPP_INFO(this->get_logger(), "Patrol visualizer initialized");
}

void PatrolVisualizer::visualizeWaypoints(const std::vector<PatrolPoint>& waypoints)
{
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Create individual waypoint markers
    for (size_t i = 0; i < waypoints.size(); ++i) {
        marker_array.markers.push_back(createWaypointMarker(waypoints[i], i));
    }
    
    // Create patrol path line
    marker_array.markers.push_back(createPathMarker(waypoints));
    
    marker_pub_->publish(marker_array);
}

void PatrolVisualizer::visualizeCurrentTarget(const PatrolPoint& current, size_t index)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "current_target";
    marker.id = 999;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Position
    marker.pose.position.x = current.x;
    marker.pose.position.y = current.y;
    marker.pose.position.z = current.z + 0.5;
    
    // Orientation (pointing down)
    marker.pose.orientation.x = 0.707;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.707;
    
    // Scale (large arrow)
    marker.scale.x = 0.5;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    
    // Color (bright yellow)
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    marker.lifetime = rclcpp::Duration::from_seconds(0);  // Persist until deleted
    
    current_target_pub_->publish(marker);
    
    RCLCPP_INFO(this->get_logger(), 
        "Visualizing target %zu: %s", index, current.name.c_str());
}

visualization_msgs::msg::Marker PatrolVisualizer::createWaypointMarker(
    const PatrolPoint& point, 
    size_t index)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "waypoints";
    marker.id = static_cast<int>(index);
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Position
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = 0.0;
    
    // Orientation
    marker.pose.orientation.w = 1.0;
    
    // Scale
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.1;
    
    // Color (blue with transparency)
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 0.7;
    
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    
    return marker;
}

visualization_msgs::msg::Marker PatrolVisualizer::createPathMarker(
    const std::vector<PatrolPoint>& waypoints)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "patrol_path";
    marker.id = 1000;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Create line through all waypoints
    for (const auto& wp : waypoints) {
        geometry_msgs::msg::Point p;
        p.x = wp.x;
        p.y = wp.y;
        p.z = 0.05;  // Slightly above ground
        marker.points.push_back(p);
    }
    
    // Close the loop
    if (!waypoints.empty()) {
        geometry_msgs::msg::Point p;
        p.x = waypoints[0].x;
        p.y = waypoints[0].y;
        p.z = 0.05;
        marker.points.push_back(p);
    }
    
    // Line appearance
    marker.scale.x = 0.05;  // Line width
    
    // Color (cyan)
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5;
    
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    
    return marker;
}

}  // namespace patrol_navigation_project_cpp