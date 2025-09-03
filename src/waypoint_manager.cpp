#include "patrol_navigation_project_cpp/waypoint_manager.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

namespace patrol_navigation_project_cpp
{

WaypointManager::WaypointManager()
    : Node("waypoint_manager")
{
    loadPatrolPoints();
}

void WaypointManager::loadPatrolPoints()
{
    try {
        // Get package share directory
        std::string package_share_directory = 
            ament_index_cpp::get_package_share_directory("patrol_navigation_project_cpp");
        std::string yaml_file = package_share_directory + "/config/patrol_points.yaml";
        
        // Load YAML file
        YAML::Node config = YAML::LoadFile(yaml_file);
        
        if (config["patrol_points"]) {
            for (const auto& point : config["patrol_points"]) {
                PatrolPoint patrol_point;
                patrol_point.name = point["name"].as<std::string>();
                patrol_point.x = point["x"].as<double>();
                patrol_point.y = point["y"].as<double>();
                patrol_point.z = point["z"].as<double>();
                
                patrol_points_.push_back(patrol_point);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "Loaded %zu patrol points", patrol_points_.size());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), 
            "Failed to load patrol points: %s", e.what());
        
        // Default points if file loading fails
        patrol_points_ = {
            {"point_1", 2.0, 0.0, 0.0},
            {"point_2", 2.0, 2.0, 0.0},
            {"point_3", 0.0, 2.0, 0.0},
            {"point_4", 0.0, 0.0, 0.0}
        };
    }
}

std::vector<PatrolPoint> WaypointManager::getPatrolPoints() const
{
    return patrol_points_;
}

std::pair<size_t, PatrolPoint> WaypointManager::getNextPoint(size_t current_index) const
{
    size_t next_index = (current_index + 1) % patrol_points_.size();
    return std::make_pair(next_index, patrol_points_[next_index]);
}

}  // namespace patrol_navigation_project_cpp