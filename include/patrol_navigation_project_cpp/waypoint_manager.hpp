#ifndef PATROL_NAVIGATION_PROJECT_CPP__WAYPOINT_MANAGER_HPP_
#define PATROL_NAVIGATION_PROJECT_CPP__WAYPOINT_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <memory>

namespace patrol_navigation_project_cpp
{

struct PatrolPoint
{
    std::string name;
    double x;
    double y;
    double z;
};

class WaypointManager : public rclcpp::Node
{
public:
    WaypointManager();
    ~WaypointManager() = default;

    void loadPatrolPoints();
    std::vector<PatrolPoint> getPatrolPoints() const;
    std::pair<size_t, PatrolPoint> getNextPoint(size_t current_index) const;

private:
    std::vector<PatrolPoint> patrol_points_;
};

}  // namespace patrol_navigation_project_cpp

#endif  // PATROL_NAVIGATION_PROJECT_CPP__WAYPOINT_MANAGER_HPP_