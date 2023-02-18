#include <ros/ros.h>

#include "mav_swarm_commander/path.h"

std::vector<Eigen::Vector3d> sampleSegment(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const double desired_step_length)
{
  std::vector<Eigen::Vector3d> result;

  const Eigen::Vector3d dir = (p2 - p1).normalized();
  const double segment_length = (p2 - p1).norm();
  const std::size_t num_steps = std::round(segment_length / desired_step_length);

  const double step_length = segment_length / num_steps;

  for (int j = 0; j < num_steps + 1; j++)
  {
    result.push_back(p1 + dir * step_length * j);
  }

  return result;
}

std::pair<int, int> Path::closestSegment(const Eigen::Vector3d& p) const
{
  double min_dist = std::numeric_limits<double>::infinity();
  std::pair<int, int> best_segment{-1, -1};

  for (std::size_t i = 0; i < points_.size() - 1; i++)
  {
    const auto line_segment = LineSegment(points_.at(i), points_.at(i + 1));
    const double dist = line_segment.pointDistance(p);
    if (dist < min_dist)
    {
      min_dist = dist;
      best_segment.first = i;
      best_segment.second = i + 1;
    }
  }

  return best_segment;
}

void Path::setFromMsg(const manager_msgs::OffboardPathSetpoint& msg)
{
  points_.clear();
  for (const auto& p : msg.waypoints)
  {
    this->addPoint(Eigen::Vector3d(p.x, p.y, p.z));
  }
}

void Path::addPoint(const Eigen::Vector3d& p) 
{ 
  points_.push_back(p); 
}


visualization_msgs::MarkerArray Path::visualizationMarkerMsg(const std_msgs::ColorRGBA& color, const double scale) const
{
  visualization_msgs::MarkerArray marker_array;

  // 1. The Waypoints
  {
    visualization_msgs::Marker waypoint_marker;
    waypoint_marker.header.frame_id = "odom";
    waypoint_marker.ns = "waypoints";
    waypoint_marker.id = 0;
    waypoint_marker.type = visualization_msgs::Marker::CUBE_LIST;
    waypoint_marker.scale.x = waypoint_marker.scale.y = waypoint_marker.scale.z = 0.2 * scale;
    waypoint_marker.action = visualization_msgs::Marker::ADD;
    waypoint_marker.pose.orientation.w = 1.0;

    waypoint_marker.color = color;

    for (const auto& p : points_)
    {
      geometry_msgs::Point ros_p;
      ros_p.x = p.x();
      ros_p.y = p.y();
      ros_p.z = p.z();
      waypoint_marker.points.push_back(ros_p);
    }
    marker_array.markers.push_back(waypoint_marker);
  }

  // 2. Connections between the Waypoints
  {
    visualization_msgs::Marker connection_marker;
    connection_marker.header.frame_id = "odom";
    connection_marker.ns = "connections";
    connection_marker.id = 0;
    connection_marker.type = visualization_msgs::Marker::LINE_STRIP;
    connection_marker.scale.x = 0.1 * scale;
    connection_marker.action = visualization_msgs::Marker::ADD;
    connection_marker.pose.orientation.w = 1.0;

    connection_marker.color = color;

    for (const auto& p : points_)
    {
      geometry_msgs::Point ros_p;
      ros_p.x = p.x();
      ros_p.y = p.y();
      ros_p.z = p.z();
      connection_marker.points.push_back(ros_p);
    }
    marker_array.markers.push_back(connection_marker);
  }

  return marker_array;
}
