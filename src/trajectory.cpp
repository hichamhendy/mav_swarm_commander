#include <ros/ros.h>

#include "mav_swarm_commander/trajectory.h"


void Trajectory::setFromMsg(const manager_msgs::OffboardTrajectorySetpoint& msg)
{
    points_.clear();
    for (const auto& p : msg.points)
    {
      Eigen::VectorXd point_holder(8);
      point_holder[0] = p.positions[0];
      point_holder[1] = p.positions[1];
      point_holder[2] = p.positions[2];
      point_holder[3] = p.velocities[0];
      point_holder[4] = p.velocities[1];
      point_holder[5] = p.velocities[2];
      point_holder[6] = p.accelerations[0];
      point_holder[7] = p.accelerations[1];
      point_holder[8] = p.accelerations[2];

      this->addPoint(point_holder);
    }
}

void Trajectory::addPoint(const Eigen::VectorXd& p) 
{ 
    points_.push_back(p); 
}


visualization_msgs::MarkerArray Trajectory::visualizationMarkerMsg(const std_msgs::ColorRGBA& color, const double scale) const
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
        ros_p.x = p[0];
        ros_p.y = p[1];
        ros_p.z = p[2];
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
        ros_p.x = p[0];
        ros_p.y = p[1];
        ros_p.z = p[2];
        connection_marker.points.push_back(ros_p);
      }
      marker_array.markers.push_back(connection_marker);
    }

    return marker_array;
}
