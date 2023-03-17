#pragma once

#include <deque>

#include <manager_msgs/OffboardTrajectorySetpoint.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mav_swarm_commander/line_segment.h"


struct Trajectory
{
    /***
     * Contructor of the struc
    */
    Trajectory() = default;

    /**
     * The idea to recieve a message of the waypoints from a global planner 
     * @note The message should meant to be:
     *  Header header
        geometry_msgs/Point destination
        float64 desired_yaw
        float64 min_obstacle_distance  
    */
    void setFromMsg(const manager_msgs::OffboardTrajectorySetpoint& msg);

    /**
     * just a pushback instead on a deque
    */
    void addPoint(const Eigen::VectorXd& p);

    /**
     *  A validation method on aerial ground station
    */
    visualization_msgs::MarkerArray visualizationMarkerMsg(const std_msgs::ColorRGBA& color, const double scale = 1.0) const;

    /*
    * the waypoint of the trajectory put in a deque to assure fast insertion and deletion at both its 
    * beginning and its end. 
    */
    std::deque<Eigen::VectorXd> points_;
};

