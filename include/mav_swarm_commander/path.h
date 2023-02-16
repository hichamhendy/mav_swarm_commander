#pragma once


#include <deque>

#include <manager_msgs/OffboardPathSetpoint.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "LineSegment.h"

struct Path;

std::vector<Eigen::Vector3d> sampleSegment(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                                           const double desired_step_length);

struct Path
{
    /***
     * Contructor of the struc
    */
    Path() = default;

    /**
     * The idea to recieve a message of the waypoints  from a global planner 
     * @note The message should meant to be:
     *  Header header
        geometry_msgs/Point destination
        float64 desired_yaw
        float64 min_obstacle_distance  
    */
    void setFromMsg(const manager_msgs::OffboardPathSetpoint& msg);

    /**
     * just a pushback instead on a deque
    */
    void addPoint(const Eigen::Vector3d& p);

    /*
    * In some cases planner tends to make big steps, from previous experience,
    * that proves some agressivness; thus lays some inbetween points.
    *
    * @note limited to 2 points. Additionally duplicatios must get removed here
    */
    std::vector<Eigen::Vector3d> sampledPoints(const double desired_step_length) const;

    /**
     * The function serves the purpose of retrieving the closest segment to a waypoint
     * This function is made to be asked parallely about by which the segment we fly
    */
    std::pair<int, int> closestSegment(const Eigen::Vector3d& p) const;

    /**
     *  A validation method on aerial ground station
    */
    visualization_msgs::MarkerArray visualizationMarkerMsg(const std_msgs::ColorRGBA& color, const double scale = 1.0) const;

    /*
    * the waypoint put in a deque to assurea fast insertion and deletion at both its 
    * beginning and its end. 
    */
    std::deque<Eigen::Vector3d> points_;
};

