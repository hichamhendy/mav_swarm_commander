#include "mav_swarm_commander/SwarmCommander.h"

SwarmCommander::SwarmCommander(const ros::NodeHandle& nh, const ros::NodeHandle& nh_priv,
                               const ros::NodeHandle& nh_waypoint_planning, const ros::NodeHandle& nh_trajectory_planning):
                               nh_(nh), nh_private_(nh_priv), nh_waypoint_planning_(nh_waypoint_planning), 
                               nh_trajectory_planning_(nh_trajectory_planning), flyto_server_(nh_trajectory_planning_, "flyto_action", false)

{
    initial_path_pub_ = nh_trajectory_planning_.advertise<visualization_msgs::MarkerArray>("initial_path", 1);
    current_path_pub_ = nh_trajectory_planning_.advertise<visualization_msgs::MarkerArray>("current_path", 1);
    final_path_pub_ = nh_trajectory_planning_.advertise<visualization_msgs::MarkerArray>("final_path", 1);
    path_setpoint_pub_ = nh_trajectory_planning_.advertise<drakula_msgs::OffboardPathSetpoint>("path_setpoint", 1);

    topological_planning_service_client_ = nh_trajectory_planning_.serviceClient<drakula_msgs::PlanWaypoints>("/plan_waypoints");

    color_initial_path_.r = 1.0;
    color_initial_path_.g = 0.5;
    color_initial_path_.b = 0.25;
    color_initial_path_.a = 1.0;

    color_current_path_.r = 1.0;
    color_current_path_.g = 0.0;
    color_current_path_.b = 0.25;
    color_current_path_.a = 1.0;

    color_final_path_.r = 0.0;
    color_final_path_.g = 0.5;
    color_final_path_.b = 0.1;
    color_final_path_.a = 1.0;

    flyto_server_.registerGoalCallback(boost::bind(&FlightCommander::goalCallback, this));
    flyto_server_.registerPreemptCallback(boost::bind(&FlightCommander::preemptCallback, this));
    flyto_server_.start();

    dynamic_reconfigure_server_.setCallback(boost::bind(&SwarmCommander::reconfigure, this, _1, _2));

    trajectory_planning_timer_ = nh_trajectory_planning_.createTimer(ros::Duration(1.0), boost::bind(&FlightCommander::trajectoryPlanningCallback, this));
}

SwarmCommander::goalCallback()
{
    current_goal_ = flyto_server_.acceptNewGoal();
    destination_point_ = Eigen::Vector3d(current_goal_->desination.x, current_goal_->desination.y, current_goal_->desination.z);
    destination_frame_id_ = current_goal_->frame_id;

    // clear relevanr place holders
    initial_path_.points_.clear();
    current_path_.points_.clear();
    current_safe_path_.points_.clear();

    trajectoryPlanningCallback(); // initate the trajectory builder based on the received goal
}