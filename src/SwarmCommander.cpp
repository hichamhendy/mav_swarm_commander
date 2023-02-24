#include "mav_swarm_commander/SwarmCommander.h"

SwarmCommander::SwarmCommander(const ros::NodeHandle& nh, const ros::NodeHandle& nh_priv,
                               const ros::NodeHandle& nh_waypoint_planning, const ros::NodeHandle& nh_trajectory_planning):
                               nh_(nh), nh_private_(nh_priv), nh_waypoint_planning_(nh_waypoint_planning), 
                               nh_trajectory_planning_(nh_trajectory_planning), flyto_server_(nh_trajectory_planning_, "flyto_action", false)

{
    initial_path_pub_ = nh_trajectory_planning_.advertise<visualization_msgs::MarkerArray>("/initial_path", 1);
    current_path_pub_ = nh_trajectory_planning_.advertise<visualization_msgs::MarkerArray>("/current_path", 1);
    final_path_pub_ = nh_trajectory_planning_.advertise<visualization_msgs::MarkerArray>("/final_path", 1);
    path_setpoint_pub_ = nh_trajectory_planning_.advertise<manager_msgs::OffboardPathSetpoint>("/path_setpoint", 1);
    offboard_mode_position_setpoint_marker_pub_ = nh_trajectory_planning_.advertise<geometry_msgs::PoseStamped>("/offboard_position_setpoint", 1);

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

    flyto_server_.registerGoalCallback(boost::bind(&SwarmCommander::goalCallback, this));
    flyto_server_.registerPreemptCallback(boost::bind(&SwarmCommander::preemptCallback, this));
    flyto_server_.start();

    dynamic_reconfigure_server_.setCallback(boost::bind(&SwarmCommander::reconfigure, this, _1, _2));

    trajectory_planning_timer_ = nh_trajectory_planning_.createTimer(ros::Duration(1.0), boost::bind(&SwarmCommander::trajectoryPlanningCallback, this));

    ROS_INFO_STREAM("==================================================================================");
    ROS_INFO_STREAM("SwarmCommander has been successfully contructed");
    ROS_INFO_STREAM("==================================================================================");
}

bool SwarmCommander::updateCopterPosition()
{
    try
    {
        const geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0));
        current_copter_position_ = {transform_stamped.transform.translation.x, transform_stamped.transform.translation.y,
                                    transform_stamped.transform.translation.z};
        current_copter_orientation_ = {transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x,
                                    transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z};

    if (!loiter_position_ || !loiter_orientation_)
    {
        loiter_position_ = current_copter_position_;
        loiter_orientation_ = current_copter_orientation_;
    }

        return true;
    }
    catch (tf2::TransformException& ex)
    {
        ROS_ERROR_STREAM("Could not get the tf transform odom -> base_link");
        return false;
    }
}

void SwarmCommander::goalCallback()
{
    current_goal_ = flyto_server_.acceptNewGoal();
    destination_point_ = Eigen::Vector3d(current_goal_->destination.x, current_goal_->destination.y, current_goal_->destination.z);
    destination_frame_id_ = current_goal_->frame_id;

    // clear relevant place holders
    initial_path_.points_.clear();
    current_path_.points_.clear();
    current_safe_path_.points_.clear();

    trajectoryPlanningCallback(); // initate the trajectory builder based on the received goal
}

void SwarmCommander::preemptCallback()
{
    flyto_server_.setPreempted(); // preempting

    // clear relevant place holders
    initial_path_.points_.clear();
    current_path_.points_.clear();
    current_safe_path_.points_.clear();

    if (updateCopterPosition())
    {
        // TODO: insert the current position in safe path
        // safe_path.addPoint(current_copter_position_);
    }
    // another idea code be reached by acuring 
}

void SwarmCommander::finishCurrentGoal()
{
    manager_msgs::FlyToResult result;
    result.success = true;
    result.dist_to_destination =  (current_copter_position_ - destination_point_).norm();

    flyto_server_.setSucceeded(result); // send result als suceeded over the erver to set the status of the active goal.

    // clear relevant place holders
    initial_path_.points_.clear();
    current_path_.points_.clear();
    current_safe_path_.points_.clear();

    initial_path_pub_.publish(Path().visualizationMarkerMsg(color_initial_path_));
    final_path_pub_.publish(Path().visualizationMarkerMsg(color_final_path_));
}

void SwarmCommander::abortCurrentGoal()
{
    manager_msgs::FlyToResult result;
    result.success = false;
    result.dist_to_destination = (current_copter_position_ - destination_point_).norm();

    flyto_server_.setAborted(result);

    // clear relevant place holders
    initial_path_.points_.clear();
    current_path_.points_.clear();
    current_safe_path_.points_.clear();

    initial_path_pub_.publish(Path().visualizationMarkerMsg(color_initial_path_));
    final_path_pub_.publish(Path().visualizationMarkerMsg(color_final_path_));
}


void SwarmCommander::trajectoryPlanningCallback()
{

}

void SwarmCommander::reconfigure(mav_swarm_commander::SwarmCommanderConfig& config, uint32_t level)
{
	config_ = config;
}