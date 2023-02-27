#include "mav_swarm_commander/SwarmCommander.h"

SwarmCommander::SwarmCommander(const ros::NodeHandle& nh, const ros::NodeHandle& nh_priv,const ros::NodeHandle& nh_interface,
                               const ros::NodeHandle& nh_waypoint_planning,
                               const ros::NodeHandle& nh_esdf_map):
                               nh_(nh), nh_private_(nh_priv), nh_interface_(nh_interface), nh_waypoint_planning_(nh_waypoint_planning), 
                               nh_esdf_map_(nh_esdf_map),
                               flyto_server_(nh_private_, "flyto_action", false)

{
    sdf_map_.reset(new VoxelGridMap);
    sdf_map_->initMap(nh_esdf_map);

    mav_interface.reset(new nut::MavCommander(nh_interface_, true, false, true));

    initial_path_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/initial_path", 1);
    current_path_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/current_path", 1);
    final_path_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("/final_path", 1);
    path_setpoint_pub_ = nh_private_.advertise<manager_msgs::OffboardPathSetpoint>("/path_setpoint", 1);
    offboard_mode_position_setpoint_marker_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>("/offboard_position_setpoint", 1);

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
    flyto_server_.start(); // regarding auto_start in the construction the boolean value that tells the ActionServer wheteher or not to start publishing as soon as it comes up. THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS and start() should be called after construction of the server.

    dynamic_reconfigure_server_.setCallback(boost::bind(&SwarmCommander::reconfigure, this, _1, _2));

    // trajectory_planning_timer_ = nh_private_.createTimer(ros::Duration(1.0), boost::bind(&SwarmCommander::globalPlanner, this)); // in case the planner applied a horizon shift

    run_thread = std::thread(&SwarmCommander::run, this);

    ROS_INFO_STREAM("==================================================================================");
    ROS_INFO_STREAM("SwarmCommander has been successfully contructed");
    ROS_INFO_STREAM("==================================================================================");
}

SwarmCommander::~SwarmCommander()
{
    if(run_thread.joinable())
		run_thread.join();

    ros::shutdown();
}

void SwarmCommander::run()
{
	ros::Duration(2.0).sleep();
    // connect to standard prefix and with standard timeout
    ROS_INFO_STREAM(kStreamPrefix << "Trying to connect to MAV.");
    if (mav_interface->connect())
    {
        ROS_INFO_STREAM(kStreamPrefix << "Connected to MAV.");
    }
    else
    {
        ROS_ERROR_STREAM(kStreamPrefix << "Unable to connect to MAV, timeout!");
        // exit run
        return;
    }

    ros::Duration(2.0).sleep();
    ROS_INFO_STREAM(kStreamPrefix << "Trying to take off.");
    if (mav_interface->takeoff(config_.exploration_elevation))
    {
        ROS_INFO_STREAM(kStreamPrefix << "Takeoff successfull.");
    }
    else
    {
        ROS_ERROR_STREAM(kStreamPrefix << "Takeoff failed.");
        // exit run
        return;
    }
    ros::Duration(4.0).sleep();


	ros::Rate run_freq(10);
	while(ros::ok()) 
	{
		run_freq.sleep();
	}

	land();
}

void SwarmCommander::land()
{
	// call land command
    ROS_INFO_STREAM(kStreamPrefix << "Trying to land.");
    if (mav_interface->land())
    {
        ROS_INFO_STREAM(kStreamPrefix << "Landing successfull.");
    }
    else
    {
        ROS_ERROR_STREAM(kStreamPrefix << "Landing failed.");
        // exit run
        return ;
    }

    // call disarm command
    ROS_INFO_STREAM(kStreamPrefix << "Trying to disarm.");
    if (mav_interface->disarm())
    {
        ROS_INFO_STREAM(kStreamPrefix << "Disarming successfull.");
    }
    else
    {
        ROS_ERROR_STREAM(kStreamPrefix << "Disarming failed.");
        // exit run
        return;
    }

    // shut down the system
    ROS_INFO_STREAM(kStreamPrefix << "Shutdown successfull.");

	ros::shutdown();
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
        ROS_ERROR_STREAM(kStreamPrefix << "Could not get the tf transform odom -> base_link");
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

    globalPlanner(); // initate the trajectory builder based on the received goal
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
    // final_path_pub_.publish(Path().visualizationMarkerMsg(color_final_path_));
}

void SwarmCommander::globalPlanner()
{
    // Try to get the current copter position
    if (!updateCopterPosition())
    {
        ROS_ERROR_STREAM("Could not get the current copter position => Action failed!");
        abortCurrentGoal();
        ROS_INFO_STREAM("==================================================================================");
        return;
    }

    ROS_DEBUG_STREAM("current_copter_position_: " << current_copter_position_.transpose());
    ROS_INFO_STREAM("flyto_server_.isActive()=" << flyto_server_.isActive());

    // Temporary solution
    if (initial_path_.points_.empty())
    {
      initial_path_.addPoint(current_copter_position_);
      initial_path_.addPoint(destination_point_);
    }

    current_path_ = resamplePath(initial_path_);

    ceres::Solver::Summary ceres_solver_summary;
    current_path_ = trajectoryPlanning(current_path_, &ceres_solver_summary);
    ROS_DEBUG_STREAM("Solver Summary" << ceres_solver_summary.FullReport());

    ROS_DEBUG_STREAM("optimized path:");
    for (const auto& p : current_path_.points_)
    {
        ROS_DEBUG_STREAM(p.transpose());
    }

    current_path_pub_.publish(current_path_.visualizationMarkerMsg(color_current_path_));

    ros::Rate setpoint_loop_rate(10);
    for(std::size_t i = 0; i < current_path_.points_.size() - 1; i++)
    {   
        mav_interface->setPositionYaw(current_path_.points_[i] , 0);
        setpoint_loop_rate.sleep();
    }
}


Path SwarmCommander::trajectoryPlanning(const Path& initial_path, ceres::Solver::Summary* summary)
{
    Path optimized_path = initial_path;

    ceres::Problem problem;

    for (int i = 0; i < optimized_path.points_.size() - 1; i++)
    {
        const Eigen::Vector3d p1 = optimized_path.points_[i];
        const Eigen::Vector3d p2 = optimized_path.points_[i + 1];

        const Eigen::Vector3d vector_dir = (p2-p1).normalized();
        const double vector_mag = (p2-p1).norm();
        const double step_length = 0.1;
        std::size_t num_steps = std::round(vector_mag/step_length);

        // Distance to obstacles residuals
        // auto cost_functor = new DistanceToObstacleFunctor( *sdf_map_, num_steps, config_.min_obstacle_distance + config_.additional_min_obstacle_distance,
        // config_.min_obstacle_distance, config_.soft_obstacle_distance_weight, config_.hard_obstacle_distance_weight);

        // problem.AddResidualBlock(new DistanceToObstacleFunctor::CostFunction(cost_functor, ceres::TAKE_OWNERSHIP, num_steps), nullptr,
        //                      optimized_path.points_[i].data(), optimized_path.points_[i + 1].data());

        // Tracking 
        problem.AddResidualBlock(new TrackingGlobalPlannerFunctor::CostFunction(new TrackingGlobalPlannerFunctor(config_.path_length_weight)),
        nullptr, optimized_path.points_[i].data(), optimized_path.points_[i + 1].data());
    }

    // Set the start-point constant!
    problem.SetParameterBlockConstant(optimized_path.points_[0].data());
    problem.SetParameterBlockConstant(optimized_path.points_.back().data());

    // Test Eval
    {    
        ceres::Problem::EvaluateOptions eval_options;
    
        double cost;
        std::vector<double> residuals;
        std::vector<double> gradient;
        ceres::CRSMatrix jacobian;
    
        problem.Evaluate(eval_options, &cost, &residuals,
                        // nullptr, nullptr);
                        &gradient, &jacobian);
        ROS_INFO_STREAM(kStreamPrefix << "Cost: " << cost);
    
        ROS_INFO_STREAM(kStreamPrefix <<"Residuals: ");
        for (const auto& r : residuals)
        {
        ROS_INFO_STREAM(r);
        }
    
        ROS_INFO_STREAM(kStreamPrefix << "Gradients: ");
        for (const auto& g : gradient)
        {
        ROS_INFO_STREAM(g);
        }
    
        ROS_INFO_STREAM(kStreamPrefix << "Jacobian Matrices");
        for (const auto& j : jacobian.values)
        {
        ROS_INFO_STREAM(j);
        }
    }

    // Run the solver!
    {
        ceres::Solver::Options options;
        options.minimizer_type = ceres::LINE_SEARCH;
        options.line_search_type = ceres::ARMIJO;
        options.line_search_direction_type = ceres::STEEPEST_DESCENT;
        options.max_num_iterations = config_.max_num_iterations;
        options.minimizer_progress_to_stdout = false;
        ceres::Solve(options, &problem, summary);
    }


    return optimized_path;
}


void SwarmCommander::reconfigure(mav_swarm_commander::SwarmCommanderConfig& config, uint32_t level)
{
	config_ = config;
}

void SwarmCommander::commandTrajectory(const Path& path, const double yaw_setpoint_rad)
{
    if (path.points_.size() < 2)
    {
        ROS_ERROR_STREAM("To perform path following, at least 2 waypoints are required!");
        return;
    }

    const Eigen::AngleAxisd angle_axis_orientation_setpoint(yaw_setpoint_rad, Eigen::Vector3d::UnitZ());
    const Eigen::Quaterniond quat_orientation_setpoint(angle_axis_orientation_setpoint);
    publish(path, quat_orientation_setpoint);

    const Eigen::ParametrizedLine<double, 3> current_3d_line_segment = Eigen::ParametrizedLine<double, 3>::Through(path.points_[0], path.points_[1]);
    const Eigen::Vector3d projected_copter_position = current_3d_line_segment.projection(current_copter_position_);

}


void SwarmCommander::publish(const Path& path, const Eigen::Quaterniond& quat_orientation_setpoint)
{
    geometry_msgs::PoseStamped PoseStamped_position_setpoint;
    PoseStamped_position_setpoint.header.stamp = ros::Time::now();
    PoseStamped_position_setpoint.header.frame_id = "odom";
    PoseStamped_position_setpoint.pose.position.x = path.points_[1].x();
    PoseStamped_position_setpoint.pose.position.y = path.points_[1].y();
    PoseStamped_position_setpoint.pose.position.z = path.points_[1].z();
    PoseStamped_position_setpoint.pose.orientation.x = quat_orientation_setpoint.x();
    PoseStamped_position_setpoint.pose.orientation.y = quat_orientation_setpoint.y();
    PoseStamped_position_setpoint.pose.orientation.z = quat_orientation_setpoint.z();
    PoseStamped_position_setpoint.pose.orientation.w = quat_orientation_setpoint.w();
    offboard_mode_position_setpoint_marker_pub_.publish(PoseStamped_position_setpoint);
}

Path SwarmCommander::resamplePath(const Path& initial_path, const double max_path_length)
{
    Path resampled_path;
    resampled_path.addPoint(*initial_path.points_.begin());
    for (int i = 0; i < initial_path.points_.size() - 1; i++)
    {
        // Start- and end-point of the current segment
        auto p1 = initial_path.points_[i];
        const auto p2 = initial_path.points_[i + 1];
        while ((p2 - p1).norm() > max_path_length)
        {
        p1 = p1 + (p2 - p1).normalized() * 0.5 * max_path_length;
        resampled_path.addPoint(p1);
        }
        resampled_path.addPoint(p2);
    }
    return resampled_path;
}