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
    velocity_sub_ = nh_private_.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 1, boost::bind(&SwarmCommander::velocityCallback, this, _1));

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
    flyto_server_.start(); // regarding auto_start in the construction, the boolean value that tells the ActionServer wheteher or not to start publishing as soon as it comes up. THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS and start() should be called after construction of the server.

    dynamic_reconfigure_server_.setCallback(boost::bind(&SwarmCommander::reconfigure, this, _1, _2));

    // trajectory_planning_timer_ = nh_private_.createTimer(ros::Duration(1.0), boost::bind(&SwarmCommander::globalPlanner, this)); // in case the planner applied a horizon shift

    run_thread = std::thread(&SwarmCommander::run, this);
    // goal_reached_thread = std::thread(&SwarmCommander::goalReached, this);

    // ros::AsyncSpinner spinner(2); // Use multi threads
    // spinner.start();

    ROS_INFO_STREAM("==================================================================================");
    ROS_INFO_STREAM("SwarmCommander has been successfully contructed");
    ROS_INFO_STREAM("==================================================================================");
}

SwarmCommander::~SwarmCommander()
{
    if(run_thread.joinable())
		run_thread.join();

    if(goal_reached_thread.joinable())
		goal_reached_thread.join();

    // ros::waitForShutdown();
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


void SwarmCommander::goalReached()
{
	ros::Rate run_freq(1/dt);
	while(ros::ok()) 
	{
           // Try to get the current copter position
        if (!updateCopterPosition())
        {
            abortCurrentGoal();
        }

        const double dist_to_destination = (current_copter_position_ - destination_point_).norm();
        if (dist_to_destination < config_.goal_vicinity)
        {
            ROS_INFO_STREAM(kStreamPrefix << "Distance to destination in meter = " << dist_to_destination);
            ROS_INFO_STREAM(kStreamPrefix << "Goal Reaching Thread: Drone is located in goal vicinity; Initiating a planning process ain't needed");
            finishCurrentGoal();
            ROS_INFO_STREAM(kStreamPrefix << "==================================================================================");
            return;
        }
		run_freq.sleep();
	}
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
    final_path_pub_.publish(Path().visualizationMarkerMsg(color_final_path_));
}

void SwarmCommander::globalPlanner()
{
    // Try to get the current copter position
    if (!updateCopterPosition())
    {
        ROS_ERROR_STREAM(kStreamPrefix << "Could not get the current copter position => Action failed!");
        abortCurrentGoal();
        ROS_INFO_STREAM(kStreamPrefix <<"==================================================================================");
        return;
    }

    ROS_DEBUG_STREAM("current_copter_position_: " << current_copter_position_.transpose());
    ROS_DEBUG_STREAM("current_copter_orientation_: " << current_copter_euler_orientation_.transpose());
    ROS_INFO_STREAM(kStreamPrefix << "Fly To Server activity is on: " << flyto_server_.isActive());

    if (!flyto_server_.isActive())
    {
        ROS_INFO_STREAM(kStreamPrefix << "=========================Server ain't active======================================");
        return;
    }

    const double dist_to_destination = (current_copter_position_ - destination_point_).norm();
    ROS_INFO_STREAM(kStreamPrefix << "Distance to destination in meter = " << dist_to_destination);

    if (dist_to_destination < config_.goal_vicinity)
    {
        ROS_INFO_STREAM(kStreamPrefix << "Drone is located in goal vicinity; Initiating a planning process ain't needed");
        finishCurrentGoal();
        ROS_INFO_STREAM(kStreamPrefix << "==================================================================================");
        return;
    }



    // Temporary solution
    if (initial_path_.points_.empty())
    {
      initial_path_.addPoint(current_copter_position_);
      initial_path_.addPoint(destination_point_);
    }
    initial_path_pub_.publish(initial_path_.visualizationMarkerMsg(color_initial_path_));

    current_path_ = resamplePath(initial_path_);


    // // Ceres Solution Non linear least square
    // ceres::Solver::Summary ceres_solver_summary;
    // current_path_ = trajectoryPlanning(current_path_, &ceres_solver_summary);
    // ROS_DEBUG_STREAM("Solver Summary" << ceres_solver_summary.FullReport());

    // Model predictive planning
    current_path_ = modelPredictivePlanning(current_path_);


    ROS_DEBUG_STREAM("optimized path:");
    for (const auto& p : current_path_.points_)
    {
        ROS_DEBUG_STREAM(p.transpose());
    }

    current_path_pub_.publish(current_path_.visualizationMarkerMsg(color_current_path_));

    // ros::Rate setpoint_loop_rate(10);
    // for(std::size_t i = 0; i < current_path_.points_.size() - 1; i++)
    // {   
    //     mav_interface->setPositionYaw(current_path_.points_[i] , 0);
    //     setpoint_loop_rate.sleep();
    // }
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
    
        // ROS_INFO_STREAM(kStreamPrefix << "Gradients: ");
        // for (const auto& g : gradient)
        // {
        // ROS_INFO_STREAM(g);
        // }
    
        // ROS_INFO_STREAM(kStreamPrefix << "Jacobian Matrices");
        // for (const auto& j : jacobian.values)
        // {
        // ROS_INFO_STREAM(j);
        // }
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


Path SwarmCommander::modelPredictivePlanning(const Path& initial_path)
{ 
    Path optimized_path = initial_path;
    Eigen::Vector3d setting_point;

    //
    size_t N = 6;
    double dt = 0.025;
    const size_t x_start = 0;
    const size_t y_start = x_start + N;
    const size_t z_start = y_start + N;
    const size_t x_dot_start = z_start + N;
    const size_t y_dot_start = x_dot_start + N;
    const size_t z_dot_start = y_dot_start + N;
    const size_t roll_start = z_dot_start + N;
    const size_t pitch_start = roll_start + N;
    const size_t roll_command_start = pitch_start + N;
    const size_t pitch_command_start = roll_command_start + N - 1;
    const size_t thrust_command_start = pitch_command_start + N - 1;
    // 

    
    const size_t n_vars = N * 8 + (N - 1) * 3; // number of independent variables 8 states and 3 inputs (domain dimension for f and g)
    const size_t n_constraints = N * 8; // Number of constraints
    SystemConstants sys_constants;

    // options
    std::string options;
    options += "Integer print_level  0\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // options += "Numeric max_cpu_time          0.5\n";
    // options += "Integer max_iter     10\n"; // // maximum number of iterations
    // options += "Numeric tol          1e-6\n";
    // options += "String  derivative_test            second-order\n";
    // options += "Numeric point_perturbation_radius  0.\n";
    
    ros::WallTime start_time = ros::WallTime::now();
    for (int i_ = 0; i_ < initial_path.points_.size() - 1 && updateCopterPosition() && ros::ok(); i_++)
    {  
        typedef CPPAD_TESTVECTOR(double) Dvector;

        const Eigen::Vector3d p1 = initial_path.points_[i_];
        const Eigen::Vector3d p2 = initial_path.points_[i_ + 1];

        // double yaw_init = current_copter_euler_orientation_.z(); // not needed

        // Initial value of the independent variables "vars".
        // Should be 0 except for the initial values.
        Dvector vars(n_vars);
        for (int i = 0; i < n_vars; ++i) 
        {
            vars[i] = 0.0;
        }

        vars[x_start] = current_copter_position_.x();
        vars[y_start] = current_copter_position_.y();
        vars[z_start] = current_copter_position_.z();
        vars[x_dot_start] = current_copter_velocity_.x();
        vars[y_dot_start] = current_copter_velocity_.y();
        vars[z_dot_start] = current_copter_velocity_.z();
        vars[roll_start] = current_copter_euler_orientation_.x();
        vars[pitch_start] = current_copter_euler_orientation_.y();

        // Set all non-actuators upper and lowerlimits to the max negative and positive values.
        // Lower and upper limits for x
        Dvector vars_lowerbound(n_vars);
        Dvector vars_upperbound(n_vars);
        for (int i = 0; i < roll_start; ++i) 
        {
            vars_lowerbound[i] = -1.0e19;
            vars_upperbound[i] = 1.0e19;
        }

         for (int i = roll_start; i < thrust_command_start; ++i) 
        {
            vars_lowerbound[i] = - sys_constants.maxmin_angle;
            vars_upperbound[i] =   sys_constants.maxmin_angle;
        }

        // control input constraints
        for (int i = thrust_command_start; i < n_vars; i++)
        {
            vars_lowerbound[i] = - sys_constants.min_thrust;
            vars_upperbound[i] =   sys_constants.max_thrust;
        }

        // Set boundary conditions.
        // Lower and upper limits for constraints
        Dvector constraints_lowerbound(n_constraints);
        Dvector constraints_upperbound(n_constraints);
        for (int i = 0; i < n_constraints; ++i) 
        {
            // A motion manifold should be created 
            constraints_lowerbound[i] = -1.0e19;
            constraints_upperbound[i] = 1.0e19;
        }
 
        constraints_lowerbound[x_start] = current_copter_position_.x();
        constraints_lowerbound[y_start] = current_copter_position_.y();
        constraints_lowerbound[z_start] = current_copter_position_.z();
        constraints_lowerbound[x_dot_start] = current_copter_velocity_.x();
        constraints_lowerbound[y_dot_start] = current_copter_velocity_.y();
        constraints_lowerbound[z_dot_start] = current_copter_velocity_.z();
        constraints_lowerbound[roll_start] = current_copter_euler_orientation_.x();
        constraints_lowerbound[pitch_start] = current_copter_euler_orientation_.y();


        constraints_upperbound[x_start] = current_copter_position_.x();
        constraints_upperbound[y_start] = current_copter_position_.y();
        constraints_upperbound[z_start] = current_copter_position_.z();
        constraints_upperbound[x_dot_start] = current_copter_velocity_.x();
        constraints_upperbound[y_dot_start] = current_copter_velocity_.y();
        constraints_upperbound[z_dot_start] = current_copter_velocity_.z();
        constraints_upperbound[roll_start] = current_copter_euler_orientation_.x();
        constraints_upperbound[pitch_start] = current_copter_euler_orientation_.y();

        // object that computes objective and constraints
        FG_eval fg_eval(p1,  p2);

        // place to return solution
        CppAD::ipopt::solve_result<Dvector> solution;

        try
        {
            // solve the problem
            CppAD::ipopt::solve<Dvector, FG_eval>(
                options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
                    constraints_upperbound, fg_eval, solution);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
        // bool memory_ok = CppAD::thread_alloc::free_all();
        // std::string group = "swarm/Node";
        // size_t      width = 20;
        // CppAD::test_boolofvoid	Run(group, width);
        // bool ok_sum = Run.summary(memory_ok);
        // ROS_INFO_STREAM(kStreamPrefix <<"Summary (mem): "<< ok_sum);
        // return static_cast<int>( ! ok_sum );
        //
        // Check some of the solution values
        //
        bool ok = true;
        ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
        if(CppAD::ipopt::solve_result<Dvector>::success)
            ROS_INFO_STREAM(kStreamPrefix <<"Solution found");
        else
            ROS_ERROR_STREAM(kStreamPrefix << "No solution found!!! Revise the Solver!");

        auto cost = solution.obj_value;
        ROS_INFO_STREAM(kStreamPrefix <<"Cost: "<< cost);

        ros::Rate setpoint_loop_rate(1/dt);
        for(std::size_t i = 0; i < N; i++)
        {   
            // cout << "Solution (collective): " << std::endl;
            // std::cout << solution.x[x_start + i] << " " << solution.x[y_start + i] << " " << solution.x[z_start + i] << std::endl;
            setting_point << solution.x[x_start + i], solution.x[y_start + i], solution.x[z_start + i];
            optimized_path.addPoint(setting_point);
            mav_interface->setPositionYaw(setting_point, 0);
        
            setpoint_loop_rate.sleep();
        }

/*         if (!updateCopterPosition())
            abortCurrentGoal();
        const double dist_to_destination = (current_copter_position_ - destination_point_).norm();
        ROS_INFO_STREAM(kStreamPrefix << "MPC: Distance to destination in meter = " << dist_to_destination);
        if (dist_to_destination < config_.goal_vicinity)
        {
            ROS_INFO_STREAM(kStreamPrefix << "Modelpredictive Planner: Drone is located in goal vicinity; Initiating a planning process ain't needed");
            finishCurrentGoal();
            ROS_INFO_STREAM(kStreamPrefix << "==================================================================================");
            return optimized_path;;
        } */

        // while((ros::WallTime::now() - start_time).toSec() < ros::WallDuration(dt).toSec())
        //     ros::Duration(0.005).sleep();
    }
    return optimized_path;
}

// void SwarmCommander::optimize(const Path& initial_path)
// {
//     Path optimized_control_points = initial_path;
//     // bool optimize_time_ = cost_function_ & MINTIME; // time optimization ?? 
//     // variable_num_ = optimize_time_ ? dim_ * 3 + 1 : dim_ * 3;

//     ocp ocp_cost;
//     nlopt::opt optimizer = nlopt::opt(nlopt::GN_ISRES, 3);
//     optimizer.set_min_objective(ocp_cost.costFunction, this);
//     optimizer.set_maxeval(config_.max_num_iterations);
//     optimizer.set_maxtime(config_.max_iteration_time);
//     optimizer.set_xtol_rel(1e-5);


//     vector<double> q(3);


//     try 
//     {
//         double final_cost;
//         nlopt::result result = opt.optimize(q, final_cost);
//     } 
//     catch (std::exception& e) 
//     {
//         std::cout << e.what() << endl;
//     }

//     // Eigen::Vector3d bmin, bmax;

// }

// double SwarmCommander::costFunction(const std::vector<double>& x, std::vector<double>& grad,
//                                       void* func_data)
// {

//     double cost;

//     return cost;
// }