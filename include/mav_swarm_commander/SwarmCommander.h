#pragma once

#include <actionlib/server/simple_action_server.h>

#include <manager_msgs/FlyToAction.h>
#include <manager_msgs/OffboardPathSetpoint.h>
#include <manager_msgs/OffboardTrajectorySetpoint.h>
#include <dynamic_reconfigure/server.h>
#include <mav_swarm_commander/SwarmCommanderConfig.h>
#include <mavros_msgs/HomePosition.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/ceres.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

// #include <GeographicLib/LocalCartesian.hpp>

#include <thread>
#include <cmath>

#include "mav_swarm_commander/path.h"
#include "mav_swarm_commander/trajectory.h"
#include "mav_swarm_commander/cost_functionals.h"
#include "mav_swarm_commander/state_space_ODE.h"


#include "esdf_map/signed_distance_field.h"
#include <mav_commander/mav_commander.h>
#include <mav_planner/path_planner.h>

#include <future> // to name the type of the mutex used




typedef actionlib::SimpleActionServer<manager_msgs::FlyToAction> FlyToServer; // https://docs.ros.org/en/diamondback/api/actionlib/html/classactionlib_1_1SimpleActionServer.html


class SwarmCommander
{
    public:
        SwarmCommander(const ros::NodeHandle& nh, const ros::NodeHandle& nh_priv,const ros::NodeHandle& nh_interface,const ros::NodeHandle& nh_waypoint_planning, const ros::NodeHandle& nh_esdf_map);

        /**
         * @brief Destroy the Swarm Commander object and closes the threads opnend in the constructor
         * 
         */
        ~SwarmCommander();
        
        /**
         * Copy operator for such a class shouldn't happen
        */
        SwarmCommander(const SwarmCommander&) = delete;

        /**
         * Copy assignment for such a class shouldn't happen
        */
        SwarmCommander& operator=(SwarmCommander&) = delete;

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_; // at the mean time it is made to take care of internal matters
        ros::NodeHandle nh_interface_;
        ros::NodeHandle nh_waypoint_planning_;      // to handle the waypoint planner 
        ros::NodeHandle nh_esdf_map_;
        

        VoxelGridMap::Ptr sdf_map_;
        nut::MavCommander::Ptr mav_interface;

        nut::Planner* mav_planner;

        // The timer triggering the main trajectory planning loop
        ros::Timer trajectory_planning_timer_;

        // Current state
        Eigen::Vector3d current_copter_position_;
        Eigen::Vector3d current_copter_velocity_;
        Eigen::Quaterniond current_copter_orientation_;
        Eigen::Vector3d current_copter_euler_orientation_;

        /** Service and Action Servers
         * @note In General:
         * SimpleActionServer implements a singe goal policy on top of the ActionServer class. 
         * The specification of the policy is as follows: only one goal can have an active status at a time, 
         * new goals preempt previous goals based on the stamp in their GoalID field.
         * More infor to be found under:
         *  https://docs.ros.org/en/diamondback/api/actionlib/html/classactionlib_1_1SimpleActionServer.html#a4964ef9e28f5620e87909c41f0458ecb
         * Proposed action format:
         * # Define the goal
            string frame_id
            geometry_msgs/Point destination
            float64 desired_yaw
            bool relative_heading
            ---
            # Define the result
            bool success
            float32 dist_to_destination #Meter
            ---
            # Define a feedback message
            float32 percentage_of_completion

        */
        FlyToServer flyto_server_;

        // ROS-Publishers & Service-Clients
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_{tf_buffer_};
        ros::Publisher initial_path_pub_;
        ros::Publisher current_path_pub_;
        ros::Publisher final_path_pub_;
        ros::Publisher sampled_path_pub_;
        ros::Publisher path_setpoint_pub_;
        ros::Publisher minsnaptrajectory_pub_;
        ros::Publisher offboard_mode_position_setpoint_marker_pub_;
        ros::ServiceClient topological_planning_service_client_; // the client is temporary till I integrate the planner
        ros::Subscriber velocity_sub_;
        ros::Subscriber goal_sub_;
        ros::ServiceClient rrt_planning_service_client_;

        // Visualization Parameters
        std_msgs::ColorRGBA color_initial_path_;
        std_msgs::ColorRGBA color_current_path_;
        std_msgs::ColorRGBA color_final_path_;
        std_msgs::ColorRGBA color_trajectory_;

        // Reconfigure
        dynamic_reconfigure::Server<mav_swarm_commander::SwarmCommanderConfig> dynamic_reconfigure_server_;
        void reconfigure(mav_swarm_commander::SwarmCommanderConfig& config, uint32_t level);
        mav_swarm_commander::SwarmCommanderConfig config_;

        // Planning
        boost::shared_ptr<const manager_msgs::FlyToGoal> current_goal_;
        Eigen::Vector3d destination_point_;
        std::string destination_frame_id_;
        Path initial_path_;
        Path current_path_;
        Path current_safe_path_;
        Trajectory min_snap_trajectory_;
        Trajectory current_trajectory_;
        mutable std::mutex current_safe_path_mutex_;
        std::string px4_flight_mode_;
        boost::optional<Eigen::Vector3d> loiter_position_;
        boost::optional<Eigen::Vector3d> loiter_velocity_;
        boost::optional<Eigen::Quaterniond> loiter_orientation_;
        boost::optional<manager_msgs::OffboardPathSetpoint> path_setpoint_msg_;

        std::thread run_thread, goal_reached_thread;

        const std::string kStreamPrefix = "[Swarm Commanding]: ";
        
        ros::Timer publish_position_setpoint_timer_; // Setpoint

        bool post_init;
        Eigen::MatrixXf prev_states;
        
        size_t N;
        double dt;
        size_t x_start, y_start, z_start, x_dot_start, y_dot_start, z_dot_start, roll_start, pitch_start, roll_command_start, pitch_command_start, thrust_command_start;

        size_t n_vars; // number of independent variables 8 states and 3 inputs (domain dimension for f and g)
        size_t n_constraints; // Number of constraints
        SystemConstants sys_constants;

        // options
        std::string options;

        /**
         * @brief 
         * 
         */
        void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

        /**
         * @brief The function uses a server 
         * https://docs.ros.org/en/diamondback/api/actionlib/html/classactionlib_1_1SimpleActionServer.html#a4964ef9e28f5620e87909c41f0458ecb
         * The callback assigns the goal and Start planning and flying immediately.
         * acceptNewGoal() @return s boost::shared_ptr< const typename SimpleActionServer< ActionSpec >::Goal > -> boost::shared_ptr<const manager_msgs::FlyToGoal>
        */
        void goalCallback();

        /** Allows users to register a callback to be invoked when a new preempt request is available.
         * https://docs.ros.org/en/diamondback/api/actionlib/html/classactionlib_1_1SimpleActionServer.html#a8074dcf85b8c3f57d711870446323446
         * The callback prevents copter from flying
        */
        void preemptCallback();

        /**
         * @brief The callback is reposnsible for Ceres initiation
         * Steps:
         * 1- Try to get the current copter position
         * 2- Do nothing, if we do not have any Goal at the moment
         * 3- Check if we have already reached our goal position (beeing in a close vicinity of the goal)
         * 4- Abort Replanning, if necessary
         * 5- call planGlobalPath; Use the planned initial path as the best available current path, any way path should be resampled
         * 6- Visualize the initial path
         * 7- Optimize the currently planned path using intervention of an artifical potential function if needed call optimizePath()
         * 8- Check the path before commanding it (if some criterion aren't fullfilled, fall back on a kinda of getSafePath)
         * 9- Check if we have reached the end of the current safe path
         * 10- Visualize the optimized path use kinda of publishSafePath()
        */
        void trajectoryPlanningCallback();

        /**
         * TODO
        */
        void publishSafePath();

        /**
         * @brief Updates the copter current position
        */
        bool updateCopterPosition();

        /**
         * @brief The function gets called from way point planner 
        */
        Path optimizePath(const Path& initial_path, ceres::Solver::Summary* summary);

        /**
         * @brief This function is meant for the far future.
         * I will be integrating my path planner node here. What will happen till then is to pass a list of points given somehow
         * @return path
        */
        Path planGlobalPath(const Eigen::Vector3d& destination_point);

        /**
         * @brief The functions gives a runaway in case the planner failed or becmes unreasoable
         * Steps:
         * 1- Add the first input point as a "safe point"; usually this point is the current copter position anyway, so its always "safe"
         * 2- Extend a line and check by every line segment if the point comes near to an obstacle or not

        */
        Path getSafePath(const Path& path);
        
        /**
         * @brief Sets the status of the active goal to succeeded.
        */
        void finishCurrentGoal();

        /**
         * Sets the status of the active goal to aborted.
        */
        void abortCurrentGoal();

        /**
         * sampling the path to avoid aggressiveness 
        */
        Path resamplePath(const Path& initial_path, const double max_path_length = 0.75);

        /**
         * @brief based on current_safe_path_ , a path will be commanded on the interface
         * Steps:
         * 1- set the initial as a reference for the optimization 
         * 2- Build the problem
         * 3- Add residuals for the paths between two consecutive waypoints by looping on the waypoints
         * 4- Enforce hard constraints
         * 5- Run the solver
         */
        Path trajectoryPlanning(const Path& initial_path, ceres::Solver::Summary* summary);

        /**
         * @brief handling the RRT* module
         * 
         */
        void globalPlanner();

        /**
         * @brief publish the path to be commanded
         * 
         */
        void publish(const Path& path, const Eigen::Quaterniond& quat_orientation_setpoint);

        /**
         * @brief The function gives setpoints to PX4 over the interface
         * 
         * @param path 
         * @param yaw_setpoint_rad 
         */
        void commandTrajectory(const Path& path, const double yaw_setpoint_rad);

        /**
         * @brief The functions runs the core of the singelton
         * 
         */
        void run();

        /**
         * @brief safe landing process
         * 
         */
        void land();

        /**
         * @brief solver takes all the state variables and actuator
         * variables in a singular vector. Thus, we should to establish
         * when one variable starts and another ends to make our lifes easier.
         * @note https://coin-or.github.io/CppAD/doc/ipopt_solve_get_started.cpp.htm
        */
        Trajectory modelPredictivePlanning(const Trajectory& initial_trajectory);

        /**
         * @brief The target is to generate a natural trajectory from the perspective of the quadrotor. revise: https://ieeexplore.ieee.org/document/5980409
         * @note Mellinger and Kumar proved that there are conditions to fly a quadrotor along a trajec-
            tory, given a sequence of waypoints in 3D Space; otherwise, a good tracking performance can not
            be implemented [MK11]. The waypoints represent a piecewise-linear path. Therefore, the intuitive
            idea is to interpolate between the waypoints using straight lines and constant velocity to form a
            trajectory. Yet, the trajectory, in this case, will be inefficient because of the infinite curvature
            at the trajectory’s keyframes/waypoints [MK11]. The authors have demonstrated that polynomial
            trajectories provide smooth transint n, via discontinuous inputs. System inputs (Ft, Mx, My, Mz)
            and states will be reconstructed by PX4 through feedforwarding the trajectory in output space exploiting the
            differential flatness property. The polynomials of the trajectory need then to be built in the way for
            which the fourth-derivative of position, also called snap, is minimized and consequently the change
            in inputs, as well.
         * 	For the first polynomial, there are four conditions at the initial state (position at the start and three
            derivatives, set to null) and seven at the end (one position at the end and six derivatives). For all the
            intermediary polynomials, there are the same seven conditions at the beginning (one position and
            six derivatives) and similarly seven conditions at the end so that the polynomials can be matched
            with each other. For the last polynomial, the boundary conditions are similar to the first polynomial
            except that there are seven conditions at the beginning of the polynomial and four at the end of
            the polynomial. 
            Every polynomial will be parametrized by the dimensionless time which starts at -1 (try 0) and ends at 1.
            That should prevent the element values of the polynomial from increasing due to the increase in the
            time since some elements of the polynomial are raised to the seventh order.
         * 
         * @param msg taken from the planner 
         */
        Eigen::MatrixXf minSnapTraj(const Path& initial_path) const;

        /**
         * @brief function to differentiate a 7th order polynomial
         * @param k is differentation order, t is the time instant (to be made dimentionless)
         * @note The function differentiates a 7th order polynomial. Based on the the k, we want to
         * to reach a polynmial gets calcuklated. Polynmials are function of time which also will be base 
         * of the power will should raise to. 
         */
        Eigen::MatrixXf differentiate( int k, double t) const;

        /**
         * @brief 
         * 
         */
        Eigen::Vector3d ToEulerAngles(const Eigen::Quaterniond q);

        /**
         * @brief transfrom to
         * 
         */
        double deg2rad(double deg);
    
        /**
         * @brief 
         * 
         */
        Eigen::VectorXd polyFit2D(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order);

        /**
         * @brief 
         * 
         * @param coeffs 
         * @param x 
         * @return double 
         */
        double polyEval2D(const Eigen::VectorXd &coeffs, double x);

        /**
         * @brief 
         * 
         */
        void goalReached();
};
