#pragma once

#include <actionlib/server/simple_action_server.h>

#include <manager_msgs/FlyToAction.h>
#include <manager_msgs/OffboardPathSetpoint.h>
#include <dynamic_reconfigure/server.h>
#include <flight_commander/FlightCommanderConfig.h>
#include <mavros_msgs/HomePosition.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/ceres.h>

#include <GeographicLib/LocalCartesian.hpp>

#include "mav_swarm_commander/Path.h"

typedef actionlib::SimpleActionServer<manager_msgs::FlyToAction> FlyToServer; // https://docs.ros.org/en/diamondback/api/actionlib/html/classactionlib_1_1SimpleActionServer.html

class SwarmCommander
{
    public:
        SwarmCommander(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, const ros::NodeHandle& nh_waypoint_planning);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::NodeHandle nh_waypoint_planning_; 
        ros::NodeHandle nh_mav_interface_;

        // The timer triggering the main planning loop
        ros::Timer planning_timer_;

        // Current state
        Eigen::Vector3d current_copter_position_;
        Eigen::Quaterniond current_copter_orientation_;

        /** Service and Action Servers
         * @note In General:
         * SimpleActionServer implements a singe goal policy on top of the ActionServer class. 
         * The specification of the policy is as follows: only one goal can have an active status at a time, 
         * new goals preempt previous goals based on the stamp in their GoalID field.
         * More infor to be found under:
         *  https://docs.ros.org/en/diamondback/api/actionlib/html/classactionlib_1_1SimpleActionServer.html#a4964ef9e28f5620e87909c41f0458ecb
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
        ros::ServiceClient topological_planning_service_client_;

        // Visualization Parameters
        std_msgs::ColorRGBA color_initial_path_;
        std_msgs::ColorRGBA color_current_path_;
        std_msgs::ColorRGBA color_final_path_;

        // Reconfigure
        dynamic_reconfigure::Server<flight_commander::SwarmCommanderConfig> dynamic_reconfigure_server_;
        void reconfigure(swarm_commander::SwarmCommanderConfig& config, uint32_t level);
        swarm_commander::SwarmCommanderConfig config_;

        // Planning
        boost::shared_ptr<const manager_msgs::FlyToGoal> current_goal_;
        Eigen::Vector3d destination_point_;
        std::string destination_frame_id_;
        Path initial_path_;
        Path current_path_;
        Path current_safe_path_;
        mutable std::mutex current_safe_path_mutex_;

        
        ros::Timer publish_position_setpoint_timer_; // Setpoint

        /**
         * The function uses a server 
         * https://docs.ros.org/en/diamondback/api/actionlib/html/classactionlib_1_1SimpleActionServer.html#a4964ef9e28f5620e87909c41f0458ecb
         * The callback assigns the goal and Start planning and flying immediately
        */
        void goalCallback();

        /** Allows users to register a callback to be invoked when a new preempt request is available.
         * https://docs.ros.org/en/diamondback/api/actionlib/html/classactionlib_1_1SimpleActionServer.html#a8074dcf85b8c3f57d711870446323446
         * The callback prevents copter from flying
        */
        void preemptCallback();

        /**
         * The callback is reposnsible for Ceres initiation
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
        void waypointPlanningCallback();

        /**
         * TODO
        */
        void publishSafePath();

        /**
         * Updates the copter current position
        */
        bool updateCopterPosition();

        /**
         * The function gets called from way point planner 
        */
        Path optimizePath(const Path& initial_path, ceres::Solver::Summary* summary);

        /**
         * This function is meant for the far future.
         * I will be integrating my path planner node here. What will happen till then is to pass a list of points given somehow
         * @return path
        */
        Path planGlobalPath(const Eigen::Vector3d& destination_point);

        /**
         * The functions gives a runaway in case the planner failed or becmes unreasoable
         * Steps:
         * 1- Add the first input point as a "safe point"; usually this point is the current copter position anyway, so its always "safe"
         * 2- Extend a line and check by every line segment if the point comes near to an obstacle or not

        */
        Path getSafePath(const Path& path);
        
        /**
         * Sets the status of the active goal to succeeded.
        */
        void finishCurrentGoal();

        
        /**
         * Sets the status of the active goal to aborted.
        */
        void abortCurrentGoal();

        /**
         * sampling the path to avoid aggressiveness 
        */
        Path resamplePath(const Path& initial_path, const double max_path_length);
};
