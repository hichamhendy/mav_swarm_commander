#include <ros/ros.h>

#include "mav_swarm_commander/SwarmCommander.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mav_swarm_commander_node");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");


    // Thread 1
    ros::NodeHandle mav_interface("~");
    ros::CallbackQueue mav_interface_callback_queue;
    mav_interface.setCallbackQueue(&mav_interface_callback_queue);

    // Thread 2
    ros::NodeHandle mav_planner("~");
    ros::CallbackQueue mav_planner_callback_queue;
    mav_planner.setCallbackQueue(&mav_planner_callback_queue);

    // Thread 3
    ros::NodeHandle mav_sdf("~");
    ros::CallbackQueue obstacles_esdf_callback_queue;
    mav_sdf.setCallbackQueue(&obstacles_esdf_callback_queue);

    SwarmCommander swarm_commander(nh, nh_private, mav_interface, mav_planner, mav_sdf);


    std::thread interface_thread( [&mav_interface_callback_queue] () {
        ros::Rate looping_rate(10);
        while (ros::ok())
        {
            mav_interface_callback_queue.callAvailable();
            looping_rate.sleep();
        }
        
    });

    std::thread planner_thread( [&mav_planner_callback_queue] () {
    ros::Rate looping_rate(10);
        while (ros::ok())
        {
            mav_planner_callback_queue.callAvailable();
            looping_rate.sleep();
        }
        
    });

    std::thread sdf_thread( [&obstacles_esdf_callback_queue] () {
    ros::Rate looping_rate(10);
        while (ros::ok())
        {
            obstacles_esdf_callback_queue.callAvailable();
            looping_rate.sleep();
        }
        
    });


    ros::spin();


    if(interface_thread.joinable())
        interface_thread.join();

    if(planner_thread.joinable())
        planner_thread.join();

    if(sdf_thread.joinable())
        sdf_thread.join();

    return 0;
}