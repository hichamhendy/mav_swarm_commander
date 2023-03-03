
#include "mav_swarm_commander/SwarmCommander.h"


Eigen::Vector3d SwarmCommander::ToEulerAngles(const Eigen::Quaterniond q) 
{
    Eigen::Vector3d angles;    //yaw pitch roll

    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[2] = std::atan2(siny_cosp, cosy_cosp);
    return angles;
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

bool SwarmCommander::updateCopterPosition()
{
    try
    {
        const geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0));
        current_copter_position_ = {transform_stamped.transform.translation.x, transform_stamped.transform.translation.y,
                                    transform_stamped.transform.translation.z};
        current_copter_orientation_ = {transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x,
                                    transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z};
        current_copter_euler_orientation_ = ToEulerAngles(current_copter_orientation_);

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


void SwarmCommander::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    current_copter_velocity_ = {msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z};

    if (!loiter_velocity_ )
        loiter_velocity_ = current_copter_velocity_;

    return;
}


double SwarmCommander::deg2rad(double deg) 
{
    return deg * M_PI / 180.0;
}