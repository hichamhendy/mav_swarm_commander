
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


Eigen::VectorXd SwarmCommander::polyFit2D(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order) 
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);

    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); ++i) 
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); ++j)
        for (int i = 0; i < order; ++i)
            A(j, i + 1) = A(j, i) * xvals(j);
        

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);

    return result;
}


double SwarmCommander::polyEval2D(const Eigen::VectorXd &coeffs, double x) 
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); ++i) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}


Eigen::MatrixXf SwarmCommander::differentiate(int k, double t) const
{
	// if ever a generalization would be needed, add n to the argument list or overload the function
	Eigen::ArrayXf T = Eigen::ArrayXf::Constant(8, 1, 1); 	// 1 as initialization
	
    // generates 'size' equally spaced values in the closed interval [low,high]-> 1 2 3 4 5 6 7 8 For integer scalar types, 
    // an even spacing is possible if and only if the length of the range, i.e., high-low is a scalar multiple of size-1,
	Eigen::ArrayXf D = Eigen::ArrayXf::LinSpaced(8, 0, 7);	
	
	for (int j = 0; j < k; ++j) // if k = 0, no differentiation for-loop won't be entered
	{
		for (int i = 0; i < 8; ++i)
		{
			T(i) *= D(i); // coefficient
			if(D(i) > 0) // if I'm gonna loop again that means nothing but higher order of differentiation; hence, reduce the order by one
				D(i) -= 1;			
		}
	}

	// Having the array of  1 2 3 4 5 6 7 8 represents a container to loop on to differentiate since by differentiating we take down the power and multiply with the base
	// if k = 0 I get p(ts) =  ts + ts^2 + ts^3  +  ts^4 + ts^5 + ts^6 + ts^7, The higher the order gets, the lower the power gets. The code shunk just before controls the oder I will raise to.
	for (int i = 0; i < 8; ++i)
	{
		T(i) *= std::pow(t, D(i));
	}
	return T.matrix().transpose();
}
