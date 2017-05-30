#ifndef __TRANS_UTILS_HH__
#define __TRANS_UTILS_HH__

#include "utils.hh"

namespace utils{
	namespace trans{
		// The below two functions transform between representations of ROS and Eigen vectors
		geometry_msgs::Quaternion quat2quat(const Eigen::Vector4d &quat);
		Eigen::Vector4d quat2quat(const geometry_msgs::Quaternion &quat);
		// This function converts an SE(3) to nav_msgs::Odomety structure
		// its corresponding SE(3) matrix. Hence this only handles
		// rotation and position. Other fields should be filled by 
		// the programmer.
		nav_msgs::Odometry se32odom(const Eigen::Matrix4d &se3, bool cancel_yaw = false, const Eigen::Matrix6d &cov = Eigen::Matrix6d::Identity());
		// This function converts nav_msgs::Odomety structure into 
		// its corresponding SE(3) matrix. Hence this only handles
		// rotation and position.
		Eigen::Matrix4d odom2se3(const nav_msgs::Odometry &odom, bool cancel_yaw = false);
		// This function converts the sensor_msgs::Imu structure into
		// its corresponding SO(3) martix. Hence this only handels
		// orientation.
		Eigen::Matrix3d imu2dcm(const sensor_msgs::Imu &imu, bool cancel_yaw = false);
		// This function converts the sensor_msgs::Imu structure into
		// its corresponding roll-pitch-yaw reps. Hence this only handels
		// orientation.
		Eigen::Vector3d imu2rpy(const sensor_msgs::Imu &imu, bool cancel_yaw = false);
		// This function converts geometry_msgs::Pose structure into 
		// its corresponding SE(3) matrix. Hence this only handles
		// rotation and position.
		Eigen::Matrix4d pose2se3(const geometry_msgs::Pose &pose, bool cancel_yaw = false);
		// The following functions transform between different
		// rotation representations.
		Eigen::Matrix3d yaw2dcm(const double &yaw);
		Eigen::Matrix3d quat2dcm(const Eigen::Vector4d &quat);
		Eigen::Vector3d quat2rpy(const Eigen::Vector4d &quat);
		Eigen::Matrix3d rpy2dcm (const Eigen::Vector3d &rpy);
		Eigen::Vector4d rpy2quat(const Eigen::Vector3d &rpy);
		Eigen::Vector4d dcm2quat(const Eigen::Matrix3d &dcm);
		Eigen::Vector3d dcm2rpy (const Eigen::Matrix3d &dcm);
		Eigen::Vector3d dcm2aaxis(const Eigen::Matrix3d &dcm);
    Eigen::Matrix3d aaxis2dcm(const Eigen::Vector3d &aaxis);
    Eigen::Vector3d quat2aaxis(const Eigen::Vector4d &quat);
		Eigen::Vector4d aaxis2quat(const Eigen::Vector3d &aaxis);

		// The following functions cancel the yaw component
		// from the given rotation.
		Eigen::Matrix3d cancel_yaw(const Eigen::Matrix3d &dcm );
		Eigen::Vector4d cancel_yaw(const Eigen::Vector4d &quat);

		// ### This function interpolates quaternions using the 
		// SLERP algorithm.
		Eigen::Vector3d slerp(const Eigen::Vector3d &quat1, const Eigen::Vector3d &quat2, double theta);
	}
}

#endif
