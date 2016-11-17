#ifndef CONTROL_CONTROL_NODE_H
#define CONTROL_CONTROL_NODE_H

#include <ros/ros.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <log4cxx/logger.h>
#include <Eigen/Eigen>
#include <mav_msgs/eigen_mav_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros_kflytelemetry/ros_kflytelemetry.h>
#include "control/lee_position_controller.h"
#include <dynamic_reconfigure/server.h>
#include <control/DynamicParametersConfig.h>

namespace control {
	class ControlNode {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		ControlNode(ros::NodeHandle&);
		~ControlNode();
		void ViconCallback(const geometry_msgs::TransformStampedPtr&);
		void KflyCallBack(const ros_kflytelemetry::IMUDataPtr&);
		void Shuttdown(ros_kflytelemetry::DirectReference);
		double Saturator(double, double, double);
		Eigen::Vector3d ViconSpeed(Eigen::Vector3d, double, double);
		Eigen::Vector3d ViconAngularVelocity(Eigen::Quaterniond, double, double);
		Eigen::Vector3d ViconAngularVelocityQuaternion(Eigen::Quaterniond, double, double);

		LeePositionController lee_position_controller_;
		ros::Publisher speed_msg_pub_;	// publisher for the kfly output signals
		ros::Subscriber kfly_sub_;
		Eigen::Vector3d rate_kfly_;
		Eigen::Vector3d position_vicon_;
		Eigen::Quaterniond orientation_vicon_;
		Eigen::Vector3d velocity_filtered_;
		Eigen::Vector3d angular_velocity_filtered_;
		Eigen::Vector3d angular_velocity_filtered_quat_;
		ros_kflytelemetry::ManageSubscription kfly_manage_sub_;
		double vicon_current_rate;
		unsigned int vicon_frame_number;
		std::ofstream debug_file;

	private:
		geometry_msgs::TransformStamped vicon_msg;
		ros::Subscriber vicon_stream_sub;
		Eigen::Vector3d old_pos;
		Eigen::Vector3d old_orientation;
		Eigen::Quaterniond previous_orientation;
		double time_stamp;
		double previous_time_stamp;

	};

	// dynamic reconfigure global parametrs
	double position_gain_;
	double velocity_gain_;
	double attitude_gain_;
	double angular_velocity_gain_;


}

#endif // CONTROL_CONTROL_NODE_H
