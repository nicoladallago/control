#include <ros/ros.h>
#include <Eigen/Eigen>
#include <mav_msgs/eigen_mav_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <control/DynamicParametersConfig.h>


#ifndef ANGULAR_RATE_CONTROLLER_H
#define ANGULAR_RATE_CONTROLLER_H

namespace control {

	class PID {
		public:
			PID();
			PID(double, double, double, double, double); // dt, P, I, D, clip
			~PID();
			void SetParameters(double, double, double, double, double); // dt, P, I, D, clip
			double Run(double);

		private:
			double dt_;
			double p_gain_;
			double i_gain_;
			double d_gain_;
			double clip_value_;
			double p_part_;
			double i_part_;
			double d_part_;
			double previous_error_;
			double output_;
	};

	class PI {
		public:
			PI();
			PI(double, double, double, double); // dt, P, I, clip
			~PI();
			void SetParameters(double, double, double, double); // dt, P, I, clip
			double Run(double);

		private:
			double dt_;
			double p_gain_;
			double i_gain_;
			double clip_value_;
			double p_part_;
			double i_part_;
			double output_;		
	};	

	class AngularRateController {
		public:
			AngularRateController();
			~AngularRateController();
			Eigen::Vector4d CalculateRotorVelocities(
				mav_msgs::EigenTrajectoryPoint, Eigen::Vector3d, Eigen::Quaterniond,
				Eigen::Vector3d, bool);
			void InitializeParameters();

			/* 
				the correct order is:
				thrust_factor, drag_factor,
				P_z, I_z, D_z, 
				P_roll, I_roll, D_roll,
				P_pitch, I_pitch, D_pitch, 
				P_yaw, I_yaw, D_yaw
				P_rate_roll, I_rate_roll,
				P_rate_pitch, I_rate_pitch,
				P_rate_yaw, I_rate_yaw,
			*/
			void ResetDynamicsParametrs(double, double, double, double, double, double, 
				double, double, double, double, double, double, double, double, double, 
				double, double, double, double, double); 

			Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond);
			Eigen::Quaterniond Euler2Quaternion(Eigen::Vector3d);
			Eigen::Vector3d KroneckerProduct(Eigen::Quaterniond, Eigen::Quaterniond);
			const unsigned int maximum_speed_ = 838;
				
		private:
			double Saturator(double, double, double);
			void MixingMatrix();
			void SetPIDs();	
		    Eigen::Matrix4d T_;
		    double thrust_factor_ = 8.54858e-06;     // DYNAMIC
		    double drag_factor_ = 1.1e-6;            // DYNAMIC
		    const double arm_length_ = 0.08225;
		    const double mass_ = 0.402;
		    const float gravity_ = 9.81;
			bool safety_control_;      // if true, engine velocity is set to zero
			bool initialized_params_;			

			// gains (DYNAMIC)
			double P_axis_x_ = -0.3;
			double I_axis_x_ = -0.02;
			double D_axis_x_ = -0.3;
			double P_axis_y_ = -0.3;
			double I_axis_y_ = -0.02;
			double D_axis_y_ = -0.3;
			double P_axis_z_ = -5;
			double I_axis_z_ = 0;		
			double D_axis_z_ = -5;

			double P_rate_roll_ = 0.5;
			double I_rate_roll_ = 0.1;
			double P_rate_pitch_ = 0.5;
			double I_rate_pitch_ = 0.1;
			double P_rate_yaw_ = 0.5;
			double I_rate_yaw_ = 0.1;		

			double P_z_ = 5;
			double I_z_ = 0.01;
			double D_z_ = 12;		
	};

	// global PIDs and PI controller
	PID PID_axis_x_;
	PID PID_axis_y_;
	PID PID_axis_z_;
	PI PI_rate_roll_;
	PI PI_rate_pitch_;
	PI PI_rate_yaw_;

	PID PID_z_;


}

#endif