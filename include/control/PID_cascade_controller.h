#include <ros/ros.h>
#include <Eigen/Eigen>
#include <mav_msgs/eigen_mav_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <control/DynamicParametersConfig.h>


#ifndef CONTROL_PID_CASCADE_CONTROLLER_H
#define CONTROL_PID_CASCADE_CONTROLLER_H

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
			PI(double, double, double, double);
			~PI();
			void SetParameters(double, double, double, double);
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

	/* Six different PIDs for the position and orientation */
	PID PID_x_;
	PID PID_y_;
	PID PID_z_;
	PID PID_roll_;
	PID PID_pitch_;
	PID PID_yaw_;

	/* three PIs for angular rate compensation */
	PI PI_rate_roll_;
	PI PI_rate_pitch_;
	PI PI_rate_yaw_;

	class PIDCascadeController {
		
		public:
			PIDCascadeController();
			~PIDCascadeController();
			void InitializeParameters();
			Eigen::Vector4d CalculateRotorVelocities(mav_msgs::EigenTrajectoryPoint, 
				Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, bool);
			double Saturator(double, double, double);
			void MixingMatrix();
			void SetPIDs();
			Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond);

			/* 
				the correct order is:
				thrust_factor, drag_factor, maximum_speed, P_roll, I_roll, D_roll, clip_roll,
				P_pitch, I_pitch, D_pitch, clip_pitch, P_yaw, I_yaw, D_yaw, clip_yaw,
				P_x, I_x, D_x, clip_x, P_y, I_y, D_y, clip_y, P_z, I_z, D_z, clip_z,
				P_rate_roll, I_rate_roll, clip_rate_roll,
				P_rate_pitch, I_rate_pitch, clip_rate_pitch,
				P_rate_yaw, I_rate_yaw, clip_rate_yaw, 
			*/
			void ResetDynamicsParametrs(double, double, unsigned int, double, double, 
				double, double, double, double, double, double, double, double, double, 
				double, double, double, double, double, double, double, double, double, 
				double, double, double, double, double, double, double, double, double, 
				double, double, double, double);   

			// vehicle and word parameters
			const double mass_ = 0.402;                  // [Kg]
			double thrust_factor_;                       // [Ns^2]   8.54858e-06 DYNAMIC
			const double arm_length_ = 0.08225;          // [m]
			const double gravity_ = 9.81;                // [m/s^2]
			double drag_factor_;                         // [Nms^2]	 1.1e-6      DYNAMIC	
			unsigned int maximum_speed_;	             // [rad/s]	 838		 DYNAMIC

		private:
		    Eigen::Matrix4d T_;
			bool safety_control_;      // if true, engine velocity is set to zero
			bool initialized_params_;

			double P_roll_;
			double I_roll_;
			double D_roll_;
			double clip_roll_;
			double P_pitch_;
			double I_pitch_;
			double D_pitch_;
			double clip_pitch_;
			double P_yaw_;
			double I_yaw_;
			double D_yaw_;
			double clip_yaw_;

			double P_x_;
			double I_x_;
			double D_x_;
			double clip_x_;
			double P_y_;
			double I_y_;
			double D_y_;
			double clip_y_;
			double P_z_;
			double I_z_;
			double D_z_;
			double clip_z_;			

			double P_rate_roll_;
			double I_rate_roll_;
			double clip_rate_roll_;
			double P_rate_pitch_;
			double I_rate_pitch_;
			double clip_rate_pitch_;
			double P_rate_yaw_;
			double I_rate_yaw_;
			double clip_rate_yaw_;						
		    
    			
	};

}

#endif