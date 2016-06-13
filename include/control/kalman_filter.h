#include <ros/ros.h>
#include <Eigen/Eigen>
#include <math.h>  
#include <tuple>

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

namespace control {

	class KalmanFilter {
		public:
			KalmanFilter();
			~KalmanFilter();
			void StartUpConditions(Eigen::Vector3d, Eigen::Vector4d);
			std::tuple <Eigen::Vector3d, double, Eigen::Vector4d, Eigen::VectorXd, 
				Eigen::Vector2d, double, Eigen::Vector4d> Update(double, Eigen::Vector4d, 
				Eigen::Vector3d, double);

		private:	
			unsigned int iteration_ = 0;
			double P_w_ = 0;
			double P_ut_ = 0;
			double P_dl_ = 0.001;
			double P_Bz_ = 1;
			double P_B_ = 1;
			double P_By_ = 1;
			double P_Btz_ = 0.1;
			double P_tau_ = 0;
			double Q_w_ = 1e-4;
			double Q_ut_ = 1e-8;
			double Q_dl_ = 1e-10;
			double Q_Bz_ = 1e-10;
			double Q_Btz_ = 1e-9;
			double Q_B_ = 1e-8;
			double Q_tau_ = 1e-8;		
			double R_w_ = pow(0.0425, 2);
  			double R_az_ = 4;	
		    Eigen::MatrixXd P_ = Eigen::MatrixXd::Zero(15, 15); // state covariance matrix
		    Eigen::MatrixXd Q_ = Eigen::MatrixXd::Zero(15, 15); // model covariance
		    Eigen::Matrix4d R_ = Eigen::Matrix4d::Zero();       // measurement covariance
		    Eigen::VectorXd x_est_ = Eigen::VectorXd::Zero(15); // state vector

	};

}

#endif