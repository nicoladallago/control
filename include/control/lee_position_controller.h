#include <ros/ros.h>
#include <iostream>
#include <fstream> 
#include <Eigen/Eigen>
#include <mav_msgs/eigen_mav_msgs.h>

#ifndef LEE_POSITION_CONTROLLER_H
#define LEE_POSITION_CONTROLLER_H

namespace control {

  class LeePositionController {
    public:
      LeePositionController();
      ~LeePositionController();
      // orientation, position, angular rate, linear rate, trajectory
      Eigen::Vector4d CalculateRotorVelocities(Eigen::Quaterniond, Eigen::Vector3d, 
        Eigen::Vector3d, Eigen::Vector3d, mav_msgs::EigenTrajectoryPoint, 
        std::ofstream&, bool); 
      Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond);
      Eigen::Quaterniond Euler2Quaternion(Eigen::Vector3d);
      void ResetParameters(double, double, double, double);

    private:
      void NormalizeControlGains();
      void InitializeParameters();
      Eigen::Matrix3d Vector2SkewMatrix(Eigen::Vector3d);
      Eigen::Vector3d SkewMatrix2Vector(Eigen::Matrix3d);
      double Saturator(double, double, double);
      
      // params (DYNAMIC)
      double k_x_ = 0.6;     // position gain          
      double k_v_ = 0.6;     // velocity gain          
      double k_R_ = 0.4;     // attitute gain          
      double k_Omega_ = 0.1; // angular velocity gain  

      //quadrotor constant
      const double d_x_ = 0.08225;
      const double d_y_ = 0.08225;
      const double mass_ = 0.402;
      const double gravity_ = 9.81;

      // estimated parameters (from Emil)
      const double beta_1_ = 17.9427;   // AF / m
      const double beta_2_ = 167.1829;  // AF / Ixx
      const double beta_3_ = 148.4353;  // AF / Iyy
      const double beta_7_ = 14.875;    // BF / Izz

      Eigen::Matrix4d mixing_matrix_;
          
  };

}

#endif
