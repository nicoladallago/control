#include "control/kalman_filter.h"

namespace control {

    KalmanFilter::KalmanFilter() {}
    KalmanFilter::~KalmanFilter() {}

    void KalmanFilter::StartUpConditions(Eigen::Vector3d w_m, Eigen::Vector4d u_in) {

        P_(0, 0) = P_w_;
        P_(1, 1) = P_w_;
        P_(2, 2) = P_w_;
        P_(3, 3) = P_ut_;
        P_(4, 4) = P_ut_;
        P_(5, 5) = P_ut_;
        P_(6, 6) = P_ut_;
        P_(7, 7) = P_dl_;
        P_(8, 8) = P_dl_;
        P_(9, 9) = P_Bz_;
        P_(10, 10) = P_B_;
        P_(11, 11) = P_B_;
        P_(12, 12) = P_By_;
        P_(13, 13) = P_Btz_;
        P_(14, 14) = P_tau_;

        Q_(0, 0) = Q_w_;
        Q_(1, 1) = Q_w_;
        Q_(2, 2) = Q_w_;
        Q_(3, 3) = Q_ut_;
        Q_(4, 4) = Q_ut_;
        Q_(5, 5) = Q_ut_;
        Q_(6, 6) = Q_ut_;
        Q_(7, 7) = Q_dl_;
        Q_(8, 8) = Q_dl_;
        Q_(9, 9) = Q_Bz_;
        Q_(10, 10) = Q_B_;
        Q_(11, 11) = Q_B_;
        Q_(12, 12) = Q_B_;
        Q_(13, 13) = Q_Btz_;
        Q_(14, 14) = Q_tau_;

        R_(0, 0) = R_w_;
        R_(1, 1) = R_w_;
        R_(2, 2) = R_w_;
        R_(3, 3) = R_az_;

        x_est_(0) = w_m.x();
        x_est_(1) = w_m.y();
        x_est_(2) = w_m.z();
        x_est_(3) = u_in(0);
        x_est_(4) = u_in(1);
        x_est_(5) = u_in(2);
        x_est_(6) = u_in(3);
        x_est_(7) = 0;
        x_est_(8) = 0;
        x_est_(9) = log(10);
        x_est_(10) = log(10);
        x_est_(11) = log(10);
        x_est_(12) = log(1);
        x_est_(13) = 0;
        x_est_(14) = log(0.05);
    }

    std::tuple <Eigen::Vector3d, double, Eigen::Vector4d, Eigen::VectorXd, Eigen::Vector2d, double, Eigen::Vector4d> KalmanFilter::Update(double dt,
        Eigen::Vector4d u_in, Eigen::Vector3d w_m, double acc_z) {

        iteration_ ++;

        Eigen::Vector3d w(x_est_(0), x_est_(1), x_est_(2));
        Eigen::Vector4d ut(x_est_(3), x_est_(4), x_est_(5), x_est_(6));
        Eigen::Vector2d dl(x_est_(7), x_est_(8));
        Eigen::VectorXd B(5);
        B(0) = x_est_(9);
        B(1) = x_est_(10);
        B(2) = x_est_(11);
        B(3) = x_est_(12);
        B(4) = x_est_(13);
        double tau = x_est_(14);

        double dl1 = dl.x();
        double dl2 = dl.y();
        double b1 = B(0);
        double b2 = B(1);
        double b3 = B(2);
        double b4 = B(3);
        double btz = B(4);

        // throttle and torque
        Eigen::Vector4d u_app(pow(ut(0), 2), pow(ut(1), 2), pow(ut(2), 2), pow(ut(3), 2));
        Eigen::MatrixXd torque(3, 4);
        torque << exp(b2)*(dl2 - 1),  exp(b2)*(dl2 - 1),  exp(b2)*(dl2 + 1),  exp(b2)*(dl2 + 1),
                 -exp(b3)*(dl1 + 1), -exp(b3)*(dl1 - 1), -exp(b3)*(dl1 - 1), -exp(b3)*(dl1 + 1),
                           -exp(b4),            exp(b4),           -exp(b4),            exp(b4);

        Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
        Eigen::Vector3d x0_2 = w + dt*(torque*u_app + btz*e_3);
        Eigen::Vector4d x3_6 =  exp(tau)/(dt + exp(tau))*ut + dt/(dt + exp(tau))*u_in;

        x_est_(0) = x0_2.x();
        x_est_(1) = x0_2.y();
        x_est_(2) = x0_2.z();
        x_est_(3) = x3_6(0);
        x_est_(4) = x3_6(1);
        x_est_(5) = x3_6(2);
        x_est_(6) = x3_6(3);
        x_est_(7) = dl.x();
        x_est_(8) = dl.y();
        x_est_(9) = B(0);
        x_est_(10) = B(1);
        x_est_(11) = B(2);
        x_est_(12) = B(3);
        x_est_(13) = B(4);
        x_est_(14) = tau;

        w << (x_est_(0), x_est_(1), x_est_(2));
        ut << (x_est_(3), x_est_(4), x_est_(5), x_est_(6));
        dl << (x_est_(7), x_est_(8));
        B << x_est_(9), x_est_(10), x_est_(11), x_est_(12), x_est_(13);
        tau = x_est_(14);

        u_app << (pow(ut(0), 2), pow(ut(1), 2), pow(ut(2), 2), pow(ut(3), 2));

        double u1 = ut(0);
        double u2 = ut(1);
        double u3 = ut(2);
        double u4 = ut(3);

        double u1_in = u_in(1);
        double u2_in = u_in(2);
        double u3_in = u_in(3);
        double u4_in = u_in(4);

        dl1 = dl.x();
        dl2 = dl.y();

        b1 = B(0);
        b2 = B(1);
        b3 = B(2);
        b4 = B(3);
        btz = B(4);

        Eigen::Vector4d exp_b1(exp(b1), exp(b1), exp(b1), exp(b1));
        double thr = exp_b1.transpose() * u_app;

        // calculate A matrix
        Eigen::MatrixXd A(15, 15);
        A << 1, 0, 0,  2*dt*u1*exp(b2)*(dl2 - 1),  2*dt*u2*exp(b2)*(dl2 - 1),  2*dt*u3*exp(b2)*(dl2 + 1),  2*dt*u4*exp(b2)*(dl2 + 1),                                                           0, dt*exp(b2)*(pow(u1,2) + pow(u2,2) + pow(u3,2) + pow(u4,2)), 0, dt*exp(b2)*(dl2*pow(u1,2) + dl2*pow(u2,2) + dl2*pow(u3,2) + dl2*pow(u4,2) - pow(u1,2) - pow(u2,2) + pow(u3,2) + pow(u4,2)),                                                                                   										0,                                       					 0,  0,                                                 0,
             0, 1, 0, -2*dt*u1*exp(b3)*(dl1 + 1), -2*dt*u2*exp(b3)*(dl1 - 1), -2*dt*u3*exp(b3)*(dl1 - 1), -2*dt*u4*exp(b3)*(dl1 + 1), -dt*exp(b3)*(pow(u1,2) + pow(u2,2) + pow(u3,2) + pow(u4,2)),                                      					0, 0,                                                                               										   0, -dt*exp(b3)*(dl1*pow(u1,2) + dl1*pow(u2,2) + dl1*pow(u3,2) + dl1*pow(u4,2) + pow(u1,2) - pow(u2,2) - pow(u3,2) + pow(u4,2)),                                       					 0,  0,                                                 0,
             0, 0, 1,           -2*dt*u1*exp(b4),            2*dt*u2*exp(b4),           -2*dt*u3*exp(b4),            2*dt*u4*exp(b4),                                       					0,                                      			        0, 0,                                                                               										   0,                                                                                   								        0, -dt*exp(b4)*(pow(u1,2) - pow(u2,2) + pow(u3,2) - pow(u4,2)), dt,                                                 0,
             0, 0, 0,     1 - dt/(dt + exp(tau)),                          0,                          0,                          0,                                       					0,                                      			    	0, 0,                                                                               										   0,                                                                              										        0,                                       					 0,  0, (dt*exp(tau)*(u1 - u1_in))/(dt + pow(exp(tau),2)),
             0, 0, 0,                          0,     1 - dt/(dt + exp(tau)),                          0,                          0,                                       					0,                                      					0, 0,                                                                             										       0,                                                                            										        0,                                       				     0,  0, (dt*exp(tau)*(u2 - u2_in))/(dt + pow(exp(tau),2)),
             0, 0, 0,                          0,                          0,     1 - dt/(dt + exp(tau)),                          0,                                       					0,                                     	 					0, 0,                                                                           										       0,                                                                                 										    0,                                       					 0,  0, (dt*exp(tau)*(u3 - u3_in))/(dt + pow(exp(tau),2)),
             0, 0, 0,                          0,                          0,                          0,     1 - dt/(dt + exp(tau)),                                       					0,                                      					0, 0,                                                                              										       0,                                                                                										    0,                                       					 0,  0, (dt*exp(tau)*(u4 - u4_in))/(dt + pow(exp(tau),2)),
             0, 0, 0,                          0,                          0,                          0,                          0,                                       					1,                                      				    0, 0,                                                                              	 								           0,                                                                              										        0,                                       					 0,  0,                                                 0,
             0, 0, 0,                          0,                          0,                          0,                          0,                                       					0,                                      					1, 0,                                                                              										       0,                                                                             										        0,                                       					 0,  0,                                                 0,
             0, 0, 0,                          0,                          0,                          0,                          0,                                       					0,                                      					0, 1,                                                                              										       0,                                                                           										        0,                                       					 0,  0,                                                 0,
             0, 0, 0,                          0,                          0,                          0,                          0,                                       					0,                                      					0, 0,                                                                              										       1,                                                                           										        0,                                       					 0,  0,                                            	    0,
             0, 0, 0,                          0,                          0,                          0,                          0,                                       					0,                                      					0, 0,                                                                               										   0,                                                                         										            1,                                       					 0,  0,                                                 0,
             0, 0, 0,                          0,                          0,                          0,                          0,                                       					0,                                      					0, 0,                                                                               										   0,                                                                            										        0,                                       					 1,  0,                                                 0,
             0, 0, 0,                          0,                          0,                          0,                          0,                                       					0,                                      					0, 0,                                                                                                                          0,                                                                           										        0,                                       					 0,  1,                                                 0,
             0, 0, 0,                          0,                          0,                          0,                          0,                                       					0,                                      					0, 0,                                                                               										   0,                                                                           								                0,                                       					 0,  0,                                                 1;

        // state covariance matrix update based on model
        P_ = A * P_ * A.transpose() + Q_;

        // measurement prediction function
        Eigen::Vector4d h(w.x(), w.y(), w.z(), thr);

        // calculate error
        Eigen::Vector4d z(w_m.x(), w_m.y(), w_m.z(), acc_z);

        Eigen::Vector4d y = z - h;

        // the H matrix maps the measurement to the states
        Eigen::MatrixXd H(4, 15);
        H << 1, 0, 0,            0,            0,            0,            0, 0, 0,                                   					  0, 0, 0, 0, 0, 0,
             0, 1, 0,            0,            0,            0,            0, 0, 0,                                   					  0, 0, 0, 0, 0, 0,
             0, 0, 1,            0,            0,            0,            0, 0, 0,                                   					  0, 0, 0, 0, 0, 0,
             0, 0, 0, 2*u1*exp(b1), 2*u2*exp(b1), 2*u3*exp(b1), 2*u4*exp(b1), 0, 0, exp(b1)*(pow(u1,2) + pow(u2,2) + pow(u3,2) + pow(u4,2)), 0, 0, 0, 0, 0;

        // measurement covariance update
        Eigen::Matrix4d S = H * P_ * H.transpose() + R_;

        // calculate Kalman gain
        Eigen::Matrix4d K = P_ * H.transpose() * S.inverse();

        // corrected model prediction
        x_est_ = x_est_ + K*y;

        // update state covariance with new knowledge
        Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
        P_ = (I - K*H) * P_;

        w << x_est_(0), x_est_(1), x_est_(2);
        ut << x_est_(3), x_est_(4), x_est_(5), x_est_(6);
        dl << x_est_(7), x_est_(8);
        B << x_est_(9), x_est_(10), x_est_(11), x_est_(12), x_est_(13);
        tau = x_est_(14);

        b1 = B(0);
        exp_b1 << exp(b1), exp(b1), exp(b1), exp(b1);
        Eigen::Vector4d ut2(pow(ut(0),2), pow(ut(1),2), pow(ut(2),2), pow(ut(3),2));
        thr = exp_b1.transpose() * ut2;

        // output varibles
        auto output = std::make_tuple(w, thr, ut, B, dl, tau, y);
        return output;            
    }
}
