#include "control/lee_position_controller.h"

namespace control {


	LeePositionController::LeePositionController() {
		InitializeParameters();
	}


	LeePositionController::~LeePositionController() {}


	/*
	Compute the inverse of the mixing matrix and normalize the parametrs according to
	the inertia and the mass.
	*/
	void LeePositionController::InitializeParameters() {

		NormalizeControlGains();

		Eigen::Matrix4d mixing;
		mixing <<       beta_1_,       beta_1_,       beta_1_,       beta_1_,
				   d_y_*beta_3_,  d_y_*beta_3_, -d_y_*beta_3_, -d_y_*beta_3_,
				  -d_x_*beta_2_,  d_x_*beta_2_,  d_x_*beta_2_, -d_x_*beta_2_,
				  	   -beta_7_,       beta_7_,      -beta_7_,       beta_7_;

		mixing_matrix_ = mixing.inverse();
		return;
	}


	/*
	Normalize the parametrs according to the quadrtotor mass and inertia
	*/
	void LeePositionController::NormalizeControlGains() {

		// normalize the parameters
		k_x_ = k_x_ / 0.402;           // normalize position gain
		k_v_ = k_v_ / 0.402;           // normalize velocity gain
		k_R_ = k_R_ * 83.3333;         // normalize angular rate gain
		k_Omega_ = k_Omega_ * 83.3333; // normalize attitude gain
	}


	/*
	Reset the value of the control parameters.
	*/
	void LeePositionController::ResetParameters(
		double position_gain,
		double velocity_gain,
		double attitude_gain,
		double angular_velocity_gain) {

		k_x_ = position_gain;
		k_v_ = velocity_gain;
		k_R_ = attitude_gain;
		k_Omega_ = angular_velocity_gain;

		InitializeParameters();
		return;
	}


	/*
	Compute the hat map.
	*/
	Eigen::Matrix3d LeePositionController::Vector2SkewMatrix(Eigen::Vector3d vector) {
		Eigen::Matrix3d matrix;
		matrix <<           0, -vector.z(),  vector.y(),
			       vector.z(),           0, -vector.x(),
		          -vector.y(),  vector.x(),           0;

		return matrix;
	}


	/*
	Compute the v map
	*/
	Eigen::Vector3d LeePositionController::SkewMatrix2Vector(Eigen::Matrix3d matrix) {
		Eigen::Vector3d vector(matrix(2, 1), matrix(0, 2), matrix(1, 0));
		return vector;
	}


	/*
	Saturator function
	*/
	double LeePositionController::Saturator(double input, double upper_limit, double lower_limit) {

		if (input > upper_limit)
			input = upper_limit;
		if (input < lower_limit)
			input = lower_limit;

		return input;
	}


	/*
	Compute the Euler's angles from a quaternion
	*/
	Eigen::Vector3d LeePositionController::Quaternion2Euler(Eigen::Quaterniond quaternion) {
		Eigen::Vector3d euler;

		double q_0 = quaternion.w();
		double q_1 = quaternion.x();
		double q_2 = quaternion.y();
		double q_3 = quaternion.z();

		euler.x() = atan2(2*(q_0*q_1 + q_2*q_3), 1-2*(pow(q_1, 2) + pow(q_2, 2)));
		euler.y() = asin(2*(q_0*q_2 - q_3*q_1));
		euler.z() = atan2(2*(q_0*q_3 + q_1*q_3), 1-2*(pow(q_2, 2) + pow(q_3, 2)));

		return euler;
	}


	/*
	Recompute the quaternion from Euler angles
	*/
	Eigen::Quaterniond LeePositionController::Euler2Quaternion(Eigen::Vector3d euler) {
		double roll2 = euler.x() / 2;
		double pitch2 = euler.y() / 2;
		double yaw2 = euler.z() / 2;

		Eigen::Quaterniond quaternion;
		quaternion.w() = cos(roll2)*cos(pitch2)*cos(yaw2) + sin(roll2)*sin(pitch2)*sin(yaw2);
		quaternion.x() = sin(roll2)*cos(pitch2)*cos(yaw2) - cos(roll2)*sin(pitch2)*sin(yaw2);
		quaternion.y() = cos(roll2)*sin(pitch2)*cos(yaw2) + sin(roll2)*cos(pitch2)*sin(yaw2);
		quaternion.z() = cos(roll2)*cos(pitch2)*sin(yaw2) - sin(roll2)*sin(pitch2)*cos(yaw2);

		return quaternion;
	}


	/*
	Compute the four rotors velocities. The result belongs to [0 1]
	There are no safety controls here.
	*/
	Eigen::Vector4d LeePositionController::CalculateRotorVelocities(
		Eigen::Quaterniond orientation_W_B,
		Eigen::Vector3d position_W,
		Eigen::Vector3d velocity_W,
		Eigen::Vector3d Omega,
		mav_msgs::EigenTrajectoryPoint desire_trajectory,
		std::ofstream& debug_file,
		bool logs){

		// compute the rotation matrix
		Eigen::Matrix3d R = orientation_W_B.toRotationMatrix();

		// compute the position and velocity errors.
		// WARNING: in the velocity use the rotation matrix if the velocity is
		// relative to the body frame
		Eigen::Vector3d position_error = position_W - desire_trajectory.position_W;
		Eigen::Vector3d velocity_error = velocity_W - desire_trajectory.velocity_W;

		// get the desire yaw
		double desire_yaw = desire_trajectory.getYaw();

		// base e3 vector
		Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

		// compute desire acceleration
		Eigen::Vector3d acceleration = k_x_*position_error + k_v_*velocity_error - gravity_*e_3 - desire_trajectory.acceleration_W;

		// compute desire rotation matrix
		Eigen::Vector3d b1_c(cos(desire_yaw), sin(desire_yaw), 0);
		Eigen::Vector3d b3_c = -acceleration / acceleration.norm();
		Eigen::Vector3d b2_c = b3_c.cross(b1_c);
		b2_c.normalize();
		Eigen::Matrix3d R_c;
		R_c.col(0) = b2_c.cross(b3_c);
		R_c.col(1) = b2_c;
		R_c.col(2) = b3_c;

		// compute the attitude error
		Eigen::Vector3d attidute_error = 0.5 *	SkewMatrix2Vector(R_c.transpose()*R - R.transpose()*R_c);

		// desire angular velocity
		Eigen::Vector3d Omega_c(Eigen::Vector3d::Zero());

		// angular velocity error
		Eigen::Vector3d angular_velocity_error = Omega - R.transpose()*R_c*Omega_c;

		// compute the simplify torque (fast implementation)
		Eigen::Vector3d M = -k_R_*attidute_error - k_Omega_*angular_velocity_error;

		// compute the force
		double f = -acceleration.transpose()*e_3;

		// full thrust-torque vector
		Eigen::Vector4d force_torque(f, M.x(), M.y(), M.z());

		Eigen::Vector4d rotors_speed;
		rotors_speed = mixing_matrix_ * force_torque;

		rotors_speed(0) = Saturator(std::sqrt(std::abs(rotors_speed(0))), 1, 0);
		rotors_speed(1) = Saturator(std::sqrt(std::abs(rotors_speed(1))), 1, 0);
		rotors_speed(2) = Saturator(std::sqrt(std::abs(rotors_speed(2))), 1, 0);
		rotors_speed(3) = Saturator(std::sqrt(std::abs(rotors_speed(3))), 1, 0);

		//logs
		if (logs) {

			if (!debug_file.is_open())
				debug_file.open("debug_logs.log");

			debug_file << "debug cs: position gain: " << k_x_ << "\n";
			debug_file << "debug cs: velocity gain: " << k_v_ << "\n";
			debug_file << "debug cs: attitude gain: " << k_R_ << "\n";
			debug_file << "debug cs: angular velocity gain: " << k_Omega_ << "\n";
			debug_file << "debug cs: position error x: " << position_error.x() << "\n";
			debug_file << "debug cs: position error y: " << position_error.y() << "\n";
			debug_file << "debug cs: position error z: " << position_error.z() << "\n";
			debug_file << "debug cs: velocity error x: " << velocity_error.x() << "\n";
			debug_file << "debug cs: velocity error y: " << velocity_error.y() << "\n";
			debug_file << "debug cs: velocity error z: " << velocity_error.z() << "\n";
			debug_file << "debug cs: attitude error roll: " << attidute_error.x() << "\n";
			debug_file << "debug cs: attitude error pitch: " << attidute_error.y() << "\n";
			debug_file << "debug cs: attitude error yaw: " << attidute_error.z() << "\n";
			debug_file << "debug cs: angular velocity error roll: " << angular_velocity_error.x() << "\n";
			debug_file << "debug cs: angular velocity error pitch: " << angular_velocity_error.y() << "\n";
			debug_file << "debug cs: angular velocity error yaw: " << angular_velocity_error.z() << "\n";
			debug_file << "debug cs: force: " << f << "\n";
			debug_file << "debug cs: torque x: " << M.x() << "\n";
			debug_file << "debug cs: torque y: " << M.y() << "\n";
			debug_file << "debug cs: torque z: " << M.z() << "\n";
			debug_file << "debug cs: motor 1: " << rotors_speed(0) << "\n";
			debug_file << "debug cs: motor 2: " << rotors_speed(1) << "\n";
			debug_file << "debug cs: motor 3: " << rotors_speed(2) << "\n";
			debug_file << "debug cs: motor 4: " << rotors_speed(3) << "\n";
		}

		return rotors_speed;
	}
}
