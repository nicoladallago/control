#include "control/PID_cascade_controller.h"

namespace control {

	///////////////////////////////////////////////////////////////////////////////////
	//                                PID CONTROLLER                                 //
	///////////////////////////////////////////////////////////////////////////////////
	PID::PID() {
		dt_ = 0.01;
		p_gain_ = 0;
		i_gain_ = 0;
		d_gain_ = 0;
		clip_value_ = 100;
		p_part_ = 0;
		i_part_ = 0;
		d_part_ = 0;
		previous_error_ = 0;
		output_ = 0;		
	}

	PID::PID(double dt, double p_gain, double i_gain, double d_gain, double clip_value) {
		dt_ = dt;
		p_gain_ = p_gain;
		i_gain_ = i_gain;
		d_gain_ = d_gain;
		clip_value_ = clip_value;
		p_part_ = 0;
		i_part_ = 0;
		d_part_ = 0;
		previous_error_ = 0;
		output_ = 0;
	}	

	PID::~PID() {}

	void PID::SetParameters(double dt,double p_gain, double i_gain, double d_gain, 
		double clip_value) {
		dt_ = dt;
		p_gain_ = p_gain;
		i_gain_ = i_gain;
		d_gain_ = d_gain;
		clip_value_ = clip_value;
	} 

	double PID::Run(double error) {
		
		p_part_ = p_gain_ * error;

      	if (i_gain_)
			i_part_ += i_gain_ * error * dt_;
		else
		    i_part_ = 0;

		d_part_ = d_gain_ * (error - previous_error_) / dt_;
		previous_error_ = error;

		if (d_part_ > clip_value_)
		    d_part_ = clip_value_;
		else if (d_part_ < -clip_value_)
		    d_part_ = -clip_value_;

		output_ = p_part_ + i_part_ + d_part_;

		if (output_ > clip_value_) {
		    i_part_ = clip_value_ - p_part_;
		    output_ = clip_value_;
		}
		else if (output_ < -clip_value_) {
		    i_part_ = -clip_value_ - p_part_;
		    output_ = -clip_value_;
		}

		return output_;		
	}


	///////////////////////////////////////////////////////////////////////////////////
	//                                 PI CONTROLLER                                 //
	///////////////////////////////////////////////////////////////////////////////////
	PI::PI() {
		dt_ = 0.01;
		p_gain_ = 0;
		i_gain_ = 0;
		clip_value_ = 100;
		p_part_ = 0;
		i_part_ = 0;
		output_ = 0;		
	}

	PI::PI(double dt, double p_gain, double i_gain, double clip_value) {
		dt_ = dt;
		p_gain_ = p_gain;
		i_gain_ = i_gain;
		clip_value_ = clip_value;
		p_part_ = 0;
		i_part_ = 0;
		output_ = 0;
	}	

	PI::~PI() {}

	void PI::SetParameters(double dt,double p_gain, double i_gain, double clip_value) {
		dt_ = dt;
		p_gain_ = p_gain;
		i_gain_ = i_gain;
		clip_value_ = clip_value;
	} 

	double PI::Run(double error) {
		
		p_part_ = p_gain_ * error;

      	if (i_gain_)
			i_part_ += i_gain_ * error * dt_;
		else
		    i_part_ = 0;

		output_ = p_part_ + i_part_;

		if (output_ > clip_value_) {
		    i_part_ = clip_value_ - p_part_;
		    output_ = clip_value_;
		}
		else if (output_ < -clip_value_) {
		    i_part_ = -clip_value_ - p_part_;
		    output_ = -clip_value_;
		}

		return output_;		
	}


	///////////////////////////////////////////////////////////////////////////////////
	//                            FULL CASCADE CONTROLLER                            //
	///////////////////////////////////////////////////////////////////////////////////
	PIDCascadeController::PIDCascadeController() {}
	PIDCascadeController::~PIDCascadeController() {}

	void PIDCascadeController::InitializeParameters() {

		SetPIDs();
	    MixingMatrix();

	    safety_control_ = false;
	    initialized_params_ = true;

	}

	// mixing matrix for a X type quadrotor
	void PIDCascadeController::MixingMatrix() {

		// compute the matrix from torque/force to square of the velocity
	    double b = thrust_factor_;    // thrust factor
	    double d = drag_factor_;      // drag factor
	    double l = arm_length_;       // distance from propeller to center of quad 		

	    // build the matrix
	    T_(0, 0) = b;    T_(0, 1) = b;    T_(0, 2) = b;    T_(0, 3) = b;
	    T_(1, 0) = l*b;  T_(1, 1) = l*b;  T_(1, 2) = -l*b; T_(1, 3) = -l*b; 
	    T_(2, 0) = -l*b; T_(2, 1) = l*b;  T_(2, 2) = l*b;  T_(2, 3) = -l*b;
	    T_(3, 0) = -d;   T_(3, 1) = d;    T_(3, 2) = -d;   T_(3, 3) = d;

	    // compute the pseudo inverse
	    T_ = T_.transpose() * (T_ * T_.transpose()).inverse();
	}

	void PIDCascadeController::SetPIDs(){
	    // initialize PIDs for the orientation
	    PID_roll_.SetParameters(0.01,  P_roll_,  I_roll_,  D_roll_,  clip_roll_);
	    PID_pitch_.SetParameters(0.01, P_pitch_, I_pitch_, D_pitch_, clip_pitch_);
	    PID_yaw_.SetParameters(0.01,   P_yaw_,   I_yaw_,   D_yaw_,   clip_yaw_);

	    // initialize PIDs for the position (dt, p_gain, i_gain, d_gain, clip)
	    PID_x_.SetParameters(0.01, P_x_, I_x_, D_x_, clip_x_);
	    PID_y_.SetParameters(0.01, P_y_, I_y_, D_y_, clip_y_);
	    PID_z_.SetParameters(0.01, P_z_, I_z_, D_z_, clip_z_);

	    PI_rate_roll_.SetParameters(0.01, P_rate_roll_, I_rate_roll_, clip_rate_roll_);
	    PI_rate_pitch_.SetParameters(0.01, P_rate_pitch_, I_rate_pitch_, clip_rate_pitch_);
	    PI_rate_yaw_.SetParameters(0.01, P_rate_yaw_, I_rate_yaw_, clip_rate_yaw_);
	} 

	void PIDCascadeController::ResetDynamicsParametrs(
		double thrust_factor, double drag_factor, unsigned int maximum_speed, 
		double P_roll, double I_roll, double D_roll, double clip_roll,
		double P_pitch, double I_pitch, double D_pitch, double clip_pitch, 
		double P_yaw, double I_yaw, double D_yaw, double clip_yaw,
		double P_x, double I_x, double D_x, double clip_x, 
		double P_y, double I_y, double D_y, double clip_y, 
		double P_z, double I_z, double D_z, double clip_z,
		double P_rate_roll, double I_rate_roll, double clip_rate_roll,
		double P_rate_pitch, double I_rate_pitch, double clip_rate_pitch,
		double P_rate_yaw, double I_rate_yaw, double clip_rate_yaw) {
		
		thrust_factor_ = thrust_factor;
		drag_factor_ = drag_factor;
		maximum_speed_ = maximum_speed;

		P_roll_ = P_roll;
		I_roll_ = I_roll;
		D_roll_ = D_roll;
		clip_roll_ = clip_roll;
		P_pitch_ = P_pitch;
		I_pitch_ = I_pitch;
		D_pitch_ = D_pitch;
		clip_pitch_ = clip_pitch;
		P_yaw_ = P_yaw;
		I_yaw_ = I_yaw;
		D_yaw_ = D_yaw;
		clip_yaw_ = clip_yaw;	
		
		P_x_ = P_x;
		I_x_ = I_x;
		D_x_ = D_x;
		clip_x_ = clip_x;
		P_y_ = P_y;
		I_y_ = I_y;
		D_y_ = D_y;
		clip_y_ = clip_y;					
		P_z_ = P_z;
		I_z_ = I_z;
		D_z_ = D_z;
		clip_z_ = clip_z;

		P_rate_roll_ = P_rate_roll;
		I_rate_roll_ = I_rate_roll;
		clip_rate_roll_ = clip_rate_roll;

		P_rate_pitch_ = P_rate_pitch;
		I_rate_pitch_ = I_rate_pitch;
		clip_rate_pitch_ = clip_rate_pitch;

		P_rate_yaw_ = P_rate_yaw;
		I_rate_yaw_ = I_rate_yaw;
		clip_rate_yaw_ = clip_rate_yaw;				

		// recompute mixing matrix
		MixingMatrix();

		//reset PIDs
		SetPIDs();
	}

	Eigen::Vector3d PIDCascadeController::Quaternion2Euler(Eigen::Quaterniond quaternion) {
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

	Eigen::Vector4d PIDCascadeController::CalculateRotorVelocities(
		mav_msgs::EigenTrajectoryPoint command_trajectory, Eigen::Vector3d position_W, 
		Eigen::Vector3d orientation_W, Eigen::Vector3d angular_rate, bool logs) {


    	//---------------------------CONTROL ALGORITHM HERE---------------------------//   
		
		// set the speed to zero at the beginning
		Eigen::Vector4d rotor_speed =  Eigen::Vector4d::Zero();  

		// if negative desire height, set the speed to zero
		if (command_trajectory.position_W.z() < 0)
			return rotor_speed; 

	    // compute the position error in the word frame
	    double x_err = command_trajectory.position_W.x() - position_W.x();
	    double y_err = command_trajectory.position_W.y() - position_W.y();
	    double z_err = command_trajectory.position_W.z() - position_W.z();

	    // extract the roll, pitch and yaw angular rate
	    double roll_rate = angular_rate.x();
	    double pitch_rate = angular_rate.y();
	    double yaw_rate = angular_rate.z();

	    // compute the compensation signals
	    double rate_roll_compensation = PI_rate_roll_.Run(roll_rate);
	    double rate_pitch_compensation = PI_rate_pitch_.Run(pitch_rate);
	    double rate_yaw_compensation = PI_rate_yaw_.Run(yaw_rate);

	    // PIDs in the position
	    double x_dd = Saturator(PID_x_.Run(x_err), 3*gravity_, -3*gravity_);
	    double y_dd = Saturator(PID_y_.Run(y_err), 3*gravity_, -3*gravity_);
	    // PID in z plus the feedforward term to compensate the gravity
	    double z_dd = Saturator(PID_z_.Run(z_err) + gravity_, 3*gravity_, -3*gravity_);	    

	    // Euler conversion
	    Eigen::Vector3d Gamma_dd;   // desire acceleration vector
	    Gamma_dd(0) = x_dd; 
	    Gamma_dd(1) = y_dd; 
	    Gamma_dd(2) = z_dd;

	    double yaw_ref = command_trajectory.getYaw();
	    double roll_ref = asin((x_dd * sin(yaw_ref) - y_dd * cos(yaw_ref)) / Gamma_dd.norm());
	    double pitch_ref = atan2(x_dd * cos(yaw_ref) + y_dd * sin(yaw_ref), z_dd);

	    // extract roll, pitch and yaw from odometry
	    double roll = orientation_W.x();
	    double pitch = orientation_W.y();
	    double yaw = orientation_W.z();

	    // PIDs in the orientation 
	    // CHECK THE SIGNS BEFOE FLY                                      
	    double torque_roll_ref = PID_roll_.Run(roll_ref - roll); //- rate_roll_compensation
	    double torque_pitch_ref = PID_pitch_.Run(pitch_ref - pitch); //- rate_pitch_compensation
	    double torque_yaw_ref = PID_yaw_.Run(yaw_ref - yaw); //- rate_yaw_compensation

	    // build the force/torque vector and normalize with mass 
	    Eigen::Vector4d force_torque;
	    force_torque(0) = z_dd * mass_;
	    force_torque(1) = torque_roll_ref;
	    force_torque(2) = torque_pitch_ref;
	    force_torque(3) = torque_yaw_ref;		    

	    // compute the square of the rotor speed
	    Eigen::Vector4d Omega2_ref;
	    Omega2_ref = T_ * force_torque;    

	    //----------------------------COMPUTE THE ROTOR SPEED HERE--------------------------//
	    // compute the square root and saturate to the maximum speed
	    rotor_speed(0) = Saturator(std::sqrt(std::abs(Omega2_ref(0))), maximum_speed_, 0);
	  	rotor_speed(1) = Saturator(std::sqrt(std::abs(Omega2_ref(1))), maximum_speed_, 0);
	  	rotor_speed(2) = Saturator(std::sqrt(std::abs(Omega2_ref(2))), maximum_speed_, 0);
	  	rotor_speed(3) = Saturator(std::sqrt(std::abs(Omega2_ref(3))), maximum_speed_, 0);

	  	// logs
		ROS_DEBUG_STREAM_COND(logs, "control side: x error:       " << x_err);
		ROS_DEBUG_STREAM_COND(logs, "control side: y error:       " << y_err);
		ROS_DEBUG_STREAM_COND(logs, "control side: z error:       " << z_err);
		ROS_DEBUG_STREAM_COND(logs, "control side: roll desire:   " << roll_ref);
		ROS_DEBUG_STREAM_COND(logs, "control side: pitch desire:  " << pitch_ref);
		ROS_DEBUG_STREAM_COND(logs, "control side: yaw desire:    " << yaw_ref);
		ROS_DEBUG_STREAM_COND(logs, "control side: roll error:    " << roll_ref - roll);
		ROS_DEBUG_STREAM_COND(logs, "control side: pitch error:   " << pitch_ref - pitch);
		ROS_DEBUG_STREAM_COND(logs, "control side: yaw error:     " << yaw_ref - yaw);
		ROS_DEBUG_STREAM_COND(logs, "rate roll compensation:      " << rate_roll_compensation);
		ROS_DEBUG_STREAM_COND(logs, "rate pitch compensation:     " << rate_pitch_compensation);
		ROS_DEBUG_STREAM_COND(logs, "rate yaw compensation:       " << rate_yaw_compensation);
		ROS_DEBUG_STREAM_COND(logs, "control side: force z:       " << force_torque(0));
		ROS_DEBUG_STREAM_COND(logs, "control side: torque roll:   " << force_torque(1));
		ROS_DEBUG_STREAM_COND(logs, "control side: torque pitch:  " << force_torque(2));
		ROS_DEBUG_STREAM_COND(logs, "control side: torque yaw :   " << force_torque(3));
	  	ROS_DEBUG_STREAM_COND(logs, "control side: rotor speed 1: " << rotor_speed(0));
	  	ROS_DEBUG_STREAM_COND(logs, "control side: rotor speed 2: " << rotor_speed(1));
	  	ROS_DEBUG_STREAM_COND(logs, "control side: rotor speed 3: " << rotor_speed(2));
	  	ROS_DEBUG_STREAM_COND(logs, "control side: rotor speed 4: " << rotor_speed(3)); 

		return rotor_speed;
	}

	double PIDCascadeController::Saturator(double input, double upper_limit,
  		double lower_limit) {

    	if (input > upper_limit)
      		input = upper_limit;
    	if (input < lower_limit)
      		input = lower_limit;

    	return input;
  	}


}