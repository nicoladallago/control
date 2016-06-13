#include "control/angular_rate_controller.h"

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
	//                   FULL CASCADE ANGULAR RATE CONTROLLER                        //
	///////////////////////////////////////////////////////////////////////////////////	
	AngularRateController::AngularRateController() {
		InitializeParameters();
	}
	
	AngularRateController::~AngularRateController() {}
	
	void AngularRateController::InitializeParameters() {
		SetPIDs();
	    MixingMatrix();

	    safety_control_ = false;
	    initialized_params_ = true;
	}
	
	void AngularRateController::MixingMatrix() {
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
	
	void AngularRateController::SetPIDs() {
		PID_axis_x_.SetParameters(0.01, P_axis_x_, I_axis_x_, D_axis_x_, 100);
		PID_axis_y_.SetParameters(0.01, P_axis_y_, I_axis_y_, D_axis_y_, 100);
		PID_axis_z_.SetParameters(0.01, P_axis_z_, I_axis_z_, D_axis_z_, 100);

		PI_rate_roll_.SetParameters( 0.01, P_rate_roll_,  I_rate_roll_,  100);
		PI_rate_pitch_.SetParameters(0.01, P_rate_pitch_, I_rate_pitch_, 100);
		PI_rate_yaw_.SetParameters(  0.01, P_rate_yaw_,   I_rate_yaw_,   100);

		PID_z_.SetParameters(0.01, P_z_, I_z_, D_z_, 100);
	}

	void AngularRateController::ResetDynamicsParametrs(
		double thrust_factor, double drag_factor,
		double P_z, double I_z, double D_z, 
		double P_roll, double I_roll, double D_roll,
		double P_pitch, double I_pitch, double D_pitch, 
		double P_yaw, double I_yaw, double D_yaw, 
		double P_rate_roll, double I_rate_roll,
		double P_rate_pitch, double I_rate_pitch,
		double P_rate_yaw, double I_rate_yaw) {
		
		thrust_factor_ = thrust_factor;
		drag_factor_ = drag_factor;

		P_z_ = P_z;
		I_z_ = I_z;
		D_z_ = D_z;

		P_axis_x_ = P_roll;
		I_axis_x_ = I_roll;
		D_axis_x_= D_roll;
		P_axis_y_ = P_pitch;
		I_axis_y_ = I_pitch;
		D_axis_y_ = D_pitch;
		P_axis_z_ = P_yaw;
		I_axis_z_= I_yaw;
		D_axis_z_ = D_yaw;	

		P_rate_roll_ = P_rate_roll;
		I_rate_roll_ = I_rate_roll;
		P_rate_pitch_ = P_rate_pitch;
		I_rate_pitch_ = I_rate_pitch;				
		P_rate_yaw_ = P_rate_yaw;
		I_rate_yaw_ = I_rate_yaw;

		// recompute mixing matrix
		MixingMatrix();

		//reset PIDs
		SetPIDs();
	}	

	Eigen::Vector3d AngularRateController::Quaternion2Euler(Eigen::Quaterniond quaternion) {
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

	Eigen::Quaterniond AngularRateController::Euler2Quaternion(Eigen::Vector3d euler) {
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

	Eigen::Vector3d AngularRateController::KroneckerProduct(Eigen::Quaterniond q, 
		Eigen::Quaterniond q_ref){

		Eigen::Matrix4d Q_q_ref;
		Q_q_ref << q_ref.w(), -q_ref.x(), -q_ref.y(), -q_ref.z(),
				   q_ref.x(),  q_ref.w(), -q_ref.z(),  q_ref.y(),
				   q_ref.y(),  q_ref.z(),  q_ref.w(), -q_ref.x(),
				   q_ref.z(), -q_ref.y(),  q_ref.x(),  q_ref.w(); 

		Eigen::Vector4d q_conj;
		q_conj << q.w(), - q.x(), -q.y(), -q.z();		   
		Eigen::Vector4d q_err = Q_q_ref * q_conj;
		int sgn = -1;
		if (q_err(0) > 0)
			sgn = 1;

		Eigen::Vector3d axis_error;
		axis_error.x() = sgn * q_err(1);
		axis_error.y() = sgn * q_err(2);
		axis_error.z() = sgn * q_err(3);

		return axis_error;
	}
	
	/*
		Safety check MUST be done in the control node!!!
	*/
	Eigen::Vector4d AngularRateController::CalculateRotorVelocities(
		mav_msgs::EigenTrajectoryPoint command_trajectory, Eigen::Vector3d position_W, 
		Eigen::Quaterniond orientation, Eigen::Vector3d rate_B, bool logs) {
		
		// set the speed to zero at the beginning
		Eigen::Vector4d rotor_speed =  Eigen::Vector4d::Zero(); 

		// compute all the control variable
		double x = position_W.x();
		double y = position_W.y();
		double z = position_W.z();
		double x_ref = command_trajectory.position_W.x();
		double y_ref = command_trajectory.position_W.y();
		double z_ref = command_trajectory.position_W.z();
		double x_err = x_ref - x;
		double y_err = y_ref - y;
		double z_err = z_ref - z;
		double rate_roll = rate_B.x();
		double rate_pitch = rate_B.y();
		double rate_yaw = rate_B.z();
		Eigen::Quaterniond orientation_ref = command_trajectory.orientation_W_B; 

		// if negative desire height, set the speed to zero
		if (command_trajectory.position_W.z() < 0)
			return rotor_speed; 						

	    /*
	    	CONTROL ALGORITHM START HERE
	    */
	    // computing quaternion error (eq 19) and converting in axis error (eq 20)
	    Eigen::Vector3d axis_error = KroneckerProduct(orientation, orientation_ref);
		double x_axis_err = axis_error.x();
	    double y_axis_err = axis_error.y();
	    double z_axis_err = axis_error.z();

	    // PID in the axis errors
	    double x_axis_control = PID_axis_x_.Run(x_axis_err);
	    double y_axis_control = PID_axis_y_.Run(y_axis_err);
	    double z_axis_control = PID_axis_z_.Run(z_axis_err);

	    // PI in the rate
	    double roll_rate_control = PI_rate_roll_.Run(rate_roll);
	    double pitch_rate_control = PI_rate_pitch_.Run(rate_pitch);
	    double yaw_rate_control = PI_rate_yaw_.Run(rate_yaw);

	    // compute the torque
	    double torque_x = x_axis_control - roll_rate_control;
	    double torque_y = y_axis_control - pitch_rate_control;
	    double torque_z = z_axis_control - yaw_rate_control;  

	    // PID in the z direction
	    double force = mass_ * (PID_z_.Run(z_err) + gravity_);

	    // force and torque vector
	    Eigen::Vector4d force_torque;
	    force_torque(0) = force;
	    force_torque(1) = torque_x;
	    force_torque(2) = torque_y;
	    force_torque(3) = torque_z;

	   	// compute the square of the rotor speed
	    Eigen::Vector4d Omega2_ref;
	    Omega2_ref = T_ * force_torque;

	    // rotor speed
	    rotor_speed(0) = Saturator(sqrt(abs(Omega2_ref(0))), maximum_speed_, 0);
	  	rotor_speed(1) = Saturator(sqrt(abs(Omega2_ref(1))), maximum_speed_, 0);
	  	rotor_speed(2) = Saturator(sqrt(abs(Omega2_ref(2))), maximum_speed_, 0);
	  	rotor_speed(3) = Saturator(sqrt(abs(Omega2_ref(3))), maximum_speed_, 0);	    

	    /*
	    	Debug variables
	    */
	    ROS_DEBUG_STREAM_COND(logs, "control side: x err: " << x_err);
	    ROS_DEBUG_STREAM_COND(logs, "control side: y err: " << y_err);
	    ROS_DEBUG_STREAM_COND(logs, "control side: z err: " << z_err);
	    ROS_DEBUG_STREAM_COND(logs, "control side: q_ref w: " << orientation_ref.w());
	    ROS_DEBUG_STREAM_COND(logs, "control side: q_ref x: " << orientation_ref.x());
	    ROS_DEBUG_STREAM_COND(logs, "control side: q_ref y: " << orientation_ref.y());
	    ROS_DEBUG_STREAM_COND(logs, "control side: q_ref z: " << orientation_ref.z());
	    ROS_DEBUG_STREAM_COND(logs, "control side: q w: " << orientation.w());
	    ROS_DEBUG_STREAM_COND(logs, "control side: q x: " << orientation.x());
	    ROS_DEBUG_STREAM_COND(logs, "control side: q y: " << orientation.y());
	    ROS_DEBUG_STREAM_COND(logs, "control side: q z: " << orientation.z());
	    ROS_DEBUG_STREAM_COND(logs, "control side: axis_err x: " << x_axis_err);
	    ROS_DEBUG_STREAM_COND(logs, "control side: axis_err y: " << y_axis_err);
	    ROS_DEBUG_STREAM_COND(logs, "control side: axis_err z: " << z_axis_err);
	    ROS_DEBUG_STREAM_COND(logs, "control side: roll rate : " << rate_roll);
	    ROS_DEBUG_STREAM_COND(logs, "control side: pitch rate: " << rate_pitch);
	    ROS_DEBUG_STREAM_COND(logs, "control side: yaw rate  : " << rate_yaw);
	    ROS_DEBUG_STREAM_COND(logs, "control side: x axis control: " << x_axis_control);
	    ROS_DEBUG_STREAM_COND(logs, "control side: y axis control: " << y_axis_control);
	    ROS_DEBUG_STREAM_COND(logs, "control side: z axis control: " << z_axis_control);
	    ROS_DEBUG_STREAM_COND(logs, "control side: roll rate control: " << roll_rate_control);
	    ROS_DEBUG_STREAM_COND(logs, "control side: pitch rate control: " << pitch_rate_control);
	    ROS_DEBUG_STREAM_COND(logs, "control side: yaw rate control: " << yaw_rate_control);
	    ROS_DEBUG_STREAM_COND(logs, "control side: torque x: " << torque_x);
	    ROS_DEBUG_STREAM_COND(logs, "control side: torque y: " << torque_y);
	    ROS_DEBUG_STREAM_COND(logs, "control side: torque z: " << torque_z);
	    ROS_DEBUG_STREAM_COND(logs, "control side: rotor speed 1: " << rotor_speed(0));
	  	ROS_DEBUG_STREAM_COND(logs, "control side: rotor speed 2: " << rotor_speed(1));
	  	ROS_DEBUG_STREAM_COND(logs, "control side: rotor speed 3: " << rotor_speed(2));
	  	ROS_DEBUG_STREAM_COND(logs, "control side: rotor speed 4: " << rotor_speed(3));


		return rotor_speed;

	}

	double AngularRateController::Saturator(double input, double upper_limit,
  		double lower_limit) {

    	if (input > upper_limit)
      		input = upper_limit;
    	if (input < lower_limit)
      		input = lower_limit;

    	return input;
  	}


}