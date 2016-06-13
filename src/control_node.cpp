#include "control_node.h"


namespace control {

	void ControlNode::ViconCallback(const geometry_msgs::TransformStampedPtr& msg) {
		position_vicon_.x() = msg->transform.translation.x;
		position_vicon_.y() = msg->transform.translation.y;
		position_vicon_.z() = msg->transform.translation.z;

		orientation_vicon_.w() = msg->transform.rotation.w;
		orientation_vicon_.x() = msg->transform.rotation.x;
		orientation_vicon_.y() = msg->transform.rotation.y;
		orientation_vicon_.z() = msg->transform.rotation.z; 

        // compute the actual rate from the vicon
        time_stamp = msg->header.stamp.toSec();
        vicon_current_rate = time_stamp - previous_time_stamp;
        previous_time_stamp = time_stamp;

        // frame number
        vicon_frame_number = msg->header.seq;
	}

    void ControlNode::KflyCallBack(const ros_kflytelemetry::IMUDataPtr& msg) {
        rate_kfly_.x() = msg->angular_rate.x;
        rate_kfly_.y() = msg->angular_rate.y;
        rate_kfly_.z() = msg->angular_rate.z;
    
        // remove bias 
        /*rate_kfly_.x() = rate_kfly_.x() - 0.0185;
        rate_kfly_.y() = rate_kfly_.y() - 0.0457;
        rate_kfly_.z() = rate_kfly_.z() - (-0.0145);*/
    }

	ControlNode::ControlNode(ros::NodeHandle& nh) {

        // wait for 2 seconds to make sure that the kfly can connect
        std::this_thread::sleep_for(std::chrono::seconds(2)); 
                                                           
		// initialize parameters for the velocity
        vicon_current_rate = 0.01;
        old_pos = Eigen::Vector3d::Zero();
        old_orientation = Eigen::Vector3d::Zero();
        previous_orientation.w() = 1;
        previous_orientation.x() = 0;
        previous_orientation.y() = 0;
        previous_orientation.z() = 0;
        velocity_filtered_ = Eigen::Vector3d::Zero();
        angular_velocity_filtered_ = Eigen::Vector3d::Zero();
        angular_velocity_filtered_quat_ = Eigen::Vector3d::Zero();

		// eneble the DEBUG level logs
		log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(
			ros::console::g_level_lookup[ros::console::levels::Debug]);
		ros::console::notifyLoggerLevelsChanged();

		// publisher for the kfly output signals
		speed_msg_pub_ = nh.advertise<ros_kflytelemetry::DirectReference>(
			"/kfly_1/DirectReference", 5);

		// subscriber for the kfly Gyrooscope signals and ManageSubscription
        kfly_manage_sub_.request.port = 1;         // 1 xbee, 0 usb
        kfly_manage_sub_.request.cmd = 44;         // 54 = getIMUData
        kfly_manage_sub_.request.subscribe = true; // true
        kfly_manage_sub_.request.delta_ms = 20; 

        kfly_sub_ = nh.subscribe("/kfly_1/GyroscopeData", 10, 
            &ControlNode::KflyCallBack, this, ros::TransportHints().tcpNoDelay());

        ros::ServiceClient client = nh.serviceClient<ros_kflytelemetry::
            ManageSubscription>("/kfly_1/ManageSubscription");
        client.call(kfly_manage_sub_);
        ROS_INFO_STREAM("subsciption response from the kfly " << kfly_manage_sub_.response);

        // check if the response of the kfly is true, otherwise shutdown
        /*if (kfly_manage_sub_.response.success == 0){
            ROS_FATAL_STREAM("False response from the kfly ManageSubscription, shutdown");
            ros::shutdown();
        }*/

    	// connect to the vicon node
    	vicon_stream_sub = nh.subscribe("/vicon/darotor/darotor", 10, 
    		&ControlNode::ViconCallback, this, ros::TransportHints().tcpNoDelay());    

        // wait for 2 seconds to make sure that the vicon can connect
        std::this_thread::sleep_for(std::chrono::seconds(2));

	}	

	ControlNode::~ControlNode() {}    	

	void ControlNode::Shuttdown(ros_kflytelemetry::DirectReference kfly_motor_speed_msg) {
		boost::array<double, 4> rotor_speed_kfly;
		rotor_speed_kfly[0] = 0;
    	rotor_speed_kfly[1] = 0;
    	rotor_speed_kfly[2] = 0;
    	rotor_speed_kfly[3] = 0;

   		kfly_motor_speed_msg.header.stamp = ros::Time::now();
    	this->speed_msg_pub_.publish(kfly_motor_speed_msg);
    	ros::shutdown();
        return;
	} 

    double ControlNode::Saturator(double input, double upper_limit, double lower_limit) {

        if (input > upper_limit)
            input = upper_limit;
        if (input < lower_limit)
            input = lower_limit;

        return input;
    }


    Eigen::Vector3d ControlNode::ViconSpeed(Eigen::Vector3d current_pos,
        double sample_time, double lp_gain) {

        Eigen::Vector3d velocity = (current_pos - old_pos) / sample_time;
		old_pos = current_pos;
        // low pass filtering
        velocity_filtered_ = (1 - lp_gain)*velocity + lp_gain*velocity_filtered_;

        return velocity_filtered_;
    }

    Eigen::Vector3d ControlNode::ViconAngularVelocity(Eigen::Quaterniond
        current_orientation, double sample_time, double lp_gain) {

        // extract roll, pitch yaw
        Eigen::Vector3d orientation = 
            lee_position_controller_.Quaternion2Euler(current_orientation);

        Eigen::Vector3d angular_velocity = (orientation -
            old_orientation) / sample_time;

        // low pass filtering
        angular_velocity_filtered_ = (1 - lp_gain)*angular_velocity + 
            lp_gain*angular_velocity_filtered_;

        old_orientation = orientation;    

        return angular_velocity_filtered_;         
    }

    Eigen::Vector3d ControlNode::ViconAngularVelocityQuaternion(Eigen::Quaterniond
        current_orientation, double sample_time, double lp_gain) {

        Eigen::Quaterniond Delta_quat = current_orientation*previous_orientation.conjugate();

        float Delta_theta = 2 * acos(Delta_quat.w());

        Eigen::Vector3d u_bar;
        u_bar.x() = (1 / sin(Delta_theta/2)) * Delta_quat.x();
        u_bar.y() = (1 / sin(Delta_theta/2)) * Delta_quat.y();
        u_bar.z() = (1 / sin(Delta_theta/2)) * Delta_quat.z();
        float norm_omega = Delta_theta / sample_time;

        Eigen::Vector3d angular_velocity = norm_omega * u_bar;

        // Fast implementation
        /*Eigen::Vector3d angular_velocity;
        angular_velocity.x() = Delta_quat.x() * 2 / sample_time;
        angular_velocity.y() = Delta_quat.y() * 2 / sample_time;
        angular_velocity.z() = Delta_quat.z() * 2 / sample_time;*/ 

        previous_orientation = current_orientation;

        // low pass filtering
        angular_velocity_filtered_quat_ = (1 - lp_gain)*angular_velocity + 
            lp_gain*angular_velocity_filtered_quat_;

        if (isnan(angular_velocity_filtered_quat_.x()))
        	angular_velocity_filtered_quat_.x() = 0;
        if (isnan(angular_velocity_filtered_quat_.y()))
        	angular_velocity_filtered_quat_.y() = 0;
        if (isnan(angular_velocity_filtered_quat_.z()))
        	angular_velocity_filtered_quat_.z() = 0;


        return angular_velocity_filtered_quat_;         
    }    


}

void DynamicReconfigureCallBack(control::DynamicParametersConfig &config, uint32_t level) {
    
    ROS_INFO("Reconfigure Request");

    control::position_gain_ = config.position_gain;
    control::velocity_gain_ = config.velocity_gain;
    control::attitude_gain_ = config.attitude_gain;
    control::angular_velocity_gain_ = config.angular_velocity_gain;

}

int main(int argc, char** argv) {

	ros::init(argc, argv, "control_node");
	ROS_INFO("control_node start");

    const bool logs = true;
    const bool logs_control_side = true;
    bool initial_calibration = false;

    unsigned int control_frequency = 50;
	const long double PI = 3.141592653589793238L;

    unsigned int iteration_number = 0;
    unsigned int old_frame_number = 0;

	ros::NodeHandle nh;
	ros::Rate loop_rate(control_frequency);  // set the frequency
	control::ControlNode control_node(nh);   // new ControlNode object


    // dynamic reconfigure
    dynamic_reconfigure::Server<control::DynamicParametersConfig> server;
    dynamic_reconfigure::Server<control::DynamicParametersConfig>::CallbackType f;
    f = boost::bind(&DynamicReconfigureCallBack, _1, _2);
    server.setCallback(f);

    /*
        Reference trajectories
    */
    Eigen::Vector3d position_ref = Eigen::Vector3d::Zero();

    // new message for the kfly
    ros_kflytelemetry::DirectReference kfly_motor_speed_msg; 

	/*
		Initial calibration
	*/
	Eigen::Vector3d initial_position;
	double initial_yaw = 0;
	double initial_time = ros::Time::now().toSec();
    boost::array<double, 4> rotor_speed_kfly; // array for the kfly output
    
    /*
    	CONTROL LOOP
    */
    while(ros::ok()) {

        control_node.lee_position_controller_.ResetParameters(
        	control::position_gain_,
        	control::velocity_gain_,
        	control::attitude_gain_,
        	control::angular_velocity_gain_);    


    	/* 
            Time
        */
    	double current_time = ros::Time::now().toSec() - initial_time;

        /*
            Ack to obtain the initial position from the vicon system
        */
        if ((!initial_calibration) && (iteration_number > 200)){  // initial calibration
            initial_position = control_node.position_vicon_;

            double q_0 = control_node.orientation_vicon_.w();
            double q_1 = control_node.orientation_vicon_.x();
            double q_2 = control_node.orientation_vicon_.y();
            double q_3 = control_node.orientation_vicon_.z();

            initial_yaw = atan2(2*(q_0*q_3 + q_1*q_3), 1-2*(pow(q_2, 2) + pow(q_3, 2)));
            initial_calibration = true;

            ROS_INFO_STREAM("initial vicon position");
            ROS_INFO_STREAM(" " << initial_position.x());
            ROS_INFO_STREAM(" " << initial_position.y());
            ROS_INFO_STREAM(" " << initial_position.z());            
        }


        /* 
            First set the speed to zero (I will use the output 0,1,2,3)
        */
        rotor_speed_kfly[0] = 0;
        rotor_speed_kfly[1] = 0;
        rotor_speed_kfly[2] = 0;
        rotor_speed_kfly[3] = 0;

    	/* 
    		Get position and orientation from the vicon system
    	*/
        Eigen::Vector3d euler_vicon = 
            control_node.lee_position_controller_.Quaternion2Euler(
            control_node.orientation_vicon_);
      	// adjust the yaw 
        double yaw = euler_vicon.z() - initial_yaw;     
    	if (yaw < -PI)
    		yaw = yaw + 2*PI;
    	else if(yaw > PI)
    		yaw = yaw - 2*PI;
        euler_vicon.z() = yaw;    	

        // rebuild the quaternion  
        Eigen::Quaterniond orientation_W = 
            control_node.lee_position_controller_.Euler2Quaternion(euler_vicon);

        // adjust the position
        Eigen::Vector3d position_W = control_node.position_vicon_ - initial_position;

        /*
            Get the velocity and angular_rate from the vicon
        */
        Eigen::Vector3d velocity_W(Eigen::Vector3d::Zero());
        Eigen::Vector3d angular_velocity_B(Eigen::Vector3d::Zero());
        Eigen::Vector3d angular_velocity_B_quat(Eigen::Vector3d::Zero());
        if ((control_node.position_vicon_ != Eigen::Vector3d::Zero()) && 
            initial_calibration) {
            unsigned int current_frame = control_node.vicon_frame_number;
        	unsigned int delta_frame = control_node.vicon_frame_number - old_frame_number;

            // probabily af the first iteration it will be happen
            if (delta_frame == 0)
                delta_frame = 1;

            float sample_time = (1/(float (control_frequency)))*delta_frame;

        	velocity_W = control_node.ViconSpeed(position_W, 
            	sample_time, 0.7);

            angular_velocity_B_quat = orientation_W.toRotationMatrix()*
                control_node.ViconAngularVelocityQuaternion(orientation_W, sample_time, 0.7);

            old_frame_number = current_frame;
        }        
 	
        /*
            Get angular rate from the kfly
        */
        Eigen::Vector3d angular_rate_kfly = control_node.rate_kfly_;


        /*
        	Safety check in the angle (roll or pitch) and position (x, y or z)
        */
	  	if (((std::abs(euler_vicon.x()) > 3.14/4) 
            || (std::abs(euler_vicon.y()) > 3.14/4)) && 
            (initial_calibration)) {
	  		ROS_FATAL_STREAM("Enter in safety mode, out of angle");
        	ROS_FATAL_STREAM("vicon roll: " << euler_vicon.x());
        	ROS_FATAL_STREAM("vicon pitch: " << euler_vicon.y());	  		
	  		control_node.Shuttdown(kfly_motor_speed_msg);
	  		ros::shutdown();
	  	}
	    if (((std::abs(position_ref.x() - position_W.x()) > 0.2) || 
            (std::abs(position_ref.y() - position_W.y()) > 0.2)  ||
            (std::abs(position_ref.z() - position_W.z()) > 0.2)) && (initial_calibration)){
	  		ROS_FATAL_STREAM("Enter in safety mode, out of position");
	  		ROS_FATAL_STREAM("error x: " << std::abs(position_ref.x() - position_W.x()));
	  		ROS_FATAL_STREAM("error y: " << std::abs(position_ref.y() - position_W.y()));
	  		ROS_FATAL_STREAM("error z: " << std::abs(position_ref.z() - position_W.z()));
	  		control_node.Shuttdown(kfly_motor_speed_msg);
	  		ros::shutdown();
	  	}


	  	if (initial_calibration){
	        
            /*
	            Build the trajectory msg
	        */
	        mav_msgs::EigenTrajectoryPoint trajectory_ref;
	        trajectory_ref.setFromYaw(0);
	        trajectory_ref.position_W = Eigen::Vector3d::Zero();
	        trajectory_ref.velocity_W = Eigen::Vector3d::Zero();
	        trajectory_ref.acceleration_W = Eigen::Vector3d::Zero();

	        /*
	            Compute motor speed
	        */
	        Eigen::Vector4d rotor_speed = 
	            control_node.lee_position_controller_.CalculateRotorVelocities(
	            orientation_W, position_W, velocity_W, angular_rate_kfly, 
	            trajectory_ref, control_node.debug_file, logs_control_side);
	        // saturate for safety
	        rotor_speed_kfly[0] = control_node.Saturator(rotor_speed[0], 1, 0);
	        rotor_speed_kfly[1] = control_node.Saturator(rotor_speed[1], 1, 0);
	        rotor_speed_kfly[2] = control_node.Saturator(rotor_speed[2], 1, 0);
	        rotor_speed_kfly[3] = control_node.Saturator(rotor_speed[3], 1, 0);       

            rotor_speed_kfly[0] = 0;
            rotor_speed_kfly[1] = 0;
            rotor_speed_kfly[2] = 0;
            rotor_speed_kfly[3] = 0;   

	        /* 
	            Message to the engines
	        */
            // comment the following lines for no engines' power    
            kfly_motor_speed_msg.header.stamp = ros::Time::now();
            kfly_motor_speed_msg.throttle = rotor_speed_kfly;  
            control_node.speed_msg_pub_.publish(kfly_motor_speed_msg);
        }         


        /*
            Debug logs
        */
        if (logs && initial_calibration) {
            if (!control_node.debug_file.is_open())
                control_node.debug_file.open("debug_logs.log");
            
	        control_node.debug_file << "debug ref position x: " << position_ref.x() << "\n";
	        control_node.debug_file << "debug ref position y: " << position_ref.y() << "\n";
	        control_node.debug_file << "debug ref position z: " << position_ref.z() << "\n";
	        control_node.debug_file << "debug vicon position x: " << position_W.x() << "\n";
	        control_node.debug_file << "debug vicon position y: " << position_W.y() << "\n";
	        control_node.debug_file << "debug vicon position z: " << position_W.z() << "\n";
	        control_node.debug_file << "debug vicon roll: " << euler_vicon.x() << "\n";
	        control_node.debug_file << "debug vicon pitch: " << euler_vicon.y() << "\n";
	        control_node.debug_file << "debug vicon yaw: " << euler_vicon.z() << "\n";
            control_node.debug_file << "debug kfly rate x: " << angular_rate_kfly.x() << "\n";
            control_node.debug_file << "debug kfly rate y: " << angular_rate_kfly.y() << "\n";
            control_node.debug_file << "debug kfly rate z: " << angular_rate_kfly.z() << "\n";     
	        control_node.debug_file << "debug vicon rate x: " << angular_velocity_B_quat.x() << "\n";
	        control_node.debug_file << "debug vicon rate y: " << angular_velocity_B_quat.y() << "\n";
	        control_node.debug_file << "debug vicon rate z: " << angular_velocity_B_quat.z() << "\n";
	        control_node.debug_file << "debug rotor speed 1: " << rotor_speed_kfly[0] << "\n";
	        control_node.debug_file << "debug rotor speed 2: " << rotor_speed_kfly[1] << "\n";
	        control_node.debug_file << "debug rotor speed 3: " << rotor_speed_kfly[2] << "\n";
	        control_node.debug_file << "debug rotor speed 4: " << rotor_speed_kfly[3] << "\n";
	        control_node.debug_file << "debug velocity x: " << velocity_W.x() << "\n";
	        control_node.debug_file << "debug velocity y: " << velocity_W.y() << "\n";
	        control_node.debug_file << "debug velocity z: " << velocity_W.z() << "\n";
	    	control_node.debug_file << "debug vicon rate: " 
	    		<< control_node.vicon_current_rate << "\n";
	        control_node.debug_file << "debug iteration number: " << iteration_number << "\n";
	        control_node.debug_file << "\n";
        }

        iteration_number++;
     	ros::spinOnce();
    	loop_rate.sleep();
    }
    return 0;
}

