#include <six_dof_arm_hardware_interface/robot_hardware_interface.h>

#define DEG_TO_PI 0.01745329;
#define PI_TO_DEG 57.29578;

void ROBOTHardwareInterface::callBack(const std_msgs::Int16MultiArray::ConstPtr& message)  {
       double read_position[8] = {(message->data[0]-95)*DEG_TO_PI,   // shoulder pan
	                      		(message->data[1]-90)*DEG_TO_PI,     // shoulder pitch
                                (message->data[2]-90)*DEG_TO_PI,     //elbow pitch
                                (message->data[3]-85)*DEG_TO_PI,     //elbow roll
                                (message->data[4]-80)*DEG_TO_PI,     //wrist pitch
                                (message->data[5])*DEG_TO_PI,        //gripper roll
                                (115-message->data[6])/80*0.025,     //gripper
                                (message->data[7]-115)/80*0.025};
        
        for (int i =0 ; i<8; i++){
            this->subscriber_position[i] = read_position[i];
        }

    // ROS_INFO("Receiving  j1: %.2f, j2: %.2f, j3: %.2f, j4: %.2f, j5: %.2f, j6: : %.2f, j7: %.2f",subscriber_position[0],subscriber_position[1], subscriber_position[2],subscriber_position[3], subscriber_position[4],subscriber_position[5],subscriber_position[6]);
	//std::vector<double> subscriber_position(position, position + n);
}

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=50;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
 
    pub = nh.advertise<std_msgs::String>("/servo_commands",1000); //buffer up 1000 messages before throwing some away.
    sub = nh.subscribe<std_msgs::Int16MultiArray>("/read_joint_state", 1000, &ROBOTHardwareInterface::callBack, this);

}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

void ROBOTHardwareInterface::init() {
    
   	num_joints_= 8;
	joint_names_[0]="shoulder_pan_joint";	
	joint_names_[1]="shoulder_pitch_joint";
	joint_names_[2]="elbow_pitch_joint";
	joint_names_[3]="elbow_roll_joint";	
	joint_names_[4]="wrist_pitch_joint";	
	joint_names_[5]="gripper_roll_joint";
	joint_names_[6]="finger_joint1";
	joint_names_[7]="finger_joint2";
	

    for (int i = 0; i < num_joints_; ++i) {

         // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create position joint interface
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        
        position_joint_interface_.registerHandle(jointPositionHandle);

	// Create position joint limits
    //joint_limits_interface::JointLimits limits;
    //joint_limits_interface::SoftJointLimits softLimits;
            
    //getJointLimits(joint_names_[i], nh_, limits);
      //joint_limits_interface::PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
      //positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
   
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
  //  registerInterface(&positionJointSoftLimitsInterface);
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}


void ROBOTHardwareInterface::read() {

//    string names = {"shoulder_pan_joint","shoulder_pitch_joint",
//"elbow_pitch_joint", "wrist_pitch_joint","gripper_roll_joint","finger_joint1","finger_joint2"};
	joint_position_[0] = this->subscriber_position[0]; 
	joint_position_[1] = this->subscriber_position[1]; 
	joint_position_[2] = this->subscriber_position[2]; 
	joint_position_[3] = this->subscriber_position[3]; 
	joint_position_[4] = this->subscriber_position[4];
	joint_position_[5] = this->subscriber_position[5]; 
	joint_position_[6] = this->subscriber_position[6]; 
	joint_position_[7] = this->subscriber_position[7]; 


    // ROS_INFO("Receiving  j1: %.2f, j2: %.2f, j3: %.2f, j4: %.2f, j5: %.2f, j6: : %.2f, j7: %.2f",subscriber_position[0],subscriber_position[1], subscriber_position[2],subscriber_position[3], subscriber_position[4],subscriber_position[5],subscriber_position[6]);
}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
    int IN[7] = {int(joint_position_command_[0]*PI_TO_DEG)+95, //shoulder_pan
		 int(joint_position_command_[1]*PI_TO_DEG)+90, //shoulder_pitch
		 int(joint_position_command_[2]*PI_TO_DEG)+90, // elbow_pitch
		 int(joint_position_command_[3]*PI_TO_DEG)+85, //elbow_roll
		 int((joint_position_command_[4])*PI_TO_DEG)+80, // wrist_pitch
		 int(joint_position_command_[5]*PI_TO_DEG), //gripper_roll
		 int(115-80/0.025*joint_position_command_[6])};  //gripper

	std::stringstream ss;
	ss << IN[0] << ',' << IN[1] << ',' << IN[2] << ',' << IN[3] << ',' << IN[4] << ',' << IN[5] << ',' << IN[6];
    
	const std::string tmp = ss.str();
	joints_pub.data = tmp.c_str();
	pub.publish(joints_pub);	
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "six_dof_arm_hardware_interface");
    ros::NodeHandle nh;

    //ros::AsyncSpinner spinner(2); 
    ros::MultiThreadedSpinner spinner(2);// 2 threads for controller publisher and for the subscriber used to get the feedback from ardiuno
    
    ROBOTHardwareInterface ROBOT(nh);

    //spinner.start();
    spinner.spin();
    //ros::spin();
    return 0;
}
