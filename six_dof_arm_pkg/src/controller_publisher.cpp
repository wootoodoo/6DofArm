#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


/**
 * To publish servo messages to /joint_states topic
header: 
  seq: 19907
  stamp: 
    secs: 1990
    nsecs: 700000000
  frame_id: ''

name: [elbow_pitch_joint, finger_joint1, finger_joint2, gripper_roll_joint, shoulder_pan_joint, shoulder_pitch_joint, wrist_pitch_joint]

position: [1.5545836262731427e-07, 4.726192249210016e-06, 2.3685536518588065e-06, -1.0482390688792975e-07, -1.9542095586189134e-07, 1.5554293675279496e-07, 1.464164496312037e-07]

velocity: [5.662307803637641e-05, 0.0023297878168340334, 0.0023297879687953216, -4.3472200011862385e-06, -4.205880274686499e-05, 5.512313418381924e-05, 5.383743719964383e-05]

effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
* 
* To publish controller messages to /servo_command topic
* joint_names: [shoulder_pan_joint, shoulder_pitch_joint, elbow_pitch_joint, wrist_pitch_joint]
desired: 
  positions: [-4.1836183797568023e-05, 0.14830048908223387, 1.4659162815941036, 1.4310166114472307]
  velocities: [0.0, 0.0, 0.0, 0.0]
  accelerations: [0.0, 0.0, 0.0, 0.0]
  * 
joint_names: [gripper_roll_joint, finger_joint1, finger_joint2]
desired: 
  positions: [9.298007785929485e-08, -2.055991289504329e-08, -2.168114193806892e-06]
  velocities: [0.0, 0.0, 0.0]
  accelerations: [0.0, 0.0, 0.0]
  effort: []



 */

std_msgs::String _servo_command;

int servo0, servo1, servo2, servo3, servo4, servo5 = 0;

void armCallback(const control_msgs::JointTrajectoryControllerState::desired::ConstPtr& msg)
{
  int servo0 = msg->positions[0] * 57.295;
  int servo1 = msg->positions[1]* 57.295;
  int servo2 = msg->positions[2] * 57.295;
  int servo3 = msg->positions[3] * 57.295;
  //ROS_INFO ("I heard %d \n" , servo0 , servo1 ,servo2 ,servo3);
}


void gripperCallback(const control_msgs::JointTrajectoryControllerState::desired::ConstPtr& msg)
{
  int servo4 = msg->positions[0] * 57.295;
  int servo5 = 115 - msg->positions[1]/0.025*80; // Map from 0 - 0.025 to 115 to 35
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "pub_joint_command_test");

  ros::NodeHandle n;

  ros::Subscriber subArm = n.subscribe("/six_dof_arm/six_dof_arm_joint_controller/state", 1000, armCallback);
  
  ros::Subscriber subGripper = n.subscribe("/six_dof_arm/gripper_controller/state", 1000, gripperCallback);
  
  ros::Publisher servo_command = n.advertise<std_msgs::String>("servo_command", 1000);

	while (ros::ok()){
	 
	ros::spinOnce();
	
	std::stringstream ss;
	ss << servo0 << ',' << servo1 << ',' << servo2 << ',' << servo3 << ',' << servo4 << ',' << servo5;
	_servo_command.data = ss.str();
	
	// To include a publisher on "servo_command"
	servo_command.publish(_servo_command);  	
	  
	ros::Duration(0.2).sleep();
	}

}
