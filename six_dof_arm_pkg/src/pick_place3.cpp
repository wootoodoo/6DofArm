#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std;
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void openGripper(moveit::planning_interface::MoveGroupInterface& gripper)
{
  /* Add both finger joints of panda robot. */
    const robot_state::JointModelGroup* joint_model_group_gripper =
      gripper.getCurrentState()->getJointModelGroup("gripper");

    moveit::core::RobotStatePtr current_state = gripper.getCurrentState();
  
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    joint_group_positions[0] = 0.025;  
    joint_group_positions[1] = -0.025;  
    gripper.setJointValueTarget(joint_group_positions);
    gripper.move();
    cout<<"gripper opened! joint1: "<< joint_group_positions[0]<< endl; 
}
void closedGripper(moveit::planning_interface::MoveGroupInterface& gripper)
{
    const robot_state::JointModelGroup* joint_model_group_gripper =
      gripper.getCurrentState()->getJointModelGroup("gripper");

    moveit::core::RobotStatePtr current_state = gripper.getCurrentState();
  
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    joint_group_positions[0] = 0.02;  
    joint_group_positions[1] = -0.020;  
    gripper.setJointValueTarget(joint_group_positions);
    gripper.move();
    cout<<"gripper closed! joint1: "<< joint_group_positions[0]<< endl; 
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "six_dof_arm_planner");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);

	spinner.start();

	moveit::planning_interface::MoveGroupInterface group("arm");
	moveit::planning_interface::MoveGroupInterface gripper("gripper");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


	string line;

	//Waiting for scene initialization
	sleep(2);


	//--- objects into the scene
	moveit::planning_interface::PlanningSceneInterface current_scene;
	geometry_msgs::Pose pose;

	//---Add grasping object to the scene
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.01;
	primitive.dimensions[1] = 0.01;
	primitive.dimensions[2] = 0.08;

    moveit_msgs::CollisionObject grasping_object;
    grasping_object.id = "grasping_object";
    pose.orientation.w = 1.0;
    pose.position.y =  0.2;
    pose.position.x =  0.2;
    pose.position.z =  0.055;

    grasping_object.primitives.push_back(primitive);
    grasping_object.primitive_poses.push_back(pose);
    grasping_object.operation = grasping_object.ADD;
    grasping_object.header.frame_id = "base_link";

	std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(grasping_object);
	//---


	// Once all of the objects (in this case just one) have been added to the
	// vector, we tell the planning scene to add our new box
	current_scene.addCollisionObjects(collision_objects);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	const robot_state::JointModelGroup *joint_model_group =
  group.getCurrentState()->getJointModelGroup("arm");
    
  //   geometry_msgs::Pose open_gripper;
  //   // open_gripper.points.resize(1);
  //   // open_gripper.points[0].positions.resize(2);
  // // open_gripper.points[0].positions.resize(2);
  // open_gripper.position.x = 0.025;
  // // open_gripper.position[1] = -0.025;
  //   gripper.setPoseTarget(open_gripper);
     openGripper(gripper);

	//---approaching
     geometry_msgs::Pose target_pose;

    // target_pose.orientation.x = -0.699;
    // target_pose.orientation.y = 0.2902;
    // target_pose.orientation.z = -0.2504;
    // target_pose.orientation.w = 0.60334;
	// target_pose.position.x = 0.124;
	// target_pose.position.y = 0.128;
	// target_pose.position.z = 0.0745;
	// group.setApproximateJointValueTarget(target_pose,"gripper_roll_link");
	// group.move();
	// sleep(1);
    // cout << "In position" << endl;
	//---grasping
    target_pose.orientation.x = 0.8594;
    target_pose.orientation.y = -0.2573;
    target_pose.orientation.z = 0.16756;
    target_pose.orientation.w = -0.408893;
	target_pose.position.x = 0.1373;
	target_pose.position.y = 0.299551;
	target_pose.position.z = 0.0995;
	group.setApproximateJointValueTarget(target_pose,"gripper_roll_link");
	group.move();
       
    cout << "completed approach" << endl;

    closedGripper(gripper);
    cout << "completed grasping" << endl;

	//---attach object to the robot
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "gripper_roll_link";
    attached_object.object = grasping_object;
    current_scene.applyAttachedCollisionObject( attached_object );

	//---move far away from the grasping position
    target_pose.orientation.x = 0.2058;
    target_pose.orientation.y = -0.6674;
    target_pose.orientation.z = 0.6936;
    target_pose.orientation.w = -0.1764;
	target_pose.position.x = 0.13531;
	target_pose.position.y = -0.2414;
	target_pose.position.z = 0.0292;
	group.setApproximateJointValueTarget(target_pose,"gripper_roll_link");
	group.move();
	sleep(1);
    
    cout << "completed moving to new position" << endl;

	//---setting down
	// target_pose.orientation.x = 0.5779;
    // target_pose.orientation.y = 0.310337;
    // target_pose.orientation.z = 0.336;
    // target_pose.orientation.w = 0.6759;
	// target_pose.position.y = -0.1313;
	// target_pose.position.x = 0.1853;
	// target_pose.position.z = 0.3131;
	// group.setApproximateJointValueTarget(target_pose,"gripper_roll_link");
	// group.move();
	// //---
    // cout << "completed placement" << endl;

    openGripper(gripper);    
	// target_pose.position.y = -0.1;
	// target_pose.position.x = 0.24;
	// target_pose.position.z = 0.32;
	// group.setPoseTarget(target_pose);
	// group.move();

    cout << "opened gripper" << endl;
	//---remove object from robot's body
    grasping_object.operation = grasping_object.REMOVE;
    attached_object.link_name = "gripper_roll_link";
    attached_object.object = grasping_object;
    current_scene.applyAttachedCollisionObject( attached_object );
	
    // Move back to original position
	target_pose.orientation.x = 0.298;
    target_pose.orientation.y = 0.2396; 
    target_pose.orientation.z = 0.6428;
    target_pose.orientation.w = 0.6637;
	target_pose.position.y = -0.00518;
	target_pose.position.x = 0.04144;
	target_pose.position.z = 0.3933;
	group.setApproximateJointValueTarget(target_pose,"gripper_roll_link");
	group.move();
    cout << "back to the original position" << endl;

	ros::shutdown();

}
