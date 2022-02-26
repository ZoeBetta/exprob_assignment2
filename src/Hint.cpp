#include "exprob_assignment2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <cstdlib>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace KCL_rosplan {
HintInterface::HintInterface(ros::NodeHandle &nh) {
// here the initialization
}
bool HintInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
// here the implementation of the action 
std::cout << "Moving to get the hint " << std::endl;
geometry_msgs::Pose target_pose1;
moveit::planning_interface::MoveGroupInterface group("arm");

if(msg->parameters[0].value == "wp1"){
target_pose1.orientation.w = 0.0;
target_pose1.orientation.x = 0.0;
target_pose1.orientation.y = 0.0;
target_pose1.orientation.z = 0.0;
target_pose1.position.x = 0.6;
target_pose1.position.y = 0.0;
target_pose1.position.z = 1.25;
}
else if (msg->parameters[0].value == "wp2"){
	target_pose1.orientation.w = 0.0;
target_pose1.orientation.x = 0.0;
target_pose1.orientation.y = 0.0;
target_pose1.orientation.z = 0.0;
target_pose1.position.x = 0.0;
target_pose1.position.y = 0.6;
target_pose1.position.z = 0.75;
}
else if (msg->parameters[0].value == "wp3"){
	target_pose1.orientation.w = 0.0;
target_pose1.orientation.x = 0.0;
target_pose1.orientation.y = 0.0;
target_pose1.orientation.z = 0.0;
target_pose1.position.x = -0.6;
target_pose1.position.y = 0.0;
target_pose1.position.z = 1.25;
}
else if (msg->parameters[0].value == "wp4"){
	target_pose1.orientation.w = 0.0;
target_pose1.orientation.x = 0.0;
target_pose1.orientation.y = 0.0;
target_pose1.orientation.z = 0.0;
target_pose1.position.x =0.0;
target_pose1.position.y = -0.6;
target_pose1.position.z = 0.75;
}
group.setStartStateToCurrentState();
group.setApproximateJointValueTarget(target_pose1, "cluedo_link");
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
group.plan(my_plan);
group.execute(my_plan);

ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	return true;
	/*
int random=rand()%10;

if(random > 5)
	{ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	return true;}
else 
	{ROS_INFO("Action (%s) WRONG", msg->name.c_str());
		return false;}*/
}
}
int main(int argc, char **argv) {
ros::init(argc, argv, "hintaction", ros::init_options::AnonymousName);
ros::NodeHandle nh("~");
KCL_rosplan::HintInterface my_aci(nh);
my_aci.runActionInterface();
ros::AsyncSpinner spinner(1);
spinner.start();
sleep(2.0);
return 0;
}
