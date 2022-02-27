#include "exprob_assignment2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <cstdlib>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <exprob_assignment2/ErlOracle.h>
int received=0;
int previous=1;
void hintCallback( const exprob_assignment2::ErlOracle x)
{
	received=1;
}
int movehigh()
{
	geometry_msgs::Pose target_pose1;
moveit::planning_interface::MoveGroupInterface group("arm");
group.setEndEffectorLink("cluedo_link");
group.setPoseReferenceFrame("arm_base_link");
group.setPlannerId("RRTstar");
group.setNumPlanningAttempts(10);
group.setPlanningTime(10.0);
group.allowReplanning(true);
group.setGoalPositionTolerance(0.01);
	target_pose1.position.x = 0.0;
	target_pose1.position.y = 0.5;
	target_pose1.position.z = 1.25;
	group.setStartStateToCurrentState();
	group.setApproximateJointValueTarget(target_pose1, "cluedo_link");
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	group.plan(my_plan);
	group.execute(my_plan);
	return 0;
}
int movelow()
{
	geometry_msgs::Pose target_pose1;
moveit::planning_interface::MoveGroupInterface group("arm");
group.setEndEffectorLink("cluedo_link");
group.setPoseReferenceFrame("arm_base_link");
group.setPlannerId("RRTstar");
group.setNumPlanningAttempts(10);
group.setPlanningTime(10.0);
group.allowReplanning(true);
group.setGoalPositionTolerance(0.01);
	target_pose1.position.x = 0.0;
	target_pose1.position.y = 0.5;
	target_pose1.position.z = 0.75;
	group.setStartStateToCurrentState();
	group.setApproximateJointValueTarget(target_pose1, "cluedo_link");
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	group.plan(my_plan);
	group.execute(my_plan);
	return 0;
}
namespace KCL_rosplan {
HintInterface::HintInterface(ros::NodeHandle &nh) {
// here the initialization
}
bool HintInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
// here the implementation of the action 
std::cout << "Moving to get the hint " << std::endl;


if(msg->parameters[0].value == "wp1")
	{
		while(received==0)
			{
				if (previous==1)
					{
						movelow();
						previous=0;
					}
				else if (previous ==0)
					{
						movehigh();
						previous=1;
					}
			}
		// elabora indizio
		received=0;
			
	}
else if (msg->parameters[0].value == "wp2")
	{
		while (received==0)
			{
				if (previous==1)
					{
						movelow();
						previous=0;
					}
				else if (previous ==0)
					{
						movehigh();
						previous=1;
					}
			}
		// elabora indizio
		received=0;
			
	}
else if (msg->parameters[0].value == "wp3")
	{
		while (received==0)
			{
				if (previous==1)
					{
						movelow();
						previous=0;
					}
				else if (previous ==0)
					{
						movehigh();
						previous=1;
					}
			}
		// elabora indizio
		received=0;
	}
else if (msg->parameters[0].value == "wp4")
	{
		while (received==0)
			{
				if (previous==1)
					{
						movelow();
						previous=0;
					}
				else if (previous ==0)
					{
						movehigh();
						previous=1;
					}
			}
		// elabora indizio
		received=0;
	}


ROS_INFO("Action performed: completed!");
	return true;
	
/*int random=rand()%10;

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
ros::NodeHandle n;
ros:: Subscriber reached= n.subscribe("/oracle_hint", 1000, hintCallback);
KCL_rosplan::HintInterface my_aci(nh);
my_aci.runActionInterface();
ros::AsyncSpinner spinner(1);
spinner.start();
sleep(2.0);
return 0;
}
