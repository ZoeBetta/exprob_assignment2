#include "exprob_assignment2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <cstdlib>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <exprob_assignment2/ErlOracle.h>
//#include <exprob_assignment2/HintElaboration.h>
int received=0;
int previous=1;
exprob_assignment2::ErlOracle temp;
int movehigh();
int movelow();
void hintCallback( const exprob_assignment2::ErlOracle x);
int calculatePos();

namespace KCL_rosplan {
HintInterface::HintInterface(ros::NodeHandle &nh) {
// here the initialization
}
bool HintInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
// here the implementation of the action 
std::cout << "Moving to get the hint " << std::endl;

calculatePos();

ROS_INFO("Action performed: completed!");
	return true;

}
}
int main(int argc, char **argv) {
ros::init(argc, argv, "hintaction", ros::init_options::AnonymousName);
ros::NodeHandle nh("~");
ros::NodeHandle n;
ros:: Subscriber reached= n.subscribe("/oracle_hint", 1000, hintCallback);
//ros::ServiceClient client = n.serviceClient<exprob_assignment2::HintElaboration>("/hint");
KCL_rosplan::HintInterface my_aci(nh);
my_aci.runActionInterface();
ros::AsyncSpinner spinner(1);
spinner.start();
sleep(2.0);
return 0;
}

int calculatePos()
{
	int once=0;
	int i=0;
	while(received==0 & once==0)
		{
			if (previous==1)
				{
					movelow();
					previous=0;
					sleep(1);
					i=i+1;
					if (i>1)
						{once=1;}
				}
			else if (previous ==0)
				{
					movehigh();
					previous=1;
					sleep(1);
					i=i+1;
					if (i>1)
					{once=1;}
				}
		}
	if (received==1)
		int i=0;
	// elabora indizio
	received=0;	
	return 0;
}

void hintCallback( const exprob_assignment2::ErlOracle x)
{
	received=1;
	temp.ID=x.ID;
	temp.key=x.key;
	temp.value=x.value;

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
