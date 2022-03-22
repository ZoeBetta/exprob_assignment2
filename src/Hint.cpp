/** @ package exprob_assignment2
* 
*  \file Hint.cpp
*  \brief 
*
*  \author Zoe Betta
*  \version 1.0
*  \date 22/03/2022
*  \details
*   
*  Subscribes to: <BR>
*	/oracle_hint
*
*  Publishes to: <BR>
*	 None
*
*  Services: <BR>
*    None
* 
*   Client Services: <BR>
*   /hint
*    
*  Action Services: <BR>
*    go_to_point
*
*  Description: <BR>
*  This program implements the real action to be completed when the planner
* dispatches the action (take_hint). When the robot is in position it moves 
* the arm to retrieve one hint and then sends the hint to the server that elaborates the received hint. 
*/

#include "exprob_assignment2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <cstdlib>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <exprob_assignment2/ErlOracle.h>
#include <exprob_assignment2/HintElaboration.h>
#include <string.h>

// global variables
int received=0;
int previous=1;
exprob_assignment2::ErlOracle temp;

// function declaration 
int movehigh();
int movelow();
void hintCallback( const exprob_assignment2::ErlOracle x);
int calculatePos();

namespace KCL_rosplan 
{
HintInterface::HintInterface(ros::NodeHandle &nh) 
	{
		// here the initialization
	}
	
/**
 * \brief: HintInterface callback
 * \param msg : the variables received from the plan dispatcher
 * 
 * \return true
 * 
 * This function implements the behaviour for the robot when the planner dispatches
 * the action take_hint. It moves the arm to retrieve the hint and calls the server for its elaboration.
 */	
bool HintInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) 
	{
		// here the implementation of the action 
		std::cout << "Moving to get the hint " << std::endl;
		// calculate the position of the arm to retrieve the hint
		calculatePos();
		ROS_INFO("Action performed: completed!");
		return true;
	}
}

/**
 * \brief: main function
 * \param : None
 * 
 * \return 0
 * 
 * This is the main function, it initializes the node and the interface to 
 * associate the action of the ros plan to the actual implementation.
 */
int main(int argc, char **argv) 
{
	ros::init(argc, argv, "hintaction", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	// initialization of the node handle for the subscriber on the topic /oracle_hint
	ros::NodeHandle n;
	ros:: Subscriber reached= n.subscribe("/oracle_hint", 1000, hintCallback);
	KCL_rosplan::HintInterface my_aci(nh);
	my_aci.runActionInterface();
	ros::AsyncSpinner spinner(1);
	spinner.start();
	sleep(2.0);
	return 0;
}

/**
 * \brief: It controls the motion of the arm
 * \param : None
 * 
 * \return 0
 * 
 * This function calculates the desired position of the arm. The arm changes
 *  position from the one before if the hint has not been received yet.
 * It changes position twice before not receiving the hint. 
 */
int calculatePos()
{
	// initialization of the node handle for the service client on the topic /hint
	ros::NodeHandle n1;
	ros::ServiceClient client = n1.serviceClient<exprob_assignment2::HintElaboration>("/hint");
	// initialization of the variables for controlling the arm
	int once=0;
	int i=0;
	// while the hint has not been received ( received is a global variable) 
	// and only for two repetitions max
	while(received==0 & once==0)
		{
			// if the current position is high
			if (previous==1)
				{
					std::cout << "Moving" << std::endl;
					// move the arm low
					movelow();
					// update with the current position
					previous=0;
					// wait one second
					sleep(1);
					// increase the counter
					i=i+1;
					if (i>1)
						{once=1;}
				}
			// if the current position is low
			else if (previous ==0)
				{
					std::cout << "Moving" << std::endl;
					// move the arm high
					movehigh();
					// update with the current position
					previous=1;
					//wait one second
					sleep(1);
					//increase the counter
					i=i+1;
					if (i>1)
						{once=1;}
				}
		}
	// if the hint has been received
	if (received==1)
		{
			// initializa a requent for the server
			exprob_assignment2::HintElaboration msg;
			// save the correct values in the corresponding fields
			msg.request.ID=temp.ID;
			msg.request.key=temp.key;
			msg.request.value=temp.value;
			// call the client server
			client.call(msg);
		}
	// reset the global variable received	
	received=0;	
	return 0;
}

/**
 * \brief: callback for the topic /oracle_hint
 * \param x : the message received on the /oracle_hint topic
 * 
 * \return None
 * 
 * This is the main function, it initializes the node and the interface to 
 * associate the action of the ros plan to the actual implementation.
 */
void hintCallback( const exprob_assignment2::ErlOracle x)
{
	// set the global variable received to 1
	received=1;
	// set the values of the global variable
	temp.ID=x.ID;
	temp.key=x.key;
	temp.value=x.value;

}

/**
 * \brief: moves the arm on the position high
 * \param : None
 * 
 * \return 0
 * 
 * This function calls moveit and sets the target position for the high position of the arm.
 * To retrieve the hint on the highest set position.
 */
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

/**
 * \brief: moves the arm on the position low
 * \param : None
 * 
 * \return 0
 * 
 * This function calls moveit and sets the target position for the low position of the arm.
 * To retrieve the hint on the lowest set position.
 */
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
