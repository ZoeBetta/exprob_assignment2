/** @ package exprob_assignment2
* 
*  \file ToHomeAction.cpp
*  \brief 
*
*  \author Zoe Betta
*  \version 1.0
*  \date 22/03/2022
*  \details
*   
*  Subscribes to: <BR>
*	None 
*
*  Publishes to: <BR>
*	 None
*
*  Services: <BR>
*    None
* 
*   Client Services: <BR>
*   None
*    
*
*  Action Services: <BR>
*    go_to_point
*
*  Description: <BR>
*  This program implements the real action to be completed when the planner
* dispatches the action (go_to_waypoint). It moves the robot to the desired waypoint.
*/
#include "exprob_assignment2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>

namespace KCL_rosplan 
{
	ToHomeInterface::ToHomeInterface(ros::NodeHandle &nh) 
	{
		// here the initialization
	}
/**
 * \brief: ToHomeInterface callback
 * \param msg : the variables received from the plan dispatcher
 * 
 * \return 0
 * 
 * This function implements the behaviour for the robot when the planner dispatches
 * the action go_home. it sends the request to move the robot to the home position
 */		
	bool ToHomeInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) 
	{
		// here the implementation of the action 
		// prints to the screen the path the robot should do
		std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		// initialization of the action server client
		actionlib::SimpleActionClient<motion_plan::PlanningAction> ac("/go_to_point", true);
		// initialize the variable to send to the action server
		motion_plan::PlanningGoal goal;
		ac.waitForServer();
		// set the goal coordinates
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 0.0;
		// send the goal to the action server
		ac.sendGoal(goal);
		// wait for the result from the action server
		ac.waitForResult();
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
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
	ros::init(argc, argv, "tohomeaction", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::ToHomeInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}
