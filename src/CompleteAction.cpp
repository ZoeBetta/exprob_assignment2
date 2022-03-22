/** @ package exprob_assignment2
* 
*  \file CompleteAction.cpp
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
*   /checkcomplete
*    
*
*  Action Services: <BR>
*    None
*
*  Description: <BR>
*  This program implements the real action to be completed when the planner
* dispatches the action (checkcomplete). It calls the server to check if there
* is at least one action that is complete and handles the response.
*/

#include "exprob_assignment2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <exprob_assignment2/Complete.h>

namespace KCL_rosplan 
{
	CompleteInterface::CompleteInterface(ros::NodeHandle &nh)
		{
			// here the initialization
		}

/**
 * \brief: CompleteInterface callback
 * \param msg : the variables received from the plan dispatcher
 * 
 * \return 0
 * 
 * This function implements the behaviour for the robot when the planner dispatches
 * the action checkcomplete
 */
	bool CompleteInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) 
	{
		// here the implementation of the action 
		// initialization of the node handle for the service Client on the topic /checkcomplete
		ros::NodeHandle n1;
		ros::ServiceClient client = n1.serviceClient<exprob_assignment2::Complete>("/checkcomplete");
		//initialize the variable to send as a request to the server
		exprob_assignment2::Complete data;
		// set the start to true
		data.request.start=true;
		// call the service Client
		client.call(data);
		// read the response to the server
		// if the response is true
		if (data.response.ret==true)
			{
				//print that the action was successful and return true to the planning server
				ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
				return true;
			}
		else 
			{
				// print that the action was not successfull and return false
				ROS_INFO("Action (%s) performed: wrong!", msg->name.c_str());
				return false;
			}
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
	ros::init(argc, argv, "checkaction", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::CompleteInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}
