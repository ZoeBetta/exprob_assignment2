#include "exprob_assignment2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <exprob_assignment2/Hypothesis.h>

namespace KCL_rosplan {
CorrectInterface::CorrectInterface(ros::NodeHandle &nh) {
// here the initialization
}
bool CorrectInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
// here the implementation of the action 

	ros::NodeHandle n1;
    ros::ServiceClient client = n1.serviceClient<exprob_assignment2::Hypothesis>("/correcthypothesis");
    exprob_assignment2::Hypothesis data;
    data.request.start=true;
    client.call(data);
    if (data.response.ret==true)
		{ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
return true;}
    else {ROS_INFO("Action (%s) performed: wrong!", msg->name.c_str());
return false;}
}
}
int main(int argc, char **argv) {
ros::init(argc, argv, "correctaction", ros::init_options::AnonymousName);
ros::NodeHandle nh("~");
KCL_rosplan::CorrectInterface my_aci(nh);
my_aci.runActionInterface();
return 0;
}
