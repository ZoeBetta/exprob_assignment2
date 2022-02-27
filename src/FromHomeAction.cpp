#include "exprob_assignment2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>

namespace KCL_rosplan {
FromHomeInterface::FromHomeInterface(ros::NodeHandle &nh) {
// here the initialization
}
bool FromHomeInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
// here the implementation of the action 
ROS_INFO("in the action");
std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
actionlib::SimpleActionClient<motion_plan::PlanningAction> ac("/go_to_point", true);

motion_plan::PlanningGoal goal;
ac.waitForServer();
if(msg->parameters[1].value == "wp1"){
goal.target_pose.pose.position.x = 2.5;
goal.target_pose.pose.position.y = 0.0;
goal.target_pose.pose.orientation.w = -3.14/2;
}
else if (msg->parameters[1].value == "wp2"){
goal.target_pose.pose.position.x = 0.0;
goal.target_pose.pose.position.y = 2.5;
goal.target_pose.pose.orientation.w = 0.0;
}
else if (msg->parameters[1].value == "wp3"){
goal.target_pose.pose.position.x = -2.5;
goal.target_pose.pose.position.y = 0.0;
goal.target_pose.pose.orientation.w = 3.14/2;
}
else if (msg->parameters[1].value == "wp4"){
goal.target_pose.pose.position.x = 0.0;
goal.target_pose.pose.position.y = -2.5;
goal.target_pose.pose.orientation.w = 3.14;
}
ac.sendGoal(goal);
ac.waitForResult();

ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
return true;
}
}
int main(int argc, char **argv) {
ros::init(argc, argv, "fromhomeaction", ros::init_options::AnonymousName);
ros::NodeHandle nh("~");
KCL_rosplan::FromHomeInterface my_aci(nh);
my_aci.runActionInterface();
return 0;
}
