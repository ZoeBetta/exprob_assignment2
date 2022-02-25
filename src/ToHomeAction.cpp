#include "exprob_assignment2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>

namespace KCL_rosplan {
ToHomeInterface::ToHomeInterface(ros::NodeHandle &nh) {
// here the initialization
}
bool ToHomeInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
// here the implementation of the action 
std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
actionlib::SimpleActionClient<motion_plan::PlanningAction> ac("/go_to_point", true);

motion_plan::PlanningGoal goal;
ac.waitForServer();
ROS_INFO("server on");
goal.target_pose.pose.position.x = 0.0;
goal.target_pose.pose.position.y = 0.0;
goal.target_pose.pose.orientation.w = 0.0;

ac.sendGoal(goal);
ac.waitForResult();

ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
return true;
}
}
int main(int argc, char **argv) {
ros::init(argc, argv, "tohomeaction", ros::init_options::AnonymousName);
ros::NodeHandle nh("~");
KCL_rosplan::ToHomeInterface my_aci(nh);
my_aci.runActionInterface();
return 0;
}
