#include "exprob_assignment2/MoveAction.h"
#include <unistd.h>
namespace KCL_rosplan {
MoveInterface::MoveInterface(ros::NodeHandle &nh) {
// here the initialization
}
bool MoveInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
// here the implementation of the action 
std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
sleep(5);
ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
return true;
}
}
int main(int argc, char **argv) {
ros::init(argc, argv, "my_rosplan_action", ros::init_options::AnonymousName);
ros::NodeHandle nh("~");
KCL_rosplan::MoveInterface my_aci(nh);
my_aci.runActionInterface();
return 0;
}
