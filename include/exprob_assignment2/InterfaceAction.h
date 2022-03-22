#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"
namespace KCL_rosplan {
class MoveInterface: public RPActionInterface
{
private:
public:
/* constructor */
MoveInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

class FromHomeInterface: public RPActionInterface
{
private:
public:
/* constructor */
FromHomeInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

class ToHomeInterface: public RPActionInterface
{
private:
public:
/* constructor */
ToHomeInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};


class HintInterface: public RPActionInterface
{
private:
public:
/* constructor */
HintInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

class CompleteInterface: public RPActionInterface
{
private:
public:
/* constructor */
CompleteInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

class CorrectInterface: public RPActionInterface
{
private:
public:
/* constructor */
CorrectInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

}
