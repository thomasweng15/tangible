#ifndef TANGIBLE_GRIPPER
#define TANGIBLE_GRIPPER

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"

// NOTE: this code mainly comes from http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20gripper

namespace tangible
{

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper
{
private:
	GripperClient* gripper_client;

public:
	Gripper(char left_or_right);
	~Gripper();

	bool open();
	bool close();
};

};

#endif