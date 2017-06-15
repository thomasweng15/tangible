#include "tangible/gripper.h"

namespace tangible
{

Gripper::Gripper(char left_or_right)
{
	 switch(left_or_right)
	 {
	 	case 'r':
	 		gripper_client = new GripperClient("r_gripper_controller/gripper_action", true);
	 		break;
	 	case 'l':
	 		gripper_client = new GripperClient("l_gripper_controller/gripper_action", true);
	 		break;
	 	default:
	 		ROS_ERROR("invalid gripper %c specified", left_or_right);
	 		break;
	 }

	 // TO-DO use stringstream to concatenate left_or_right to "_gripper_controller/gripper_action",
	 // combine 'r' and 'l' cases and make sure you proceed to the following line only if a 
	 // GripperClient is instantiated.
	 
	 while(!gripper_client->waitForServer(ros::Duration(5.0)))
	 	ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
}

Gripper::~Gripper()
{
	delete gripper_client;
}

bool Gripper::open()
{
	pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.08;
    open.command.max_effort = -1.0;  // Do not limit effort (negative)

    ROS_INFO("Sending open goal");
    gripper_client->sendGoal(open);
    gripper_client->waitForResult();
    bool success = false;

    if(gripper_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
       gripper_client->getState() == actionlib::SimpleClientGoalState::ABORTED)
    	// TO-DO look iff these conditions reflect successfull object gripping
    {
    	success = true;
    	ROS_INFO("The gripper opened!");
    }
    else
  	    ROS_INFO("The gripper failed to open.");

  	return success;
}

bool Gripper::close()
{
	pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.0;
    squeeze.command.max_effort = 50.0;  // Close gently

    ROS_INFO("Sending squeeze goal");
    gripper_client->sendGoal(squeeze);
    gripper_client->waitForResult();
    bool success = false;


    if(gripper_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
       gripper_client->getState() == actionlib::SimpleClientGoalState::ABORTED)
    	// TO-DO look iff these conditions reflect successfull object releasing
    {
        success = true;
        ROS_INFO("The gripper closed!");
    }
    else
        ROS_INFO("The gripper failed to close.");

    return success;
}

}