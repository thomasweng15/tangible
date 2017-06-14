#ifndef TANGIBLE_ARM_MOTION
#define TANGIBLE_ARM_MOTION

#include "ros/ros.h"

// msg's and srv's
#include "tangible_msgs/GetMovements.h"
#include "tangible_msgs/StopMovements.h"
#include "moveit/move_group_interface/move_group.h"

namespace tangible
{

class ArmMotion
{
private:
	const static bool SUCCESSUL = true;
	const static bool UNSUCCESSFUL = false;

	ros::NodeHandle node_handle;
	ros::ServiceServer srv_move;
	ros::ServiceServer srv_stop;

	moveit::planning_interface::MoveGroup right_arm;
	// TO-DO later on also instantiate the group for left arm;
	
	bool status, stopped;

public:
	ArmMotion(ros::NodeHandle& n);
	~ArmMotion();

	bool move_callback(tangible_msgs::GetMovements::Request& req, tangible_msgs::GetMovements::Response& res);
	bool stop_callback(tangible_msgs::StopMovements::Request& req, tangible_msgs::StopMovements::Response& res);

};

};

#endif