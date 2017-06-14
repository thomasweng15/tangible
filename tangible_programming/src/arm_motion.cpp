#include "tangible/arm_motion.h"

namespace tangible
{

ArmMotion::ArmMotion(ros::NodeHandle& n) : right_arm("right_arm")
{
	node_handle = n;
	srv_move = node_handle.advertiseService("move_arm", &ArmMotion::move_callback, this);
	srv_stop = node_handle.advertiseService("stop_arm", &ArmMotion::stop_callback, this);

	status = SUCCESSUL;
	stopped = false;
}

ArmMotion::~ArmMotion() {}

bool ArmMotion::move_callback(tangible_msgs::GetMovements::Request& req, tangible_msgs::GetMovements::Response& res)
{
	ROS_INFO("planning motion");
	stopped = false;
	// TO-DO
	res.status = status;
	return true;
}

bool ArmMotion::stop_callback(tangible_msgs::StopMovements::Request& req, tangible_msgs::StopMovements::Response& res)
{
	ROS_INFO("stopping all movements");
	stopped = true;
	right_arm.stop();
	return true;
}

}