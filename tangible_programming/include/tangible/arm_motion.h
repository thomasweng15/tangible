#ifndef TANGIBLE_ARM_MOTION
#define TANGIBLE_ARM_MOTION

#include "ros/ros.h"

// user-defined classes
#include "tangible/gripper.h"

// msg's and srv's
#include "tangible_msgs/GetMovements.h"
#include "tangible_msgs/ControlMovements.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "moveit/move_group_interface/move_group.h"

namespace tangible
{

class ArmMotion
{
private:
	const static bool SUCCESSUL = true;
	const static bool UNSUCCESSFUL = false;
	const static bool ENABLED = true;
	const static bool DISABLED = false;

	ros::NodeHandle node_handle;
	ros::ServiceServer srv_move;
	ros::ServiceServer srv_control;

	moveit::planning_interface::MoveGroup right_arm;
	tangible::Gripper right_gripper;
	// TO-DO later on also support left arm & gripper;
	
	bool status, motion;

	geometry_msgs::Point get_relative_point(geometry_msgs::Point org, geometry_msgs::Vector3Stamped vec, double mag);

public:
	ArmMotion(ros::NodeHandle& n);
	~ArmMotion();

	bool move_callback(tangible_msgs::GetMovements::Request& req, tangible_msgs::GetMovements::Response& res);
	bool control_callback(tangible_msgs::ControlMovements::Request& req, tangible_msgs::ControlMovements::Response& res);

};

};

#endif