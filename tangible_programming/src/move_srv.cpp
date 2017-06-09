#include "ros/ros.h"

#include "tangible_msgs/GetMovements.h"

bool move_callback(tangible_msgs::GetMovements::Request& req, tangible_msgs::GetMovements::Response& res)
{
	res.success = true;
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "arm_movement_server");

	ros::NodeHandle n;
	ros::ServiceServer srv = n.advertiseService("move_arm", move_callback);
	ros::spin();
	ROS_INFO("fake service to move the arm");

	return 0;
}