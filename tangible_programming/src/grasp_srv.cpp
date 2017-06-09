#include "ros/ros.h"

#include "tangible_msgs/GetGrasps.h"
#include "moveit_msgs/Grasp.h"

bool grasp_callback(tangible_msgs::GetGrasps::Request& req, tangible_msgs::GetGrasps::Response& res)
{
	moveit_msgs::Grasp g;
	res.grasps.push_back(g);
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "grasp_generation_service");

	ros::NodeHandle n;
	ros::ServiceServer srv = n.advertiseService("get_grasp", grasp_callback);
	ros::spin();
	ROS_INFO("fake service to generate grasps");

	return 0;
}