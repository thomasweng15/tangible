#include "ros/ros.h"

#include "tangible_msgs/GetReleases.h"
#include "moveit_msgs/Grasp.h"

bool grasp_callback(tangible_msgs::GetReleases::Request& req, tangible_msgs::GetReleases::Response& res)
{
	moveit_msgs::Grasp g;
	res.releases.push_back(g);
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "release_generation_service");

	ros::NodeHandle n;
	ros::ServiceServer srv = n.advertiseService("get_release", grasp_callback);
	ROS_INFO("fake service to generate releases");
	ros::spin();

	return 0;
}