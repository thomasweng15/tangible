#include "ros/ros.h"

#include "tangible_msgs/GetScene.h"

bool scene_callback(tangible_msgs::GetScene::Request& req, tangible_msgs::GetScene::Response& res)
{
	tangible_msgs::Scene scene;

	tangible_msgs::BoundingBox bb;
	bb.pose.pose.position.x = 1;
	bb.pose.pose.position.y = 1;
	bb.pose.pose.position.z = 0;
	bb.dimensions.x = 2;
	bb.dimensions.y = 2;
	bb.dimensions.z = 3;


	ROS_INFO("scene service callback finished");
	return true;
}


int main (int argc, char** argv)
{
	ros::init(arg, argv, "get_scene_server");

	ros::NodeHandle n;
	ros::ServiceServer server = n.advertise("get_scene", scene_callback);
	ROS_INFO("fake service to provide scene information");
	ros::spin();

	return 0;
}