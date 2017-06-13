#include <vector>

#include "ros/ros.h"

#include "tangible_msgs/GetScene.h"
#include "tangible_msgs/Surface.h"
#include "tangible_msgs/SceneObject.h"
#include "tangible_msgs/BoundingBox.h"

bool scene_callback(tangible_msgs::GetScene::Request& req, tangible_msgs::GetScene::Response& res)
{
	tangible_msgs::Scene scene;
	tangible_msgs::Surface srf;
	tangible_msgs::SceneObject obj;
	tangible_msgs::BoundingBox bb;
	
	// surface
	bb.pose.pose.position.x = 0;
	bb.pose.pose.position.y = 0;
	bb.pose.pose.position.z = 0;
	bb.dimensions.x = 10;
	bb.dimensions.y = 10;
	bb.dimensions.z = 0.5;
	scene.surface.bounding_box = bb;

	// object 1
	bb.pose.pose.position.x = 1;
	bb.pose.pose.position.y = 1;
	bb.pose.pose.position.z = 0;
	bb.dimensions.x = 2;
	bb.dimensions.y = 2;
	bb.dimensions.z = 3;
	obj.bounding_box = bb;
	scene.objects.push_back(obj);

	// object 2
	bb.pose.pose.position.x = 1;
	bb.pose.pose.position.y = 4;
	bb.pose.pose.position.z = 0;
	bb.dimensions.x = 2;
	bb.dimensions.y = 2;
	bb.dimensions.z = 2;
	obj.bounding_box = bb;
	scene.objects.push_back(obj);

	// object 3
	bb.pose.pose.position.x = 4;
	bb.pose.pose.position.y = 1;
	bb.pose.pose.position.z = 0;
	bb.dimensions.x = 1;
	bb.dimensions.y = 1;
	bb.dimensions.z = 2;
	obj.bounding_box = bb;
	scene.objects.push_back(obj);

	res.scene = scene;

	ROS_INFO("scene service callback finished");
	return true;
}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "get_scene_server");

	ros::NodeHandle n;
	ros::ServiceServer server = n.advertiseService("get_scene", scene_callback);
	ROS_INFO("fake service to provide scene information");
	
	ros::spin();

	return 0;
}