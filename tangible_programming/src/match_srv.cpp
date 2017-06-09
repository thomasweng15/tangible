#include <vector>

#include "ros/ros.h"

#include "tangible_msgs/GetMatchingObjects.h"
#include "tangible_msgs/SceneObject.h"
#include "tangible_msgs/Target.h"

bool match_callback(tangible_msgs::GetMatchingObjects::Request& req, tangible_msgs::GetMatchingObjects::Response& res)
{
	std::vector<tangible_msgs::SceneObject> objs_in;
	tangible_msgs::Target target;
	std::vector<tangible_msgs::SceneObject> objs_out;

	objs_in = req.objects;
	target = req.target;

	for(int i = 0; i < objs_in.size(); i++)
	{
		switch(target.type)
		{
			case tangible_msgs::Target::POINT_LOCATION:
				// ROS_INFO("matching object at (%f, %f, %f) to location (%f, %f, %f)", objs_in[i].bounding_box.pose.pose.position.x,
				// 																 	 objs_in[i].bounding_box.pose.pose.position.y,
				// 																 	 objs_in[i].bounding_box.pose.pose.position.z,
				// 																 	 target.specified_point.point.x,
				// 																 	 target.specified_point.point.y,
				// 																 	 target.specified_point.point.z);
				if(target.specified_point.point.x == objs_in[i].bounding_box.pose.pose.position.x &&
				   target.specified_point.point.y == objs_in[i].bounding_box.pose.pose.position.y &&
				   target.specified_point.point.z == objs_in[i].bounding_box.pose.pose.position.z)
					objs_out.push_back(objs_in[i]);
				break;
			case tangible_msgs::Target::REGION:
				// TO-DO
				break;
			case tangible_msgs::Target::OBJECT_SELECTOR: // TO-DO rename to OBJECT_LOCATION
				// ROS_INFO("matching object bb(%f, %f, %f) to object bb(%f, %f, %f)", objs_in[i].bounding_box.dimensions.x,
				// 																objs_in[i].bounding_box.dimensions.y,
				// 																objs_in[i].bounding_box.dimensions.z,
				// 																target.selected_object.bounding_box.dimensions.x,
				// 																target.selected_object.bounding_box.dimensions.y,
				// 																target.selected_object.bounding_box.dimensions.z);
				if(target.selected_object.bounding_box.dimensions.x == objs_in[i].bounding_box.dimensions.x &&
				   target.selected_object.bounding_box.dimensions.y == objs_in[i].bounding_box.dimensions.y &&
				   target.selected_object.bounding_box.dimensions.z == objs_in[i].bounding_box.dimensions.z)
					objs_out.push_back(objs_in[i]);
				break;
			// case tangible_msgs::Target::OBJECTS_LOCATION: // TO-DO include after adding it to Target msg
			// 	break;
			default:
				ROS_ERROR("Invalid Target");
				break;
		}
	}

	res.objects = objs_out;
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "instruction_check_server");

	ros::NodeHandle n;
	ros::ServiceServer server = n.advertiseService("check_instruction", match_callback);
	ROS_INFO("fake service to check instruction criteria information");

	ros::spin();

	return 0;
}