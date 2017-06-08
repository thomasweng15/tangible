#include "tangible/operation.h"

namespace tangible 
{

Operation::Operation(ros::NodeHandle& n, std::vector<tangible_msgs::Instruction> ins)
{
	node_handle = n;

	instructions = ins; // NOTE that operator= does a deep copy

	for(int i = 0; i < ins.size(); i++)
		done.push_back(false);

	all_done = false;
}

Operation::~Operation() {}

std::string Operation::get_private_param(std::string param_name)
{
	ros::NodeHandle private_params("~");
	std::string param;
	private_params.getParam(param_name, param);
	return param;
}

tangible_msgs::Scene Operation::get_scene()
{
	
	std::string scene_info_service = get_private_param("scene_information_service");
	ros::ServiceClient scene_info_client = node_handle.serviceClient<tangible_msgs::GetScene>(scene_info_service);

	tangible_msgs::GetScene scene_srv;
	
	bool success = scene_info_client.call(scene_srv);

	tangible_msgs::Scene scene_info;
	if(success)
		scene_info = scene_srv.response.scene;
	else
		ROS_ERROR("failed to call service %s to obtain scene information (table top and objects)", scene_info_service.c_str());

	return scene_info;
}

void Operation::print_scene(tangible_msgs::Scene scene)
{
	ROS_INFO("surface bounding box at (%f, %f, %f) of size (%f, %f, %f)", scene.surface.bounding_box.pose.pose.position.x, 
		                                                                  scene.surface.bounding_box.pose.pose.position.y, 
		                                                                  scene.surface.bounding_box.pose.pose.position.z, 
		                                                                  scene.surface.bounding_box.dimensions.x,
		                                                                  scene.surface.bounding_box.dimensions.y, 
		                                                                  scene.surface.bounding_box.dimensions.z);

	ROS_INFO(" and the following objects: ");
	for(int i = 0; i < scene.objects.size(); i++)
		ROS_INFO("object %d at (%f, %f, %f) of size (%f, %f, %f)", i, scene.objects[i].bounding_box.pose.pose.position.x, 
																	  scene.objects[i].bounding_box.pose.pose.position.y, 
																	  scene.objects[i].bounding_box.pose.pose.position.z, 
																	  scene.objects[i].bounding_box.dimensions.x,
																	  scene.objects[i].bounding_box.dimensions.y, 
																	  scene.objects[i].bounding_box.dimensions.z);
}

bool Operation::is_done()
{
	return all_done;
}

void Operation::reset() 
{
	for(int i = 0; i < done.size(); i++)
		done[i] = false;

	all_done = false;
	
	ROS_INFO("operation reset.");
}

}