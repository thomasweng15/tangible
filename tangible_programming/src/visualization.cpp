#include "tangible/visualization.h"

namespace tangible
{

Visualization::Visualization(ros::NodeHandle& n)
{
	node = n;
	
	std::string viz_srv_name, viz_topic_name;
	viz_srv_name = get_private_param("object_visualization_service");
	viz_topic_name = get_private_param("object_visualization_topic");
	srv_viz = node.advertiseService(viz_srv_name, &Visualization::visualization_callback, this);
	viz_pub = node.advertise<visualization_msgs::Marker>(viz_topic_name, 1);

	// TO-DO name_space and viz_color from parameter server
	name_space = "object_viz";
	viz_color.r = 0;
	viz_color.g = 0.75;
	viz_color.b = 0;
	viz_color.a = 1;
}

Visualization::~Visualization() {}

std::string Visualization::get_private_param(std::string param_name)
{
	ros::NodeHandle private_params("~");
	std::string param;
	private_params.getParam(param_name, param);
	return param;
}

bool Visualization::visualization_callback(tangible_msgs::VisualizeObjects::Request& req, tangible_msgs::VisualizeObjects::Response& res)
{
	clear_object_markers();
	draw_object_markers(req.objects);
	return true;
}


void Visualization::clear_object_markers()
{
	for(int i = 0; i < object_markers.size(); i++) {
		object_markers[i].action = visualization_msgs::Marker::DELETE;
		viz_pub.publish(object_markers[i]);
	}
	object_markers.clear();
}

void Visualization::draw_object_markers(std::vector<tangible_msgs::SceneObject> objs)
{
	for(int i = 0; i < objs.size(); i++) {
		visualization_msgs::Marker box =
	        rapid::viz::Marker::Box(NULL, objs[i].bounding_box.pose, objs[i].bounding_box.dimensions).marker();
	    box.ns = name_space;
	    box.id = i + 1;
		box.color = viz_color;
		box.action = visualization_msgs::Marker::ADD;
		object_markers.push_back(box);
		viz_pub.publish(box);
	}
}

}