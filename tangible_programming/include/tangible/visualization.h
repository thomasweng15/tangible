#ifndef TANGIBLE_VISUALIZATION
#define TANGIBLE_VISUALIZATION

#include "ros/ros.h"

// user-defined classes
#include "rapid_viz/markers.h"

// msg's and srv's
#include "visualization_msgs/Marker.h"
#include "tangible_msgs/VisualizeObjects.h"
#include "tangible_msgs/SceneObject.h"

namespace tangible
{

class Visualization
{
private:
	const static int ADD = 0;
	const static int MODIFY = 1;
	const static int DELETE = 2;
	
	ros::NodeHandle node;
	ros::ServiceServer srv_viz;
	ros::Publisher viz_pub;

	std::string name_space;
	std_msgs::ColorRGBA viz_color;

	std::vector<visualization_msgs::Marker> object_markers;

	std::string get_private_param(std::string param_name);

	void clear_object_markers();
	void draw_object_markers(std::vector<tangible_msgs::SceneObject> objs);

public:
	Visualization(ros::NodeHandle& n);
	~Visualization();

	bool visualization_callback(tangible_msgs::VisualizeObjects::Request& req, tangible_msgs::VisualizeObjects::Response& res);

};

};

#endif