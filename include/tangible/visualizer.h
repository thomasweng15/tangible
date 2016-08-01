#ifndef TANGIBLE_VISUALIZER
#define TANGIBLE_VISUALIZER

#include <string>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "tangible/program.h"
#include "rapid_perception/object.h"

namespace tangible {

class Visualizer {
private:
	const static int ADD = 0;
	const static int MODIFY = 1;
	const static int DELETE = 2;

	const static int TAG_NUM = 18;
	int latest_instruction_num;
	int latest_object_num;

	ros::NodeHandle node;

	std::string frame_id;

	ros::Publisher ar_label_pub;
	ros::Publisher scene_pub;
	ros::Publisher program_pub;

	void setHeader(visualization_msgs::Marker& marker);
	void setNamespace(visualization_msgs::Marker& marker, std::string name);
	void setID(visualization_msgs::Marker& marker, int ID);
	void setAction(visualization_msgs::Marker& marker, int action);
	void setPose(visualization_msgs::Marker& marker, geometry_msgs::PoseStamped ps);
	void setScale(visualization_msgs::Marker& marker, geometry_msgs::Vector3 scale);
	void setColor(visualization_msgs::Marker& marker, double r, double g, double b, double a);
	void setPoints(visualization_msgs::Marker& marker, Instruction& ins);
public:
	Visualizer(ros::NodeHandle& n, std::string id);
	~Visualizer();

	void update(Program p);
	void update(std::vector<rapid::perception::Object> objects);
	//TO-DO overload update function for AR tags and scene
	
	void clear();
};

}

#endif