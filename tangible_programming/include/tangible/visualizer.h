#ifndef TANGIBLE_VISUALIZER
#define TANGIBLE_VISUALIZER

#include <string>
#include <vector>
#include <map>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "tangible/tag.h"
#include "tangible/program.h"
#include "rapid_perception/object.h"

namespace tangible {

class Visualizer {
private:
	const static int LABEL_POSITION_OFFSET = 0.02;
	const static int ADD = 0;
	const static int MODIFY = 1;
	const static int DELETE = 2;

	const static int TAG_NUM = 18;

	std::map<int, std::string> LABELS_TXT;

	ros::NodeHandle node;

	std::string frame_id;

	ros::Publisher ar_label_pub;
	ros::Publisher scene_pub;
	ros::Publisher program_pub;

	std::vector<visualization_msgs::Marker> scene_elements;
	std::vector<visualization_msgs::Marker> program_elements;

	void fillGroupingMarker(visualization_msgs::Marker& marker, Instruction& ins);
	void fillLabelMarker(visualization_msgs::Marker& marker, Tag& tag);

	void setHeader(visualization_msgs::Marker& marker);
	void setNamespace(visualization_msgs::Marker& marker, std::string name);
	void setID(visualization_msgs::Marker& marker, int ID);
	void setAction(visualization_msgs::Marker& marker, int action);
	void setPose(visualization_msgs::Marker& marker, geometry_msgs::PoseStamped ps);
	void setPose(visualization_msgs::Marker& marker, Tag& tag);
	void setScale(visualization_msgs::Marker& marker, geometry_msgs::Vector3 scale);
	void setColor(visualization_msgs::Marker& marker, double r, double g, double b, double a);
	void setPoints(visualization_msgs::Marker& marker, Instruction& ins);
public:
	Visualizer(ros::NodeHandle& n, std::string id);
	~Visualizer();

	void update(Program p);
	void update(std::vector<rapid::perception::Object> objects);
	//TO-DO overload update function for AR tags and scene
	
	void clearProgram();
	void clearScene();
	void clear();
};

}

#endif