#ifndef TANGIBLE_TAG_EXTRACTOR
#define TANGIBLE_TAG_EXTRACTOR

#include <vector>

#include "ros/ros.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

namespace tangible {

struct Position {
	double x;
	double y;
	double z;
};

struct Quaternion {
	double x;
	double y;
	double z;
	double w;
};

struct Axis {
	double x;
	double y;
	double z;
};

struct Tag {
	Position center;
	Quaternion orientation;
	Axis x_axis;
	Axis y_axis;
	Axis z_axis;
	int id;
};

class TagExtractor {
private:
	ros::NodeHandle node;
	ros::Subscriber ar_sub;

	std::vector<Tag> tags;

	void ARcallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr msg);
	
	void fillCenter(Tag& t, geometry_msgs::PoseStamped& p);
	void fillOrientation(Tag& t, geometry_msgs::PoseStamped& p);
	void fillAxes(Tag& t, geometry_msgs::PoseStamped& p);
public:
	TagExtractor(ros::NodeHandle& n);
	~TagExtractor();

	std::vector<Tag> get_tags();
};

}

#endif

//TO-DO publisher to publish tag information. Would be helpful for debugging