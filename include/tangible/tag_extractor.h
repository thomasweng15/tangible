#ifndef TANGIBLE_TAG_EXTRACTOR
#define TANGIBLE_TAG_EXTRACTOR

#include <vector>

#include "ros/ros.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#include "tangible/GetBlocks.h"
#include "tangible/Block.h"
#include "Eigen/Geometry"
#include "geometry_msgs/PoseStamped.h"

namespace tangible {

class TagExtractor {
private:
	ros::NodeHandle node;

	std::vector<tangible::Block> blocks;
	void initBlock(geometry_msgs::PoseStamped& p, int _id, tangible::Block* block);
	void setAxes(geometry_msgs::PoseStamped& p, tangible::Block* block);


public:
	TagExtractor(ros::NodeHandle& n);
	~TagExtractor();
	void tagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr msg);
	bool parseCallback(tangible::GetBlocks::Request& req,
                 tangible::GetBlocks::Response& res);
};

}

#endif

//TO-DO publisher to publish tag information. Would be helpful for debugging