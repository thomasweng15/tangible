#ifndef TANGIBLE_TAG_EXTRACTOR
#define TANGIBLE_TAG_EXTRACTOR

#include <vector>

#include "ros/ros.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#include "tangible/tag.h"

namespace tangible {

class TagExtractor {
private:
	ros::NodeHandle node;
	ros::Subscriber ar_sub;

	std::vector<Tag> tags;

	void ARcallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr msg);
public:
	TagExtractor(ros::NodeHandle& n);
	~TagExtractor();

	std::vector<Tag> get_tags();
};

}

#endif

//TO-DO publisher to publish tag information. Would be helpful for debugging