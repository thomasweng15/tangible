#include "tangible/tag_extractor.h"

namespace tangible {

TagExtractor::TagExtractor(ros::NodeHandle& n) {
	node = n;
	ar_sub = node.subscribe("/ar_pose_transformed", 10, &TagExtractor::ARcallback, this);
}

TagExtractor::~TagExtractor () {}

void TagExtractor::ARcallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr msg) {
	tags.clear();
	for(int i = 0; i < msg->markers.size(); i++) {
		geometry_msgs::PoseStamped p = msg->markers[i].pose;
		Tag t(p, msg->markers[i].id);
		ROS_INFO("\n*** ID = %1d: X(%2.3lf, %2.3lf, %2.3lf) Y(%2.3lf, %2.3lf, %2.3lf) Z(%2.3lf, %2.3lf, %2.3lf) ***",
			     t.getID(), t.getX().x, t.getX().y, t.getX().z,
			                t.getY().x, t.getY().y, t.getY().z,
			                t.getZ().x, t.getZ().y, t.getZ().z);
		tags.push_back(t);
	}
}

std::vector<Tag> TagExtractor::get_tags() { return tags; }

}