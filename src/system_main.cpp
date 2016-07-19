#include "ros/ros.h"
#include "tangible/frame_transformer.h"

int main (int argc, char** argv) {
	ros::init(argc, argv, "tangible_pbd");
	ros::NodeHandle node;
	tangible::FrameTransformer trns(node, "base_footprint");
	ros::spin();
	return 0;
}