#include "ros/ros.h"
#include "tangible/frame_transform.h"

int main (int argc, char** argv) {
	ros::init(argc, argv, "tangible_pbd");
	ros::NodeHandle node;
	tangible::FrameTransform trns(node, "base_footprint");
	ros::spin();
	return 0;
}