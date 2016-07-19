#include "ros/ros.h"
#include "tangible/frame_transformer.h"
#include "tangible/scene_parser.h"

int main (int argc, char** argv) {
	ros::init(argc, argv, "tangible_pbd");
	ros::NodeHandle node;
	tangible::FrameTransformer trns(node, "base_footprint");
	tangible::SceneParser parser(node);
	ros::spin();
	return 0;
}