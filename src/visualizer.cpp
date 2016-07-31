#include "tangible/visualizer.h"

namespace tangible {

Visualizer::Visualizer(ros::NodeHandler& n) {
	node = n;

	ar_label_pub = n.advertise<visualization_msgs::Marker>("ar_label_viz", 10);
	scene_pub = n.advertise<visualization_msgs::Marker>("scene_viz", 10);
	program_pub = n.advertise<visualization_msgs::Marker>("program_viz", 10);
}

void Visualizer::update(Program p) {
	//
	//program_pub.pub()
}

}