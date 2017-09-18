#include "ros/ros.h"

#include "tangible/visualization.h"

int main(int argc, char** argv)
{
	const static int MAX_BLOCKING_THREAD_NUM = 2;
	ros::init(argc, argv, "visualization_node");

	ros::NodeHandle viz_node;
	ros::AsyncSpinner spinner(MAX_BLOCKING_THREAD_NUM);
	spinner.start();

	tangible::Visualization viz(viz_node);

	ros::waitForShutdown();
	spinner.stop();

	return 0;
}