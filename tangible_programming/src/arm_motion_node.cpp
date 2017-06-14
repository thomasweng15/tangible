#include "ros/ros.h"
#include "tangible/arm_motion.h"

int main(int argc, char** argv)
{
	const static int MAX_BLOCKING_THREAD_NUM = 2;
	ros::init(argc, argv, "arm_motion_node");

	ros::NodeHandle srv_node;
	ros::AsyncSpinner spinner(MAX_BLOCKING_THREAD_NUM);
	spinner.start();

	tangible::ArmMotion pr2_arms(srv_node);

	ros::waitForShutdown();
	spinner.stop();

	return 0;
}