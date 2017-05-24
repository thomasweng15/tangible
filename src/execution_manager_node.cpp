#include "ros/ros.h"
#include "tangible/execution_manager.h"

int main(int argc, char** argv) {
	const static int MAX_BLOCKING_THREAD_NUM = 5;
	ros::init(argc, argv, "execution_manager_node");

	ros::NodeHandle exec_node;
	ros::AsyncSpinner spinner(MAX_BLOCKING_THREAD_NUM);
	spinner.start();

	tangible::ExecutionManager exec_mng(exec_node);

	ros::waitForShutdown();
	spinner.stop();

	return 0;
}