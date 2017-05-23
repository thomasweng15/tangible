#include "ros/ros.h"
#include "tangible/execution_manager.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "execution_manager_node");
	ros::NodeHandle n;

	tangible::ExecutionManager exec_mng(n);
	return 0;
}