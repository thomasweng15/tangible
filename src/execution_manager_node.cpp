#include "ros/ros.h"
#include "tangible/execution_manager.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "execution_manager_node");

	ros::NodeHandle exec_node;
	tangible::ExecutionManager exec_mng(exec_node);

	ros::NodeHandle private_params("~");
	std::string system_mode_topic;
	private_params.getParam("system_mode_topic", system_mode_topic);
	// NOTE: system_mode_topic is tpbd_mode

	ros::Subscriber mode_sub = exec_node.subscribe(system_mode_topic, 1000, &tangible::ExecutionManager::mode_callback, &exec_mng);
	ros::spin();

	// TO-DO:
	// subscriber inside exec_mng with spinner.start() and spinner.stop() [see system_main for example]
	// priate_params inside exec_mng

	return 0;
}