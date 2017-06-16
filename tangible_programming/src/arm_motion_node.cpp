#include "ros/ros.h"
#include "tangible/arm_motion.h"

std::string get_private_param(std::string param_name);

int main(int argc, char** argv)
{
	const static int MAX_BLOCKING_THREAD_NUM = 2;
	ros::init(argc, argv, "arm_motion_node");

	ros::NodeHandle srv_node;
	ros::AsyncSpinner spinner(MAX_BLOCKING_THREAD_NUM);
	spinner.start();

	std::string move_srv_name, control_srv_name;
	move_srv_name = get_private_param("arm_movement_service");
	control_srv_name = get_private_param("arm_movement_control_service");

	tangible::ArmMotion pr2_arms(srv_node, move_srv_name, control_srv_name);

	ros::waitForShutdown();
	spinner.stop();

	return 0;
}

std::string get_private_param(std::string param_name)
{
	ros::NodeHandle private_params("~");
	std::string param;
	private_params.getParam(param_name, param);
	return param;
}