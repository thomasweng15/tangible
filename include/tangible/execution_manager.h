#ifndef TANGIBLE_EXECUTION_MANAGER
#define TANGIBLE_EXECUTION_MANAGER

#include <vector>

#include "ros/ros.h"
#include "tangible/Mode.h"
#include "tangible/GetProgram.h"
#include "tangible/Operation.h"

namespace tangible {

class ExecutionManager {
private:
	ros::NodeHandle node_handle;
	ros::Subscriber exec_mode;

	int mode;
	tangible::Program program;

	bool executing;

	bool get_program();
	bool get_scene();

	void start_execution();
	void stop_execution();

	std::string get_private_param(std::string param_name);
	
public:
	ExecutionManager(ros::NodeHandle& n);
	~ExecutionManager();

	void mode_callback(const tangible::Mode::ConstPtr& mode_msg);
};

};

#endif