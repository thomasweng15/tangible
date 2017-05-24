#ifndef TANGIBLE_EXECUTION_MANAGER
#define TANGIBLE_EXECUTION_MANAGER

#include <vector>

#include "ros/ros.h"
#include "tangible/Mode.h"

namespace tangible {

class ExecutionManager {
private:
	int mode;
	std::vector<tangible::Operation> program;


	void get_program();
	void get_scene();

	void start_execution();
	void stop_execution();
	
public:
	ExecutionManager(ros::NodeHandle& n);
	~ExecutionManager();

	void mode_callback(const tangible::Mode::ConstPtr& mode_msg);
};

};

#endif