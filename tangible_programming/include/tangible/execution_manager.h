#ifndef TANGIBLE_EXECUTION_MANAGER
#define TANGIBLE_EXECUTION_MANAGER

#include <vector>

#include "ros/ros.h"

// user-defined classes
#include "tangible/operation.h"
#include "tangible/pick_and_release.h"

// msg's and srv's
#include "tangible_msgs/Mode.h"
#include "tangible_msgs/Program.h"
#include "tangible_msgs/GetProgram.h"

namespace tangible
{

class ExecutionManager
{
private:
	const static int STOPPED = -1;
	
	ros::NodeHandle node_handle;
	ros::Subscriber exec_mode;

	int mode;
	std::vector<tangible::Operation*> program;

	bool executing;
	int current_operation_index;

	std::string get_private_param(std::string param_name);

	void setup_program(tangible_msgs::Program p);
	void clear_program();

	bool get_program();

	void start_execution();
	void stop_execution();
	
public:
	ExecutionManager(ros::NodeHandle& n);
	~ExecutionManager();

	void mode_callback(const tangible_msgs::Mode::ConstPtr& mode_msg);
};

};

#endif