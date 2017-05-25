#ifndef TANGIBLE_EXECUTION_MANAGER
#define TANGIBLE_EXECUTION_MANAGER

#include <vector>

#include "ros/ros.h"

// user-defined classes


// msg's and srv's
#include "tangible/Mode.h"
#include "tangible/Program.h"
#include "tangible/GetProgram.h"

namespace tangible
{

class ExecutionManager
{
private:
	ros::NodeHandle node_handle;
	ros::Subscriber exec_mode;

	int mode;
	tangible::Program program;
	//QUESTION: how is tangible::Program recognized without including tanigble/Program.h 
	//          BUT tangible::Mode is not recognized without including tangible/Mode.h?
	// I suspect it's because Program.h is available as GetProgam includes it
	//TO-DO: this should be std::vector<tangible::Operation> program
	//       and should be populated based on the program message for each program acquisition

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