#ifndef TANGIBLE_EXECUTION_MANAGER
#define TANGIBLE_EXECUTION_MANAGER

#include "ros/ros.h"

namespace tangible {

class ExecutionManager {
private:
	int mode;
	// TO-DO
	// fields to store
	//   - program to execute
	//   - progress in executing the program.
	
public:
	ExecutionManager(ros::NodeHandle& n);
	~ExecutionManager();

	// TO-DO: subscribe to mode topic
};

};

#endif