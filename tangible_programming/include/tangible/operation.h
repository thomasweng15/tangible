#ifndef TANGIBLE_OPERATION
#define TANGIBLE_OPERATION

#include <vector>

#include "ros/ros.h"

// msg's and srv's
#include "tangible_msgs/Scene.h"
#include "tangible_msgs/Instruction.h"
#include "tangible_msgs/GetScene.h"

namespace tangible
{

class Operation
{
protected:
	const static int OPERATION_MAX_ATTEMPTS = 2;
	const static int INSTRUCTION_MAX_ATTEMPTS = 2;

	ros::NodeHandle node_handle;

	bool all_done;
	std::vector<tangible_msgs::Instruction> instructions;
	std::vector<bool> done;

	std::string get_private_param(std::string param_name);
	
	tangible_msgs::Scene get_scene();

	// for testing --- should be removed afterwards
	void print_scene(tangible_msgs::Scene scene);

public:
	Operation(ros::NodeHandle& n, std::vector<tangible_msgs::Instruction> ins);
	virtual ~Operation() = 0;

	bool is_done();

	virtual bool execute() = 0;
	virtual void stop() = 0;
	void reset();
};

};

#endif