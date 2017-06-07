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
private:
	ros::NodeHandle node_handle;
	std::string get_private_param(std::string param_name);

protected:
	bool all_done;
	std::vector<tangible_msgs::Instruction> instructions;
	std::vector<bool> done;

	tangible_msgs::Scene get_scene();

public:
	Operation(ros::NodeHandle& n, std::vector<tangible_msgs::Instruction> ins);
	virtual ~Operation() = 0;

	virtual bool execute() = 0;

	virtual void stop() = 0;

	bool is_done();

	void reset();
};

};

#endif