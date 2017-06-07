#ifndef TANGIBLE_PICK_AND_PLACE
#define TANGIBLE_PICK_AND_PLACE

#include "tangible/operation.h"

#include "tangible_msgs/Instruction.h"

namespace tangible
{

class PickAndPlace : public Operation
{
private:
	const static int PICK = 0;
	const static int PLACE = 1;

	bool once;

public:
	PickAndPlace(ros::NodeHandle& n, std::vector<tangible_msgs::Instruction> ins);
	~PickAndPlace();

	bool execute();

	void stop();

};

};

#endif