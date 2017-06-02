#ifndef TANGIBLE_PICK_AND_PLACE
#define TANGIBLE_PICK_AND_PLACE

#include "tangible/operation.h"

#include "tangible_msgs/Instruction.h"

namespace tangible
{

class PickAndPlace : public Operation
{
private:
	tangible_msgs::Instruction pick;
	tangible_msgs::Instruction place;

public:
	PickAndPlace(std::vector<tangible_msgs::Instruction> ins);
	~PickAndPlace();

	bool execute();

	void stop();

};

};

#endif