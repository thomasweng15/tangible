#include "tangible/operation.h"

namespace tangible 
{

Operation::Operation(std::vector<tangible_msgs::Instruction> ins)
{
	instructions = ins;
	//QUESTION: is this fine or should I do the following to make sure even when
	//          ins is going out of scope we have access to the data it contained
	//for(int i = 0; i  < ins.size(); i++)
	//	instructions.push_back(ins[i]);
	done = false;
}

Operation::~Operation() {}

bool Operation::is_done()
{
	return done;
}

void Operation::reset() 
{
	done = false;
	//TO-DO: mark as undone all the instructions of an operation, in addition to the operation itself
}

}