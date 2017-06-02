#ifndef TANGIBLE_OPERATION
#define TANGIBLE_OPERATION

#include "ros/ros.h"


// msg's and srv's
#include "tangible_msgs/Instruction.h"

namespace tangible
{

//QUESTION: can I use the same name as a message for my classes?
//          I suppose not. How would the compiler know which type is intended
//          if they both use the same name
class Operation
{
protected:
	bool done;
	std::vector<tangible_msgs::Instruction> instructions;
	//NOTE: tangible::Instruction is not a message, it's the interface I have defined

public:
	Operation(std::vector<tangible_msgs::Instruction> ins);
	//TO-DO: initialize done to false
	virtual ~Operation() = 0;

	virtual bool execute() = 0;

	virtual void stop() = 0;

	bool is_done();

	void reset();
};

};

#endif