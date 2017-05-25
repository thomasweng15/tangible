#ifndef TANGIBLE_OPERATION
#define TANGIBLE_OPERATION

#include "ros/ros.h"

// user-defined classes
#include "tangible/instruction.h"

// msg's and srv's
#include "tangible/Operation.h"

namespace tangible
{

//QUESTION: can I use the same name as a message for my classes?
//          I suppose not. How would the compiler know which type is intended
//          if they both use the same name
class Operation
{
private:
	bool done;
	std::vactor<tangible::Instruction> instructions;
	//NOTE: tangible::Instruction is not a message, it's the interface I have defined

public:
	Operation(tangible::Operation op);
	//TO-DO: initialize done to false
	~Operation();

	bool execute();

	void reset();
	//TO-DO: mark as undone all the instructions of an operation, and the operation itself
};

};

#endif