#ifndef TANGIBLE_INSTRUCTION
#define TANGIBLE_INSTRUCTION

namespace tangible
{

class Instruction
{
protected:
	ActionTarget target;
	bool done;

public:
	Instruction();
	virtual ~Instruction() = 0;

	virtual bool execute() = 0;
}

};

#endif