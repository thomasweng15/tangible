#ifndef TANGIBLE_ACTION_TARGET
#define TANGIBLE_ACTION_TARGET

namespace tangible
{

class ActionTarget
{
public:
	ActionTarget();
	virtual ~ActionTarget();

	virtual bool match() = 0;
};

};

#endif