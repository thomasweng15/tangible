#ifndef TANGIBLE_ACTION_TARGET
#define TANGIBLE_ACTION_TARGET

namespace tangible
{

class ActionTarget
{
protected:
public:
	ActionTarget();
	~ActionTarget();

	virtual bool match() = 0;
};

};

#endif