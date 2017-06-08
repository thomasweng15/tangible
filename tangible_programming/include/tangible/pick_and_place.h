#ifndef TANGIBLE_PICK_AND_PLACE
#define TANGIBLE_PICK_AND_PLACE

#include "tangible/operation.h"

#include "tangible_msgs/Instruction.h"
#include "tangible_msgs/Scene.h"
#include "tangible_msgs/SceneObject.h"

namespace tangible
{

class PickAndPlace : public Operation
{
private:
	const static int PICK = 0;
	const static int PLACE = 1;

	bool once;

	bool attempt_pick();
	bool attempt_place();

	bool match_obj2criteria(tangible_msgs::SceneObject obj, tangible_msgs::Target trg);

public:
	PickAndPlace(ros::NodeHandle& n, std::vector<tangible_msgs::Instruction> ins);
	~PickAndPlace();

	bool execute();
	void stop();
	void reset();

};

};

#endif