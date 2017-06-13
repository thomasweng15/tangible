#ifndef TANGIBLE_PICK_AND_PLACE
#define TANGIBLE_PICK_AND_PLACE

#include <vector>

#include "tangible/operation.h"

// msg's and srv's
#include "tangible_msgs/Instruction.h"
#include "tangible_msgs/Scene.h"
#include "tangible_msgs/SceneObject.h"
#include "tangible_msgs/GetMatchingObjects.h"
#include "tangible_msgs/GetGrasps.h"
#include "tangible_msgs/GetReleases.h"
#include "tangible_msgs/GetMovements.h"
#include "moveit_msgs/Grasp.h"

namespace tangible
{

class PickAndPlace : public Operation
{
private:
	const static int PICK = 0;
	const static int PLACE = 1;

	const static int NO_PICK = 0; // there has not be any pick possible
	const static int FAILED_GRASP = 1; // a possible pick has failed.

	bool once;

	tangible_msgs::SceneObject obj_of_op;

	bool attempt_pick(tangible_msgs::SceneObject& obj_under_op, int& pick_status);
	bool attempt_place(tangible_msgs::SceneObject obj_held);

	bool match_obj2criteria(tangible_msgs::SceneObject obj, tangible_msgs::Target trg);

	std::vector<moveit_msgs::Grasp> get_grasp(tangible_msgs::SceneObject obj, tangible_msgs::Scene scene);
	std::vector<moveit_msgs::Grasp> get_release(tangible_msgs::Target target, tangible_msgs::SceneObject obj, tangible_msgs::Scene scene, int release_type);
	bool move(std::vector<moveit_msgs::Grasp> poses);

public:
	PickAndPlace(ros::NodeHandle& n, std::vector<tangible_msgs::Instruction> ins);
	~PickAndPlace();

	bool execute();
	void stop();
	void reset();

};

};

#endif