#include "tangible/pick_and_place.h"

namespace tangible
{

PickAndPlace::PickAndPlace(ros::NodeHandle& n, std::vector<tangible_msgs::Instruction> ins) : Operation (n, ins) 
{
	//TO-DO error-checking: make sure there are exactly two instructions within the operation
	once = false; 
}

PickAndPlace::~PickAndPlace() {}

bool PickAndPlace::execute() 
{
	ROS_INFO("executing pick and place operation");


	while(!done[PLACE])
	{
		if(!done[PICK])
		{
			ROS_INFO("obtain table top and objects on top of it.");
		}
	}

	return all_done;
	//TO-DO what to return is debatable
}

void PickAndPlace::stop()
{
	ROS_INFO("stop Pick and Place operation");
	//TO-DO
}

}