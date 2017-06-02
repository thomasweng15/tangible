#include "tangible/pick_and_place.h"

namespace tangible
{

PickAndPlace::PickAndPlace(std::vector<tangible_msgs::Instruction> ins) : Operation (ins)
{
	pick = instructions[0];
	place = instructions[1];
}

PickAndPlace::~PickAndPlace() {}

bool PickAndPlace::execute() 
{
	ROS_INFO("executing pick and place operation");

	// executing pick
	//TO-DO if(!pick_done)
	//{
		ROS_INFO("call a service to obtain scene information (segmentation of the table top and objects on top of it.");
		ROS_INFO("for each object in the scene:");
		ROS_INFO("   call a service with that object and pick instruction. The service retruns true if the object satisfies the instruction criteria.");
		//NOTE: this service may internally call other services, e.g. to match object
		ROS_INFO("   if matched with pick criteria, call the same service with that object and place.");
		ROS_INFO("   if matched with place criteria, continue to the next object.");
		ROS_INFO("   if not matched mark the object as pickable (by storing its index) and exit the loop.");
		//TO-DO if no such object is found for region selector after the first succesful pick and place, then can move to the next operation setting this one as done 
		ROS_INFO("call a service with the pickable object to get the grasp for it. Repeat until successful.");
		//TO-DO may need to define max number of attempts and retrun false if reached that number without succeeding at get a grasp
		ROS_INFO("call a service with grasp information to move the robot. Repeat until successful.");
		//TO-DO may need to define max number of attempts and retrun false if reached that number without succeeding at moving the robot
		//TO-DO pick_done = true;
	//}
	
	//TO-DO if(pick_done)
	//{
		ROS_INFO("call a service to obtain scene information (segmentation of the table top and objects on top of it.");
		ROS_INFO("call a service with scene information and place instruction to get the release information. Repeat until successful.");
		//TO-DO may need to define max number of attempts and retrun false if reached that number without succeeding at get a release
		ROS_INFO("call a service with realease information to move the robot. Repeat until successful.");
		//TO-DO may need to define max number of attempts and retrun false if reached that number without succeeding at moving the robot
		//place_done = true;
		//TO-DO need to play with this to handle the case for region selection
	//}

	return true;
	//TO-DO true impelementation
	//NOTES: 
	//   - done becomes true only if both pick and place are accomplished
}

void PickAndPlace::stop()
{
	ROS_INFO("stop Pick and Place operation");
	//TO-DO
}

}