#include "tangible/pick_and_place.h"

namespace tangible
{

PickAndPlace::PickAndPlace(ros::NodeHandle& n, std::vector<tangible_msgs::Instruction> ins) : Operation (n, ins) 
{
	// TO-DO error-checking: make sure there are exactly two instructions within the operation

	// ROS_INFO("a pick and place operation created with the following instructions");
	// for(int i = 0; i < instructions.size(); i++)
	// 	switch(instructions[i].type)
	// 	{
	// 		case tangible_msgs::Instruction::PICK:
	// 			ROS_INFO("pick");
	// 			break;
	// 		case tangible_msgs::Instruction::PLACE:
	// 			ROS_INFO("place");
	// 			break;
	// 		case tangible_msgs::Instruction::DROP:
	// 			ROS_INFO("drop");
	// 			break;
	// 		default:
	// 			ROS_ERROR("invalid instruction");
	// 	}

	once = false; 
}

PickAndPlace::~PickAndPlace() {}

bool PickAndPlace::execute() 
{
	ROS_INFO("executing pick and place operation");

	int attempt = 0;

	while(!done[PLACE] && attempt <= OPERATION_MAX_ATTEMPTS)
	{
		
		if(!done[PICK])
			done[PICK] = attempt_pick();

		if(done[PICK])
			done[PLACE] = attempt_place();

		attempt++;

		if(done[PLACE] && !once)
			once = true;

		if(!(instructions[PICK].target.type == tangible_msgs::Target::POINT_LOCATION || 
		   instructions[PICK].target.type == tangible_msgs::Target::OBJECT_SELECTOR))
			// TO-DO better to clearly state the conditions for OBJECT_SELECTOR and OBJECTS_SELECTOR
		{
			if(done[PLACE])
			{
				done[PICK] = false;
				done[PLACE] = false;
				attempt = 0;
			}
			else if(!done[PICK] && once)
				done[PLACE] = true;
		}

	}

	if(done[PLACE])
		all_done = true;

	return all_done;
	// TO-DO what to return is debatable
}

bool PickAndPlace::attempt_pick()
{
	tangible_msgs::Scene scene = get_scene();
	//print_scene(scene);
	
	for(int i = 0; i < scene.objects.size(); i++)
	{

		if(!match_obj2criteria(scene.objects[i], instructions[PICK].target))
			continue;
		
		bool already_placed = false;

		if(instructions[PLACE].target.type == tangible_msgs::Target::REGION ||  
		   instructions[PLACE].target.type == tangible_msgs::Target::POINT_LOCATION)

			already_placed = match_obj2criteria(scene.objects[i], instructions[PLACE].target);
			
		else 
		// TO-DO better to clearly state the conditions for OBJECT_SELECTOR and OBJECTS_SELECTOR
		{

			for(int j = 0; j < scene.objects.size(); j++)
			{
				if(!match_obj2criteria(scene.objects[j], instructions[PLACE].target))
					continue;
				
				tangible_msgs::Target place_target;
				place_target.type = tangible_msgs::Target::POINT_LOCATION;
				place_target.specified_point.point = scene.objects[j].bounding_box.pose.pose.position;

				if(match_obj2criteria(scene.objects[i], place_target))
				{
					already_placed = true;
					break;
				}
			}

		}

		if(already_placed)
			continue;

		int pick_attempt = 0;
		while(pick_attempt <= INSTRUCTION_MAX_ATTEMPTS)
		{
			// TO-DO generate grasp
			// if non empty break
			pick_attempt++;
		}

		pick_attempt = pick_attempt > INSTRUCTION_MAX_ATTEMPTS ? pick_attempt : 0;

		while(pick_attempt <= INSTRUCTION_MAX_ATTEMPTS)
		{
			// if here, there has been a grasp

			// TO-DO move the arm
			// if succeeded (done[PICK] = true and break) --> return true
			pick_attempt++; 
		}

	}

	return false;
}

bool PickAndPlace::attempt_place()
{
	tangible_msgs::Scene scene = get_scene();

	std::vector<tangible_msgs::Target> place_target;

	if(instructions[PLACE].target.type == tangible_msgs::Target::REGION ||  
	   instructions[PLACE].target.type == tangible_msgs::Target::POINT_LOCATION)

		place_target.push_back(instructions[PLACE].target);

	else 
	// TO-DO better to clearly state the conditions for OBJECT_SELECTOR and OBJECTS_SELECTOR
	{
		
		for(int i = 0; i < scene.objects.size(); i++)
			if(match_obj2criteria(scene.objects[i], instructions[PLACE].target))
			{
				tangible_msgs::Target target;
				target.type = tangible_msgs::Target::OBJECT_SELECTOR;
				target.selected_object = scene.objects[i];
				place_target.push_back(target);
			}

	}

	// TO-DO possibly rank the palcement targets before attempting at placing at them

	for(int i = 0; i < place_target.size(); i++)
	{
		
		int place_attempt = 0;
		while(place_attempt <= INSTRUCTION_MAX_ATTEMPTS)
		{
			// TO-DO generate release
			// if not empty break
			place_attempt++;
		}

		place_attempt = place_attempt > INSTRUCTION_MAX_ATTEMPTS ? place_attempt : 0;

		while(place_attempt <= INSTRUCTION_MAX_ATTEMPTS)
		{
			// if here, there has been a release

			// TO-DO move the arm
			// if succeeded (done[PLACE] = true and break) --> return true;
			place_attempt++;
		}

	}

	return false;
}

bool PickAndPlace::match_obj2criteria(tangible_msgs::SceneObject obj, tangible_msgs::Target trg)
{
	std::vector<tangible_msgs::SceneObject> objs_to_check;
	objs_to_check.push_back(obj);

	std::string ins_check_service = get_private_param("instruction_check_criteria_service");
	ros::ServiceClient ins_check_client = node_handle.serviceClient<tangible_msgs::GetMatchingObjects>(ins_check_service);

	tangible_msgs::GetMatchingObjects criteria_srv;
	criteria_srv.request.objects = objs_to_check;
	criteria_srv.request.target = trg;

	bool success = ins_check_client.call(criteria_srv);

	std::vector<tangible_msgs::SceneObject> objs_checked;
	if(success)
		objs_checked = criteria_srv.response.objects;
	else
	{
		// Edited by Sarah to make it compile, but may want to change back
		ROS_ERROR("failed to call service %s to obtain matching object information", ins_check_service.c_str());
		return false;
	}

	return !objs_checked.empty();
}

void PickAndPlace::stop()
{
	ROS_INFO("stop Pick and Place operation");
	// TO-DO
}

void PickAndPlace::reset()
{
	Operation::reset();
	
	once = false;
	
	ROS_INFO("pick_and_place reset.");
}

}