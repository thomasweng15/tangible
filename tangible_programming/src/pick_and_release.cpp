#include "tangible/pick_and_release.h"

namespace tangible
{

PickAndRelease::PickAndRelease(ros::NodeHandle& n, std::vector<tangible_msgs::Instruction> ins) : Operation (n, ins) 
{
	// TO-DO error-checking
	// make sure there are exactly two instructions within the operation
	// the first instruction is a pick and the second one is a release

	// ROS_INFO("a pick and release operation created with the following instructions");
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
	//			break;
	// 	}

	once = false; 
}

PickAndRelease::~PickAndRelease() {}

void PickAndRelease::start()
{
	std::string start_service = get_private_param("arm_movement_control_service");
	ros::ServiceClient start_client = node_handle.serviceClient<tangible_msgs::ControlMovements>(start_service);

	tangible_msgs::ControlMovements start_srv;
	start_srv.request.type = tangible_msgs::ControlMovements::Request::ENABLE;

	bool success = start_client.call(start_srv);

	if(success)
		ROS_INFO("successfully called service %s to start arm motions.", start_service.c_str());
	else
		ROS_ERROR("failed to call service %s to start arm motions.", start_service.c_str());
}

bool PickAndRelease::execute() 
{
	ROS_INFO("executing pick and release operation");

	int attempt = 0;
	
	while(!done[RELEASE] && attempt <= OPERATION_MAX_ATTEMPTS)
	{
		
		ROS_INFO("attempt #%d", attempt+1);
		int pick_status = NO_PICK;

		if(!done[PICK])
			done[PICK] = attempt_pick(obj_of_op, pick_status);

		if(done[PICK])
			// NOTE: the content of obj_of_op is only used when it holds  
			// valid information so no need to clean it up from one attempt 
			// of operation to another or from one iteration over the entire 
			// program of operations to another.	
			done[RELEASE] = attempt_release(obj_of_op);

		attempt++;

		if(done[RELEASE] && !once)
			once = true;

		if(!(instructions[PICK].target.type == tangible_msgs::Target::POINT_LOCATION || 
		   instructions[PICK].target.type == tangible_msgs::Target::OBJECT_SELECTOR))
			// TO-DO better to clearly state the conditions for OBJECT_SELECTOR and OBJECTS_SELECTOR
		{
			if(done[RELEASE])
			{
				done[PICK] = false;
				done[RELEASE] = false;
				attempt = 0;
				ROS_INFO("moving to the next object of operation...");
			}
			else if(!done[PICK] && once && pick_status == NO_PICK)
			// TO-DO check whether the condition can be simplified. done[PICK] and pick_status seem redundant
			{
				done[RELEASE] = true;
				ROS_INFO("no more objects for operation.");
			}
		}

	}

	if(done[RELEASE])
		all_done = true;

	ROS_INFO("execution of operation was %s", all_done ? "SUCCESSFUL" : "UNSUCCESSFUL");

	return all_done;
	// TO-DO what to return is debatable
}

bool PickAndRelease::attempt_pick(tangible_msgs::SceneObject& obj_under_op, int& pick_status)
{
	ROS_INFO("   pick...");

	tangible_msgs::Scene scene = get_scene();
	display_objects(scene.objects);
	
	for(int i = 0; i < scene.objects.size(); i++)
	{
		// TO-DO highlight the object being considered
		ROS_INFO("   object %d bb(%f, %f, %f) at (%f, %f, %f):", i,
															  	 scene.objects[i].bounding_box.dimensions.x,
															  	 scene.objects[i].bounding_box.dimensions.y,
															  	 scene.objects[i].bounding_box.dimensions.z,
															  	 scene.objects[i].bounding_box.pose.pose.position.x,
															  	 scene.objects[i].bounding_box.pose.pose.position.y,
															  	 scene.objects[i].bounding_box.pose.pose.position.z);

		if(!match_obj2criteria(scene.objects[i], instructions[PICK].target))
		{
			ROS_INFO("      does not match pick criteria ==> skipped ");
			continue;
		}
		ROS_INFO("      matched pick criteria");

		bool already_placed = false;

		if(instructions[RELEASE].target.type == tangible_msgs::Target::REGION ||  
		   instructions[RELEASE].target.type == tangible_msgs::Target::POINT_LOCATION)

			already_placed = match_obj2criteria(scene.objects[i], instructions[RELEASE].target);
			
		else 
		// TO-DO better to clearly state the conditions for OBJECT_SELECTOR and OBJECTS_SELECTOR
		{

			for(int j = 0; j < scene.objects.size(); j++)
			{
				// NOTE: make an exclusive comparison (remove what you are comparing from candidate pool
				// as everything matches itself perfectly.)
				if(j == i)
					continue;

				if(!match_obj2criteria(scene.objects[j], instructions[RELEASE].target))
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
		{
			ROS_INFO("      but is already at release target ==> skipped ");
			continue;
		}
		ROS_INFO("      is not already at release target");

		int pick_attempt = 0;
		std::vector<moveit_msgs::Grasp> grasps;
		while(pick_attempt <= INSTRUCTION_MAX_ATTEMPTS)
		{
			grasps = get_grasp(scene.objects[i], scene);
			if(!grasps.empty())
				break;
			pick_attempt++;
		}

		pick_attempt = pick_attempt > INSTRUCTION_MAX_ATTEMPTS ? pick_attempt : 0;

		if(pick_attempt == 0)
			ROS_INFO("      has a grasp sequence.");
		else
		{
			ROS_INFO("      unfortunately, does not have a grasp sequence. ==> skipped ");
			pick_status = FAILED_GRASP;
		}

		while(pick_attempt <= INSTRUCTION_MAX_ATTEMPTS)
		{
			// if here, there has been a grasp
			if(move(grasps, PICK))
			{
				ROS_INFO("      is successfully picked.");
				obj_under_op = scene.objects[i];
				return true;
			}
			pick_attempt++; 
		}

		ROS_INFO("      unfortunately, pick was not successful. ==> skipped ");
		pick_status = FAILED_GRASP;

	}

	return false;
}

bool PickAndRelease::attempt_release(tangible_msgs::SceneObject obj_held)
{
	ROS_INFO("   release...");
	ROS_INFO("   object bb(%f, %f, %f) at (%f, %f, %f):", obj_held.bounding_box.dimensions.x,
														  obj_held.bounding_box.dimensions.y,
														  obj_held.bounding_box.dimensions.z,
														  obj_held.bounding_box.pose.pose.position.x,
														  obj_held.bounding_box.pose.pose.position.y,
														  obj_held.bounding_box.pose.pose.position.z);

	tangible_msgs::Scene scene = get_scene();
	display_objects(scene.objects);

	std::vector<tangible_msgs::Target> place_target;

	if(instructions[RELEASE].target.type == tangible_msgs::Target::REGION ||  
	   instructions[RELEASE].target.type == tangible_msgs::Target::POINT_LOCATION)

		place_target.push_back(instructions[RELEASE].target);

	else 
	// TO-DO better to clearly state the conditions for OBJECT_SELECTOR and OBJECTS_SELECTOR
	{
		
		for(int i = 0; i < scene.objects.size(); i++)
			if(match_obj2criteria(scene.objects[i], instructions[RELEASE].target))
			{
				tangible_msgs::Target target;
				target.type = tangible_msgs::Target::OBJECT_SELECTOR;
				target.selected_object = scene.objects[i];
				place_target.push_back(target);
			}

	}

	ROS_INFO("number of valid placement targets: %d", (int) place_target.size());

	// TO-DO possibly rank the palcement targets before attempting at placing at them

	for(int i = 0; i < place_target.size(); i++)
	{
		//TO-DO highlight the object being considered
		
		std::vector<moveit_msgs::Grasp> releases;
		int place_attempt = 0;
		while(place_attempt <= INSTRUCTION_MAX_ATTEMPTS)
		{
			releases = get_release(place_target[i], obj_held, scene, instructions[RELEASE].type);
			if(!releases.empty())
				break;
			place_attempt++;
		}

		place_attempt = place_attempt > INSTRUCTION_MAX_ATTEMPTS ? place_attempt : 0;

		if(place_attempt == 0)
			ROS_INFO("      has a release sequence");
		else
			ROS_INFO("      does not have a release sequence.");

		while(place_attempt <= INSTRUCTION_MAX_ATTEMPTS)
		{
			// if here, there has been a release
			if(move(releases, RELEASE))
			{
				ROS_INFO("      is successfully released.");
				return true;
			}
			place_attempt++;
		}

		ROS_INFO("      unfortunately, release was unsuccessful.");	

	}

	return false;
}

bool PickAndRelease::match_obj2criteria(tangible_msgs::SceneObject obj, tangible_msgs::Target trg)
{
	std::vector<tangible_msgs::SceneObject> objs_to_check;
	objs_to_check.push_back(obj);

	std::string ins_check_service = get_private_param("instruction_check_criteria_service");
	ros::ServiceClient ins_check_client = node_handle.serviceClient<tangible_msgs::GetMatchingObjects>(ins_check_service);

	tangible_msgs::GetMatchingObjects criteria_srv;
	criteria_srv.request.objects = objs_to_check;
	criteria_srv.request.target = trg;

	// ROS_INFO("      object (%f, %f, %f) at (%f, %f, %f) matched target %d?", obj.bounding_box.dimensions.x,
	// 					   												     obj.bounding_box.dimensions.y,
	// 																   		 obj.bounding_box.dimensions.z, 
	// 																   		 obj.bounding_box.pose.pose.position.x, 
	// 																   		 obj.bounding_box.pose.pose.position.y, 
	// 																   		 obj.bounding_box.pose.pose.position.z,
	// 																   		 trg.type);

	bool success = ins_check_client.call(criteria_srv);

	std::vector<tangible_msgs::SceneObject> objs_checked;
	if(success)
		objs_checked = criteria_srv.response.objects;
	else
	{
		ROS_ERROR("failed to call service %s to check instruction criteria.", ins_check_service.c_str());
		return false;
	}

	// if(objs_checked.empty())
	// 	ROS_INFO("      - NO");
	// else
	// 	ROS_INFO("      - YES");

	return !objs_checked.empty();
}

std::vector<moveit_msgs::Grasp> PickAndRelease::get_grasp(tangible_msgs::SceneObject obj, tangible_msgs::Scene scene)
{
	std::string grasp_service = get_private_param("grasp_acquisition_service");
	ros::ServiceClient grasp_acquisition_client = node_handle.serviceClient<tangible_msgs::GetGrasps>(grasp_service);

	tangible_msgs::GetGrasps grasp_srv;
	grasp_srv.request.object = obj;
	grasp_srv.request.scene = scene;

	bool success = grasp_acquisition_client.call(grasp_srv);

	std::vector<moveit_msgs::Grasp> grasps;
	if(success)
		grasps = grasp_srv.response.grasps;
	else
		ROS_ERROR("failed to call service %s to obtain grasp poses.", grasp_service.c_str());

	return grasps;
}

std::vector<moveit_msgs::Grasp> PickAndRelease::get_release(tangible_msgs::Target target, tangible_msgs::SceneObject obj, tangible_msgs::Scene scene, int release_type)
{
	std::string release_service = get_private_param("release_acquisition_service");
	ros::ServiceClient release_acquisition_client = node_handle.serviceClient<tangible_msgs::GetReleases>(release_service);

	tangible_msgs::GetReleases release_srv;
	if(release_type == tangible_msgs::Instruction::PLACE)
		release_srv.request.type = tangible_msgs::GetReleases::Request::PLACE;
	else if(release_type == tangible_msgs::Instruction::DROP)
		release_srv.request.type = tangible_msgs::GetReleases::Request::DROP;
	release_srv.request.num_orientations = 4;
	release_srv.request.region_sample_spacing = 0.3;
	release_srv.request.target = target;
	release_srv.request.object = obj;
	release_srv.request.scene = scene;

	bool success = release_acquisition_client.call(release_srv);

	std::vector<moveit_msgs::Grasp> releases;
	if(success)
		releases = release_srv.response.releases;
	else
		ROS_ERROR("failed to call service %s to obtain release poses.", release_service.c_str());

	return releases;
}

bool PickAndRelease::move(std::vector<moveit_msgs::Grasp> poses, int type)
{
	std::string move_service = get_private_param("arm_movement_service");
	ros::ServiceClient move_client = node_handle.serviceClient<tangible_msgs::GetMovements>(move_service);

	tangible_msgs::GetMovements move_srv;
	move_srv.request.poses = poses;
	if(type == PICK)
		move_srv.request.type = tangible_msgs::GetMovements::Request::PICK;
	else if(type == RELEASE)
		move_srv.request.type = tangible_msgs::GetMovements::Request::RELEASE;
	else
		ROS_ERROR("invalid movement type passed to arm_movement_service");

	bool success = move_client.call(move_srv);

	bool movement_success = false;
	if(success)
		movement_success = move_srv.response.status;
	else
		ROS_ERROR("failed to call service %s to move the arm.", move_service.c_str());

	return movement_success;
}

void PickAndRelease::stop()
{
	std::string stop_service = get_private_param("arm_movement_control_service");
	ros::ServiceClient stop_client = node_handle.serviceClient<tangible_msgs::ControlMovements>(stop_service);

	tangible_msgs::ControlMovements stop_srv;
	stop_srv.request.type = tangible_msgs::ControlMovements::Request::DISABLE;

	bool success = stop_client.call(stop_srv);

	if(success)
		ROS_INFO("successfully called service %s to stop arm motions.", stop_service.c_str());
	else
		ROS_ERROR("failed to call service %s to stop arm motions.", stop_service.c_str());
}

void PickAndRelease::reset()
{
	once = false;

	Operation::reset();
	
	ROS_INFO("pick_and_release reset");
}

std::string PickAndRelease::print()
{
	std::stringstream ss;

	for(int i = 0; i < instructions.size(); i++)
	{
		switch(instructions[i].type)
		{
			case tangible_msgs::Instruction::PICK:
				ss << "pick";
				switch(instructions[i].target.type)
				{
					case tangible_msgs::Target::POINT_LOCATION:
						ss << " from point ";
						break;
					case tangible_msgs::Target::OBJECT_SELECTOR:
						ss << " the  object";
						break;
					case tangible_msgs::Target::REGION:
						ss << " from region";
						break;
					// TO-DO the case of objects selector
					default:
						ROS_ERROR("invalid pick and release selection; abort printing");
						return "";
				}
				break;
			case tangible_msgs::Instruction::PLACE:
				ss << "place";
				switch(instructions[i].target.type)
				{
					case tangible_msgs::Target::POINT_LOCATION:
						ss << " at point ";
						break;
					case tangible_msgs::Target::OBJECT_SELECTOR:
						ss << " on object";
						break;
					case tangible_msgs::Target::REGION:
						ss << " in region";
						break;
					// TO-DO the case of objects selector
					default:
						ROS_ERROR("invalid pick and release selection; abort printing");
						return "";
				}
				break;
			case tangible_msgs::Instruction::DROP:
				ss << "drop";
				switch(instructions[i].target.type)
				{
					case tangible_msgs::Target::POINT_LOCATION:
						ss << " at point ";
						break;
					case tangible_msgs::Target::OBJECT_SELECTOR:
						ss << " in object";
						break;
					case tangible_msgs::Target::REGION:
						ss << " in region";
						break;
					// TO-DO the case of objects selector
					default:
						ROS_ERROR("invalid pick and release selection; abort printing");
						return "";
				}
				break;
			default:
				ROS_ERROR("invalid pick and release action; abort printing");
				return "";
				break;
		}

		switch(instructions[i].target.type)
		{
			case tangible_msgs::Target::POINT_LOCATION:
				ss << " (" << instructions[i].target.specified_point.point.x;
				ss << ", " << instructions[i].target.specified_point.point.y;
				ss << ", " << instructions[i].target.specified_point.point.z << ")";
				break;
			case tangible_msgs::Target::OBJECT_SELECTOR:
				ss << " bb(" << instructions[i].target.selected_object.bounding_box.dimensions.x;
				ss << ", "   << instructions[i].target.selected_object.bounding_box.dimensions.y;
				ss << ", "   << instructions[i].target.selected_object.bounding_box.dimensions.z << ")";
				break;
			case tangible_msgs::Target::REGION:
				ss << "(";
				for(int j = 0; j < instructions[i].target.region_corners.size(); j++)
				{
					ss <<          instructions[i].target.region_corners[j].point.x;
					ss << ", "  << instructions[i].target.region_corners[j].point.y;
					ss << ", "  << instructions[i].target.region_corners[j].point.z << " ^ ";
				}
				ss << ")";
				break;
			// TO-DO the case of objects selector
		}

		ss << " then ";
	}

	return ss.str();
}

void PickAndRelease::display_objects(std::vector<tangible_msgs::SceneObject> objs)
{
	std::string display_service = get_private_param("object_visualization_service");
	ros::ServiceClient display_client = node_handle.serviceClient<tangible_msgs::VisualizeObjects>(display_service);

	tangible_msgs::VisualizeObjects display_srv;
	display_srv.request.objects = objs;

	bool success = display_client.call(display_srv);

	if(success)
		ROS_INFO("successfully called service %s to visualize objects.", display_service.c_str());
	else
		ROS_ERROR("failed to call service %s to visualize objects.", display_service.c_str());
}

}