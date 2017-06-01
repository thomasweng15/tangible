#include "tangible/execution_manager.h"

namespace tangible
{

ExecutionManager::ExecutionManager(ros::NodeHandle& n)
{
	node_handle = n;

	std::string system_mode_topic = get_private_param("system_mode_topic");
	exec_mode = node_handle.subscribe(system_mode_topic, 1000, &tangible::ExecutionManager::mode_callback, this);

	executing = false;

	ROS_INFO("Execution node is instantiated and listens to %s topic.", system_mode_topic.c_str());
	// NOTE: system_mode_topic is tpbd_mode
}

ExecutionManager::~ExecutionManager() {}

void ExecutionManager::mode_callback(const tangible_msgs::Mode::ConstPtr& mode_msg) 
{
	mode = mode_msg->mode;
	
	switch(mode)
	{
		case tangible_msgs::Mode::IDLE:
			ROS_INFO("Idle Mode");
			executing = false;

			// stop all movements
			stop_execution();

			break;

		case tangible_msgs::Mode::EDIT:
			ROS_INFO("Edit Mode");
			executing = false;

			// stop all movements
			stop_execution();
			
			// clear the program. A new program should be obtained after the edit
			program.operations.clear();

			break;

		case tangible_msgs::Mode::EXECUTE:
			ROS_INFO("Execution Mode");
			executing = true;

			// if the program is not defined, obtain the program
			// else resume its execution from the first incompelete instruction.
			// NOTE: instructions are the atomic units of normal program execution.
			if(program.operations.empty())
				executing = get_program();

			start_execution();

			break;

		default:
			ROS_ERROR("Invalid Mode");
	}
}

std::string ExecutionManager::get_private_param(std::string param_name)
{
	ros::NodeHandle private_params("~");
	std::string param;
	private_params.getParam(param_name, param);
	return param;
}

bool ExecutionManager::get_program() 
{
	std::string get_program_srv = get_private_param("program_acquisition_service");
	ros::ServiceClient program_acquisition_client = node_handle.serviceClient<tangible_msgs::GetProgram>(get_program_srv);

	tangible_msgs::GetProgram program_srv;
	bool success = program_acquisition_client.call(program_srv);
	if(success)
	{
		ROS_INFO("program received");
		//TO-DO: make sure the program in the message is not empty before modifying the exiting program
		program = program_srv.response.program;
		//TO-DO: program should be a std::vector<tangible::Operation>
		//       that is populated based on the program message.
	}
	else
	{
		ROS_ERROR("Failed to call service %s to obtain the compiled program.", get_program_srv.c_str());
	}

	return success;
}

bool ExecutionManager::get_scene()
{
	ROS_INFO("request scene");
	//TO-DO
	return true;
}

void ExecutionManager::start_execution()
{
	ROS_INFO("start moving the robot");
	//TO-DO
	// should always condition on executing
}

void ExecutionManager::stop_execution()
{
	ROS_INFO("stop moving the robot");
	//TO-DO
}

}