#include "tangible/execution_manager.h"

namespace tangible
{

ExecutionManager::ExecutionManager(ros::NodeHandle& n)
{
	node_handle = n;

	std::string system_mode_topic = get_private_param("system_mode_topic");
	exec_mode = node_handle.subscribe(system_mode_topic, 1000, &tangible::ExecutionManager::mode_callback, this);

	current_operation_index = STOPPED;

	ROS_INFO("Execution node is instantiated and listens to %s topic.", system_mode_topic.c_str());
	// NOTE: system_mode_topic is tpbd_mode
}

ExecutionManager::~ExecutionManager()
{
	clear_program();
}

void ExecutionManager::mode_callback(const tangible_msgs::Mode::ConstPtr& mode_msg) 
{
	mode = mode_msg->mode;
	
	switch(mode)
	{
		case tangible_msgs::Mode::IDLE:
			ROS_INFO("Idle Mode");

			// stop all movements
			stop_execution();

			break;

		case tangible_msgs::Mode::EDIT:
			ROS_INFO("Edit Mode");

			// stop all movements
			stop_execution();
			
			// clear the program. A new program should be obtained after the edit
			// NOTE: this requires the compilation node to always hold on to the last valid program
			clear_program();

			break;

		case tangible_msgs::Mode::EXECUTE:
			ROS_INFO("Execution Mode");

			// a program is a sequence of operations. If the program is not defined, obtain 
			// the program. Else, resume its execution from the first incompelete instruction.
			// NOTE: instructions are the atomic units of normal program execution.
			if(program.empty())
				get_program();

			start_execution();

			break;

		default:
			ROS_ERROR("Invalid Mode");
			break;
	}
}

std::string ExecutionManager::get_private_param(std::string param_name)
{
	ros::NodeHandle private_params("~");
	std::string param;
	private_params.getParam(param_name, param);
	return param;
}

void ExecutionManager::setup_program(tangible_msgs::Program p)
{
	ROS_INFO("Setting up the program operations.");

	for(int i = 0; i < p.operations.size(); i++)
	{
		tangible_msgs::Operation op = p.operations[i];
		if (op.instructions.size() == 2 &&
			op.instructions[0].type == tangible_msgs::Instruction::PICK &&
			(op.instructions[1].type == tangible_msgs::Instruction::PLACE ||
			 op.instructions[1].type == tangible_msgs::Instruction::DROP))
			program.push_back(new PickAndRelease(node_handle, op.instructions));
	}
}

void ExecutionManager::clear_program()
{
	ROS_INFO("Deleting the current program operations.");
	
	Operation* op_ptr;
	for(int i = 0; i < program.size(); i++)
	{
		op_ptr = program[i];
		delete op_ptr;
	}

	program.clear();
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
		
		if(program_srv.response.program.operations.empty())
		{
			success = false;
			ROS_ERROR("Empty program received and discarded.");
		}
		else
		{
			setup_program(program_srv.response.program);
			current_operation_index = 0;
		}
		
	}
	else
	{
		ROS_ERROR("Failed to call service %s to obtain the compiled program.", get_program_srv.c_str());
	}

	return success;
}

void ExecutionManager::start_execution()
{
	ROS_INFO("executing the program");
	
	bool full_program_iteration = false;

	// TO-DO can be more efficient if current_operation_index is only reset upon clearing programs (currently
	// at stopping programs) so we can start this loop from current_operation_index instead of the beginning of
	// of the program and skipping over the programs that are done.
	for(int i = 0; i < program.size(); i++)
	{
		if(program[i] -> is_done())
			continue;
		// TO-DO remove if the loop starts at current_operation_index

		current_operation_index = i;
		
		program[i] -> start();
		full_program_iteration = program[i] -> execute();

		// NOTE: should not move to the next operation if the most recent operation failed
		if(!full_program_iteration)
			break;
	}


	if(full_program_iteration)
		for(int i = 0; i < program.size(); i++)
			program[i] -> reset();
	
}

void ExecutionManager::stop_execution()
{
	ROS_INFO("stop moving the robot at operation# %d", current_operation_index);

	if(!program.empty() && current_operation_index != STOPPED)
		program[current_operation_index] -> stop();
	current_operation_index = STOPPED;
}

}