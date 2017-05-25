#include "ros/ros.h"

// msg's and srv's
#include "tangible/GetProgram.h"
#include "tangible/Program.h"
#include "tangible/Operation.h"
#include "tangible/Instruction.h"

bool program_callback (tangible::GetProgram::Request& req, tangible::GetProgram::Response& res)
{
	tangible::Program prg;
	tangible::Operation op;
	tangible::Instruction ins;

	// pick
	ins.type = tangible::Instruction::PICK;
	ins.act_at.type = tangible::ActAt::POINT_LOCATION;
	ins.act_at.specified_point.point.x = 0;
	ins.act_at.specified_point.point.y = 0;
	ins.act_at.specified_point.point.z = 0;
	op.instructions.push_back(ins);


	ins.type = tangible::Instruction::PLACE;
	ins.act_at.type = tangible::ActAt::POINT_LOCATION;
	ins.act_at.specified_point.point.x = 1;
	ins.act_at.specified_point.point.y = 1;
	ins.act_at.specified_point.point.z = 1;
	op.instructions.push_back(ins);

	prg.operations.push_back(op);

	res.program = prg;
	
	ROS_INFO("service callback finished");
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "get_program_server");

	ros::NodeHandle n;
	ros::ServiceServer srv = n.advertiseService("get_program", program_callback);
	ROS_INFO("fake service to provide compiled programs");
	ros::spin();

	return 0;
}