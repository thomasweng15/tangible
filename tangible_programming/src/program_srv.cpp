#include "ros/ros.h"

// msg's and srv's
#include "tangible_msgs/GetProgram.h"
#include "tangible_msgs/Program.h"
#include "tangible_msgs/Operation.h"
#include "tangible_msgs/Instruction.h"

bool program_callback (tangible_msgs::GetProgram::Request& req, tangible_msgs::GetProgram::Response& res)
{
	tangible_msgs::Program prg;
	tangible_msgs::Operation op;
	tangible_msgs::Instruction ins;

	// pick
	ins.type = tangible_msgs::Instruction::PICK;
	ins.act_at.type = tangible_msgs::ActAt::POINT_LOCATION;
	ins.act_at.specified_point.point.x = 0;
	ins.act_at.specified_point.point.y = 0;
	ins.act_at.specified_point.point.z = 0;
	op.instructions.push_back(ins);


	ins.type = tangible_msgs::Instruction::PLACE;
	ins.act_at.type = tangible_msgs::ActAt::POINT_LOCATION;
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