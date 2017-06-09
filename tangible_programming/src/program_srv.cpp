#include <vector>

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
	ins.target.type = tangible_msgs::Target::POINT_LOCATION;
	ins.target.specified_point.point.x = 1;
	ins.target.specified_point.point.y = 1;
	ins.target.specified_point.point.z = 0;
	op.instructions.push_back(ins);

	// place
	ins.type = tangible_msgs::Instruction::PLACE;
	ins.target.type = tangible_msgs::Target::POINT_LOCATION;
	ins.target.specified_point.point.x = 2;
	ins.target.specified_point.point.y = 2;
	ins.target.specified_point.point.z = 0;
	op.instructions.push_back(ins);

	prg.operations.push_back(op);

	res.program = prg;
	
	ROS_INFO("program service callback finished");
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "get_program_server");

	ros::NodeHandle n;
	ros::ServiceServer server = n.advertiseService("get_program", program_callback);
	ROS_INFO("fake service to provide compiled programs");
	
	ros::spin();

	return 0;
}