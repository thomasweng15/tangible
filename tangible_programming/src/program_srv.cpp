#include <vector>

#include "ros/ros.h"

// msg's and srv's
#include "tangible_msgs/GetProgram.h"
#include "tangible_msgs/Program.h"
#include "tangible_msgs/Operation.h"
#include "tangible_msgs/Instruction.h"

#include "geometry_msgs/PointStamped.h"

bool program_callback (tangible_msgs::GetProgram::Request& req, tangible_msgs::GetProgram::Response& res)
{
	tangible_msgs::Program prg;
	tangible_msgs::Operation op;
	tangible_msgs::Instruction ins;

	// // pick(point) & place(point)
	// // pick
	// ins.type = tangible_msgs::Instruction::PICK;
	// ins.target.type = tangible_msgs::Target::POINT_LOCATION;
	// ins.target.specified_point.point.x = 1;
	// ins.target.specified_point.point.y = 1;
	// ins.target.specified_point.point.z = 0;
	// op.instructions.push_back(ins);

	// // place
	// ins.type = tangible_msgs::Instruction::PLACE;
	// ins.target.type = tangible_msgs::Target::POINT_LOCATION;
	// ins.target.specified_point.point.x = 2;
	// ins.target.specified_point.point.y = 2;
	// ins.target.specified_point.point.z = 0;
	// op.instructions.push_back(ins);

	// // pick(point) & place(object)
	// // pick
	// ins.type = tangible_msgs::Instruction::PICK;
	// ins.target.type = tangible_msgs::Target::POINT_LOCATION;
	// ins.target.specified_point.point.x = 1;
	// ins.target.specified_point.point.y = 1;
	// ins.target.specified_point.point.z = 0;
	// op.instructions.push_back(ins);

	// // place
	// ins.type = tangible_msgs::Instruction::PLACE;
	// ins.target.type = tangible_msgs::Target::OBJECT_SELECTOR;
	// ins.target.selected_object.bounding_box.dimensions.x = 2;
	// ins.target.selected_object.bounding_box.dimensions.y = 2;
	// ins.target.selected_object.bounding_box.dimensions.z = 2;
	// op.instructions.push_back(ins);

	// // pick(point) & place(region)
	// // pick
	// ins.type = tangible_msgs::Instruction::PICK;
	// ins.target.type = tangible_msgs::Target::POINT_LOCATION;
	// ins.target.specified_point.point.x = 1;
	// ins.target.specified_point.point.y = 4;
	// ins.target.specified_point.point.z = 0;
	// op.instructions.push_back(ins);

	// // place
	// geometry_msgs::PointStamped corner;
	// ins.type = tangible_msgs::Instruction::PLACE;
	// ins.target.type = tangible_msgs::Target::REGION;
	// corner.point.x = 0; corner.point.y = 0; corner.point.z = 0;
	// ins.target.region_corners.push_back(corner);
	// corner.point.x = 5; corner.point.y = 2; corner.point.z = 0;
	// ins.target.region_corners.push_back(corner);
	// op.instructions.push_back(ins);

	// // pick(object) & place(point)
	// // pick
	// ins.type = tangible_msgs::Instruction::PICK;
	// ins.target.type = tangible_msgs::Target::OBJECT_SELECTOR;
	// ins.target.selected_object.bounding_box.dimensions.x = 2;
	// ins.target.selected_object.bounding_box.dimensions.y = 2;
	// ins.target.selected_object.bounding_box.dimensions.z = 2;
	// op.instructions.push_back(ins);

	// // place
	// ins.type = tangible_msgs::Instruction::PLACE;
	// ins.target.type = tangible_msgs::Target::POINT_LOCATION;
	// ins.target.specified_point.point.x = 1;
	// ins.target.specified_point.point.y = 1;
	// ins.target.specified_point.point.z = 0;
	// op.instructions.push_back(ins);

	// pick(region) & place(region)
	geometry_msgs::PointStamped corner;
	// pick
	ins.type = tangible_msgs::Instruction::PICK;
	ins.target.type = tangible_msgs::Target::REGION;
	corner.point.x = 0; corner.point.y = 0; corner.point.z = 0;
	ins.target.region_corners.push_back(corner);
	corner.point.x = 5; corner.point.y = 2; corner.point.z = 0;
	ins.target.region_corners.push_back(corner);
	op.instructions.push_back(ins);

	// place
	ins.type = tangible_msgs::Instruction::PLACE;
	ins.target.type = tangible_msgs::Target::REGION;
	ins.target.region_corners.clear();
	corner.point.x = 1; corner.point.y = 5; corner.point.z = 0;
	ins.target.region_corners.push_back(corner);
	corner.point.x = 3; corner.point.y = 7; corner.point.z = 0;
	ins.target.region_corners.push_back(corner);
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