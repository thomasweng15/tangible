#include "ros/ros.h"
#include "tangible/scene_parser.h"
#include "tangible/tag.h"
#include "tangible/program.h"
#include "tangible/visualizer.h"

#include <vector>
#include <iostream>

#include "geometry_msgs/PoseStamped.h"

#include "tangible/utils.h"

const int ARROW_ONLY = 1;
const int CORNER_ONLY = 2;
const int MIXED = 3;

const int SELECTION_TAG = 1;
const int ACTION_TAG = 2;
const int NUMBER_TAG = 3;
const int SELECTION_2ND_TAG = 4;
const int NUMBER_2ND_TAG = 5;

const int X_AXIS = 0;
const int Y_AXIS = 1;
const int Z_AXIS = 2;

void findOOI(std::vector<rapid::perception::Object> objects, tangible::Visualizer vis);

void make_arrow_aligned_below(std::vector<tangible::Tag>& tags);
void make_corner_aligned_above(std::vector<tangible::Tag>& tags);

tangible::Tag createTag(double x, double y, double z,
	                    tangible::Axis x_axis, tangible::Axis y_axis, tangible::Axis z_axis,
	                    int id);

std::string compileProgram(std::vector<tangible::Tag>& tags,
	                       std::vector<rapid::perception::Object> objects);

//NOTE make sure to play a rosbag file containing topic cloud_transformed before running this
int main (int argc, char** argv) {
	ros::init(argc, argv, "tangible_pbd_matching_test");

	const static int MAX_BLOCKING_THREAD_NUM = 5;
	
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(MAX_BLOCKING_THREAD_NUM);
	spinner.start();

	tangible::SceneParser parser(node);
	tangible::Visualizer vis(node, "base_footprint");

	ros::Duration(5).sleep();

	std::vector<tangible::Tag> tags;
	std::vector<rapid::perception::Object> objects;
	std::string output_msg = "cannot parse the scene";

	if(parser.isSuccessful()) {
		objects = parser.getObjects();
		findOOI(objects, vis);
		
		make_arrow_aligned_below(tags);
		output_msg = compileProgram(tags, objects);
		std::cout << output_msg << "\n";

		tags.clear();
		make_corner_aligned_above(tags);
		output_msg = compileProgram(tags, objects);
		std::cout << output_msg << "\n";
	}

	//ros::Rate interval(3);
	//while(ros::ok()) {
	//	if(parser.isSuccessful())
	//		vis.update(parser.getObjects());
	//	interval.sleep();
	//}

	ros::waitForShutdown();
	//ros::Duration(5).sleep();

	return 0;
}

// helps find the object of interest to use for creating tags
//NOTE in file 160801-1101-subset-cloud_trnsformed.bag this object is roughly at
//     (x, y, z) = (0.689311, -0.00294057, 0.822762) [wrt base_footprint]
void findOOI(std::vector<rapid::perception::Object> objects, tangible::Visualizer vis) {
	ROS_INFO("updating the scene with %d objects", (int)(objects.size()));
	vis.update(objects);
	ros::Duration(3).sleep();
	std::vector<rapid::perception::Object> objects_of_interest;
	for(int i = 0; i < objects.size(); i++) {
		std::cout << "object " << i << "\\";
		std::cout << "(" << objects[i].pose().pose.position.x << ", "
		                 << objects[i].pose().pose.position.y << ", "
		                 << objects[i].pose().pose.position.z << ")";
		if(tangible::inRange(-0.003, 0.003, objects[i].pose().pose.position.y)){
			std::cout << " --- is of interest.";
			objects_of_interest.push_back(objects[i]);
		}
		std::cout << "\n";
	}
	vis.clearScene();
	std::cout << "visualizing objects of interest\n";
	ros::Duration(3).sleep();
	vis.update(objects_of_interest);
}

void make_arrow_aligned_below(std::vector<tangible::Tag>& tags) {
	int selectionID, actionID, numberID;
	selectionID = tangible::Tag::SELECT_OBJECT_ID;
	actionID = tangible::Tag::TOP_PICK_ID;
	numberID = tangible::Tag::NUMBER_ID_MIN;

	double x, y, z;
	x = 0.689311; y = -0.00294057; z = 0.822762;

	tangible::Axis x_axis; x_axis.x = 1; x_axis.y = 0; x_axis.z = 0;
	tangible::Axis y_axis; y_axis.y = 0; y_axis.y = 1; y_axis.z = 0;
	tangible::Axis z_axis; z_axis.y = 0; z_axis.y = 0; z_axis.z = 1;

	// selection
	y -= tangible::Tag::ARROW_SELECTION_LEN;
	tags.push_back(createTag(x, y, z, x_axis, y_axis, z_axis, selectionID));

	// action
	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, x_axis, y_axis, z_axis, actionID));

	// number
	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, x_axis, y_axis, z_axis, numberID));
}

void make_corner_aligned_above(std::vector<tangible::Tag>& tags) {
	int selectionID, actionID, numberID;
	selectionID = tangible::Tag::SELECT_OBJECTS_ID;
	actionID = tangible::Tag::TOP_PICK_ID;
	numberID = tangible::Tag::NUMBER_ID_MIN;

	double x, y, z;
	x = 0.689311; y = -0.00294057; z = 0.822762;

	double w, h;
	w = 0.05; h = 0.1;

	tangible::Axis x_axis; x_axis.x = 1; x_axis.y = 0; x_axis.z = 0;
	tangible::Axis y_axis; y_axis.y = 0; y_axis.y = 1; y_axis.z = 0;
	tangible::Axis z_axis; z_axis.y = 0; z_axis.y = 0; z_axis.z = 1;

	// selection
	x -= w; y += h;
	tags.push_back(createTag(x, y, z, x_axis, y_axis, z_axis, selectionID));

	// action
	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, x_axis, y_axis, z_axis, actionID));

	// number
	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, x_axis, y_axis, z_axis, numberID));

	// secondary selection
	x = 0.689311; y = -0.00294057; z = 0.822762;
	x += w; y -= h;

	x_axis.x = 0; x_axis.y = -1; x_axis.z = 0;
	y_axis.y = 1; y_axis.y = 0; y_axis.z = 0;

	tags.push_back(createTag(x, y, z, x_axis, y_axis, z_axis, tangible::Tag::SELECTION_2ND_ID));

	// second number tag
	y += tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, x_axis, y_axis, z_axis, numberID));

}

tangible::Tag createTag(double x, double y, double z,
	                    tangible::Axis x_axis, tangible::Axis y_axis, tangible::Axis z_axis,
	                    int id) {
	tangible::Tag tag;
	geometry_msgs::PoseStamped ps;
	ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.position.z = z;

	tag.setCenter(ps);
	tag.setX(x_axis.x, x_axis.y, x_axis.z);
	tag.setY(y_axis.x, y_axis.y, y_axis.z);
	tag.setZ(z_axis.x, z_axis.y, z_axis.z);
	tag.setID(id);

	return tag;
}

std::string compileProgram(std::vector<tangible::Tag>& tags,
	                       std::vector<rapid::perception::Object> objects) {
	tangible::Program program(tags, objects);

	return program.error().empty() ? program.printInstructions() : program.error();			
}