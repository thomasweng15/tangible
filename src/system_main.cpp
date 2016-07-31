#include "ros/ros.h"
#include "tangible/frame_transformer.h"
#include "tangible/tag_extractor.h"
#include "tangible/scene_parser.h"
#include "tangible/program.h"
#include "tangible/visualizer.h"

//YSS for testing
#include <vector>
#include <iostream>
#include <stdlib.h>

#include "geometry_msgs/PoseStamped.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"
#include "Eigen/Geometry"
#include "pcl/filters/crop_box.h"

const int ARROW_ONLY = 1;
const int CORNER_ONLY = 2;
const int MIXED = 3;

const int SELECTION_TAG = 1;
const int ACTION_TAG = 2;
const int NUMBER_TAG = 3;
const int SELECTION_2ND_TAG = 4;
const int NUMBER_2ND_TAG = 5;

const int SELECTION_TAG_NUM = tangible::Tag::SELECTION_ID_MAX - tangible::Tag::SELECTION_ID_MIN + 1;
const int ACTION_TAG_NUM = tangible::Tag::ACTION_ID_MAX - tangible::Tag::ACTION_ID_MIN + 1;
const int NUMBER_TAG_NUM = tangible::Tag::NUMBER_ID_MAX - tangible::Tag::NUMBER_ID_MIN + 1;

const int X_AXIS = 0;
const int Y_AXIS = 1;
const int Z_AXIS = 2;

const double CLOUD_DENSITY = 0.1;

void testSyntheticSetup(int caseID, int instruction_num);
void make_no_instructions(std::vector<tangible::Tag>& tags,
	                      std::vector<rapid::perception::Object>& objects);
void make_instructions_no_selection(std::vector<tangible::Tag>& tags,
	                                std::vector<rapid::perception::Object>& objects);
void make_instructions_no_action(std::vector<tangible::Tag>& tags,
	                             std::vector<rapid::perception::Object>& objects);
void make_instructions_no_number(std::vector<tangible::Tag>& tags,
	                             std::vector<rapid::perception::Object>& objects);
void make_instructions_missing_action(std::vector<tangible::Tag>& tags,
	                                  std::vector<rapid::perception::Object>& objects);
void make_instructions_missing_2ndary(std::vector<tangible::Tag>& tags,
	                                  std::vector<rapid::perception::Object>& objects);
void make_instructions_missing_number(std::vector<tangible::Tag>& tags,
	                                  std::vector<rapid::perception::Object>& objects);
void make_instructions_skip_step(std::vector<tangible::Tag>& tags,
	                             std::vector<rapid::perception::Object>& objects);
void make_instructions_repeat_step(std::vector<tangible::Tag>& tags,
	                               std::vector<rapid::perception::Object>& objects);
void make_instruction_dangling_number(std::vector<tangible::Tag>& tags,
	                                  std::vector<rapid::perception::Object>& objects);
void make_instruction_place_1st(std::vector<tangible::Tag>& tags,
	                            std::vector<rapid::perception::Object>& objects);
void make_instruction_pick_2nd(std::vector<tangible::Tag>& tags,
	                           std::vector<rapid::perception::Object>& objects);
void make_instruction_dangling_action(std::vector<tangible::Tag>& tags,
		                              std::vector<rapid::perception::Object>& objects);
void make_instruction_dangling_selection(std::vector<tangible::Tag>& tags,
	                                     std::vector<rapid::perception::Object>& objects);
void make_instruction_invalid_pair(std::vector<tangible::Tag>& tags,
	                               std::vector<rapid::perception::Object>& objects);


void make_arrow_instructions(std::vector<tangible::Tag>& tags,
	                         std::vector<rapid::perception::Object>& objects);
void make_corner_instructions(std::vector<tangible::Tag>& tags,
	                          std::vector<rapid::perception::Object>& objects);
void make_colocated_instructions(std::vector<tangible::Tag>& tags,
	                             std::vector<rapid::perception::Object>& objects);


void tagIDs(int& selectionID, int& actionID, int& numberID, int step, int caseID);
tangible::Tag createTag(double x, double y, double z, int id, int type);
void setupAxis(tangible::Axis&, int type);
bool setupTag(tangible::Tag& t,
	          int id,
	          geometry_msgs::PoseStamped ps,
	          tangible::Axis a,
	          int axis_id);
rapid::perception::Object createObject(double x, double y, double z, int count, std::string name);
//YSS end for testing

int main (int argc, char** argv) {
	const static int MAX_BLOCKING_THREAD_NUM = 5;
	ros::init(argc, argv, "tangible_pbd");
	
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(MAX_BLOCKING_THREAD_NUM);
	spinner.start();

	tangible::FrameTransformer trns(node, "base_footprint");
	tangible::TagExtractor tagext(node);
	ros::Duration(5).sleep();
	tangible::Visualizer vis(node);
	//NOTE: wait is necessary here so tags are filled before the call to tagext.get_tags()
	//TO-DO change the whole architecture to avoid this race condition
	//      e.g. use ros::topic::waitForMessage<msg_type>("topic_name", timeOut);

	
	std::string err = "error";
	std::vector<tangible::Tag> tags;
	std::vector<rapid::perception::Object> objects;
	//do {		
		tags = tagext.get_tags();
		ROS_INFO("the following tags are detected:");
		for(int i = 0; i < tags.size(); i++)
			ROS_INFO("%s%s", tags[i].printID().c_str(), tags[i].printCenter().c_str());
		//tangible::SceneParser parser(node);
		//if(parser.isSuccessful) {
		    //objects = parser.getObjects();
			tangible::Program program(tags, objects);
			err = program.error();
			if(!err.empty()) {
				ROS_ERROR("\n%s", err.c_str());
				vis.clear();
			} else {
				ROS_INFO("\n%s", program.printInstructionTags().c_str());
				vis.update(program);
			}
		//}
	//} while(!err.empty());

	//TO-DO later on there should be a mechanism for refreshing a program
	//ros::spin();
	ros::waitForShutdown();

	/*YSS testing
	make_no_instructions(tags, objects); // prints ERROR - TAG GROUPING - no tags to group. 
	make_instructions_no_selection(tags, objects); // prints ERROR - TAG GROUPING - no selection tag.
	make_instructions_no_action(tags, objects); // prints ERROR - TAG GROUPING - no action tag.
	make_instructions_no_number(tags, objects); // prints ERROR - TAG GROUPING - no number tag.
	make_instructions_missing_action(tags, objects); // prints ERROR - TAG GROUPING - too few action or two many selection tags.
	make_instructions_missing_2ndary(tags, objects); // prints ERROR - TAG GROUPING - too many or too few secondary selection tags.
	make_instructions_missing_number(tags, objects); // prints ERROR - TAG GROUPING - too many or too few number tags.
	make_instructions_skip_step(tags, objects); // prints ERROR - TAG GROUPING - missing number tag.
	make_instructions_repeat_step(tags, objects); // prints ERROR - TAG GROUPING - too many repeated number tags.
	make_instruction_dangling_number(tags, objects); // prints ERROR - TAG GROUPING - singular number tag.
	make_instruction_place_1st(tags, objects); // prints ERROR - TAG GROUPING - expected a pick action.
	make_instruction_pick_2nd(tags, objects); // prints ERROR - TAG GROUPING - expected a place action.
	make_instruction_dangling_action(tags, objects); // prints ERROR - TAG GROUPING - singular action tag.
	make_instruction_dangling_selection(tags, objects); // prints ERROR - TAG GROUPING - dangling selection tag.
	make_instruction_invalid_pair(tags, objects); //prints ERROR - TAG GROUPING - tags inavlidly paired.

	make_arrow_instructions(tags, objects); // prints instructions
	make_corner_instructions(tags, objects); // prints instructions
	make_colocated_instructions(tags, objects); // prints instructions
	
	for(int i = 0; i < tags.size(); i++)
		std::cout << tags[i].printID() << tags[i].printCenter() << ", ";
	std::cout << "\n";
	
	std::cout << "\ncompiling the tags to build the program...\n";
	tangible::Program program(tags, objects);

	std::cout << (program.error().empty() ? program.printInstructionTags() : program.error()) << "\n";
	*/

	return 0;
}

void make_no_instructions(std::vector<tangible::Tag>& tags,
	                      std::vector<rapid::perception::Object>& objects) {
	// no tag is created and pushed to tags
	return;
}

void make_instructions_no_selection(std::vector<tangible::Tag>& tags,
	                                std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	// no selection tag is created and pushed to tags

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));
}

void make_instructions_no_action(std::vector<tangible::Tag>& tags,
	                             std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	// no action tag is created and pushed to tags

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));
}

void make_instructions_no_number(std::vector<tangible::Tag>& tags,
	                             std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	// no number tag is created and pushed to tags
	
}

void make_instructions_missing_action(std::vector<tangible::Tag>& tags,
	                                  std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	// instruction #2
	tagIDs(selectionID, actionID, numberID, 1, ARROW_ONLY);

	x = 7*tangible::Tag::EDGE_SIZE; y = 2*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	// no action tag is created and pushed to tags for the 2nd instruction

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));
}

void make_instructions_missing_2ndary(std::vector<tangible::Tag>& tags,
	                                  std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;
	double w = 2*tangible::Tag::EDGE_SIZE; double h = 3*tangible::Tag::EDGE_SIZE;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, CORNER_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	x = 4*tangible::Tag::EDGE_SIZE + w; y = 4*tangible::Tag::EDGE_SIZE - h;
	// no secondary selection tag and its associated number tag are added

	y += tangible::Tag::EDGE_SIZE;
	// no number tag
}

void make_instructions_missing_number(std::vector<tangible::Tag>& tags,
	                                  std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;
	double w = 2*tangible::Tag::EDGE_SIZE; double h = 3*tangible::Tag::EDGE_SIZE;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, CORNER_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	x = 4*tangible::Tag::EDGE_SIZE + w; y = 4*tangible::Tag::EDGE_SIZE - h;
	tags.push_back(createTag(x, y, z, tangible::Tag::SELECTION_2ND_ID, SELECTION_2ND_TAG));

	y += tangible::Tag::EDGE_SIZE;
	// no number tag
}

void make_instructions_skip_step(std::vector<tangible::Tag>& tags,
	                             std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	// instruction #2 --- wrongly numbered
	tagIDs(selectionID, actionID, numberID, 3, ARROW_ONLY);

	x = 7*tangible::Tag::EDGE_SIZE; y = 2*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	//TO-DO try out with the second instruction at (6, 5)*EDGE_SIZE
}

void make_instructions_repeat_step(std::vector<tangible::Tag>& tags,
	                               std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	// instruction #2
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);

	x = 7*tangible::Tag::EDGE_SIZE; y = 2*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	// instruction #3
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);

	x = 2*tangible::Tag::EDGE_SIZE; y = 7*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));
}

void make_instruction_dangling_number(std::vector<tangible::Tag>& tags,
	                                  std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	// instruction #2
	tagIDs(selectionID, actionID, numberID, 1, ARROW_ONLY);

	x = 7*tangible::Tag::EDGE_SIZE; y = 2*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));
	

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	y += tangible::Tag::EDGE_SIZE; // so the number is not properly aligned with action
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));
}

void make_instruction_place_1st(std::vector<tangible::Tag>& tags,
	                            std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);
	actionID = tangible::Tag::POSITION_ID; // place action

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));
}

void make_instruction_pick_2nd(std::vector<tangible::Tag>& tags,
	                           std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	// instruction #2
	tagIDs(selectionID, actionID, numberID, 1, ARROW_ONLY);
	actionID = tangible::Tag::SIDE_PICK_ID;

	x = 7*tangible::Tag::EDGE_SIZE; y = 2*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));
}

void make_instruction_dangling_action(std::vector<tangible::Tag>& tags,
	                                  std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= 1.5*tangible::Tag::EDGE_SIZE; // so the action is far from selection
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));
}

void make_instruction_dangling_selection(std::vector<tangible::Tag>& tags,
	                                     std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	// instruction #2 - colocated w/ instruction #1 (i.e. they share the selection tag)
	tagIDs(selectionID, actionID, numberID, 1, ARROW_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;

	y -= 2*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	// dangling selection tag
	x = 7*tangible::Tag::EDGE_SIZE; y = 2*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));
}

void make_instruction_invalid_pair(std::vector<tangible::Tag>& tags,
	                               std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	// instruction #2
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);

	x = 7*tangible::Tag::EDGE_SIZE; y = 2*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));
}

void make_arrow_instructions(std::vector<tangible::Tag>& tags,
	                         std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));
	//TO-DO add object

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	// instruction #2
	tagIDs(selectionID, actionID, numberID, 1, ARROW_ONLY);

	x = 7*tangible::Tag::EDGE_SIZE; y = 2*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));
	//TO-DO add object

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	//TO-DO try out with the second instruction at (6, 5)*EDGE_SIZE
}

void make_corner_instructions(std::vector<tangible::Tag>& tags,
	                          std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;
	double w = 2*tangible::Tag::EDGE_SIZE; double h = 3*tangible::Tag::EDGE_SIZE;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, CORNER_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));
	//TO-DO add object

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	x = 4*tangible::Tag::EDGE_SIZE + w; y = 4*tangible::Tag::EDGE_SIZE - h;
	tags.push_back(createTag(x, y, z, tangible::Tag::SELECTION_2ND_ID, SELECTION_2ND_TAG));

	y += tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_2ND_TAG));

	// instruction #2
	tagIDs(selectionID, actionID, numberID, 1, CORNER_ONLY);

	x = 10*tangible::Tag::EDGE_SIZE; y = 11*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));
	//TO-DO add object

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	x = 10*tangible::Tag::EDGE_SIZE + w; y = 11*tangible::Tag::EDGE_SIZE - h;
	tags.push_back(createTag(x, y, z, tangible::Tag::SELECTION_2ND_ID, SELECTION_2ND_TAG));

	y += tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_2ND_TAG));
}

void make_colocated_instructions(std::vector<tangible::Tag>& tags,
	                             std::vector<rapid::perception::Object>& objects) {
	int selectionID, actionID, numberID;
	double x, y, z = 0;
	double w = 2*tangible::Tag::EDGE_SIZE; double h = 3*tangible::Tag::EDGE_SIZE;

	// instruction #1
	tagIDs(selectionID, actionID, numberID, 0, ARROW_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));
	//TO-DO add object

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	// instruction #2
	tagIDs(selectionID, actionID, numberID, 1, CORNER_ONLY);

	x = 10*tangible::Tag::EDGE_SIZE; y = 11*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, selectionID, SELECTION_TAG));
	//TO-DO add object

	y -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	x = 10*tangible::Tag::EDGE_SIZE + w; y = 11*tangible::Tag::EDGE_SIZE - h;
	tags.push_back(createTag(x, y, z, tangible::Tag::SELECTION_2ND_ID, SELECTION_2ND_TAG));

	y += tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_2ND_TAG));

	// instruction #3 - colocated w/ instruction #1 (i.e. they share the selection tag)
	tagIDs(selectionID, actionID, numberID, 2, ARROW_ONLY);

	x = 4*tangible::Tag::EDGE_SIZE; y = 4*tangible::Tag::EDGE_SIZE;
	//TO-DO add object

	y -= 2*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));

	// instruction #4 - colocated w/ instruction #2 (i.e. they share the selection tag)
	tagIDs(selectionID, actionID, numberID, 3, CORNER_ONLY);

	x = 10*tangible::Tag::EDGE_SIZE; y = 11*tangible::Tag::EDGE_SIZE;
	//TO-DO add object

	y -= 2*tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, actionID, ACTION_TAG));

	x -= tangible::Tag::EDGE_SIZE;
	tags.push_back(createTag(x, y, z, numberID, NUMBER_TAG));
}

void tagIDs(int& selectionID, int& actionID, int& numberID, int step, int caseID) {
	selectionID = tangible::Tag::SELECTION_ID_MIN;
	switch(caseID) {
		case ARROW_ONLY:
			selectionID += (std::rand()%2)*2;
			break;
		case CORNER_ONLY:
			selectionID += (std::rand()%2)*2+1;
			break;
		default:
			selectionID += std::rand()%SELECTION_TAG_NUM;
			break;
	}

	actionID = tangible::Tag::ACTION_ID_MIN;
	actionID += std::rand()%2 + (step%2)*2;

	numberID = step + tangible::Tag::NUMBER_ID_MIN;
}

tangible::Tag createTag(double x, double y, double z, int id, int type) {
	geometry_msgs::PoseStamped ps;
	ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.position.z = z;

	tangible::Axis axis;
	setupAxis(axis, type);

	tangible::Tag tag;
	if(type == SELECTION_TAG || type == ACTION_TAG || type == SELECTION_2ND_TAG)
		setupTag(tag, id, ps, axis, Y_AXIS);
	else if(type == NUMBER_TAG || type == NUMBER_2ND_TAG)
		setupTag(tag, id, ps, axis, X_AXIS);

	return tag;
}

void setupAxis(tangible::Axis& axis, int type) {
	if(type == SELECTION_TAG || type == ACTION_TAG) {
		axis.x = 0; axis.y = 1; axis.z = 0;
	} else if(type == NUMBER_TAG || type == SELECTION_2ND_TAG) {
		axis.x = 1; axis.y = 0; axis.z = 0;
	} else if(type == NUMBER_2ND_TAG) {
		axis.x = 0; axis.y = -1; axis.z = 0;
	}
}

bool setupTag(tangible::Tag& t,
	          int id,
	          geometry_msgs::PoseStamped ps,
	          tangible::Axis a,
	          int axis_id) {
	t.setID(id);
	t.setCenter(ps);
	if(axis_id == X_AXIS)
		t.setX(a.x, a.y, a.z);
	else if(axis_id == Y_AXIS)
		t.setY(a.x, a.y, a.z);
	else if(axis_id == Z_AXIS)
		t.setZ(a.x, a.y, a.z);
	else {
		std::cout << "invalid axis ID " << axis_id;
		return false;
	}
	return true;
}

//YSS unless I figure out a way to fake the indices, I cannot use this
rapid::perception::Object createObject(double x, double y, double z, int count, std::string name) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(int i = 0; i < count; i++)
		for(int j = 0; j < count; j++)
			for(int k = 0; k < count; k++)
				cloud->push_back(pcl::PointXYZRGB(x + i * CLOUD_DENSITY,
				                                  y + j * CLOUD_DENSITY,
				                                  z + k * CLOUD_DENSITY));
	rapid::perception::Object obj;
	pcl::PointIndices::Ptr indices;
	obj.SetCloud(cloud, indices);
	obj.set_name(name);

	//YSS run-time error likely because I don't set the point indices when I try using obj
	//std::cout << obj.GetCloud()->points.size() << "\n";

	return obj;
	//YSS not sure if this is an organized point cloud
}