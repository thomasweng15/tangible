#include "ros/ros.h"
#include "tangible/frame_transformer.h"
#include "tangible/tag_extractor.h"
#include "tangible/scene_parser.h"
#include "tangible/program.h"

//YSS for testing
#include <vector>
#include <iostream>
#include <stdlib.h>

#include <geometry_msgs/PoseStamped.h>

void testSyntheticValid(int caseID, int instruction_num);
void tagIDs(int& selectionID, int& actionID, int& numberID, int step, int caseID);
bool tagSetup(tangible::Tag& t,
	          int id,
	          geometry_msgs::PoseStamped ps,
	          tangible::Axis a,
	          int axis_id);

const int ARROW_ONLY = 1;
const int REGION_ONLY = 2;
const int MIXED = 3;

const int SELECTION_TAG_NUM = tangible::Tag::SELECTION_ID_MAX - tangible::Tag::SELECTION_ID_MIN + 1;
const int ACTION_TAG_NUM = tangible::Tag::ACTION_ID_MAX - tangible::Tag::ACTION_ID_MIN + 1;
const int NUMBER_TAG_NUM = tangible::Tag::NUMBER_ID_MAX - tangible::Tag::NUMBER_ID_MIN + 1;

const int SCENE_SIZE = 50;

const int MAX_REGION_LEN = 10;

const int X_AXIS = 0;
const int Y_AXIS = 1;
const int Z_AXIS = 2;

//YSS end for testing

int main (int argc, char** argv) {
	ros::init(argc, argv, "tangible_pbd");
	
	/*ros::NodeHandle node;
	tangible::FrameTransformer trns(node, "base_footprint");
	tangible::TagExtractor tagext(node);
	//tangible::SceneParser parser(node);
	tangible::Program program(tagext.get_tags());
	ros::spin();*/

	//YSS testing
	int caseID, instruction_num = 2;
	do {
		std::cout << "enter the desired test case ID or 0 to quit\n";
		std::cout << "\t  1: only arrow selection\n";
		std::cout << "\t  2: only region selection\n";
		std::cout << "\t  3: mixed arrow and region selections\n";
		std::cin >> caseID;
		if(caseID <= 0)
			break;
		std::cout << "enter the number of instructions ( <= 7 )\n";
		std::cin >> instruction_num;
		if(instruction_num < 1 || instruction_num > 7) {
			std::cout << "\t number of instructions can be {1, 2, 3, 4, 5, 6, 7}\n";
			continue;
		}
		testSyntheticValid(caseID, instruction_num);
	} while(caseID > 0);

	return 0;
}

void testSyntheticValid(int caseID, int instruction_num) {
	std::vector<tangible::Tag> tags;
	
	int selectionID, actionID, numberID;
	geometry_msgs::PoseStamped ps; double x, y, z = 0; tangible::Axis axis;
	for(int i = 0; i < instruction_num; i++) {
		tagIDs(selectionID, actionID, numberID, i, caseID);

		x = std::rand()%SCENE_SIZE + 4*tangible::Tag::EDGE_SIZE;
		y = std::rand()%SCENE_SIZE + 4*tangible::Tag::EDGE_SIZE;
		ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.position.z = z;
		axis.x = 0; axis.y = 1; axis.z = 0;
		tangible::Tag selection;
		if(!tagSetup(selection, selectionID, ps, axis, Y_AXIS))	break;
		std::cout << selection.printID() << ": " << selection.printCenter() << ", ";
		tags.push_back(selection);

		
		ps.pose.position.y -= tangible::Tag::EDGE_SIZE;
		axis.x = 0; axis.y = 1; axis.z = 0;
		tangible::Tag action;
		if(!tagSetup(action, actionID, ps, axis, Y_AXIS)) break;
		std::cout << action.printID() << ": " << action.printCenter() << ", ";
		tags.push_back(action);

		
		ps.pose.position.x -= tangible::Tag::EDGE_SIZE; 
		axis.x = 1; axis.y = 0; axis.z = 0;
		tangible::Tag number;
		if(!tagSetup(number, numberID, ps, axis, X_AXIS)) break;
		std::cout << number.printID() << ": " << number.printCenter() << ", ";
		tags.push_back(number);
		
		if(selectionID == tangible::Tag::SELECT_REGION_ID ||
		   selectionID == tangible::Tag::SELECT_OBJECTS_ID) { // a region task
			int w = std::rand()%MAX_REGION_LEN; x += w; 
			int h = std::rand()%MAX_REGION_LEN; y -= h;
			ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.position.z = z;
			axis.x = 1; axis.y = 0; axis.z = 0;
			tangible::Tag selection2nd;
			if(!tagSetup(selection2nd, tangible::Tag::SELECTION_2ND_ID, ps, axis, Y_AXIS)) break;
			std::cout << selection2nd.printID() << ": " << selection2nd.printCenter() << ", ";
			tags.push_back(selection2nd);
			
			ps.pose.position.y += tangible::Tag::EDGE_SIZE;
			axis.x = 0; axis.y = -1; axis.z = 0;
			if(!tagSetup(number, numberID, ps, axis, X_AXIS)) break;
			std::cout << number.printID() << ": " << number.printCenter() << ", ";
			tags.push_back(number);
		}		
	}
	std::cout << "\ncompiling the tags to build the program...\n";
	
	tangible::Program program(tags);
}

void tagIDs(int& selectionID, int& actionID, int& numberID, int step, int caseID) {
	selectionID = tangible::Tag::SELECTION_ID_MIN;
	switch(caseID) {
		case ARROW_ONLY:
			selectionID += (std::rand()%2)*2;
			break;
		case REGION_ONLY:
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

bool tagSetup(tangible::Tag& t,
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