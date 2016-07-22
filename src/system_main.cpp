#include "ros/ros.h"
#include "tangible/frame_transformer.h"
#include "tangible/tag_extractor.h"
#include "tangible/scene_parser.h"
#include "tangible/program.h"

//YSS just for testing
#include <vector>
#include <iostream>
#include <stdlib.h>

#include <geometry_msgs/PoseStamped.h>

int main (int argc, char** argv) {
	ros::init(argc, argv, "tangible_pbd");
	
	/*ros::NodeHandle node;
	tangible::FrameTransformer trns(node, "base_footprint");
	tangible::TagExtractor tagext(node);
	//tangible::SceneParser parser(node);
	tangible::Program program(tagext.get_tags());
	ros::spin();*/

	int SELECTION_TAG_NUM =4;
	int ACTION_TAG_NUM = 4;
	int NUMBER_TAG_NUM = 7;

	int SCENE_SIZE = 100;

	std::vector<tangible::Tag> tags;
	
	int instructionNum = 2;
	int selectionID, actionID, numberID;
	geometry_msgs::PoseStamped ps; double x, y, z;
	for(int i = 0; i < instructionNum; i++) {
		selectionID = std::rand()%SELECTION_TAG_NUM + 0;
		tangible::Tag selection;
		selection.setID(selectionID);
		std::cout << selection.getID() << ", ";
		tags.push_back(selection);

		actionID = std::rand()%ACTION_TAG_NUM + SELECTION_TAG_NUM + 1;
		x = i*2+5; y = i*2+5; z = 0;
		ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.position.z = z;
		tangible::Tag action;
		action.setID(actionID);
		action.setCenter(ps);
		std::cout << action.getID() 
		          << ": (" << x << ", " << y << ", " << z << "), ";
		tags.push_back(action);

		numberID = std::rand()%NUMBER_TAG_NUM + SELECTION_TAG_NUM + 1 + ACTION_TAG_NUM;
		ps.pose.position.x -= tangible::Tag::EDGE_SIZE; 
		tangible::Tag number;
		number.setID(numberID);
		number.setCenter(ps);
		number.setX(1, 0, 0);
		std::cout << number.getID() 
		          << ": (" << x-tangible::Tag::EDGE_SIZE << ", " << y << ", " << z << ") ,";
		tags.push_back(number);
		
		if(selectionID % 2 == 1) { // a region task
			x = i*3+10; y = i+10; z = 0;
			ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.position.z = z;
			tangible::Tag selection2nd;
			selection2nd.setID(SELECTION_TAG_NUM);
			selection2nd.setCenter(ps);
			std::cout << SELECTION_TAG_NUM
			          << ": (" << x << ", " << y << ", " << z << "), ";
			tags.push_back(selection2nd);
			
			ps.pose.position.x -= tangible::Tag::EDGE_SIZE; 
			number.setCenter(ps);
			std::cout << number.getID() 
		          << ": (" << x-tangible::Tag::EDGE_SIZE << ", " << y << ", " << z << ") ,";
			tags.push_back(number);
		}		
	}
	std::cout << "\n";
	
	tangible::Program program(tags);


	return 0;
}