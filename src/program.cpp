#include "tangible/program.h"

#include <algorithm>
#include <math.h>

#include "Eigen/Geometry"
#include "pcl/filters/crop_box.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "tangible/utils.h"

namespace tangible {

Program::Program(std::vector<Tag>& tgs, std::vector<rapid::perception::Object> objs) { 
	tags = tgs;
	objects = objs;
	if(tag2Instruction())
		matchObjects();
}
Program::~Program() {}

void Program::refresh(std::vector<Tag>& tgs, std::vector<rapid::perception::Object> objs) {
	tags = tgs;
	objects = objs;
	if(!tag2Instruction()) return;
	matchObjects();
}

//grouping tags to form instructions:
//  - sort tags ascendingly based on id
//  - find the range for selection, action, and number tags
//  - for each pair of numbers and actions/secodndary selections tags
//      - find the vector connecting the centers of the two tags and normalize it
//      - consider the two as grouped if
//          - the center to center distance is ~TAG_EDGE
//          - the dot product of the normalized center to center and the x-axis is ~1
//            alternativly, the dot product of x-axes of the two tags is ~1
//      ! the grouping is 1-1 => visited (i.e. already grouped) can be removed from the pool
//  - for each pair of actions and selections tags
//      similar to number-action/secondary selection but should find the minimum 
//      center-2-center distance and should check alignment with y-axis
//  - number tags with equal id's are selection and secondary selection that should be grouped
//  - instructions are formed based on the tags grouped together
bool Program::tag2Instruction() {
	instructions.clear();
	error_msg.clear();
	//NOTE: this ensures no instruction is formed for invalid tag settings

	int tag_num = tags.size();
	if(tag_num == 0) {
		error_msg = "ERROR - TAG GROUPING - no tags to group.";
		std::cout << error_msg << "\n";
		return false;
	}

	std::sort(tags.begin(), tags.end());

	int selection_count = 0;
	int selection2nd_count = 0;
	int action_count = 0;
	int number_count = 0;
	int other_count = 0;
	int regionID_count = 0;
	int grouped[tag_num];
	for(int i = 0; i < tag_num; i++) {
		if(inRange(Tag::SELECTION_ID_MIN, Tag::SELECTION_ID_MAX, tags.at(i).getID()))
			selection_count++;
		else if(tags.at(i).getID() == Tag::SELECTION_2ND_ID)
			selection2nd_count++;
		else if(inRange(Tag::ACTION_ID_MIN, Tag::ACTION_ID_MAX, tags.at(i).getID()))
			action_count++;
		else if(inRange(Tag::NUMBER_ID_MIN, Tag::NUMBER_ID_MAX, tags.at(i).getID()))
			number_count++;
		else if(tags.at(i).getID() > Tag::NUMBER_ID_MAX)
			other_count++;

		if(tags.at(i).getID() == Tag::SELECT_REGION_ID ||
		   tags.at(i).getID() == Tag::SELECT_OBJECTS_ID)
			regionID_count++;
		
		grouped[i] = -1;
	}

	if(selection_count == 0) {
		error_msg = "ERROR - TAG GROUPING - no selection tag.";
		std::cout << error_msg << "\n";
		return false;
	}

	if(action_count == 0) {
		error_msg = "ERROR - TAG GROUPING - no action tag.";
		std::cout << error_msg << "\n";
		return false;
	}

	if(number_count == 0) {
		error_msg = "ERROR - TAG GROUPING - no number tag.";
		std::cout << error_msg << "\n";
		return false;
	}

	if(action_count < selection_count) {
		error_msg = "ERROR - TAG GROUPING - too few action or two many selection tags.";
		std::cout << error_msg << "\n";
		return false;
	}

	if(selection2nd_count != regionID_count) {
		error_msg = "ERROR - TAG GROUPING - too many or too few secondary selection tags.";
		std::cout << error_msg << "\n";
		return false;
	}

	if(selection2nd_count + action_count != number_count) {
		error_msg = "ERROR - TAG GROUPING - too many or too few number tags.";
		std::cout << error_msg << "\n";
		return false;
	}

	// index where selection tags start
	int selection_ind = 0;
	// index where secondary selection tags start
	int selection2nd_ind = selection_ind + selection_count; 
	// index where action tags start
	int action_ind = selection2nd_ind + selection2nd_count;
	// index where number tags start
	int number_ind = action_ind + action_count;
	//TO-DO to handle additional tags (e.g. loop)
	int other_ind  = number_ind + number_count;

	int curr = Tag::NUMBER_ID_MIN;
	for(int i = number_ind; i < other_ind - 1; i++) {
		if(tags.at(i).getID() != curr) {
			error_msg = "ERROR - TAG GROUPING - missing number tag.";
			std::cout << error_msg << "\n";
			return false;
		}
		if(tags.at(i).getID() == tags.at(i-1).getID() &&
		   tags.at(i).getID() == tags.at(i+1).getID()) {
		// NOTE: at this point there is at least one selection and one action tags so 
		// number_ind >= 2 and i-1 will be valid.
			error_msg = "ERROR - TAG GROUPING - too many repeated number tags.";
			std::cout << error_msg << "\n";
			return false;
		}

		if(tags.at(i).getID() != tags.at(i+1).getID())
			curr++;
	}

	//std::cout << "selection tags @ " << selection_ind << ", " 
	//          << "2ndary selection tags @ " << selection2nd_ind << ", " 
	//          << "ation tags @ " << action_ind << ", " 
	//          << "number tags @ " << number_ind << "\n";

	//for(int i = 0; i < tag_num; i++) {
	//	if(i == selection_ind || i == selection2nd_ind || i == action_ind || i == number_ind)
	//		std::cout << "| ";
	//	else
	//		std::cout << ", ";
	//	std::cout << tags.at(i).getID();
	//}
	//std::cout << " |\n";

	for(int i = number_ind; i < other_ind; i++) {
		Tag number = tags.at(i);
		
		//std::cout << "number " 
		//          << number.printID() 
		//          << number.printCenter()
		//          << " at " << i << " to\n";
		
		for(int j = selection2nd_ind; j < number_ind; j++) {
			if(grouped[j] > -1) // action/secondary selection tag is already grouped
				continue;

			Tag action_or_2ndary = tags.at(j);
			
			//std::cout << "\t action or secondary selection  " 
			//          << action_or_2ndary.printID()
			//          << action_or_2ndary.printCenter()
			//          << " at " << j << "\n";
			
			double distance = number.dist(action_or_2ndary);
			
			//std::cout << "\t\tdistance: " << distance << "\n";
			
			if(!inRange(Tag::EDGE_SIZE - DIST_ERR_MARGIN,
				        Tag::EDGE_SIZE + DIST_ERR_MARGIN,
				        distance)) // action/secondary selection tag is too close/far
				continue;
			
			// normalized center-to-center vector
			Eigen::Vector3d n2a = number.vect(action_or_2ndary) / distance;

			Eigen::Vector3d ux = number.getXvect();
			
			//std::cout << "\t\tx axis: " << ux.transpose() << "\n";
			
			double inner_product = ux.dot(n2a);
			
			//std::cout << "\t\tinner_product: " << inner_product << "\n";
			
			if(!inRange(1 - ROTATE_ERR_MARGIN,
			            1 + ROTATE_ERR_MARGIN,
			            inner_product)) //action/secondary selection tag is not aligned w/ x-axis
				continue;
			
			// number tag is grouped with action/secondary selection tag
			grouped[i] = j; 
			grouped[j] = i;
			
			//std::cout << "number at " << i << " grouped with action or secondary selection at at " << j << "\n";
			
			break;
		}

		if(grouped[i] == -1) {
			error_msg = "ERROR - TAG GROUPING - singular number tag.";
			std::cout << error_msg << "\n";
			return false;
		}

		//NOTE: decided to enforce the following:
		//  - the even steps are pick actions and the odd steps are place actions.
		//    This allows us to consider each pick and the subsequent place as a block
		//    We cannot thus accpet nested blocks to support such cases as picking up a tool
		//    for later pick&place (later pick&place blocks are nested in tool pickup block)
		//TO-DO once decided to support nested blocks should remove this check

		if(number.getID()%2 == 1 && 
		   (tags.at(grouped[i]).getID() != Tag::TOP_PICK_ID ||
		   	tags.at(grouped[i]).getID() != Tag::SIDE_PICK_ID ||
		   	tags.at(grouped[i]).getID() != Tag::SELECTION_2ND_ID)) {
			error_msg = "ERROR - TAG GROUPING - expected a pick action." ;
			std::cout << error_msg << "\n";
			return false;
		}

		if(number.getID()%2 == 0 && 
		   (tags.at(grouped[i]).getID() != Tag::POSITION_ID ||
		   	tags.at(grouped[i]).getID() != Tag::DROP_ID ||
		   	tags.at(grouped[i]).getID() != Tag::SELECTION_2ND_ID)) {
			error_msg = "ERROR - TAG GROUPING - expected a place action." ;
			std::cout << error_msg << "\n";
			return false;
		}
	}

	for(int i = selection2nd_ind; i < action_ind; i++)
		if(grouped[i] == -1) {
			error_msg = "ERROR - TAG GROUPING - secondary selection tag not numbered.";
			std::cout << error_msg << "\n";
			return false;
		}

	for(int i = action_ind; i < number_ind; i++)
		if(grouped[i] == -1) {
			error_msg = "ERROR - TAG GROUPING - action tag not numbered.";
			std::cout << error_msg << "\n";
			return false;
		}

	for(int i = action_ind; i < number_ind; i++) {
		Tag action = tags.at(i);

		//std::cout << "action " 
		//          << action.printID()
		//          << action.printCenter()
		//          << " at " << i 
		//          << " with y-axis " << action.getYvect().transpose() << "\n";

		double minDist = MAX_WORKSPACE_DIST; int temp_grouped = -1;
		for(int j = selection_ind; j < selection2nd_ind; j++) {
			Tag selection = tags.at(j);

			//std::cout << "\tselection " 
			//          << selection.printID()
			//          << selection.printCenter()
			//          << " at " << j << "\n";

			double distance = action.dist(selection);

			//std::cout << "\t\tdistance: " << distance << "\n";

			// normalized center-to-center vector
			Eigen::Vector3d a2s = action.vect(selection) / distance;

			double quantizedDist = round(distance/Tag::EDGE_SIZE);		
			
			//std::cout << "\tideal distance: " << quantizedDist*Tag::EDGE_SIZE << "\n";

			if(!inRange(quantizedDist*Tag::EDGE_SIZE - DIST_ERR_MARGIN,
			            quantizedDist*Tag::EDGE_SIZE + DIST_ERR_MARGIN,
			            distance)) // distance of selection tag is not a multiple of EDGE_SIZE
				continue;

			Eigen::Vector3d uy = action.getYvect();
			double inner_product = uy.dot(a2s);
			if(!inRange(1 - ROTATE_ERR_MARGIN,
				        1 + ROTATE_ERR_MARGIN,
				        inner_product)) // selection tag is not aligned w/ y-axis
				continue;
			
			if(distance < minDist) {
				minDist = distance;
				temp_grouped = j;
			}
		}

		//std::cout << "\tmin distance: " << minDist << " with tag at " << temp_grouped << "\n";

		if(temp_grouped == -1) {
			error_msg = "ERROR - TAG GROUPING - singular action tag.";
			std::cout << error_msg << "\n";
			return false;
		}

		//std::cout << "action at " << i << " grouped with selection at " << temp_grouped << "\n";

		grouped[i] = temp_grouped;
		if(grouped[temp_grouped] == -1)
			grouped[temp_grouped] = i;
	}

	for(int i = selection_ind; i < selection2nd_ind; i++)
		if(grouped[i] == -1) {
			error_msg = "ERROR - TAG GROUPING - singular selection tag.";
			std::cout << error_msg << "\n";
			return false;
		}

	//TO-DO return false for the following error cases
	//   - the order of action tags grouped w/ the same selection tag does not follow
	//     their distances

	for(int i = number_ind; i < other_ind-1; i++) {
		Tag num1 = tags.at(i);
		Tag num2 = tags.at(i+1);
		if(num1.getID() == num2.getID()) {
			
			//std::cout << num1.printID() << " @" << i 
			//          << " --> " << tags.at(grouped[i]).printID() << " @" << grouped[i];
			//std::cout << " ---- ";
			//std::cout << num2.printID() << " @" << i+1 
			//          << " --> " << tags.at(grouped[i+1]).printID() << " @" << grouped[i+1];
			//std::cout << "\n";
			
			// of two successive tags with the same id, one is grouped with an action and
	        // another with a secondary selection tool
			if((tags.at(grouped[i]).getID() == Tag::SELECTION_2ND_ID &&
			    tags.at(grouped[i+1]).getID() == Tag::SELECTION_2ND_ID) ||
			   (tags.at(grouped[i]).getID() != Tag::SELECTION_2ND_ID &&
			    tags.at(grouped[i+1]).getID() != Tag::SELECTION_2ND_ID))  {
				error_msg = "ERROR - TAG GROUPING - region tag invalidly numbered.";
				std::cout << error_msg << "\n";
				return false;
			}

			if(tags.at(grouped[i]).getID() == Tag::SELECTION_2ND_ID) {
				grouped[grouped[i]] = grouped[grouped[i+1]];
				std::cout << "secondary selection at " << i 
				          << " grouped with selectio at" << grouped[grouped[i+1]] << "\n";
			} else {
				grouped[grouped[i+1]] = grouped[grouped[i]];
				//std::cout << "secondary selection at " << i+1 
				//          << " grouped with selection at " << grouped[grouped[i]] << "\n";
			}
		}
	}

	//TO-DO return false for the following error cases
	//   - the number grouped w/ the secondary selection tag is not equal to the smallest
	//     number grouped w/ an action grouped with the primary selection

	int instruction_num = action_count;
	for(int i = 0; i < instruction_num; i++) {
		Instruction instruction;
		instructions.push_back(instruction);
	}
	
	for(int i = number_ind; i < other_ind; i++) {
		int index, action_at, selection_at;
		index = tags.at(i).getID() - Tag::NUMBER_ID_MIN;
		Instruction instruction = instructions.at(index);

		action_at = grouped[i];
		selection_at = grouped[action_at];

		if(tags.at(action_at).getID() == Tag::SELECTION_2ND_ID) {
			instruction.selection2nd = tags.at(action_at);
		} else {
			instruction.number = tags.at(i);
			instruction.action = tags.at(action_at);
			instruction.selection = tags.at(selection_at);
		}

		instructions.at(index) = instruction;
	}

	if(instructions.at(0).action.getID() != Tag::SIDE_PICK_ID &&
	   instructions.at(0).action.getID() != Tag::TOP_PICK_ID) {
		error_msg = "ERROR - TAG GROUPING - invalid first action (not a pick).";
		std::cout << error_msg << "\n";
		instructions.clear();
		return false;
	}

	for(int i = 0; i < instructions.size(); i++) {
		if((instructions.at(i).selection.getID() == Tag::SELECT_REGION_ID ||
		    instructions.at(i).selection.getID() == Tag::SELECT_OBJECTS_ID) &&
		   instructions.at(i).selection2nd.getID() == -1) {
		   	error_msg = "ERROR - TAG GROUPING - missing secondary selection tag.";
			std::cout << error_msg << "\n";
			instructions.clear();
			return false;
		}
	}

	std::cout << "\tnumber" << "\t\tselection" << "\tsecondary" << "\taction\n";
	for(int i = 0; i < instruction_num; i++) {
		Instruction instruction = instructions.at(i);
		std::cout << i
		          << "\t" << instruction.number.printID() << instruction.number.printCenter()
		          << "\t" << instruction.selection.printID() << instruction.selection.printCenter()
		          << "\t" << instruction.selection2nd.printID() << instruction.selection2nd.printCenter()
		          << "\t" << instruction.action.printID() << instruction.action.printCenter() << "\n";
	}

	//TO-DO handling additional other tags
	return true;
}

// matching objects to instructions if necessary
// for each instruction with a relative selection element
//    - if SELECT_OBJECT_ID, for each object
//        - copy the point cloud in separate variable so the original cloud is intact
//        - use CropBox to find the overlap of the cloud with the box in front of the arrow
//        - count the number of points remianing after the filter
//     keep track of the max overlap. Match the object with the max overlap to the instruction
//     if the overlap is larger that a threshold. 
//     error if no object with THRESHOLD points is found: no object to select
//    - if SELECT_OBJECTS_ID: match to all objects whose point cloud is fully contaied
//      within a region.
bool Program::matchObjects() {
	for(int i = 0; i < instructions.size(); i++) {
		Tag selection = instructions.at(i).selection;
		if(selection.getID() == Tag::SELECT_OBJECT_ID) {
			Eigen::Vector4f tip;
			tip << selection.getCenter().x,
			       selection.getCenter().y,
			       selection.getCenter().z,
			       1;
			Eigen::Vector4f temp;
			//temp << 
			Eigen::Vector4f min, max;

			int max_overlap = -1; int max_overlap_index = -1;
			for(int j = 0; j < objects.size(); j++) {
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(objects.at(j).GetCloud());
				
				pcl::CropBox<pcl::PointXYZRGB> cbox;
				cbox.setInputCloud(cloud);
				cbox.setMin(min);
				cbox.setMax(max);
				cbox.filter(*cloud); //YSS will this change object's original point cloud?
				
				if(cloud->points.size() >= MIN_POINTCLOUD_OVERLAP &&
				   cloud->points.size() > max_overlap) {
					max_overlap = cloud->points.size();
					max_overlap_index = j;
				}
			}

			if(max_overlap_index == -1) {
				error_msg = "ERROR - TAG GROUPING - no object to select.";
				std::cout << error_msg << "\n";
				return false;
			}

			instructions.at(i).objects.push_back(objects.at(max_overlap_index));
		} else if (instructions.at(i).selection.getID() == Tag::SELECT_OBJECTS_ID){
			// for each object
			//    - test if point cloud is fully within the ROI
			//      can use CropBox and compare the point before and after
		}
	}
	return true;
}

std::string Program::error() { return error_msg; }

}
