#include "tangible/program.h"

#include <algorithm>
#include <math.h>
#include <sstream>

#include "Eigen/Geometry"

#include "pcl/point_cloud.h"

#include "tangible/utils.h"

namespace tangible {

Program::Program(std::vector<Tag>& tgs, std::vector<rapid::perception::Object> objs) { 
	tags = tgs;
	objects = objs;
	tag2Instruction();
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
		if(inRange(Tag::SELECTION_ID_MIN, Tag::SELECTION_ID_MAX, tags[i].getID()))
			selection_count++;
		else if(tags[i].getID() == Tag::SELECTION_2ND_ID)
			selection2nd_count++;
		else if(inRange(Tag::ACTION_ID_MIN, Tag::ACTION_ID_MAX, tags[i].getID()))
			action_count++;
		else if(inRange(Tag::NUMBER_ID_MIN, Tag::NUMBER_ID_MAX, tags[i].getID()))
			number_count++;
		else if(tags[i].getID() > Tag::NUMBER_ID_MAX)
			other_count++;

		if(tags[i].getID() == Tag::SELECT_REGION_ID ||
		   tags[i].getID() == Tag::SELECT_OBJECTS_ID)
			regionID_count++;
		
		grouped[i] = -1;
	}

	if(selection_count == 0) {
		error_msg = "ERROR - TAG GROUPING - no selection tag.";
		return false;
	}

	if(action_count == 0) {
		error_msg = "ERROR - TAG GROUPING - no action tag.";
		return false;
	}

	if(number_count == 0) {
		error_msg = "ERROR - TAG GROUPING - no number tag.";
		return false;
	}

	if(action_count < selection_count) {
		error_msg = "ERROR - TAG GROUPING - too few action or two many selection tags.";
		return false;
	}

	if(selection2nd_count != regionID_count) {
		error_msg = "ERROR - TAG GROUPING - too many or too few secondary selection tags.";
		return false;
	}

	if(selection2nd_count + action_count != number_count) {
		error_msg = "ERROR - TAG GROUPING - too many or too few number tags.";
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

	for(int i = number_ind; i < other_ind - 1; i++) {
		int prev = tags[i-1].getID();
		// NOTE: at this point there is at least one selection and one action tags so 
		// number_ind >= 2 and i-1 will be valid.
		int curr = tags[i].getID();
		int next = tags[i+1].getID();
		if(next > (curr+1)) {
			error_msg = "ERROR - TAG GROUPING - missing number tag.";
			return false;
		}
		if(prev == next) {
			error_msg = "ERROR - TAG GROUPING - too many repeated number tags.";
			return false;
		}
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
	//	std::cout << tags[i].getID();
	//}
	//std::cout << " |\n";

	for(int i = number_ind; i < other_ind; i++) {
		Tag number = tags[i];
		
		//std::cout << "number " 
		//          << number.printID() 
		//          << number.printCenter()
		//          << " at " << i << " to\n";
		
		for(int j = selection2nd_ind; j < number_ind; j++) {
			if(grouped[j] > -1) // action/secondary selection tag is already grouped
				continue;

			Tag action_or_2ndary = tags[j];
			
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
			error_msg = "ERROR - TAG GROUPING - dangling number tag.";
			return false;
		}

		//NOTE: decided to enforce the following:
		//  - the even steps are pick actions and the odd steps are place actions.
		//    This allows us to consider each pick and the subsequent place as a block
		//    We cannot thus accpet nested blocks to support such cases as picking up a tool
		//    for later pick&place (later pick&place blocks are nested in tool pickup block)
		//TO-DO once decided to support nested blocks should remove this check

		if(number.getID()%2 == 1 && 
		   (tags[grouped[i]].getID() != Tag::TOP_PICK_ID &&
		   	tags[grouped[i]].getID() != Tag::SIDE_PICK_ID &&
		   	tags[grouped[i]].getID() != Tag::SELECTION_2ND_ID)) {
			error_msg = "ERROR - TAG GROUPING - expected a pick action." ;
			return false;
		}

		if(number.getID()%2 == 0 && 
		   (tags[grouped[i]].getID() != Tag::POSITION_ID &&
		   	tags[grouped[i]].getID() != Tag::DROP_ID &&
		   	tags[grouped[i]].getID() != Tag::SELECTION_2ND_ID)) {
			error_msg = "ERROR - TAG GROUPING - expected a place action." ;
			return false;
		}
	}

	//YSS cannot think of a case where a selection2nd is not grouped yet none of the earlier
	//    error cases is triggered. I think this check is redundant.
	//    - extra selection2nd ---> selection2nd_count != regionID_count
	//    - selection2nd not numbered ---> selection2nd_count + action_count != number_count
	//    - selection2nd and number improperly placed ---> dangling number tag
	//    but I leave it just in case
	for(int i = selection2nd_ind; i < action_ind; i++)
		if(grouped[i] == -1) {
			error_msg = "ERROR - TAG GROUPING - secondary selection tag not numbered (WEIRD).";
			return false;
		}

	//YSS again cannot think of a case where a selection2nd is not grouped yet none of the 
	//    earlier error cases is triggered. I think this check is redundant.
	//    - extra action ---> selection2nd_count + action_count != number_count
	//    - action not numbered ---> selection2nd_count + action_count != number_count
	//    - action and number improperly placed ---> dangling number tag
	//    but I leave it just in case
	for(int i = action_ind; i < number_ind; i++)
		if(grouped[i] == -1) {
			error_msg = "ERROR - TAG GROUPING - action tag not numbered (WEIRD).";
			return false;
		}

	//std::cout << "number-action grouping:\t\t";
	//for(int i = 0; i < tag_num; i++)
	//	std::cout << grouped[i] << ", ";
	//std::cout << "\n";

	for(int i = action_ind; i < number_ind; i++) {
		Tag action = tags[i];

		//std::cout << "action " 
		//          << action.printID()
		//          << action.printCenter()
		//          << " at " << i 
		//          << " with y-axis " << action.getYvect().transpose() << "\n";

		double minDist = MAX_WORKSPACE_DIST; int temp_grouped = -1;
		for(int j = selection_ind; j < selection2nd_ind; j++) {
			Tag selection = tags[j];

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
			error_msg = "ERROR - TAG GROUPING - dangling action tag.";
			return false;
		}

		//std::cout << "action at " << i << " grouped with selection at " << temp_grouped << "\n";

		grouped[i] = temp_grouped;
		if(grouped[temp_grouped] == -1)
			grouped[temp_grouped] = i;
	}

	for(int i = selection_ind; i < selection2nd_ind; i++)
		if(grouped[i] == -1) {
			error_msg = "ERROR - TAG GROUPING - dangling selection tag.";
			return false;
		}

	//TO-DO return false for the following error cases
	//   - the order of action tags grouped w/ the same selection tag does not follow
	//     their distances

	//std::cout << "action-selection grouping:\t";
	//for(int i = 0; i < tag_num; i++)
	//	std::cout << grouped[i] << ", ";
	//std::cout << "\n";

	for(int i = number_ind; i < other_ind-1; i++) {
		Tag num1 = tags[i]; int num1_action_or_2ndary_at = grouped[i];
		Tag num2 = tags[i+1]; int num2_action_or_2ndary_at = grouped[i+1];
		if(num1.getID() == num2.getID()) {
			
			//std::cout << num1.printID() << " @" << i 
			//          << " --> " << tags[grouped[i]].printID() << " @" << grouped[i];
			//std::cout << " ---- ";
			//std::cout << num2.printID() << " @" << i+1 
			//          << " --> " << tags[grouped[i+1]].printID() << " @" << grouped[i+1];
			//std::cout << "\n";
			
			// of two successive number tags with the same id, one is grouped with an action
	        // and another with a secondary selection tool. The action is grouped with a
	        // region selection tool
			if((tags[num1_action_or_2ndary_at].getID() == Tag::SELECTION_2ND_ID &&
			    tags[num2_action_or_2ndary_at].getID() == Tag::SELECTION_2ND_ID) ||
			   (tags[num1_action_or_2ndary_at].getID() != Tag::SELECTION_2ND_ID &&
			    tags[num2_action_or_2ndary_at].getID() != Tag::SELECTION_2ND_ID) ||
			   tags[grouped[num1_action_or_2ndary_at]].getID() == Tag::SELECT_POSITION_ID ||
			   tags[grouped[num1_action_or_2ndary_at]].getID() == Tag::SELECT_OBJECT_ID ||
			   tags[grouped[num2_action_or_2ndary_at]].getID() == Tag::SELECT_POSITION_ID ||
			   tags[grouped[num2_action_or_2ndary_at]].getID() == Tag::SELECT_OBJECT_ID) {
				error_msg = "ERROR - TAG GROUPING - tags inavlidly paired.";
				return false;
			}

			if(tags[num1_action_or_2ndary_at].getID() == Tag::SELECTION_2ND_ID) {
				grouped[num1_action_or_2ndary_at] = grouped[num2_action_or_2ndary_at];
				grouped[grouped[num2_action_or_2ndary_at]] = num1_action_or_2ndary_at;
				//std::cout << "secondary selection at " << i 
				//          << " grouped with selection at " << grouped[grouped[i+1]] << "\n";
			} else {
				grouped[num2_action_or_2ndary_at] = grouped[num1_action_or_2ndary_at];
				grouped[grouped[num1_action_or_2ndary_at]] = num2_action_or_2ndary_at;
				//std::cout << "secondary selection at " << i+1 
				//          << " grouped with selection at " << grouped[grouped[i]] << "\n";
			}
		}
	}

	//std::cout << "full grouping:\t\t\t";
	//for(int i = 0; i < tag_num; i++)
	//	std::cout << grouped[i] << ", ";
	//std::cout << "\n";

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
		index = tags[i].getID() - Tag::NUMBER_ID_MIN;
		Instruction instruction = instructions[index];

		action_at = grouped[i];
		selection_at = grouped[action_at];

		if(tags[action_at].getID() == Tag::SELECTION_2ND_ID) {
			instruction.selection2nd = tags[action_at];
		} else {
			instruction.number = tags[i];
			instruction.action = tags[action_at];
			instruction.selection = tags[selection_at];
			if(tags[selection_at].getID() == Tag::SELECT_REGION_ID ||
			   tags[selection_at].getID() == Tag::SELECT_OBJECTS_ID) {
				int selection2nd_at = grouped[selection_at];
				instruction.selection2nd = tags[selection2nd_at];
			}
		}

		instructions[index] = instruction;
	}

	//YSS this is already enforced by requiring all picks be on even steps
	if(instructions[0].action.getID() != Tag::SIDE_PICK_ID &&
	   instructions[0].action.getID() != Tag::TOP_PICK_ID) {
		error_msg = "ERROR - TAG GROUPING - invalid first action (not a pick).";
		instructions.clear();
		return false;
	}

	for(int i = 0; i < instructions.size(); i++) {
		if((instructions[i].selection.getID() == Tag::SELECT_REGION_ID ||
		    instructions[i].selection.getID() == Tag::SELECT_OBJECTS_ID) &&
		   instructions[i].selection2nd.getID() == -1) {
		   	error_msg = "ERROR - TAG GROUPING - missing secondary selection tag.";
			instructions.clear();
			return false;
		}
	}

	//std::cout << printInstructionTags();

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
//    - if SELECT_OBJECTS_ID, for each object
//        - test if point cloud is fully within the ROI (crop objects against the ROI crop box)
//        - if little is lost after cropping, assign the object to the instruction 
bool Program::matchObjects() {
	for(int i = 0; i < instructions.size(); i++) {
		Tag selection = instructions[i].selection;

		if(selection.getID() == Tag::SELECT_OBJECT_ID) {	
			pcl::CropBox<pcl::PointXYZRGB> cbox;
			setupFilterBox(cbox, selection);

			int max_overlap = -1; int max_overlap_index = -1;
			for(int j = 0; j < objects.size(); j++) {
				//std::cout << "object \\" << j;

			    int cloud_size = filterObject(cbox, objects[j]);
				
				//std::cout << " pass min_overlap? (" << MIN_POINT_OVERLAP << ") " << (cloud_size >= MIN_POINT_OVERLAP);
				//std::cout << " pass max_so_far? (" << max_overlap << ") " << (cloud_size > max_overlap);
				
				if(cloud_size >= MIN_POINT_OVERLAP &&
				   cloud_size > max_overlap) {
					max_overlap = cloud_size;
					max_overlap_index = j;
					
					//std::cout << " <--- new max. ";
				}

				//std::cout << "\n";
			}

			if(max_overlap_index == -1) {
				error_msg = "ERROR - TAG GROUPING - no object to select.";
				return false;
			}

			instructions[i].objects.push_back(objects[max_overlap_index]);
		} else if (instructions.at(i).selection.getID() == Tag::SELECT_OBJECTS_ID){
			Tag selection2nd = instructions[i].selection2nd;

			pcl::CropBox<pcl::PointXYZRGB> cbox;
			setupFilterBox(cbox, selection, selection2nd);
			
			for(int j = 0; j < objects.size(); j++) {
				//std::cout << "object \\" << j;

				int size_before_crop = objects[j].GetCloud()->points.size();
				
				int size_after_crop = filterObject(cbox, objects[j]);

				//std::cout << "cloud size (after crop: " << size_after_crop << ")";

				//std::cout << " ratio = " << (size_after_crop * 1.0 / size_before_crop);

				if((size_after_crop * 1.0 / size_before_crop) < MIN_REGION_OVERLAP_RATIO) {
					//std::cout << "\n";
					continue;
				}
				//TO-DO this check does not address oversized segments

				//std::cout << " <--- within the region\n";

				instructions[i].objects.push_back(objects[j]);
			}

			if(instructions[i].objects.size() == 0) {
				error_msg = "ERROR - TAG GROUPING - no object in region to select.";
				return false;
			}
		}
	}
	return true;
}

void Program::setupFilterBox(pcl::CropBox<pcl::PointXYZRGB>& cbox, Tag& selection) {
	Eigen::Vector3d x_axis = selection.getXvect();
	Eigen::Vector4f x_axis_extended(x_axis(0), x_axis(1), x_axis(2), 0);
	Eigen::Vector3d y_axis = selection.getYvect();
	Eigen::Vector4f y_axis_extended(y_axis(0), y_axis(1), y_axis(2), 0);
	Eigen::Vector3d z_axis = selection.getZvect();
	Eigen::Vector4f z_axis_extended(z_axis(0), z_axis(1), z_axis(2), 0);

	//std::cout << "selection at (" << selection.getCenter().x << ", "
	//                              << selection.getCenter().y << ", "
	//                              << selection.getCenter().z << ")\n";

	Eigen::Vector4f tip (selection.getCenter().x, 
		                 selection.getCenter().y,
		                 selection.getCenter().z,
		                 1);
	tip += y_axis_extended * Tag::ARROW_SELECTION_LEN;

	//std::cout << "tip at (" << tip(0) << ", " << tip(1) << ", " << tip(2) << ")\n";
	
	Eigen::Vector4f min_, max_;
	min_ = tip - OBJECT_SELECTION_BOX_SIZE * x_axis_extended;
	max_ = tip + OBJECT_SELECTION_BOX_SIZE * (x_axis_extended + 
		                                      y_axis_extended + 
		                                      z_axis_extended);
	
	//std::cout << "min at (" << min_(0) << ", " << min_(1) << ", " << min_(2) << ")\n";
	//std::cout << "max at (" << max_(0) << ", " << max_(1) << ", " << max_(2) << ")\n";

	cbox.setMin(min_);
	cbox.setMax(max_);
}

void Program::setupFilterBox(pcl::CropBox<pcl::PointXYZRGB>& cbox,
	                           Tag& selection, Tag& selection2nd) {
	Eigen::Vector3d y_axis = selection.getYvect();
	Eigen::Vector4f y_axis_extended(y_axis(0), y_axis(1), y_axis(2), 0);

	//std::cout << "selection at (" << selection.getCenter().x << ", "
	//                              << selection.getCenter().y << ", "
	//                              << selection.getCenter().z << ")\n";

	Eigen::Vector4f primary_corner (selection.getCenter().x,
		                            selection.getCenter().y,
		                            selection.getCenter().z,
		                            1);
	primary_corner += y_axis_extended * Tag::CORNER_SELECTION_LEN;

	//std::cout << "primary corner at (" << primary_corner(0) << ", " 
	//                                   << primary_corner(1) << ", " 
	//                                   << primary_corner(2) << ")\n";

	//std::cout << "2nd selection at (" << selection2nd.getCenter().x << ", "
	//                                  << selection2nd.getCenter().y << ", "
	//                                  << selection2nd.getCenter().z << ")\n";

	y_axis = selection2nd.getYvect();
	y_axis_extended(0) = y_axis(0);
	y_axis_extended(1) = y_axis(1);
	y_axis_extended(2) = y_axis(2);
	Eigen::Vector4f secondary_corner (selection2nd.getCenter().x,
		                              selection2nd.getCenter().y,
		                              selection2nd.getCenter().z,
		                              1);
	
	secondary_corner += y_axis_extended * Tag::CORNER_SELECTION_LEN;

	//std::cout << "secondary corner at (" << secondary_corner(0) << ", " 
	//                                     << secondary_corner(1) << ", " 
	//                                     << secondary_corner(2) << ")\n";

	Eigen::Vector4f min_, max_;
	min_(0) = std::min(primary_corner(0), secondary_corner(0));
	min_(1) = std::min(primary_corner(1), secondary_corner(1));
	min_(2) = primary_corner(2);
	//YSS this should almost the same os secondary_corner(2)

	max_(0) = std::max(primary_corner(0), secondary_corner(0));
	max_(1) = std::max(primary_corner(0), secondary_corner(2));
	max_(2) = primary_corner(2) + MAX_WORKSPACE_HEIGHT;

	cbox.setMin(min_);
	cbox.setMax(max_);
}

int Program::filterObject(pcl::CropBox<pcl::PointXYZRGB>& cbox,
	                      rapid::perception::Object& obj) {
	//std::cout << "(" << obj.pose().pose.position.x << ", "
    //                 << obj.pose().pose.position.y << ", "
    //                 << obj.pose().pose.position.z << ") ";

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud = *obj.GetCloud();
	//NOTE make a copy of the cloud to ensure the original cloud is intact 

	int cloud_size = cloud->points.size();

    //std::cout << "cloud size (before crop: " << cloud_size << ") ";
	
	cbox.setInputCloud(cloud);
	cbox.filter(*cloud);

	cloud_size = cloud->points.size();

	//std::cout << "cloud size (after crop: " << cloud_size << ")";

	return cloud_size;
}

std::vector<Instruction> Program::getInstructions() { return instructions; }

std::string Program::error() { return error_msg; }

std::string Program::printInstructions() {
	std::stringstream ss;
	
	ss << "index\tnumber, selection, secondary, action\n";
	ss << "\tobjects:\n";
	for(int i = 0; i < instructions.size(); i++) {
		Instruction instruction = instructions[i];
		ss << i << "\t";
		ss << instruction.number.printID() << instruction.number.printCenter() << ", ";
		ss << instruction.selection.printID() << instruction.selection.printCenter() << ", ";
		ss << instruction.selection2nd.printID() << instruction.selection2nd.printCenter() << ", ";
		ss << instruction.action.printID() << instruction.action.printCenter() << "\n";
		ss << "\tobjects:";
		for(int j = 0; j < instruction.objects.size(); j++) {
			ss << "(" << instruction.objects[j].pose().pose.position.x << ", "
			          << instruction.objects[j].pose().pose.position.y << ", "
			          << instruction.objects[j].pose().pose.position.z << "), ";
		}
		ss << "\n";
	}
	
	return ss.str();
}

}
