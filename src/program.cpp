#include "tangible/program.h"

#include <algorithm>
#include <math.h>

#include "Eigen/Geometry"
#include "tangible/utils.h"

namespace tangible {

Program::Program(std::vector<Tag>& tgs) { 
	tags = tgs;
	groupTags();
}
Program::~Program() {}

bool Program::refreshTags(std::vector<Tag>& tgs) {
	tags = tags;
	return groupTags();
}

//grouping:
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
bool Program::groupTags() {
	instructions.clear();
	//NOTE: this ensures no instruction is formed for invalid tag settings

	int tagNum = tags.size();
	if(tagNum == 0) {
		error_msg = "ERROR - TAG GROUPING - no tags to group.";
		std::cout << error_msg << "\n";
		return false;
	}

	std::sort(tags.begin(), tags.end());

	int selectionCount = 0;
	int selection2ndCount = 0;
	int actionCount = 0;
	int numberCount = 0;
	int otherCount = 0;
	int regionIDCount = 0;
	int grouped[tagNum];
	for(int i = 0; i < tagNum; i++) {
		if(inRange(Tag::SELECTION_ID_MIN, Tag::SELECTION_ID_MAX, tags.at(i).getID()))
			selectionCount++;
		else if(tags.at(i).getID() == Tag::SELECTION_2ND_ID)
			selection2ndCount++;
		else if(inRange(Tag::ACTION_ID_MIN, Tag::ACTION_ID_MAX, tags.at(i).getID()))
			actionCount++;
		else if(inRange(Tag::NUMBER_ID_MIN, Tag::NUMBER_ID_MAX, tags.at(i).getID()))
			numberCount++;
		else if(tags.at(i).getID() > Tag::NUMBER_ID_MAX)
			otherCount++;

		if(tags.at(i).getID() == Tag::SELECT_REGION_ID ||
		   tags.at(i).getID() == Tag::SELECT_OBJECTS_ID)
			regionIDCount++;
		
		grouped[i] = -1;
	}

	if(selectionCount == 0) {
		error_msg = "ERROR - TAG GROUPING - no selection tag.";
		std::cout << error_msg << "\n";
		return false;
	}

	if(actionCount == 0) {
		error_msg = "ERROR - TAG GROUPING - no action tag.";
		std::cout << error_msg << "\n";
		return false;
	}

	if(numberCount == 0) {
		error_msg = "ERROR - TAG GROUPING - no number tag.";
		std::cout << error_msg << "\n";
		return false;
	}

	if(actionCount < selectionCount) {
		error_msg = "ERROR - TAG GROUPING - too few action or two many selection tags.";
		std::cout << error_msg << "\n";
		return false;
	}

	if(selection2ndCount != regionIDCount) {
		error_msg = "ERROR - TAG GROUPING - too many or too few secondary selection tags.";
		std::cout << error_msg << "\n";
		return false;
	}

	if(selection2ndCount + actionCount != numberCount) {
		error_msg = "ERROR - TAG GROUPING - too many or too few number tags.";
		std::cout << error_msg << "\n";
		return false;
	}

	// index where selection tags start
	int selectionInd = 0;
	// index where secondary selection tags start
	int selection2ndInd = selectionInd + selectionCount; 
	// index where action tags start
	int actionInd = selection2ndInd + selection2ndCount;
	// index where number tags start
	int numberInd = actionInd + actionCount;
	//TO-DO to handle additional tags (e.g. loop)
	int otherInd  = numberInd + numberCount;

	if(tags.at(actionInd).getID() != Tag::SIDE_PICK_ID &&
	   tags.at(actionInd).getID() != Tag::TOP_PICK_ID) {
		error_msg = "ERROR - TAG GROUPING - invalid first action (not a pick).";
		std::cout << error_msg << "\n";
		return false;
	}

	int currNum = Tag::NUMBER_ID_MIN;
	for(int i = numberInd; i < otherInd - 1; i++) {
		if(tags.at(i).getID() != currNum) {
			error_msg = "ERROR - TAG GROUPING - missing/out of order number tag.";
			std::cout << error_msg << "\n";
			return false;
		}
		if(tags.at(i).getID() == tags.at(i-1).getID() &&
		   tags.at(i).getID() == tags.at(i+1).getID()) {
		// NOTE: at this point there is at least one selection and one action tags so 
		// numberInd >= 2 and i-1 will be valid.
			error_msg = "ERROR - TAG GROUPING - too many repeated number tags.";
			std::cout << error_msg << "\n";
			return false;
		}

		if(tags.at(i).getID() != tags.at(i+1).getID())
			currNum++;
	}

	//std::cout << "selection tags @ " << selectionInd << ", " 
	//          << "2ndary selection tags @ " << selection2ndInd << ", " 
	//          << "ation tags @ " << actionInd << ", " 
	//          << "number tags @ " << numberInd << "\n";

	//for(int i = 0; i < tagNum; i++) {
	//	if(i == selectionInd || i == selection2ndInd || i == actionInd || i == numberInd)
	//		std::cout << "| ";
	//	else
	//		std::cout << ", ";
	//	std::cout << tags.at(i).getID();
	//}
	//std::cout << " |\n";

	for(int i = numberInd; i < otherInd; i++) {
		Tag number = tags.at(i);
		
		//std::cout << "number " 
		//          << number.printID() 
		//          << number.printCenter()
		//          << " at " << i << " to\n";
		
		for(int j = selection2ndInd; j < numberInd; j++) {
			if(grouped[j] > -1) // action/secondary selection tag is already grouped
				continue;

			Tag actionOr2ndarySelection = tags.at(j);
			
			//std::cout << "\t action or secondary selection  " 
			//          << actionOr2ndarySelection.printID()
			//          << actionOr2ndarySelection.printCenter()
			//          << " at " << j << "\n";
			
			double distance = number.dist(actionOr2ndarySelection);
			
			//std::cout << "\t\tdistance: " << distance << "\n";
			
			if(!inRange(Tag::EDGE_SIZE - DIST_ERR_MARGIN,
				        Tag::EDGE_SIZE + DIST_ERR_MARGIN,
				        distance)) // action/secondary selection tag is too close/far
				continue;
			
			// normalized center-to-center vector
			Eigen::Vector3d n2a = number.vect(actionOr2ndarySelection) / distance;

			Eigen::Vector3d ux = number.getXvect();
			
			//std::cout << "\t\tx axis: " << ux.transpose() << "\n";
			
			double innerProduct = ux.dot(n2a);
			
			//std::cout << "\t\tinnerProduct: " << innerProduct << "\n";
			
			if(!inRange(1 - ROTATE_ERR_MARGIN,
			            1 + ROTATE_ERR_MARGIN,
			            innerProduct)) //action/secondary selection tag is not aligned w/ x-axis
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
	}

	for(int i = selection2ndInd; i < actionInd; i++)
		if(grouped[i] == -1) {
			error_msg = "ERROR - TAG GROUPING - secondary selection tag not numbered.";
			std::cout << error_msg << "\n";
			return false;
		}

	for(int i = actionInd; i < numberInd; i++)
		if(grouped[i] == -1) {
			error_msg = "ERROR - TAG GROUPING - action tag not numbered.";
			std::cout << error_msg << "\n";
			return false;
		}

	double latestActionDist[selectionCount];
	for(int i = actionInd; i < numberInd; i++) {
		Tag action = tags.at(i);

		//std::cout << "action " 
		//          << action.printID()
		//          << action.printCenter()
		//          << " at " << i 
		//          << " with y-axis " << action.getYvect().transpose() << "\n";

		double minDist = MAX_WORKSPACE_DIST; int tempGrouped = -1;
		for(int j = selectionInd; j < selection2ndInd; j++) {
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
			            distance)) // distance of selection tag is not a multiples of EDGE_SIZE
				continue;

			Eigen::Vector3d uy = action.getYvect();
			double innerProduct = uy.dot(a2s);
			if(!inRange(1 - ROTATE_ERR_MARGIN,
				        1 + ROTATE_ERR_MARGIN,
				        innerProduct)) // selection tag is not aligned w/ y-axis
				continue;
			
			if(distance < minDist) {
				minDist = distance;
				tempGrouped = j;
			}
		}

		//std::cout << "\tmin distance: " << minDist << " with tag at " << tempGrouped << "\n";

		if(tempGrouped == -1) {
			error_msg = "ERROR - TAG GROUPING - singular action tag.";
			std::cout << error_msg << "\n";
			return false;
		}

		//std::cout << "action at " << i << " grouped with selection at " << tempGrouped << "\n";

		grouped[i] = tempGrouped;
		if(grouped[tempGrouped] == -1)
			grouped[tempGrouped] = i;
	}

	for(int i = selectionInd; i < selection2ndInd; i++)
		if(grouped[i] == -1) {
			error_msg = "ERROR - TAG GROUPING - singular selection tag.";
			std::cout << error_msg << "\n";
			return false;
		}

	//TO-DO return false for the following error cases
	//   - the order of action tags grouped w/ the same selection tag does not follow
	//     their distances

	for(int i = numberInd; i < otherInd-1; i++) {
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

	int instructionNum = actionCount;
	for(int i = 0; i < instructionNum; i++) {
		Instruction instruction;
		instructions.push_back(instruction);
	}
	
	for(int i = numberInd; i < otherInd; i++) {
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

	//NOTE: decided not to enforce the following:
	//  - the even steps are pick actions and the odd steps are place actions.
	//    This allows us to support picking up a tool for later pick&place

	std::cout << "\tnumber" << "\t\tselection" << "\tsecondary" << "\taction\n";
	for(int i = 0; i < instructionNum; i++) {
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

std::string Program::error() { return error_msg; }

}
