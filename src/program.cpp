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
//  ! repeated numbers ---> selection and secondary selection that should be grouped but this
//    can wait to be done during instruction generation
bool Program::groupTags() {
	int tagNum = tags.size();
	if(tagNum == 0) {
		return false;
		//TO-DO provide error message: zero tags to group
	}

	std::sort(tags.begin(), tags.end());

	int selectionInd = tagNum; // index where selection tags start
	int selection2ndInd = tagNum; // index where secondary selection tags start
	int actionInd = tagNum; // index where action tags start
	int numberInd = tagNum; // index where number tags start
	int otherInd  = tagNum; //YSS: to handle additional tags (e.g. loop)
	int grouped[tagNum];
	for(int i = 0; i < tagNum; i++) {
		if(inRange(Tag::SELECTION_ID_MIN, Tag::SELECTION_ID_MAX, tags.at(i).getID()) 
		    && i < selectionInd) selectionInd = i;
		else if(tags.at(i).getID() == Tag::SELECTION_2ND_ID
			&& i < selection2ndInd) selection2ndInd = i;
		else if(inRange(Tag::ACTION_ID_MIN, Tag::ACTION_ID_MAX, tags.at(i).getID())
			&& i < actionInd) actionInd = i;
		else if(inRange(Tag::NUMBER_ID_MIN, Tag::NUMBER_ID_MAX, tags.at(i).getID())
			&& i < numberInd) numberInd = i;
		else if(tags.at(i).getID() > Tag::NUMBER_ID_MAX
			&& i < otherInd) otherInd = i;
		grouped[i] = -1;
	}

	//TO-DO return false for the following error cases
	//   - only primary selection tags
	//   - only primary and secondary tas
	//   - only selection and action but no number tags
	//   - number of number tags ~= number of actions + number of region selection tags
	//   - number of action tags is odd
	//   - number of pick actions ~= number of place actions

	//std::cout << "selection tags @ " << selectionInd << ", " 
	//          << "2ndary selection tags @ " << selection2ndInd << ", " 
	//          << "ation tags @ " << actionInd << ", " 
	//          << "number tags @ " << numberInd << "\n";

	for(int i = 0; i < tagNum; i++) {
		if(i == selectionInd || i == selection2ndInd || i == actionInd || i == numberInd)
			std::cout << "| ";
		else
			std::cout << ", ";
		std::cout << tags.at(i).getID();
	}
	std::cout << " |\n";

	for(int i = numberInd; i < otherInd; i++) {
		Tag number = tags.at(i);
		
		//std::cout << "number " 
		//          << number.printID() 
		//          << number.printCenter()
		//          << " at " << i << " to\n";
		
		for(int j = selection2ndInd; j < numberInd; j++) {
			if(grouped[j] > -1) // action or secondary selection tag is already grouped
				continue;
			Tag actionOr2ndarySelection = tags.at(j);
			
			//std::cout << "\taction " 
			//          << actionOr2ndarySelection.printID()
			//          << actionOr2ndarySelection.printCenter()
			//          << " at " << j << "\n";
			
			double distance = number.dist(actionOr2ndarySelection);
			
			//std::cout << "\t\tdistance: " << distance << "\n";
			
			if(!inRange(Tag::EDGE_SIZE - DIST_ERR_MARGIN,
				        Tag::EDGE_SIZE + DIST_ERR_MARGIN,
				        distance)) // action or secondary selection tag is too close/far
				continue;
			
			// normalized center-to-center vector
			Eigen::Vector3d n2a = number.vect(actionOr2ndarySelection) / distance;
			Eigen::Vector3d ux = number.getXvect();
			
			//std::cout << "\t\tx axis: " << ux.transpose() << "\n";
			
			double innerProduct = ux.dot(n2a);
			
			//std::cout << "\t\tinnerProduct: " << innerProduct << "\n";
			
			if(!inRange(1 - ROTATE_ERR_MARGIN,
			            1 + ROTATE_ERR_MARGIN,
			            innerProduct)) // action or secondary selection tag is not along x-axis
				continue;
			
			// number tag is grouped with action or secondary selection tag
			grouped[i] = j; 
			grouped[j] = i;
			
			std::cout << "number at " << i << " grouped with action at " << j << "\n";
			
			break;
		}
	}

	//TO-DO return false for the following error cases
	//   - there is a number tag not yet grouped
	//   - there is a secondary selection tag not yet grouped
	//   - there is an action tag not yet grouped
	//   - id of more that two successive number tags is equal
	//   - of two successive tags with the same id, both are grouped with an action or 
	//     secondary selection tool
	
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

			// normalized the center-to-center vector
			Eigen::Vector3d a2s = action.vect(selection) / distance;
			Eigen::Vector3d uy = action.getYvect();
			double innerProduct = uy.dot(a2s);
			if(!inRange(1 - ROTATE_ERR_MARGIN,
				        1 + ROTATE_ERR_MARGIN,
				        innerProduct)) // selection tag is not along the y-axis
				continue;
			
			if(distance < minDist) {
				minDist = distance;
				tempGrouped = j;
			}
		}

		//std::cout << "\tmin distance: " << minDist << " with tag at " << tempGrouped << "\n";

		double quantizedDist = round(minDist/Tag::EDGE_SIZE);

		//std::cout << "\tclosest ideal distance: " << quantizedDist*Tag::EDGE_SIZE << "\n";

		if(!inRange(quantizedDist*Tag::EDGE_SIZE - DIST_ERR_MARGIN,
			        quantizedDist*Tag::EDGE_SIZE + DIST_ERR_MARGIN,
			        minDist)) // selection tag is not located in multiples of EDGE_SIZE
			continue;

		std::cout << "action at " << i << " grouped with selection at " << tempGrouped << "\n";

		grouped[i] = tempGrouped;
		if(grouped[tempGrouped] == -1)
			grouped[tempGrouped] = i;
	}

	//TO-DO return false for the following error cases
	//   - there is an action tag not yet grouped
	//   - there is a selection tag not yet grouped
	//   - the order of action tags assigned to the same selection tag does not follow
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
			if(tags.at(grouped[i]).getID() == Tag::SELECTION_2ND_ID &&
			   tags.at(grouped[i+1]).getID() == Tag::SELECTION_2ND_ID)
				return false;

			if(tags.at(grouped[i]).getID() != Tag::SELECTION_2ND_ID &&
			   tags.at(grouped[i+1]).getID() != Tag::SELECTION_2ND_ID)
				return false;			

			//YSS don't need to check the above conditions if earlier error handling exists

			if(tags.at(grouped[i]).getID() == Tag::SELECTION_2ND_ID){
				grouped[grouped[i]] = grouped[grouped[i+1]];
				std::cout << "secondary selection at " << i 
				          << " grouped with selectio at" << grouped[grouped[i+1]] << "\n";
			}
			else{
				grouped[grouped[i+1]] = grouped[grouped[i]];
				std::cout << "secondary selection at " << i+1 
				          << " grouped with selectio at " << grouped[grouped[i]] << "\n";
			}
		}
	}

	return true;
}

void Program::printTag(Tag& t) {

}

}
