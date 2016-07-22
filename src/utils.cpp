#include "tangible/utils.h"

namespace tangible {

bool inRange(int LB, int UB, int number) {
	if(number < LB) return false;
	if(number > UB) return false;
	return true;
}

bool inRange(double LB, double UB, double number) {
	if(number < LB) return false;
	if(number > UB) return false;
	return true;
}

}