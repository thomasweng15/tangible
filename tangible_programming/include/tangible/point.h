#ifndef TANGIBLE_LOCATION
#define TANGIBLE_LOCATION

#include "geometry_msgs/PointStamped.h"

#include "tangible/action_target.h"

namespace tangible
{

class Point : ActionTarget
{
private:
	geometry_msgs::PointStamped point;
	double margin;

	//TO-DO: should either use the procedure in program.cpp to match the object to the point
	//       or some other code provide the point and margin of an object to check whether it is at a point
	bool is_eqaul(geometry_msgs::PointStamped p1, geometry_msgs::PointStamped p2, double err_margin);

public:
	Point(geometry_msgs::PointStamped p);
	~Point();

	void set_margin(double m);

	bool match(geometry_msgs::PointStamped p);
};

};

#endif