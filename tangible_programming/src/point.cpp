#include "tangible/point.h"

namespace tangible
{


Point::Point(geometry_msgs::PointStamped p)
{
	point = p;

	//TO-DO: get the default margin from parameter server
	margin = 0.01;
}

Point::~Point() {}

void Point::set_margin(double m)
{
	margin = m;
}

bool Point::match(geometry_msgs::PointStamped p)
{
	return is_eqaul(point, p, margin);
}

bool Point::is_eqaul(geometry_msgs::PointStamped p1, geometry_msgs::PointStamped p2, double errMargin)
{
	if(p1.point.x != p2.point.x) return false;
	if(p1.point.y != p2.point.y) return false;
	if(p1.point.z != p2.point.z) return false;
	return true;
}

}