#include "tangible/tag.h"

#include <sstream>
#include <numeric>

#include "Eigen/Geometry"

namespace tangible {

Tag::Tag() : center(), orientation(), x_axis(), y_axis(), z_axis(), id(-1) {}

Tag::Tag(geometry_msgs::PoseStamped& p) : Tag(p, -1) {}

Tag::Tag(geometry_msgs::PoseStamped& p, int _id) {
	setCenter(p);
	setOrientation(p);
	setAxes(p);
	setID(_id);
}

Tag::~Tag() {}

void Tag::setCenter(geometry_msgs::PoseStamped& p) {
	center.x = p.pose.position.x;
	center.y = p.pose.position.y;
	center.z = p.pose.position.z;
}

void Tag::setOrientation(geometry_msgs::PoseStamped& p) {
	orientation.x = p.pose.orientation.x;
	orientation.y = p.pose.orientation.y;
	orientation.z = p.pose.orientation.z;
	orientation.w = p.pose.orientation.w;
}

void Tag::setAxes(geometry_msgs::PoseStamped& p) {
	Eigen::Quaterniond q(p.pose.orientation.w,
		                 p.pose.orientation.x, 
		                 p.pose.orientation.y,
		                 p.pose.orientation.z);
	q.normalize();
	Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
	x_axis.x = rotation_matrix(0, 0);
	x_axis.y = rotation_matrix(1, 0);
	x_axis.z = rotation_matrix(2, 0);
	y_axis.x = rotation_matrix(0, 1);
	y_axis.y = rotation_matrix(1, 1);
	y_axis.z = rotation_matrix(2, 1);
	z_axis.x = rotation_matrix(0, 2);
	z_axis.y = rotation_matrix(1, 2);
	z_axis.z = rotation_matrix(2, 2);
}

void Tag::setID(int _id) { id = _id; }

Position Tag::getCenter() { return center; }
Quaternion Tag::getOrientation() { return orientation; }
Axis Tag::getX() { return x_axis; }
Axis Tag::getY() { return y_axis; }
Axis Tag::getZ() { return z_axis; }
//double[] Tag::getXvect();
//double[] Tag::getYvect();
//double[] Tag::getZvect();
int Tag::getID() { return id; }

double Tag::dist(Tag& otherTag) {
	double this2that[3];
	this2that[0] = otherTag.center.x - center.x;
	this2that[1] = otherTag.center.y - center.x;
	this2that[2] = otherTag.center.z - center.z;
	return std::inner_product(this2that, this2that+3, this2that, 0.0);
}

}