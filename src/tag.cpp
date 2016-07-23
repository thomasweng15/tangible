#include "tangible/tag.h"

#include <sstream>

namespace tangible {

Tag::Tag() : center(), orientation(), x_axis(), y_axis(), z_axis(), id(-1) {}

Tag::Tag(geometry_msgs::PoseStamped& p) : Tag(p, -1) {}
//NOTE this requires c++11 to compile

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

void Tag::setX(double x, double y, double z) {
	x_axis.x = x;
	x_axis.y = y;
	x_axis.z = z;
}

void Tag::setY(double x, double y, double z) {
	y_axis.x = x;
	y_axis.y = y;
	y_axis.z = z;
}

void Tag::setZ(double x, double y, double z) {
	z_axis.x = x;
	z_axis.y = y;
	z_axis.z = z;
}

void Tag::setID(int _id) { id = _id; }

Position Tag::getCenter() { return center; }
Quaternion Tag::getOrientation() { return orientation; }
Axis Tag::getX() { return x_axis; }
Axis Tag::getY() { return y_axis; }
Axis Tag::getZ() { return z_axis; }

Eigen::Vector3d Tag::getXvect() {
	Eigen::Vector3d v;
	v << x_axis.x, x_axis.y, x_axis.z;
	return v;
}

Eigen::Vector3d Tag::getYvect() {
	Eigen::Vector3d v;
	v << y_axis.x, y_axis.y, y_axis.z;
	return v;
}

Eigen::Vector3d Tag::getZvect() {
	Eigen::Vector3d v;
	v << z_axis.x, z_axis.y, z_axis.z;
	return v;
}

int Tag::getID() { return id; }

Eigen::Vector3d Tag::vect(Tag& otherTag) {
	Eigen::Vector3d this2that;
	this2that << otherTag.center.x - center.x,
	             otherTag.center.y - center.y,
	             otherTag.center.z - center.z;
	return this2that;
}

double Tag::dist(Tag& otherTag) {
	return vect(otherTag).norm();
}

bool Tag::operator<(const Tag& otherTag) const {
	return id < otherTag.id;
}

//YSS just for testing
std::string Tag::printCenter() {
	std::stringstream ss;
	ss << "(" << center.x;
	ss << ", " << center.y;
	ss << ", " << center.z << ")";
	return ss.str();
}

std::string Tag::printOrientation() {
	std::stringstream ss;
	ss << "(" << orientation.x;
	ss << ", " << orientation.y;
	ss << ", " << orientation.z;
	ss << ", " << orientation.w << ")";
	return ss.str();
}

std::string Tag::printX() {
	std::stringstream ss;
	ss << "(" << x_axis.x;
	ss << ", " << x_axis.y;
	ss << ", " << x_axis.z << ")";
	return ss.str();
}

std::string Tag::printY() {
	std::stringstream ss;
	ss << "(" << y_axis.x;
	ss << ", " << y_axis.y;
	ss << ", " << y_axis.z << ")";
	return ss.str();
}

std::string Tag::printZ() {
	std::stringstream ss;
	ss << "(" << z_axis.x;
	ss << ", " << z_axis.y;
	ss << ", " << z_axis.z << ")";
	return ss.str();
}

std::string Tag::printID() {
	std::stringstream ss;
	ss << "\\" << id;
	return ss.str();	
}

}