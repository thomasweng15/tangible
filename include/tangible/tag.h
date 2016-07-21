#ifndef TANGIBLE_TAG
#define TANGIBLE_TAG

#include "Eigen/Geometry"

#include "geometry_msgs/PoseStamped.h"

namespace tangible {

struct Position {
	double x;
	double y;
	double z;
};

struct Quaternion {
	double x;
	double y;
	double z;
	double w;
};

struct Axis {
	double x;
	double y;
	double z;
};

class Tag {
private:
	Position center;
	Quaternion orientation;
	Axis x_axis;
	Axis y_axis;
	Axis z_axis;
	int id;
public:
	Tag();
	Tag(geometry_msgs::PoseStamped& p);
	Tag(geometry_msgs::PoseStamped& p, int _id);
	~Tag();

	void setCenter(geometry_msgs::PoseStamped& p);
	void setOrientation(geometry_msgs::PoseStamped& p);
	void setAxes(geometry_msgs::PoseStamped& p);
	void setID(int _id);

	Position getCenter();
	Quaternion getOrientation();
	Axis getX();
	Axis getY();
	Axis getZ();
	Eigen::Vector3d getXvect();
	Eigen::Vector3d getYvect();
	Eigen::Vector3d getZvect();
	int getID();

	Eigen::Vector3d vect(Tag& otherTag);
	double dist(Tag& otherTag);
};

}

#endif
