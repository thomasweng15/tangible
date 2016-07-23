#ifndef TANGIBLE_TAG
#define TANGIBLE_TAG

#include <string>

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
	const static double EDGE_SIZE = 3;
	// ID of selection tags {0, 1, 2, 3}
	const static int SELECTION_ID_MIN = 0;
	const static int SELECTION_ID_MAX = 3;
	// ID of secondary selection tag = 4
	const static int SELECTION_2ND_ID = 4;
	// ID of action tags {5, 6, 7, 8}
	const static int ACTION_ID_MIN = 5;
	const static int ACTION_ID_MAX = 8;
	// ID of number tags {9, 10, ..., 15}
	const static int NUMBER_ID_MIN = 9;
	const static int NUMBER_ID_MAX = 15;

	Tag();
	Tag(geometry_msgs::PoseStamped& p);
	Tag(geometry_msgs::PoseStamped& p, int _id);
	~Tag();

	void setCenter(geometry_msgs::PoseStamped& p);
	void setOrientation(geometry_msgs::PoseStamped& p);
	void setAxes(geometry_msgs::PoseStamped& p);
	void setX(double x, double y, double z);
	void setY(double x, double y, double z);
	void setZ(double x, double y, double z);
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

	bool operator<(const Tag& otherTag) const;

	
	std::string printCenter();
	std::string printOrientation();
	std::string printX();
	std::string printY();
	std::string printZ();
	std::string printID();

};

}

#endif
