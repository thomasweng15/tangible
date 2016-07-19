#ifndef TANGIBLE_SCENE_PARSER
#define TANGIBLE_SCENE_PARSER

#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

#include "rapid_perception/scene.h"
#include "rapid_perception/scene_parsing.h"

#include "rapid_utils/math.h"

namespace tangible {

class SceneParser {
private:
	ros::NodeHandle node;
	ros::Subscriber cloud_sub;

	bool successful;
	rapid::perception::Scene scene;

	void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	rapid::perception::ParseParams retrieveParams();
public:
	SceneParser(ros::NodeHandle& n);
	~SceneParser();

	bool isSuccessful();
	rapid::perception::HSurface getTableTop() const; // valid when isSuccessful is true
	std::vector<rapid::perception::Object> getObjects() const; // valid when isSuccessful is true

};

}

#endif

//TO-DO publisher to publish scene elements. Would be helpful for debugging