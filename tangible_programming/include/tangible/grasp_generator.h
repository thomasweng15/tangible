#ifndef TANGIBLE_SCENE_PARSER
#define TANGIBLE_SCENE_PARSER

#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

#include <tf/transform_listener.h>

#include "rapid_perception/scene.h"
#include "rapid_perception/scene_parsing.h"

#include "rapid_utils/math.h"

#include "tangible_msgs/BoundingBox.h"
#include "tangible_msgs/Scene.h"
#include "tangible_msgs/SceneObject.h"
#include "tangible_msgs/Surface.h"
#include "tangible_msgs/GetScene.h"
#include "tangible_msgs/GetGrasps.h"

#include "moveit_msgs/Grasp.h"


namespace tangible {

class GraspGenerator {
private:

	ros::NodeHandle node_handle;
	void publishMarkers(tangible_msgs::Scene scene);

	ros::Publisher marker_pub;
	std::vector<moveit_msgs::Grasp> grasps;
	tangible_msgs::Scene scene;
	//distance from wrist to gripper on PR2
  	const static double palm_dist = 0.12;
  	const static double pre_grasp_dist = 0.15;
  	const static double post_grasp_dist = 0.15;
  	void getGrasps();


public:
	GraspGenerator(ros::NodeHandle& n);
	~GraspGenerator();

	bool graspCallback(tangible_msgs::GetGrasps::Request& req,
                 tangible_msgs::GetGrasps::Response& res);

};

};

#endif

//TO-DO publisher to publish scene elements. Would be helpful for debugging