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

#include "tangible/BoundingBox.h"
#include "tangible/Scene.h"
#include "tangible/SceneObject.h"
#include "tangible/Surface.h"
#include "tangible/GetScene.h"
#include "tangible/GetGrasps.h"

#include "moveit_msgs/Grasp.h"


namespace tangible {

class GraspGenerator {
private:

	ros::NodeHandle node_handle;
	void publishMarkers(tangible::Scene scene);

	ros::Publisher marker_pub;
	std::vector<moveit_msgs::Grasp> grasps;
	tangible::Scene scene;
	//distance from wrist to gripper on PR2
  	const static double palm_dist = 0.12;
  	const static double pre_grasp_dist = 0.15;
  	const static double post_grasp_dist = 0.15;
  	void getGrasps();


public:
	GraspGenerator(ros::NodeHandle& n);
	~GraspGenerator();

	bool graspCallback(tangible::GetGrasps::Request& req,
                 tangible::GetGrasps::Response& res);

};

};

#endif

//TO-DO publisher to publish scene elements. Would be helpful for debugging