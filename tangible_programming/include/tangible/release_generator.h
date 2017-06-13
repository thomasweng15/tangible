#ifndef TANGIBLE_RELEASE_GENERATOR
#define TANGIBLE_RELEASE_GENERATOR

#include <vector>
#include <algorithm>
#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "tangible_msgs/SetStaticTransform.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cstdlib>
#include <ctime>
// #include "stdlib.h"

#include "rapid_perception/scene.h"
#include "rapid_perception/scene_parsing.h"

#include "rapid_utils/math.h"

#include "tangible_msgs/BoundingBox.h"
#include "tangible_msgs/Scene.h"
#include "tangible_msgs/SceneObject.h"
#include "tangible_msgs/Surface.h"
#include "tangible_msgs/GetScene.h"
#include "tangible_msgs/GetReleases.h"
#include "tangible_msgs/Target.h"

#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/filters/filter.h"
#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "pcl/common/distances.h"
#include "pcl/common/pca.h"
#include "pcl/filters/filter.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/conditional_euclidean_clustering.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/region_growing_rgb.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/point_types_conversion.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/console/parse.h>

#include "moveit_msgs/Grasp.h"


namespace tangible {

struct Box {
	double min_x;
	double min_y;
	double min_z;
	double max_x;
	double max_y;
	double max_z;
};

class ReleaseGenerator {
private:

	ros::NodeHandle node_handle;
	void publishMarkers();

	ros::Publisher marker_pub;
	std::vector<moveit_msgs::Grasp> releases;
	tangible_msgs::SceneObject obj;
  tangible_msgs::Target target;
  int release_type;
  int num_orientations;
  double region_sample_spacing;
	//distance from wrist to gripper on PR2
	const static double palm_dist = 0.12;
	const static double pre_release_dist = 0.15;
	const static double post_release_dist = 0.15;
  const static double drop_offset = 0.15;
  // const static int min_points_in_gripper = 50;
  //  // max number of points in cluster that can intersect with fingers
  //  const static int max_finger_collision_points = 7;
  //  const static int max_palm_collision_points = 6;
  //  // approximately half hand thickness
  //  const static double half_gripper_height = 0.03;
  // approximate distance from palm frame origin to palm surface
  const static double dist_to_palm = 0.12;
  // approximate distance from palm frame origin to fingertip with gripper closed
  const static double dist_to_fingertips = 0.20;
  // distance above surface to release
  const static double vertical_release_offset = 0.03;
  // approx dist between fingers when gripper open
  // const static double gripper_palm_width = 0.08;
  // // approx height of pads of fingertips
  // const static double gripper_finger_height = 0.03;

  // const static double y_offset = 0.005;
	void getReleases();
  bool isInside(std::vector<geometry_msgs::PointStamped> corners, geometry_msgs::PoseStamped pose);
  bool matchesRegion(tangible_msgs::SceneObject obj, tangible_msgs::Target target);
  bool matchesObjSelector (tangible_msgs::SceneObject obj, tangible_msgs::Target target);
  float random(float low, float high);
	// bool hasCollision(moveit_msgs::Grasp grasp, sensor_msgs::PointCloud2 pc2);
	// int findPointsInBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Box box, std::string frame);
  ros::ServiceClient set_static_tf_client;
  geometry_msgs::PoseStamped poseFromVec(geometry_msgs::PoseStamped pose, geometry_msgs::Vector3 vec, float dist);


public:
	ReleaseGenerator(ros::NodeHandle& n);
	~ReleaseGenerator();

	bool releaseCallback(tangible_msgs::GetReleases::Request& req,
                 tangible_msgs::GetReleases::Response& res);

};

};

#endif

//TO-DO publisher to publish scene elements. Would be helpful for debugging
