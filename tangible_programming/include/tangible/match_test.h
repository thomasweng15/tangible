#ifndef TANGIBLE_MATCH_TEST
#define TANGIBLE_MATCH_TEST

#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "rapid_perception/scene.h"
#include "rapid_perception/scene_parsing.h"

#include "rapid_utils/math.h"

#include "tangible_msgs/BoundingBox.h"
#include "tangible_msgs/Scene.h"
#include "tangible_msgs/SceneObject.h"
#include "tangible_msgs/Surface.h"
#include "tangible_msgs/GetScene.h"
#include "tangible_msgs/GetGrasps.h"
#include "tangible_msgs/GetMatchingObjects.h"
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

class MatchTest {
private:

	ros::NodeHandle node_handle;
  ros::ServiceClient scene_client;
  ros::ServiceClient match_client;


public:
	MatchTest(ros::NodeHandle& n, std::string scene_service_name, std::string object_matching_service_name);
	~MatchTest();

	bool testCallback(std_srvs::Empty::Request& req,
                 std_srvs::Empty::Response& res);

};

};

#endif

//TO-DO publisher to publish scene elements. Would be helpful for debugging