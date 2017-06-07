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
#include "tangible_msgs/Target.h"
#include "tangible_msgs/GetMatchingObjects.h"

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
// #include <pcl_ros/point_cloud.h>

#include "tf/transform_listener.h"
#include <Eigen/Dense>
#include <algorithm>
#include <math.h>
#include <vector>

namespace tangible {

class ObjectMatching {
private:

	ros::Publisher cloud_marker_pub;
	ros::Publisher box_marker_pub;
	ros::NodeHandle node_handle;
	void publishMarkers(); // TODO
	double point_location_threshold;
	double box_dimension_threshold;
	bool matchesPointLocation(tangible_msgs::SceneObject obj, tangible_msgs::Target target);
	int orientation(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r);
	bool doIntersect(geometry_msgs::Point p1, geometry_msgs::Point q1, 
		geometry_msgs::Point p2, geometry_msgs::Point q2);
	bool isInside(std::vector<geometry_msgs::PointStamped> corners, geometry_msgs::PoseStamped pose);
	bool matchesRegion(tangible_msgs::SceneObject obj, tangible_msgs::Target target);
	bool matchesObjSelector (tangible_msgs::SceneObject obj, tangible_msgs::Target target);
	bool onSegment(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r);


public:
	ObjectMatching(ros::NodeHandle& n, double point_location_thresh, double box_dimension_thresh);
	~ObjectMatching();

	bool matchCallback(tangible_msgs::GetMatchingObjects::Request& req,
                 tangible_msgs::GetMatchingObjects::Response& res);

};

};

#endif

//TO-DO publisher to publish scene elements. Would be helpful for debugging