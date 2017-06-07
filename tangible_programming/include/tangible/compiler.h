#ifndef COMPILER_EXTRACTOR
#define COMPILER_EXTRACTOR

#include <vector>

#include "ros/ros.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#include "tangible_msgs/GetProgram.h"
#include "tangible_msgs/GetBlocks.h"
#include "tangible_msgs/GetScene.h"
#include "tangible_msgs/Target.h"
#include "tangible_msgs/Block.h"
#include "tangible_msgs/Instruction.h"
#include "tangible_msgs/Program.h"
#include "tangible_msgs/Mode.h"
#include "Eigen/Geometry"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

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

# include "math.h"

namespace tangible {


class Compiler {
private:
	ros::NodeHandle node_handle;

	int edit_id;
	int idle_id;
	int run_id;
	bool edit_state;
	bool idle_state;
	bool run_state;
	tangible_msgs::Program program;
	tangible_msgs::Program working_program;
	ros::ServiceClient block_client;
	ros::ServiceClient scene_client;
	std::string error_msg;
	bool inRange(int LB, int UB, int number);

	bool inRange(double LB, double UB, double number);

	Eigen::Vector3d getXvect(const tangible_msgs::Block &a);

	Eigen::Vector3d getYvect(const tangible_msgs::Block &a);

	Eigen::Vector3d getZvect(const tangible_msgs::Block &a);

	Eigen::Vector3d vect(const tangible_msgs::Block &a, const tangible_msgs::Block &b);

	int blockToType(const tangible_msgs::Block &a);

	// bool blockLessThan(const tangible_msgs::Block &a, const tangible_msgs::Block &b);

	double blockDist(const tangible_msgs::Block &a, const tangible_msgs::Block &b );

	std::vector<geometry_msgs::PointStamped> getRegionCorners(const tangible_msgs::Block &a, const tangible_msgs::Block &b );
	bool getIntersection(std::vector<geometry_msgs::Point> line1, 
                              std::vector<geometry_msgs::Point> line2,
                              geometry_msgs::Point* p);

	double getQuadArea(geometry_msgs::Point p1, geometry_msgs::Point p2, 
                            geometry_msgs::Point p3, geometry_msgs::Point p4);

	geometry_msgs::PointStamped getPoint(const tangible_msgs::Block &a);
	bool tags2program(std::vector<tangible_msgs::Block> blocks);
	bool addObjects(tangible_msgs::Scene scene);
	void setupFilterBox(pcl::CropBox<pcl::PointXYZRGB>& cbox, tangible_msgs::Instruction* instruction);
	int filterObject(pcl::CropBox<pcl::PointXYZRGB>& cbox,
                        tangible_msgs::SceneObject& obj);
	bool addObject(tangible_msgs::Scene scene, tangible_msgs::Instruction* instruction);
	const static double MAX_WORKSPACE_DIST = 1;
	const static double MAX_WORKSPACE_HEIGHT = 0.3;

	const static double DIST_ERR_MARGIN = 0.02;
	const static double ROTATE_ERR_MARGIN = 0.05;

	const static int MIN_POINT_OVERLAP = 15;
	const static double MIN_REGION_OVERLAP_RATIO = 0.5;
	const static double OBJECT_SELECTION_BOX_SIZE = 0.05;
	ros::Publisher box_marker_pub;
  	ros::Publisher cloud_marker_pub;
  	void publishCorner(geometry_msgs::PointStamped point, int id);
  	void publishMarkers();

public:
	Compiler(ros::NodeHandle& n, int i_id, int r_id, int e_id, 
				std::string block_service_name, std::string scene_service_name);
	~Compiler();
	void modeCallback(const tangible_msgs::Mode::ConstPtr msg);
	bool programCallback(tangible_msgs::GetProgram::Request& req,
                 tangible_msgs::GetProgram::Response& res);
	void compile();

};

}

#endif

//TO-DO publisher to publish tag information. Would be helpful for debugging