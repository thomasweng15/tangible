#ifndef TANGIBLE_FRAME_TRANSFORMER
#define TANGIBLE_FRAME_TRANSFORMER

#include <string>

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "pcl_conversions/pcl_conversions.h"

#include "ar_track_alvar_msgs/AlvarMarkers.h"

namespace tangible {

class FrameTransformer {
private:
	std::string frame_id;

	ros::NodeHandle node;
	tf::TransformListener tf_listener;

	ros::Subscriber ar_sub;
	ros::Publisher ar_pub;
	ros::Subscriber pcl_sub;
	ros::Publisher pcl_pub;

	void PCLcallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void ARcallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

public:
	FrameTransformer (ros::NodeHandle& n, std::string id);
	~FrameTransformer ();
};

}

#endif