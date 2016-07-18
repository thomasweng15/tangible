#include "tangible/frame_transform.h"

namespace tangible {

FrameTransform::FrameTransform(ros::NodeHandle& n, std::string id) : tf_listener() {
	node = n;
	frame_id = id;
	ROS_INFO("\n******* Allow time for TF buffer to build up the TF tree *******\n");
	ros::Duration(10).sleep();

	pcl_sub = node.subscribe("/cloud_in", 10, &FrameTransform::PCLcallback, this);
	//NOTE: does not require full addressing (tangible::FrameTransform)
	//      as its under the same name space
	pcl_pub = node.advertise<sensor_msgs::PointCloud2>("cloud_transformed", 10);

	ar_sub = node.subscribe("/ar_pose_marker", 10, &FrameTransform::ARcallback, this);
	ar_pub = node.advertise<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_transformed", 10);

}

FrameTransform::~FrameTransform() {
	ROS_INFO("FrameTransform destructor called.");
	//NOTE: this is not called upon
	//        - Ctrl+C
	//        - rosnode kill <node_name> 
}

void FrameTransform::PCLcallback(const sensor_msgs::PointCloud2::ConstPtr& msg)  {
	sensor_msgs::PointCloud2 transformedCloud;
	tf_listener.waitForTransform("/"+frame_id,
		                        msg->header.frame_id,
		                        ros::Time(0),
		                        ros::Duration(10));
	pcl_ros::transformPointCloud("/"+frame_id, *msg, transformedCloud, tf_listener);
	pcl_pub.publish(transformedCloud);
}

void FrameTransform::ARcallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
	ar_track_alvar_msgs::AlvarMarkers transformedMarkers;
	for(int i = 0; i < msg->markers.size(); i++) {
		// filling in the frame_id of each marker pose (ar_track_alvar lists frame_id
		// under the header of marker array msg but not each pose in the array)
		geometry_msgs::PoseStamped original = msg->markers[i].pose;
		original.header.frame_id = msg->markers[i].header.frame_id;

		geometry_msgs::PoseStamped transformedPose;
		tf_listener.transformPose("/"+frame_id, original, transformedPose);
		//TO-DO look into pcl_ros::tranformPointCloud

		ar_track_alvar_msgs::AlvarMarker ar_marker;
		ar_marker.header = msg->markers[i].header;
		ar_marker.header.frame_id = "/"+frame_id;
		ar_marker.id = msg->markers[i].id;
		ar_marker.confidence = msg->markers[i].confidence;
		ar_marker.pose = transformedPose;
		transformedMarkers.markers.push_back(ar_marker);
	}
	ar_pub.publish(transformedMarkers);
}

}