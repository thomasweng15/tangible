#include "tangible/scene_parser.h"

namespace tangible {

SceneParser::SceneParser(ros::NodeHandle& n, std::string frame) : scene() {
  node_handle = n;
	output_frame = frame;
  box_marker_pub = node_handle.advertise<visualization_msgs::Marker>("box_markers", 20);
  cloud_marker_pub = node_handle.advertise<sensor_msgs::PointCloud2>("cloud_markers", 20);

}

SceneParser::~SceneParser() {}

void SceneParser::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  
  // ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/head_mount_kinect/depth_registered/points", ros::Duration(20));
  ROS_INFO("got point cloud, frame_id = %s", (*msg).header.frame_id.c_str());
  sensor_msgs::PointCloud2 transformed_cloud_msg;
  tf::TransformListener tf_listener;
  tf::StampedTransform transform;
  tf_listener.waitForTransform(output_frame, (*msg).header.frame_id, ros::Time(0), ros::Duration(3.0));
  //ROS_INFO("... done waiting.");
  tf_listener.lookupTransform(output_frame, (*msg).header.frame_id, ros::Time(0), transform);
  pcl_ros::transformPointCloud(output_frame, transform, *msg, transformed_cloud_msg);
  pcl::fromROSMsg(transformed_cloud_msg, *cloud);

	successful = rapid::perception::ParseScene(cloud, retrieveParams(), &scene);
}

bool SceneParser::parseCallback(tangible_msgs::GetScene::Request& req,
                 tangible_msgs::GetScene::Response& res)
{
    ROS_INFO("Got service call!");
	 tangible_msgs::Scene scene_msg;
    scene_msg = sceneToMsg(scene);
  	res.scene = scene_msg;
    publishMarkers(scene_msg);
  	return true;
}

void SceneParser::publishMarkers(tangible_msgs::Scene scene_msg){
  for (int i = 0; i < scene_msg.objects.size(); i++){

    tangible_msgs::SceneObject obj = scene_msg.objects[i];

    cloud_marker_pub.publish(obj.point_cloud);

    uint32_t shape = visualization_msgs::Marker::CUBE;

visualization_msgs::Marker marker;
       // Set the frame ID and timestamp.  See the TF tutorials for information on these.
       marker.header.frame_id = output_frame;
       marker.header.stamp = ros::Time::now();

       // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
       marker.id = i;
       marker.ns = "bounding_box";
       // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
       marker.type = shape;

       // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
     marker.action = visualization_msgs::Marker::ADD;
       // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
 //ros::Subscriber sub = n.subscribe("range_data", 1000,rangeCallback);
       marker.pose = obj.bounding_box.pose.pose;

       // Set the scale of the marker -- 1x1x1 here means 1m on a side
       marker.scale = obj.bounding_box.dimensions;

       // Set the color -- be sure to set alpha to something non-zero!
       marker.color.r = 0.20f;
       marker.color.g = 0.3f;
       marker.color.b = 0.2f;
       marker.color.a = 0.5;

       marker.lifetime = ros::Duration();

       box_marker_pub.publish(marker);

  }


    uint32_t shape = visualization_msgs::Marker::CUBE;

visualization_msgs::Marker marker;
       // Set the frame ID and timestamp.  See the TF tutorials for information on these.
       marker.header.frame_id = output_frame;
       marker.header.stamp = ros::Time::now();

       // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
       marker.id = 0;

       // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
       marker.type = shape;

       // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
     marker.action = visualization_msgs::Marker::ADD;
       // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
 //ros::Subscriber sub = n.subscribe("range_data", 1000,rangeCallback);
       marker.pose = scene_msg.surface.bounding_box.pose.pose;

       // Set the scale of the marker -- 1x1x1 here means 1m on a side
       marker.scale = scene_msg.surface.bounding_box.dimensions;

       // Set the color -- be sure to set alpha to something non-zero!
       marker.color.r = 0.3f;
       marker.color.g = 0.2f;
       marker.color.b = 0.3f;
       marker.color.a = 0.3;

       marker.lifetime = ros::Duration();

       //box_marker_pub.publish(marker);

}

tangible_msgs::Scene SceneParser::sceneToMsg(rapid::perception::Scene scene){
	tangible_msgs::Scene scene_msg;
	tangible_msgs::Surface surface_msg;
	std::vector<rapid::perception::Object> objects = getObjects();
	for (int i = 0; i < objects.size(); i++){
		tangible_msgs::SceneObject obj_msg;
		sensor_msgs::PointCloud2 pc2;
		pcl::PointCloud<pcl::PointXYZRGB> pc_filtered;
  		std::vector<int> index;
		tangible_msgs::BoundingBox bbox;
  		
  		rapid::perception::Object obj = objects[i];
  		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc = obj.GetCloud();
  		pcl::removeNaNFromPointCloud(*pc, pc_filtered, index);
  		pcl::toROSMsg(pc_filtered, pc2 );
  		obj_msg.point_cloud =  pc2;
  		getBoundingBox(pc_filtered, &bbox);
  		obj_msg.bounding_box = bbox;
  		scene_msg.objects.push_back(obj_msg);
	}
	rapid::perception::HSurface surface = getTableTop();
	surface_msg.bounding_box.pose = surface.pose();
	surface_msg.bounding_box.dimensions = surface.scale();
	scene_msg.surface = surface_msg;
	return scene_msg;
}

void SceneParser::getBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB> pc, tangible_msgs::BoundingBox* bbox){

	// Get planar bounding box.
	geometry_msgs::Pose planar_bbox_midpoint;
	geometry_msgs::Vector3 planar_bbox_dimensions;
	getPlanarBoundingBox(pc, &planar_bbox_midpoint,
	                   &planar_bbox_dimensions);
	geometry_msgs::PoseStamped planar_bbox_midpoint_stamped;
	planar_bbox_midpoint_stamped.header.frame_id = output_frame;
	planar_bbox_midpoint_stamped.pose = planar_bbox_midpoint;
	bbox->pose = planar_bbox_midpoint_stamped;
	bbox->dimensions = planar_bbox_dimensions;
}

// taken from pr2_pick_perception
void SceneParser::getPlanarBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                          geometry_msgs::Pose* midpoint,
                          geometry_msgs::Vector3* dimensions) {
  // Project points onto XY plane.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));
  for (size_t i = 0; i < projected->points.size(); ++i) {
    pcl::PointXYZRGB& point = projected->at(i);
    point.z = 0;
  }
  // Compute PCA.
  pcl::PCA<pcl::PointXYZRGB> pca(true);
  pca.setInputCloud(projected);
  // Get eigenvectors.
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
  // Because we projected points on the XY plane, we add in the Z vector as the
  // 3rd eigenvector.
  eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));
  Eigen::Quaternionf q1(eigenvectors);
  // Find min/max x and y, based on the points in eigenspace.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr eigen_projected(
      new pcl::PointCloud<pcl::PointXYZRGB>(cloud));
  pca.project(cloud, *eigen_projected);
  pcl::PointXYZRGB eigen_min;
  pcl::PointXYZRGB eigen_max;
  pcl::getMinMax3D(*eigen_projected, eigen_min, eigen_max);
  double x_length = eigen_max.x - eigen_min.x;
  double y_length = eigen_max.y - eigen_min.y;
  // The points in eigenspace all have z values of 0. Get min/max z from the
  // original point cloud data.
  pcl::PointXYZRGB cloud_min;
  pcl::PointXYZRGB cloud_max;
  pcl::getMinMax3D(cloud, cloud_min, cloud_max);
  double z_length = cloud_max.z - cloud_min.z;
  // Compute midpoint, defined as the midpoint between the minimum and maximum
  // points, in x, y, and z directions. The centroid is an average that depends
  // on the density of points, which doesn't give you the geometric center of
  // the point cloud.
  pcl::PointXYZRGB eigen_center;
  eigen_center.x = eigen_min.x + x_length / 2;
  eigen_center.y = eigen_min.y + y_length / 2;
  eigen_center.z = 0;
  pcl::PointXYZRGB center;
  pca.reconstruct(eigen_center, center);
  center.z = z_length / 2 + cloud_min.z;
  // Output midpoint.
  midpoint->position.x = center.x;
  midpoint->position.y = center.y;
  midpoint->position.z = center.z;
  midpoint->orientation.w = q1.w();
  midpoint->orientation.x = q1.x();
  midpoint->orientation.y = q1.y();
  midpoint->orientation.z = q1.z();
  // Output dimensions.
  dimensions->x = x_length;
  dimensions->y = y_length;
  dimensions->z = z_length;
}

rapid::perception::ParseParams SceneParser::retrieveParams() {
	rapid::perception::ParseParams params;
	ros::param::param("parse/scene/min_x", params.scene.min_x, 0.2);
	ros::param::param("parse/scene/max_x", params.scene.max_x, 1.2);
	ros::param::param("parse/scene/min_y", params.scene.min_y, -1.0);
	ros::param::param("parse/scene/max_y", params.scene.max_y, 1.0);
	ros::param::param("parse/scene/min_z", params.scene.min_z, 0.3);
	ros::param::param("parse/scene/max_z", params.scene.max_z, 1.7);
	ros::param::param("parse/hsurface/distance_threshold",
		               params.hsurface.distance_threshold, 0.015);
	ros::param::param("parse/hsurface/eps_angle",
		               params.hsurface.eps_angle, rapid::utils::DegreesToRadians(5));
	ros::param::param("parse/objects/distance_threshold",
		               params.objects.distance_threshold, 0.05);
	ros::param::param("parse/objects/point_color_threshold",
		               params.objects.point_color_threshold, (double) 35);
	ros::param::param("parse/objects/region_color_threshold",
		               params.objects.region_color_threshold, (double) 20);
	ros::param::param("parse/objects/min_cluster_size", 
		               params.objects.min_cluster_size, (double) 38);
	return params;
}

bool SceneParser::isSuccessful() { return successful; }

rapid::perception::HSurface SceneParser::getTableTop() const { return scene.primary_surface(); }

std::vector<rapid::perception::Object> SceneParser::getObjects() const { 
	return getTableTop().objects();
}

}
