#include "tangible/scene_parser.h"

namespace tangible {

SceneParser::SceneParser(ros::NodeHandle& n) : scene() { //YSS: initializing scene?
	node = n;
	cloud_sub = node.subscribe("/cloud_transformed", 10, &SceneParser::cloudCallback, this);
	successful = false;
}

SceneParser::~SceneParser() {}

void SceneParser::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*msg, *cloud);
	successful = rapid::perception::ParseScene(cloud, retrieveParams(), &scene);
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