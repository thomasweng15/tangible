#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h> 
#include <visualization_msgs/Marker.h>

#include "pcl/point_cloud.h"
#include "pcl_ros/transforms.h" // pcl::transformPointCloud
#include "pcl_conversions/pcl_conversions.h" // pcl::fromROSMsg

#include "rapid_perception/scene.h"
#include "rapid_perception/scene_parsing.h"

#include "rapid_utils/math.h"

#include "rapid_viz/markers.h"

#include "ar_track_alvar_msgs/AlvarMarkers.h"

#include <vector>
#include <sstream>

class Visualization {
private:
	const static int ADD = 0;
	const static int MODIFY = 1;
	const static int DELETE = 2;
	const static int TAG_NUM = 18;
	std::string FRAME_ID;

	ros::NodeHandle node;
	tf::TransformListener tf_listener;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	rapid::perception::Scene scene;
	rapid::perception::ParseParams params;
	
	ros::Subscriber pcl_sub;
	ros::Publisher cloud_pub;
	std::vector<visualization_msgs::Marker> scene_elements;
	ros::Publisher scene_element_pub;

	ros::Subscriber ar_marker_sub;
	visualization_msgs::Marker labels[TAG_NUM];
	ros::Publisher label_pub;

	void segmentationCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
		readPointCloud(msg);
		bool success = parseScene();
		if(!success) {
			ROS_ERROR("failed to parse the scene.");
			for(int i = 0; i < scene_elements.size(); i++) {
				setAction(scene_elements.at(i), DELETE);
				scene_element_pub.publish(scene_elements.at(i));
				scene_elements.pop_back();
			}
		} else {
			ROS_INFO("displaying objects.");
			displayElements();
		}
		publishCloud();
	}

	void readPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
		sensor_msgs::PointCloud2 transformed;
		tf_listener.waitForTransform("/"+FRAME_ID, msg->header.frame_id, ros::Time(0), ros::Duration(10));
		pcl_ros::transformPointCloud("/"+FRAME_ID, *msg, transformed, tf_listener);
		pcl::fromROSMsg(transformed, *cloud);
	}

	bool parseScene() {
		setParseParams();
		return rapid::perception::ParseScene(cloud, params, &scene);
	}

	void setParseParams() {
		params.scene.min_x = 0.2;
		params.scene.max_x = 1.2;
		params.scene.min_y = -1;
		params.scene.max_y = 1;
		params.scene.min_z = 0.3;
		params.scene.max_z = 1.7;
		params.hsurface.distance_threshold = 0.015;
		params.hsurface.eps_angle = rapid::utils::DegreesToRadians(5);
		params.objects.distance_threshold = 0.05;
		params.objects.point_color_threshold = 35;
		params.objects.region_color_threshold = 20;
		params.objects.min_cluster_size = 38;
	}

	void publishCloud() {
		sensor_msgs::PointCloud2 c;
		pcl::toROSMsg(*cloud, c);
		c.header.stamp = ros::Time::now();
		cloud_pub.publish(c);
	}

	void displayElements() {
		const rapid::perception::HSurface& table_top = scene.primary_surface();
		
		if(scene_elements.size() == 0) {
			ROS_INFO("Filling scene element stack afresh...");
			// box
			visualization_msgs::Marker table_top_box = 
			    rapid::viz::Marker::Box(NULL, table_top.pose(), table_top.scale()).marker();
			setNamespace(table_top_box, "scene_visualization");
			setID(table_top_box, 0);
			setAction(table_top_box, ADD);
			scene_elements.push_back(table_top_box);


			// lable
			visualization_msgs::Marker table_top_label = 
			    rapid::viz::Marker::Text(NULL, table_top.pose(), "table top", 0.03).marker();
			setNamespace(table_top_label, "scene_visualization");
			setID(table_top_label, 1);
			setColor(table_top_label, 1, 0, 0, 1);
			setAction(table_top_label, ADD);
			scene_elements.push_back(table_top_label);

			// TO-DO add objects segmented on the table top
		} else {
			// TO-DO objects that are no longer segmented should be deleted
			//       new objects should be added
		}

		for(int i = 0; i < scene_elements.size(); i++) {
			scene_element_pub.publish(scene_elements.at(i));
		}
	}

	void labelCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
		int found[TAG_NUM];
		for(int i = 0; i < TAG_NUM; i++)
			found[i] = -1;

		for(int i = 0; i < msg->markers.size(); i++)
			found[msg->markers[i].id] = i;

		std::stringstream ss;
		for(int i = 0; i < TAG_NUM; i++) {
			ss.str(std::string());
			ss << "tag " << i;
			if(found[i] == -1) setAction(labels[i], DELETE);
			else {
				geometry_msgs::PoseStamped ps = msg->markers[found[i]].pose;
				ps.header.frame_id = msg->markers[found[i]].header.frame_id;
				fillLabel(labels[i], ps, ss.str());
				setAction(labels[i], ADD);
			}
			label_pub.publish(labels[i]);
		}
	}

	void fillLabel(visualization_msgs::Marker& marker, geometry_msgs::PoseStamped ps, std::string txt) {
		setPose(marker, ps);
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		geometry_msgs::Vector3 scale;
		scale.z = 0.03;
		setScale(marker, scale);
		setColor(marker, 1, 0, 0, 1);
		marker.text = txt;
	}

	void setPose(visualization_msgs::Marker& marker, geometry_msgs::PoseStamped ps) {
		marker.header = ps.header;
		marker.pose = ps.pose;
	}

	void setNamespace(visualization_msgs::Marker& marker, std::string name){
		marker.ns = name;
	}

	void setID(visualization_msgs::Marker& marker, int ID) {
		marker.id = ID;
	}

	void setAction(visualization_msgs::Marker& marker, int action) {
		switch(action){
			case ADD:
				marker.action = visualization_msgs::Marker::ADD;
				break;
			case MODIFY:
				marker.action = visualization_msgs::Marker::MODIFY;
				break;
			case DELETE:
				marker.action = visualization_msgs::Marker::DELETE;
			default:
				marker.action = visualization_msgs::Marker::DELETE;
				break;
		}
	}

	void setScale(visualization_msgs::Marker& marker, geometry_msgs::Vector3 scale) {
		marker.scale = scale;
	}

	void setColor(visualization_msgs::Marker& marker, double r, double g, double b, double a) {
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		marker.color.a = a;
	}

public:
	Visualization(ros::NodeHandle& n) : cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
	                                    FRAME_ID("base_footprint"),
	                                    tf_listener() {
		node = n;

		ROS_INFO("\nallow time for TF buffer to build up the TF tree...");
		ros::Duration(10).sleep();

		pcl_sub = node.subscribe("/cloud_in", 10, &Visualization::segmentationCallback, this);
		cloud_pub = node.advertise<sensor_msgs::PointCloud2>("cloud_viz", 100);
		scene_element_pub = node.advertise<visualization_msgs::Marker>("scene_viz", 100);

		ar_marker_sub = node.subscribe("/ar_pose_marker", 10, &Visualization::labelCallback, this);
		label_pub = node.advertise<visualization_msgs::Marker>("ar_label_viz", 10);
		for(int i = 0; i < TAG_NUM; i++){
			setNamespace(labels[i], "scene_visualization");
			setID(labels[i], i);
		}
		//YSS: where are the pcl deleted?
		//     they are shared pointers so pcl handles deletion itself
	}
	
	~Visualization() { //YSS: this is not called upon Ctrl+c. Why? How to force its call?

		ROS_INFO("removing the labels and objects (if any) upon exit...");
		for(int i = 0; i < TAG_NUM; i++) {
			setAction(labels[i], DELETE);
			label_pub.publish(labels[i]);
		}
		for(int i=0; i < scene_elements.size(); i++) {
			setAction(scene_elements.at(i), DELETE);
			scene_element_pub.publish(scene_elements.at(i));
		}
		ROS_INFO("done.");
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "scene_visualization");
	ros::NodeHandle node;
	Visualization viz(node);
	ros::spin();
	return 0;
}