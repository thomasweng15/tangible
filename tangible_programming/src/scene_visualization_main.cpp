#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h" 
#include "visualization_msgs/Marker.h"

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
	ros::Publisher transformed_marker_pub;

	void segmentationCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
		readPointCloud(msg);
		bool success = parseScene();
		if(!success) {
			ROS_ERROR("failed to parse the scene.");
			for(int i = 0; i < scene_elements.size(); i++) {
				setAction(scene_elements.at(i), DELETE);
				scene_element_pub.publish(scene_elements.at(i));
				scene_elements.pop_back();
				//TO-DO test
			}
		} else
			displayElements();
		publishCloud();
	}

	void readPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
		sensor_msgs::PointCloud2 transformed;
		tf_listener.waitForTransform("/"+FRAME_ID, msg->header.frame_id, ros::Time(0), ros::Duration(10));
		pcl_ros::transformPointCloud("/"+FRAME_ID, *msg, transformed, tf_listener);
		pcl::fromROSMsg(transformed, *cloud);
	}

	bool parseScene() {
		retrieveParams();
		return rapid::perception::ParseScene(cloud, params, &scene);
	}

	void retrieveParams() {
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
	}

	void publishCloud() {
		sensor_msgs::PointCloud2 c;
		pcl::toROSMsg(*cloud, c);
		c.header.stamp = ros::Time::now();
		cloud_pub.publish(c);
	}

	void displayElements() {
		const rapid::perception::HSurface& table_top = scene.primary_surface();
		//ROS_INFO("***** scene info *****\nTable top (%2.2f, %2.2f, %2.2f) with %3d objects",
		//	                                        table_top.pose().pose.position.x,
		//	                                        table_top.pose().pose.position.y,
		//	                                        table_top.pose().pose.position.z,
		//	                                        (int)(table_top.objects().size()));

		if(scene_elements.size() == 0) {
			ROS_INFO("Filling scene element stack afresh...");
			// box
			visualization_msgs::Marker table_top_box = 
			    rapid::viz::Marker::Box(NULL, table_top.pose(), table_top.scale()).marker();
			setNamespace(table_top_box, "scene_visualization");
			setColor(table_top_box,0, 1, 0, 0.25);
			setID(table_top_box, 0);
			scene_elements.push_back(table_top_box);
			
			// label
			visualization_msgs::Marker table_top_label = 
			    rapid::viz::Marker::Text(NULL, table_top.pose(), "table top", 0.03).marker();
			setNamespace(table_top_label, "scene_visualization");
			setID(table_top_label, 1);
			setColor(table_top_label, 1, 1, 1, 1);
			scene_elements.push_back(table_top_label);
			//YSS: table_top_box or table_top_label are defined on stack memory.
			//     How their push_back into a structure accessible outside the current scope
			//     is handled?
		} else {
			setPose(scene_elements.at(0), table_top.pose());
			setScale(scene_elements.at(1), table_top.scale());
		}
		setAction(scene_elements.at(0), ADD);
		setAction(scene_elements.at(1), ADD);

		const std::vector<rapid::perception::Object>& objects = table_top.objects();
		ROS_INFO("displaying %3d objects", (int)(objects.size()));

		//for(int i = 0; i < objects.size(); i++) {
		//	ROS_INFO("object-%02d (%2.2f, %2.2f, %2.2f)", i, 
		//		                objects.at(i).pose().pose.position.x,
		//		                objects.at(i).pose().pose.position.y,
		//		                objects.at(i).pose().pose.position.z);
		//}
		
		int prevObjNum = scene_elements.size()/2 - 1;
		int currObjNum = objects.size();
		std::stringstream ss;

		//ROS_INFO("%3d objects now, %3d object last time", currObjNum, prevObjNum);

		for(int i = 0; i < currObjNum; i++) {
			if(i < prevObjNum) { // update info
				//ROS_INFO("updating object %2d box and label at %2d, %2d", i, 2+i*2, 2+i*2+1);
				// box
				setPose(scene_elements.at(2+i*2), objects.at(i).pose());
				setScale(scene_elements.at(2+i*2), objects.at(i).scale());

				// label
				setPose(scene_elements.at(2+i*2+1), objects.at(i).pose());
				setScale(scene_elements.at(2+i*2+1), objects.at(i).scale());
			} else { // add new markers
				//ROS_INFO("the  new object %2d box and label at %2d, %2d", i, 2+i*2, 2+i*2+1);
				// box
				visualization_msgs::Marker box =
				    rapid::viz::Marker::Box(NULL, objects.at(i).pose(), objects.at(i).scale()).marker();
				setNamespace(box, "scene_visualization");
				setID(box, 2+i*2);
				setColor(box, 0.5, 0, 0.5, 0.45);
				scene_elements.push_back(box);

				// label
				ss.str(std::string());
				ss << "object_" << i;
				visualization_msgs::Marker label = 
				    rapid::viz::Marker::Text(NULL, objects.at(i).pose(), ss.str(), 0.02).marker();
				setNamespace(label, "scene_visualization");
				setID(label, 2+i*2+1);
				setColor(label, 0, 0, 1, 1);
				scene_elements.push_back(label);
			}
			setAction(scene_elements.at(2+i*2), ADD);
			setAction(scene_elements.at(2+i*2+1), ADD);
		}

		// objects no longer existing should not be displayed
		for(int i = currObjNum; i < prevObjNum; i++) {
			setAction(scene_elements.at(2+i*2), DELETE);
			setAction(scene_elements.at(2+i*2+1), DELETE);
		}

		//for(int i = 0; i < scene_elements.size(); i++)
		//	scene_element_pub.publish(scene_elements.at(i));
		//YSS: visualizing boxes only
		for(int i=0; i < scene_elements.size(); i+=2)
			scene_element_pub.publish(scene_elements.at(i));

		// markers associated with objects no longer existing should be removed
		for(int i = prevObjNum; i > currObjNum; i--){
			scene_elements.pop_back(); // label
			scene_elements.pop_back(); // box
		}

		//NOTE: there might be jumps in labels.
	}

	void ARMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
		int found[TAG_NUM];
		for(int i = 0; i < TAG_NUM; i++)
			found[i] = -1;

		for(int i = 0; i < msg->markers.size(); i++)
			found[msg->markers[i].id] = i;

		std::stringstream ss;
		for(int i = 0; i < TAG_NUM; i++) {
			ss.str(std::string());
			ss << "tag_" << i;
			if(found[i] == -1) setAction(labels[i], DELETE);
			else {
				geometry_msgs::PoseStamped ps = msg->markers[found[i]].pose;
				ps.header.frame_id = msg->markers[found[i]].header.frame_id;
				fillLabel(labels[i], ps, ss.str());
				setAction(labels[i], ADD);
			}
			label_pub.publish(labels[i]);
		}

		ar_track_alvar_msgs::AlvarMarkers transformedMarkers;
		for(int i = 0; i < msg->markers.size(); i++) {
			geometry_msgs::PoseStamped original = msg->markers[i].pose;
			original.header.frame_id = msg->markers[i].header.frame_id;
			
			geometry_msgs::PoseStamped transformed;
			tf_listener.transformPose("/"+FRAME_ID, original, transformed);

			ar_track_alvar_msgs::AlvarMarker ar_marker;
			ar_marker.header = msg->markers[i].header;
			ar_marker.header.frame_id = "/"+FRAME_ID;
			ar_marker.id = msg->markers[i].id;
			ar_marker.confidence = msg->markers[i].confidence;
			ar_marker.pose = transformed;
			transformedMarkers.markers.push_back(ar_marker);
		}
		transformed_marker_pub.publish(transformedMarkers);
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

		ar_marker_sub =
		    node.subscribe("/ar_pose_marker", 10, &Visualization::ARMarkerCallback, this);
		label_pub = node.advertise<visualization_msgs::Marker>("ar_label_viz", 10);
		for(int i = 0; i < TAG_NUM; i++){
			setNamespace(labels[i], "tag_visualization");
			setID(labels[i], i);
		}
		transformed_marker_pub = 
		    node.advertise<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker_transformed", 10);
		//NOTE: where are the pcl deleted?
		//      they are shared pointers so pcl handles deletion itself
	}
	
	~Visualization() { //YSS: this is not called upon Ctrl+c. Why? How to force its call?

		ROS_INFO("removing the labels and boxes (if any) upon exit...");
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