#include "tangible/tag_extractor.h"

#include "Eigen/Geometry"

namespace tangible {

TagExtractor::TagExtractor(ros::NodeHandle& n) {
	node = n;
	ar_sub = node.subscribe("/ar_pose_transformed", 10, &TagExtractor::ARcallback, this);
}

TagExtractor::~TagExtractor () {}

void TagExtractor::ARcallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr msg) {
	tags.clear();
	for(int i = 0; i < msg->markers.size(); i++) {
		Tag t;
		geometry_msgs::PoseStamped p = msg->markers[i].pose;
		fillCenter(t, p);
		fillOrientation(t, p);
		fillAxes(t, p);
		t.id = msg->markers[i].id;
		ROS_INFO("\n*** ID = %1d: X(%2.3lf, %2.3lf, %2.3lf) Y(%2.3lf, %2.3lf, %2.3lf) Z(%2.3lf, %2.3lf, %2.3lf) ***",
			t.id, t.x_axis.x, t.x_axis.y, t.x_axis.z, t.y_axis.x, t.y_axis.y, t.y_axis.z, t.z_axis.x, t.z_axis.y, t.z_axis.z);
		tags.push_back(t);
	}
}

void TagExtractor::fillCenter(Tag& t, geometry_msgs::PoseStamped& p) {
	t.center.x = p.pose.position.x;
	t.center.y = p.pose.position.y;
	t.center.z = p.pose.position.z;
}

void TagExtractor::fillOrientation(Tag& t, geometry_msgs::PoseStamped& p) {
	t.orientation.x = p.pose.orientation.x;
	t.orientation.y = p.pose.orientation.y;
	t.orientation.z = p.pose.orientation.z;
	t.orientation.w = p.pose.orientation.w;
}

void TagExtractor::fillAxes(Tag& t, geometry_msgs::PoseStamped& p) {
	Eigen::Quaterniond q(p.pose.orientation.w,
		                 p.pose.orientation.x, 
		                 p.pose.orientation.y,
		                 p.pose.orientation.z);
	q.normalize();
	Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
	t.x_axis.x = rotation_matrix(0, 0);
	t.x_axis.y = rotation_matrix(1, 0);
	t.x_axis.z = rotation_matrix(2, 0);
	t.y_axis.x = rotation_matrix(0, 1);
	t.y_axis.y = rotation_matrix(1, 1);
	t.y_axis.z = rotation_matrix(2, 1);
	t.z_axis.x = rotation_matrix(0, 2);
	t.z_axis.y = rotation_matrix(1, 2);
	t.z_axis.z = rotation_matrix(2, 2);
}

std::vector<Tag> TagExtractor::get_tags() { return tags; }

}