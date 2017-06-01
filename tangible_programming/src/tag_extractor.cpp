#include "tangible/tag_extractor.h"

namespace tangible {

TagExtractor::TagExtractor(ros::NodeHandle& n) {
	node = n;
	
}

TagExtractor::~TagExtractor () {}

void TagExtractor::tagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr msg) {
	blocks.clear();
	for(int i = 0; i < msg->markers.size(); i++) {
		geometry_msgs::PoseStamped p = msg->markers[i].pose;
		tangible::Block block;
		initBlock(p, msg->markers[i].id, &block);
		//ROS_INFO("\n*** ID = %1d: X(%2.3lf, %2.3lf, %2.3lf) Y(%2.3lf, %2.3lf, %2.3lf) Z(%2.3lf, %2.3lf, %2.3lf) ***",
		//	     t.getID(), t.getX().x, t.getX().y, t.getX().z,
		//	                t.getY().x, t.getY().y, t.getY().z,
		//	                t.getZ().x, t.getZ().y, t.getZ().z);
		blocks.push_back(block);
	}
}

bool TagExtractor::parseCallback(tangible::GetBlocks::Request& req,
                 tangible::GetBlocks::Response& res){
	res.blocks = blocks;
}

void TagExtractor::initBlock(geometry_msgs::PoseStamped& p, int _id, tangible::Block* block){
	block->pose = p;
	block->id = _id;

	setAxes(p, block);
}

void TagExtractor::setAxes(geometry_msgs::PoseStamped& p, tangible::Block* block) {
	Eigen::Quaterniond q(p.pose.orientation.w,
		                 p.pose.orientation.x, 
		                 p.pose.orientation.y,
		                 p.pose.orientation.z);
	q.normalize();
	Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
	block->x_axis.x = rotation_matrix(0, 0);
	block->x_axis.y = rotation_matrix(1, 0);
	block->x_axis.z = rotation_matrix(2, 0);
	block->y_axis.x = rotation_matrix(0, 1);
	block->y_axis.y = rotation_matrix(1, 1);
	block->y_axis.z = rotation_matrix(2, 1);
	block->z_axis.x = rotation_matrix(0, 2);
	block->z_axis.y = rotation_matrix(1, 2);
	block->z_axis.z = rotation_matrix(2, 2);
}


}