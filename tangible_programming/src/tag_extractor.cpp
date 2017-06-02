#include "tangible/tag_extractor.h"

namespace tangible {

TagExtractor::TagExtractor(ros::NodeHandle& n, int i_id, int r_id, int e_id, std::string mode_change_topic) {
	node_handle = n;
	mode_pub = node_handle.advertise<tangible::Mode>(mode_change_topic, 20);
	edit_id = e_id;
	run_id = r_id;
	idle_id = i_id;
	edit_state = false;
	run_state = false;
	idle_state = true;
}

TagExtractor::~TagExtractor () {}

void TagExtractor::tagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr msg) {
	blocks.clear();
	for(int i = 0; i < msg->markers.size(); i++) {
		geometry_msgs::PoseStamped p = msg->markers[i].pose;
		tangible::Block block;
		int id = msg->markers[i].id;
		initBlock(p, id, &block);
		//ROS_INFO("\n*** ID = %1d: X(%2.3lf, %2.3lf, %2.3lf) Y(%2.3lf, %2.3lf, %2.3lf) Z(%2.3lf, %2.3lf, %2.3lf) ***",
		//	     t.getID(), t.getX().x, t.getX().y, t.getX().z,
		//	                t.getY().x, t.getY().y, t.getY().z,
		//	                t.getZ().x, t.getZ().y, t.getZ().z);
		blocks.push_back(block);
		tangible::Mode mode_msg;
		if (id == idle_id){
        	if (!idle_state){
        		idle_state = true;
        		run_state = false;
        		edit_state = false;
        		mode_msg.mode = tangible::Mode::IDLE;
        		mode_pub.publish(mode_msg);
        	}
        }
        else if (id == run_id){ 
        	if (!run_state){
        		run_state = true;
        		idle_state = false;
        		edit_state = false;
        		mode_msg.mode = tangible::Mode::EXECUTE;
        		mode_pub.publish(mode_msg);
        	}
        }
        else if (id == edit_id) { 
       	    if (!edit_state){
        		edit_state = true;
        		run_state = false;
        		idle_state = false;
        		mode_msg.mode = tangible::Mode::EDIT;
        		mode_pub.publish(mode_msg);
        	}
    	}

	}
}

bool TagExtractor::parseCallback(tangible::GetBlocks::Request& req,
                 tangible::GetBlocks::Response& res){

//fake blocks
// 	//number
// 	tangible::Block block1;
// 	int id = 9;
// 	geometry_msgs::PoseStamped pose;
// 	pose.header.frame_id = "base_footprint";
// 	pose.pose.position.x =  0.523317205906;
// 	pose.pose.position.y =  -0.0127397447824;
// 	pose.pose.position.z =  0.839;
// 	geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,-3.14/2.0);
// 	pose.pose.orientation.x = q.x;
// 	pose.pose.orientation.y = q.y;
// 	pose.pose.orientation.z = q.z;
// 	pose.pose.orientation.w = q.w;

// 	initBlock(pose, id, &block1);
// 	blocks.push_back(block1);

// 	//pick
// 	tangible::Block block2;
// 	id = 6;
// 	pose.header.frame_id = "base_footprint";
// 	pose.pose.position.x =  0.523317205906;
// 	pose.pose.position.y =  -0.062727397447824;
// 	pose.pose.position.z =  0.839;
// 	q = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,-3.14/2.0);
// 	pose.pose.orientation.x = q.x;
// 	pose.pose.orientation.y = q.y;
// 	pose.pose.orientation.z = q.z;
// 	pose.pose.orientation.w = q.w;

// 	initBlock(pose, id, &block2);
// 	blocks.push_back(block2);

// 	// location
// 	tangible::Block block3;
// 	id = 2;
// 	pose.header.frame_id = "base_footprint";
// 	pose.pose.position.x =  0.573317205906;
// 	pose.pose.position.y =  -0.0627397447824;
// 	q = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,-3.14/2.0);
// 	pose.pose.orientation.x = q.x;
// 	pose.pose.orientation.y = q.y;
// 	pose.pose.orientation.z = q.z;
// 	pose.pose.orientation.w = q.w;

// 	initBlock(pose, id, &block3);
// 	blocks.push_back(block3);

// 	//number 
// 	tangible::Block block4;
// 	id = 10;
// 	pose.header.frame_id = "base_footprint";
// 	pose.pose.position.x =  0.93317205906;
// 	pose.pose.position.y =  -0.0127397447824;
// 	pose.pose.position.z =  0.839;
// 	q = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,-3.14/2.0);
// 	pose.pose.orientation.x = q.x;
// 	pose.pose.orientation.y = q.y;
// 	pose.pose.orientation.z = q.z;
// 	pose.pose.orientation.w = q.w;

// 	initBlock(pose, id, &block4);
// 	blocks.push_back(block4);

// 	// place
// 	tangible::Block block5;
// 	id = 8;
// 	pose.header.frame_id = "base_footprint";
// 	pose.pose.position.x =  0.93817205906;
// 	pose.pose.position.y =  -0.0627397447824;
// 	pose.pose.position.z =  0.839;

// 	// tf::Quaternion q;
// 	// q.setRPY(0.0, 0.0, 3.14/2.0);
// 	q = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,-3.14/2.0);
// 	pose.pose.orientation.x = q.x;
// 	pose.pose.orientation.y = q.y;
// 	pose.pose.orientation.z = q.z;
// 	pose.pose.orientation.w = q.w;

// 	initBlock(pose, id, &block5);
// 	blocks.push_back(block5);

// 	// location 
// 	tangible::Block block6;
// 	id = 1;
// 	pose.header.frame_id = "base_footprint";
// 	pose.pose.position.x =  0.9817205906;
// 	pose.pose.position.y =  -0.0627397447824;
// 	pose.pose.position.z =  0.839;
// q = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,-3.14/2.0);
// 	pose.pose.orientation.x = q.x;
// 	pose.pose.orientation.y = q.y;
// 	pose.pose.orientation.z = q.z;
// 	pose.pose.orientation.w = q.w;

// 	initBlock(pose, id, &block6);
// 	blocks.push_back(block6);

// 	// number
// 	tangible::Block block7;
// 	id = 10;
// 	pose.header.frame_id = "base_footprint";
// 	pose.pose.position.x =  1.18317205906;
// 	pose.pose.position.y =  0.107397447824;
// 	pose.pose.position.z =  0.839;
// q = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,-3.14/2.0);
// 	pose.pose.orientation.x = q.x;
// 	pose.pose.orientation.y = q.y;
// 	pose.pose.orientation.z = q.z;
// 	pose.pose.orientation.w = q.w;

// 	initBlock(pose, id, &block7);
// 	blocks.push_back(block7);

// 	//location
// 	tangible::Block block8;
// 	id = 4;
// 	pose.header.frame_id = "base_footprint";
// 	pose.pose.position.x =  1.18317205906;
// 	pose.pose.position.y =  0.0527397447824;
// 	pose.pose.position.z =  0.839;
// q = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,-3.14/2.0);
// 	pose.pose.orientation.x = q.x;
// 	pose.pose.orientation.y = q.y;
// 	pose.pose.orientation.z = q.z;
// 	pose.pose.orientation.w = q.w;

// 	initBlock(pose, id, &block8);
// 	blocks.push_back(block8);
	ROS_INFO("Got service call");

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