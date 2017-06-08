#include "tangible/grasp_generator.h"

namespace tangible {

GraspGenerator::GraspGenerator(ros::NodeHandle& n){
  node_handle = n;
  marker_pub = node_handle.advertise<visualization_msgs::Marker>("grasp_markers", 20);

}

GraspGenerator::~GraspGenerator() {}

bool GraspGenerator::graspCallback(tangible_msgs::GetGrasps::Request& req,
                 tangible_msgs::GetGrasps::Response& res)
{
  ROS_INFO("Got service call!");
  obj = req.object;
  getGrasps();
	res.grasps = grasps;
  // publishMarkers(scene);
	return true;
}

void GraspGenerator::getGrasps(){
  grasps.clear();

  

  //pose of bounding box gives x,y position for grasp. 
  geometry_msgs::PoseStamped grasp_pose;
  grasp_pose.header.frame_id = obj.bounding_box.pose.header.frame_id;
  grasp_pose.pose.position.x = obj.bounding_box.pose.pose.position.x;
  grasp_pose.pose.position.y = obj.bounding_box.pose.pose.position.y;

  //we want the palm to not hit the object
  //distance from wrist to palm is about 0.12 m
  //so pose needs to be at least that distance from top of object
  //let's just use the top of the bounding box for now
  grasp_pose.pose.position.z = obj.bounding_box.pose.pose.position.z + 
                                    obj.bounding_box.dimensions.z/2.0 + palm_dist;
  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(obj.bounding_box.pose.pose.orientation, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  //top grasps only for now
  //so we need a 90 degree downward pitch of the wrist
  //make one aligned with primary bounding box axis
  //and one 90 degrees from that
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0,3.14/2.0, yaw);
  grasp_pose.pose.orientation.x = q.x;
  grasp_pose.pose.orientation.y = q.y;
  grasp_pose.pose.orientation.z = q.z;
  grasp_pose.pose.orientation.w = q.w;
  moveit_msgs::Grasp grasp;
  grasp.grasp_pose = grasp_pose;
  grasp.pre_grasp_approach.desired_distance = pre_grasp_dist;
  grasp.pre_grasp_approach.direction.header.frame_id = obj.bounding_box.pose.header.frame_id;
  geometry_msgs::Vector3 pre_vec;
  pre_vec.x = 0.0;
  pre_vec.y = 0.0;
  pre_vec.z = -1.0;
  grasp.pre_grasp_approach.direction.vector = pre_vec;
  grasp.post_grasp_retreat.desired_distance = post_grasp_dist;
  grasp.post_grasp_retreat.direction.header.frame_id = obj.bounding_box.pose.header.frame_id;
  geometry_msgs::Vector3 post_vec;
  pre_vec.x = 0.0;
  pre_vec.y = 0.0;
  pre_vec.z = 1.0;
  grasp.pre_grasp_approach.direction.vector = post_vec;
  if(hasCollision(grasp, obj.point_cloud) != true){
    grasps.push_back(grasp);
  }

  q = tf::createQuaternionMsgFromRollPitchYaw(0.0,3.14/2.0, yaw + 3.14/2.0);
  grasp_pose.pose.orientation.x = q.x;
  grasp_pose.pose.orientation.y = q.y;
  grasp_pose.pose.orientation.z = q.z;
  grasp_pose.pose.orientation.w = q.w;
  grasp.grasp_pose = grasp_pose;
  if(hasCollision(grasp, obj.point_cloud) != true){
    grasps.push_back(grasp);
  }

}

void GraspGenerator::publishMarkers(tangible_msgs::Scene scene_msg){
  ;
}

bool GraspGenerator::hasCollision(moveit_msgs::Grasp grasp, sensor_msgs::PointCloud2 pc2){





    // Check if enough points will be in gripper
    double y_offset = 0.005;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(grasp.grasp_pose.pose.position.x,
                                     grasp.grasp_pose.pose.position.y, 
                                     grasp.grasp_pose.pose.position.z) );
    tf::Quaternion q(grasp.grasp_pose.pose.orientation.x,
                     grasp.grasp_pose.pose.orientation.y,
                     grasp.grasp_pose.pose.orientation.z,
                     grasp.grasp_pose.pose.orientation.w);
    // q.setRPY(0, 0, msg->theta);
    transform.setRotation(q);
    std::string grasp_frame = "grasp";
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", grasp_frame));
    ros::Duration(0.25).sleep();

    // hand boxes
    Box palm_box; 
    Box l_finger_box; 
    Box r_finger_box; 

    Box grasp_box;
    grasp_box = Box();
    grasp_box.min_x = dist_to_palm;
    grasp_box.max_x = dist_to_fingertips;
    grasp_box.min_y = -1.0 * gripper_palm_width/2.0 + y_offset;
    grasp_box.max_y = gripper_palm_width/2.0 + y_offset;
    grasp_box.min_z = -1.0 * gripper_finger_height/2.0;
    grasp_box.max_z = 1.0 * gripper_finger_height/2.0;

        

        // Check for collisions with fingers
        // Left Finger
        l_finger_box.min_x = dist_to_palm - 0.02;
        l_finger_box.max_x = dist_to_fingertips;
        l_finger_box.min_y = -1.0 * gripper_palm_width/2.0 - 0.025 + y_offset;
        l_finger_box.max_y = -1.0 * gripper_palm_width/2.0 - 0.005 + y_offset;
        l_finger_box.min_z = -1.0 * gripper_finger_height/2.0;
        l_finger_box.max_z = gripper_finger_height/2.0;


        // Right Finger
        r_finger_box.min_x = dist_to_palm - 0.02;
        r_finger_box.max_x = dist_to_fingertips;
        r_finger_box.min_y = gripper_palm_width/2.0 + 0.005 + y_offset;
        r_finger_box.max_y = gripper_palm_width/2.0 + 0.025 + y_offset;
        r_finger_box.min_z = -1.0 * gripper_finger_height/2.0;
        r_finger_box.max_z = gripper_finger_height/2.0;


        // Check for collisions with palm
        palm_box.min_x = 0.05;
        palm_box.max_x = dist_to_palm - 0.01;
        palm_box.min_y = -1.0 * gripper_palm_width/2.0 + y_offset;
        palm_box.max_y = gripper_palm_width/2.0 + y_offset;
        palm_box.min_z = -1.0 * gripper_finger_height/2.0;
        palm_box.max_z = gripper_finger_height/2.0;

        // ROS_INFO("got point cloud, frame_id = %s", (*msg).header.frame_id.c_str());
        sensor_msgs::PointCloud2 transformed_cloud_msg;
        tf::TransformListener tf_listener;
        tf::StampedTransform cloud_transform;
        tf_listener.waitForTransform(grasp_frame, pc2.header.frame_id, ros::Time(0), ros::Duration(3.0));
        //ROS_INFO("... done waiting.");
        tf_listener.lookupTransform(grasp_frame, pc2.header.frame_id, ros::Time(0), cloud_transform);
        pcl_ros::transformPointCloud(grasp_frame, cloud_transform, pc2, transformed_cloud_msg);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(transformed_cloud_msg, *cloud);

        // Moved point cloud checking out of service call
        int num_palm_points =  findPointsInBox(cloud, palm_box, grasp_frame);
        int num_l_finger_points =  findPointsInBox(cloud, l_finger_box, grasp_frame);
        int num_r_finger_points =  findPointsInBox(cloud, r_finger_box, grasp_frame);
        // self.loginfo("Number of points inside gripper: {}".format(num_points[0]))
        // self.loginfo("Number of points inside left finger: {}"
        //                 .format(num_points[1]))
        // self.loginfo("Number of points inside right finger: {}"
        //                 .format(num_points[2]))
        // self.loginfo("Number of points inside palm: {}"
        //                 .format(num_points[3]))

        if (num_palm_points < max_palm_collision_points && (num_r_finger_points + num_l_finger_points) < max_finger_collision_points){
          return false;
        }
        else {
          return true;
        }
}

int GraspGenerator::findPointsInBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Box box, std::string frame){
  int num_points = 0;
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    pcl::PointXYZRGB& point = cloud->at(i);
    if (point.x >= box.min_x &&
                    point.x < box.max_x &&
                    point.y >= box.min_y &&
                    point.y < box.max_y &&
                    point.z >= box.min_z &&
                    point.z < box.max_z){
      num_points++;
    }
                   

  }
  ROS_INFO("Collision points: %d", num_points);


  return num_points;

      
}

}
