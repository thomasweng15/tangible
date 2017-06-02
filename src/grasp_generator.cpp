#include "tangible/grasp_generator.h"

namespace tangible {

GraspGenerator::GraspGenerator(ros::NodeHandle& n){
  node_handle = n;
  marker_pub = node_handle.advertise<visualization_msgs::Marker>("grasp_markers", 20);

}

GraspGenerator::~GraspGenerator() {}

bool GraspGenerator::graspCallback(tangible::GetGrasps::Request& req,
                 tangible::GetGrasps::Response& res)
{
  ROS_INFO("Got service call!");
  scene = req.scene;
  getGrasps();
	res.grasps = grasps;
  // publishMarkers(scene);
	return true;
}

void GraspGenerator::getGrasps(){
  grasps.clear();
  if (scene.objects.size() > 1){
    ROS_WARN("More than one object in Scene msg given to GraspGenerator");
    return;
  }

  tangible::SceneObject obj = scene.objects[0];

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

  grasps.push_back(grasp);

  q = tf::createQuaternionMsgFromRollPitchYaw(0.0,3.14/2.0, yaw + 3.14/2.0);
  grasp_pose.pose.orientation.x = q.x;
  grasp_pose.pose.orientation.y = q.y;
  grasp_pose.pose.orientation.z = q.z;
  grasp_pose.pose.orientation.w = q.w;
  grasp.grasp_pose = grasp_pose;
  grasps.push_back(grasp);

}

void GraspGenerator::publishMarkers(tangible::Scene scene_msg){
  ;
}


}
