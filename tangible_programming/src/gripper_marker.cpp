#include "tangible/gripper_marker.h"

namespace tangible {

    inline geometry_msgs::Point createPointMsg(const float x, const float y, const float z) {
      geometry_msgs::Point msg;
      msg.x = x;
      msg.y = y;
      msg.z = z;
      return msg;
    }

    inline geometry_msgs::Point createPointMsg(const tf::Point &v) {
      geometry_msgs::Point msg;
      msg.x = v.x();
      msg.y = v.y();
      msg.z = v.z();
      return msg;
    }

    inline geometry_msgs::Point createPointMsg(const geometry_msgs::Vector3 &v) {
      geometry_msgs::Point msg;
      msg.x = v.x;
      msg.y = v.y;
      msg.z = v.z;
      return msg;
    }

    inline geometry_msgs::Pose createPoseMsg(const geometry_msgs::Point &v,  const geometry_msgs::Quaternion &q)
    {
      geometry_msgs::Pose msg;
      msg.position = v;
      msg.orientation = q;
      return msg;
    }

    inline geometry_msgs::Pose createPoseMsg(const geometry_msgs::Vector3 &v,  const geometry_msgs::Quaternion &q)
    {
      geometry_msgs::Pose msg;
      msg.position = createPointMsg(v.x, v.y, v.z);
      msg.orientation = q;
      return msg;
    }

    inline geometry_msgs::Pose createPoseMsg(const tf::Pose &p)
    {
      geometry_msgs::Pose msg;
      tf::poseTFToMsg(p, msg);
      return msg;
    }

    // --------------------------------------------------------------------------------------------
    inline geometry_msgs::PoseStamped createPoseStampedMsg(const geometry_msgs::Pose &p, const std::string &frame_id, const ros::Time &stamp)
    {
      geometry_msgs::PoseStamped msg;
      msg.pose = p;
      msg.header.frame_id = frame_id;
      msg.header.stamp = stamp;
      return msg;
    }

    inline geometry_msgs::PoseStamped createPoseStampedMsg(const geometry_msgs::Point &v,  const geometry_msgs::Quaternion &q, const std::string &frame_id, const ros::Time &stamp)
    {
      return createPoseStampedMsg( createPoseMsg(v, q), frame_id, stamp );
    }

    inline geometry_msgs::PoseStamped createPoseStampedMsg(const geometry_msgs::TransformStamped ts)
    {

      return createPoseStampedMsg( createPoseMsg(ts.transform.translation, ts.transform.rotation), ts.header.frame_id, ts.header.stamp );
    }

    inline geometry_msgs::PoseStamped createPoseStampedMsg(const tf::Transform &pose, const std::string &frame_id, const ros::Time &stamp)
    {

      return createPoseStampedMsg( createPoseMsg(pose), frame_id, stamp );
    }


GripperMarker::GripperMarker(ros::NodeHandle& n){

  set_static_tf_client = node_handle.serviceClient<tangible_msgs::SetStaticTransform>("set_static_transform");
}

GripperMarker::~GripperMarker() {}

std::vector<visualization_msgs::Marker> GripperMarker::generateMarkerWithColour(int start_id, geometry_msgs::PoseStamped pose, std_msgs::ColorRGBA colour, std::string grasp_pose_frame, std::string ns){
  double DEFAULT_OFFSET = 0.09;
  std_msgs::ColorRGBA COLOR_MESH_REACHABLE;
  COLOR_MESH_REACHABLE.r = 0.5;
  COLOR_MESH_REACHABLE.g =  0.5;
  COLOR_MESH_REACHABLE.b =  0.5;
  COLOR_MESH_REACHABLE.a =  0.6;
  std_msgs::ColorRGBA COLOR_MESH_UNREACHABLE;
  COLOR_MESH_UNREACHABLE.r = 0.05;
  COLOR_MESH_UNREACHABLE.g =  0.05;
  COLOR_MESH_UNREACHABLE.b =  0.05;
  COLOR_MESH_UNREACHABLE.a =  0.6;
  std_msgs::ColorRGBA COLOR_MESH_UNKNOWN;
  COLOR_MESH_UNKNOWN.r = 0.0;
  COLOR_MESH_UNKNOWN.g =  0.0;
  COLOR_MESH_UNKNOWN.b =  0.5;
  COLOR_MESH_UNKNOWN.a =  0.6;
  double ANGLE_GRIPPER_OPEN = 28.0 * 3.14 / 180.0;
  double ANGLE_GRIPPER_CLOSED = 0.0;
  std::string STR_MESH_GRIPPER_FOLDER = "package://pr2_description/meshes/gripper_v0/";
  std::string STR_GRIPPER_PALM_FILE = STR_MESH_GRIPPER_FOLDER + "gripper_palm.dae";
  std::string STR_GRIPPER_FINGER_FILE = STR_MESH_GRIPPER_FOLDER + "l_finger.dae";
  std::string STR_GRIPPER_FINGERTIP_FILE = STR_MESH_GRIPPER_FOLDER + "l_finger_tip.dae";
  double INT_MARKER_SCALE = 0.2;
  double GRIPPER_MARKER_SCALE = 1.05;
  std::string REF_FRAME = "base_link";

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) );
  tf::Quaternion q(pose.pose.orientation.x,
                     pose.pose.orientation.y,
                     pose.pose.orientation.z,
                     pose.pose.orientation.w);
  // q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", grasp_pose_frame));
  //ros::Duration(0.25).sleep();

  geometry_msgs::TransformStamped trans;
  trans.header.frame_id = "base_footprint";
  trans.child_frame_id = grasp_pose_frame;
  trans.transform.translation.x = pose.pose.position.x; 
  trans.transform.translation.y = pose.pose.position.y; 
  trans.transform.translation.z = pose.pose.position.z;
  trans.transform.rotation.x = pose.pose.orientation.x; 
  trans.transform.rotation.y = pose.pose.orientation.y; 
  trans.transform.rotation.z = pose.pose.orientation.z; 
  trans.transform.rotation.w = pose.pose.orientation.w; 
  //tf::StampedTransform(transform, ros::Time::now(), "base_footprint", grasp_frame);
  //ros::Duration(0.25).sleep();
  tangible_msgs::SetStaticTransform tf_srv;
  tf_srv.request.transform = trans;
  set_static_tf_client.call(tf_srv);


  // Set angle of meshes based on gripper open vs closed.
  double angle = ANGLE_GRIPPER_OPEN; //if is_hand_open else ANGLE_GRIPPER_CLOSED

  std::vector<visualization_msgs::Marker> markers;
  visualization_msgs::Marker mesh;
  mesh.ns = ns;
  mesh.mesh_use_embedded_materials = false;
  mesh.header.frame_id = grasp_pose_frame;
  mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
  mesh.scale.x = GRIPPER_MARKER_SCALE;
  mesh.scale.y = GRIPPER_MARKER_SCALE;
  mesh.scale.z = GRIPPER_MARKER_SCALE;
  mesh.color = colour;
  tf::Transform T1, T2;
  tf::Transform T_proximal, T_distal;

  T1.setOrigin(tf::Vector3(0.07691, 0.01, 0));
  T1.setRotation(tf::Quaternion(tf::Vector3(0,0,1),  angle));
  T2.setOrigin(tf::Vector3(0.09137, 0.00495, 0));
  T2.setRotation(tf::Quaternion(tf::Vector3(0,0,1), -angle));
  T_proximal = T1;
  T_distal = T1 * T2;

  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/gripper_palm.dae";
  mesh.pose.orientation.w = 1;
  mesh.id = start_id;
  markers.push_back( mesh );
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger.dae";
  mesh.pose = createPoseMsg(T_proximal);
  mesh.id = start_id + 1;
  markers.push_back( mesh );
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae";
  mesh.pose = createPoseMsg(T_distal);
  mesh.id = start_id + 2;
  markers.push_back( mesh );

  T1.setOrigin(tf::Vector3(0.07691, -0.01, 0));
  T1.setRotation(tf::Quaternion(tf::Vector3(1,0,0), M_PI)*tf::Quaternion(tf::Vector3(0,0,1),  angle));
  T2.setOrigin(tf::Vector3(0.09137, 0.00495, 0));
  T2.setRotation(tf::Quaternion(tf::Vector3(0,0,1), -angle));
  T_proximal = T1;
  T_distal = T1 * T2;

  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger.dae";
  mesh.pose = createPoseMsg(T_proximal);
  mesh.id = start_id + 3;
  markers.push_back( mesh );
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae";
  mesh.pose = createPoseMsg(T_distal);
  mesh.id = start_id + 4;
  markers.push_back( mesh );


  return markers;
}


std::vector<visualization_msgs::Marker> GripperMarker::generateMarker(int start_id, geometry_msgs::PoseStamped pose, int reachability, std::string grasp_pose_frame, std::string ns)
{

  double DEFAULT_OFFSET = 0.09;
  std_msgs::ColorRGBA COLOR_MESH_REACHABLE;
  COLOR_MESH_REACHABLE.r = 0.5;
  COLOR_MESH_REACHABLE.g =  0.5;
  COLOR_MESH_REACHABLE.b =  0.5;
  COLOR_MESH_REACHABLE.a =  0.6;
  std_msgs::ColorRGBA COLOR_MESH_UNREACHABLE;
  COLOR_MESH_UNREACHABLE.r = 0.05;
  COLOR_MESH_UNREACHABLE.g =  0.05;
  COLOR_MESH_UNREACHABLE.b =  0.05;
  COLOR_MESH_UNREACHABLE.a =  0.6;
  std_msgs::ColorRGBA COLOR_MESH_UNKNOWN;
  COLOR_MESH_UNKNOWN.r = 0.0;
  COLOR_MESH_UNKNOWN.g =  0.0;
  COLOR_MESH_UNKNOWN.b =  0.5;
  COLOR_MESH_UNKNOWN.a =  0.6;
  double ANGLE_GRIPPER_OPEN = 28.0 * 3.14 / 180.0;
  double ANGLE_GRIPPER_CLOSED = 0.0;
  std::string STR_MESH_GRIPPER_FOLDER = "package://pr2_description/meshes/gripper_v0/";
  std::string STR_GRIPPER_PALM_FILE = STR_MESH_GRIPPER_FOLDER + "gripper_palm.dae";
  std::string STR_GRIPPER_FINGER_FILE = STR_MESH_GRIPPER_FOLDER + "l_finger.dae";
  std::string STR_GRIPPER_FINGERTIP_FILE = STR_MESH_GRIPPER_FOLDER + "l_finger_tip.dae";
  double INT_MARKER_SCALE = 0.2;
  double GRIPPER_MARKER_SCALE = 1.05;
  std::string REF_FRAME = "base_link";

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) );
  tf::Quaternion q(pose.pose.orientation.x,
                     pose.pose.orientation.y,
                     pose.pose.orientation.z,
                     pose.pose.orientation.w);
  // q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", grasp_pose_frame));
  //ros::Duration(0.25).sleep();

  geometry_msgs::TransformStamped trans;
  trans.header.frame_id = "base_footprint";
  trans.child_frame_id = grasp_pose_frame;
  trans.transform.translation.x = pose.pose.position.x; 
  trans.transform.translation.y = pose.pose.position.y; 
  trans.transform.translation.z = pose.pose.position.z;
  trans.transform.rotation.x = pose.pose.orientation.x; 
  trans.transform.rotation.y = pose.pose.orientation.y; 
  trans.transform.rotation.z = pose.pose.orientation.z; 
  trans.transform.rotation.w = pose.pose.orientation.w; 
  //tf::StampedTransform(transform, ros::Time::now(), "base_footprint", grasp_frame);
  //ros::Duration(0.25).sleep();
  tangible_msgs::SetStaticTransform tf_srv;
  tf_srv.request.transform = trans;
  set_static_tf_client.call(tf_srv);


  // Set angle of meshes based on gripper open vs closed.
  double angle = ANGLE_GRIPPER_OPEN; //if is_hand_open else ANGLE_GRIPPER_CLOSED

  std::vector<visualization_msgs::Marker> markers;
  visualization_msgs::Marker mesh;
  mesh.ns = ns;
  mesh.mesh_use_embedded_materials = false;
  mesh.header.frame_id = grasp_pose_frame;
  mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
  mesh.scale.x = GRIPPER_MARKER_SCALE;
  mesh.scale.y = GRIPPER_MARKER_SCALE;
  mesh.scale.z = GRIPPER_MARKER_SCALE;
  if (reachability == REACHABLE){
    mesh.color = COLOR_MESH_REACHABLE;
  }
  else if (reachability == UNREACHABLE) {
    mesh.color = COLOR_MESH_UNREACHABLE;

  }
  else {
    mesh.color = COLOR_MESH_UNKNOWN;

  }
  tf::Transform T1, T2;
  tf::Transform T_proximal, T_distal;

  T1.setOrigin(tf::Vector3(0.07691, 0.01, 0));
  T1.setRotation(tf::Quaternion(tf::Vector3(0,0,1),  angle));
  T2.setOrigin(tf::Vector3(0.09137, 0.00495, 0));
  T2.setRotation(tf::Quaternion(tf::Vector3(0,0,1), -angle));
  T_proximal = T1;
  T_distal = T1 * T2;

  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/gripper_palm.dae";
  mesh.pose.orientation.w = 1;
  mesh.id = start_id;
  markers.push_back( mesh );
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger.dae";
  mesh.pose = createPoseMsg(T_proximal);
  mesh.id = start_id + 1;
  markers.push_back( mesh );
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae";
  mesh.pose = createPoseMsg(T_distal);
  mesh.id = start_id + 2;
  markers.push_back( mesh );

  T1.setOrigin(tf::Vector3(0.07691, -0.01, 0));
  T1.setRotation(tf::Quaternion(tf::Vector3(1,0,0), M_PI)*tf::Quaternion(tf::Vector3(0,0,1),  angle));
  T2.setOrigin(tf::Vector3(0.09137, 0.00495, 0));
  T2.setRotation(tf::Quaternion(tf::Vector3(0,0,1), -angle));
  T_proximal = T1;
  T_distal = T1 * T2;

  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger.dae";
  mesh.pose = createPoseMsg(T_proximal);
  mesh.id = start_id + 3;
  markers.push_back( mesh );
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae";
  mesh.pose = createPoseMsg(T_distal);
  mesh.id = start_id + 4;
  markers.push_back( mesh );


  return markers;
      
}

}
