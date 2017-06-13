#include "tangible/release_generator.h"
#include "tangible/gripper_marker.h"


namespace tangible {


bool ReleaseGenerator::isInside(std::vector<geometry_msgs::PointStamped> corners, geometry_msgs::PoseStamped pose)
{

  int i, j; 
  bool c = false;

  for(i = 0, j = corners.size() - 1; i < corners.size(); j = i++) {
    if( ( (corners[i].point.y >= pose.pose.position.y ) != (corners[j].point.y >= pose.pose.position.y) ) &&
        (pose.pose.position.x <= (corners[j].point.x - corners[i].point.x) * (pose.pose.position.y - corners[i].point.y) / (corners[j].point.y - corners[i].point.y) + corners[i].point.x)
      )
      c = !c;
  }
  return c;

}

ReleaseGenerator::ReleaseGenerator(ros::NodeHandle& n){
  node_handle = n;
  marker_pub = node_handle.advertise<visualization_msgs::Marker>("release_markers", 500);
  set_static_tf_client = node_handle.serviceClient<tangible_msgs::SetStaticTransform>("set_static_transform");

}

ReleaseGenerator::~ReleaseGenerator() {}

bool ReleaseGenerator::releaseCallback(tangible_msgs::GetReleases::Request& req,
                 tangible_msgs::GetReleases::Response& res)
{

  visualization_msgs::Marker marker;
  marker.action = 3;
  marker_pub.publish(marker);
  ROS_INFO("Got service call!");
  release_type = req.type;
  obj = req.object;
  target = req.target;
  num_orientations = req.num_orientations;
  region_sample_spacing = req.region_sample_spacing;
  getReleases();
	res.releases = releases;
  publishMarkers();
	return true;
}

float ReleaseGenerator::random(float low, float high){
  float random = ((float) rand()) / (float) RAND_MAX;
  float diff = high - low;
  float r = random * diff;
  return low + r;

}

void ReleaseGenerator::getReleases(){
  releases.clear();
  double orientation_interval = (2.0*3.14)/((float) num_orientations);
  if (target.type == tangible_msgs::Target::POINT_LOCATION){
    ROS_INFO("Point location type target");
    
    //pose of location gives x,y position for release. 
    geometry_msgs::PoseStamped release_pose;
    release_pose.header.frame_id = target.specified_point.header.frame_id;
    release_pose.pose.position.x = target.specified_point.point.x;
    release_pose.pose.position.y = target.specified_point.point.y;

    release_pose.pose.position.z = target.specified_point.point.z + 
                                      obj.bounding_box.dimensions.z + palm_dist + vertical_release_offset;
    if (release_type == tangible_msgs::GetReleases::Request::DROP){
      release_pose.pose.position.z+=drop_offset;
    }
    double yaw = 0.0; // for now                                 

    for (int i=0; i < num_orientations; i++){

      yaw += ((float) i) * orientation_interval;
      geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0,3.14/2.0, yaw);
      release_pose.pose.orientation.x = q.x;
      release_pose.pose.orientation.y = q.y;
      release_pose.pose.orientation.z = q.z;
      release_pose.pose.orientation.w = q.w;

      moveit_msgs::Grasp release;
      release.grasp_pose = release_pose;
      release.pre_grasp_approach.desired_distance = pre_release_dist;
      release.pre_grasp_approach.direction.header.frame_id = target.specified_point.header.frame_id;
      geometry_msgs::Vector3 pre_vec;
      pre_vec.x = 0.0;
      pre_vec.y = 0.0;
      pre_vec.z = 1.0; //above grasp
      release.pre_grasp_approach.direction.vector = pre_vec;

      release.post_grasp_retreat.desired_distance = post_release_dist;
      release.post_grasp_retreat.direction.header.frame_id = target.specified_point.header.frame_id;
      geometry_msgs::Vector3 post_vec;
      post_vec.x = 0.0;
      post_vec.y = 0.0;
      post_vec.z = 1.0; //above grasp
      release.post_grasp_retreat.direction.vector = post_vec;


      releases.push_back(release);
    }

  }  

  else if (target.type == tangible_msgs::Target::OBJECT_SELECTOR){
    // get point location from object 
    ROS_INFO("Object selector type target");
    geometry_msgs::PoseStamped release_pose;
    release_pose.header.frame_id = target.selected_object.bounding_box.pose.header.frame_id;
    release_pose.pose.position.x = target.selected_object.bounding_box.pose.pose.position.x;
    release_pose.pose.position.y = target.selected_object.bounding_box.pose.pose.position.y;

    release_pose.pose.position.z = target.selected_object.bounding_box.pose.pose.position.z + 
                                    target.selected_object.bounding_box.dimensions.z/2.0 + 
                                      obj.bounding_box.dimensions.z + palm_dist + vertical_release_offset;
    if (release_type == tangible_msgs::GetReleases::Request::DROP){
      release_pose.pose.position.z+=drop_offset;
    }

    double yaw = 0.0; // for now
    for (int i=0; i < num_orientations; i++){
      yaw += ((float) i) * orientation_interval;
      geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0,3.14/2.0, yaw);
      release_pose.pose.orientation.x = q.x;
      release_pose.pose.orientation.y = q.y;
      release_pose.pose.orientation.z = q.z;
      release_pose.pose.orientation.w = q.w;

      moveit_msgs::Grasp release;
      release.grasp_pose = release_pose;
      release.pre_grasp_approach.desired_distance = pre_release_dist;
      release.pre_grasp_approach.direction.header.frame_id = target.specified_point.header.frame_id;
      geometry_msgs::Vector3 pre_vec;
      pre_vec.x = 0.0;
      pre_vec.y = 0.0;
      pre_vec.z = 1.0; //above grasp
      release.pre_grasp_approach.direction.vector = pre_vec;

      release.post_grasp_retreat.desired_distance = post_release_dist;
      release.post_grasp_retreat.direction.header.frame_id = target.specified_point.header.frame_id;
      geometry_msgs::Vector3 post_vec;
      post_vec.x = 0.0;
      post_vec.y = 0.0;
      post_vec.z = 1.0; //above grasp
      release.post_grasp_retreat.direction.vector = post_vec;
      releases.push_back(release);
    }
  }
  else {
    //REGION
    // randomly pick a point in region
    // pick multiple points? 
    ROS_INFO("Region type target");
    geometry_msgs::PoseStamped release_pose;

    geometry_msgs::PointStamped c0 = target.region_corners[0];
    geometry_msgs::PointStamped c1 = target.region_corners[1];
    geometry_msgs::PointStamped c2 = target.region_corners[2];
    geometry_msgs::PointStamped c3 = target.region_corners[3];
    float min_x = std::min(c0.point.x, std::min(c1.point.x, std::min(c2.point.x, c3.point.x)));
    float max_x = std::max(c0.point.x, std::max(c1.point.x, std::max(c2.point.x, c3.point.x)));
    float min_y = std::min(c0.point.y, std::min(c1.point.y, std::min(c2.point.y, c3.point.y)));
    float max_y = std::max(c0.point.y, std::max(c1.point.y, std::max(c2.point.y, c3.point.y)));

    int x_samples = (int) ((max_x - min_x)/ region_sample_spacing);
    int y_samples = (int) ((max_y - min_y)/ region_sample_spacing);

    float x = min_x;
    float y = min_y;
    for (int i=0; i < x_samples; i++) {
      for (int j=0; j < y_samples; j++) {

        x += ((float) i) * region_sample_spacing;  
        y += ((float) j) * region_sample_spacing;  

 

        //geometry_msgs::PoseStamped release_pose;
        release_pose.header.frame_id = target.region_corners[0].header.frame_id;
        release_pose.pose.position.x = x;
        release_pose.pose.position.y = y;

        ROS_INFO("Current release pose: %f, %f, %f", 
          release_pose.pose.position.x,
          release_pose.pose.position.y,
          release_pose.pose.position.z);

        if (isInside(target.region_corners, release_pose)){
          release_pose.pose.position.z = obj.bounding_box.pose.pose.position.z - obj.bounding_box.dimensions.z/2.0 + 
                                    obj.bounding_box.dimensions.z + palm_dist + vertical_release_offset;
          if (release_type == tangible_msgs::GetReleases::Request::DROP){
            release_pose.pose.position.z+=drop_offset;
          }

          double yaw = 0.0; // for now                                 

          for (int k=0; k < num_orientations; k++){
            yaw += ((float) k) * orientation_interval;
            geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0,3.14/2.0, yaw);
            release_pose.pose.orientation.x = q.x;
            release_pose.pose.orientation.y = q.y;
            release_pose.pose.orientation.z = q.z;
            release_pose.pose.orientation.w = q.w;

            moveit_msgs::Grasp release;
            release.grasp_pose = release_pose;
            release.pre_grasp_approach.desired_distance = pre_release_dist;
            release.pre_grasp_approach.direction.header.frame_id = target.specified_point.header.frame_id;
            geometry_msgs::Vector3 pre_vec;
            pre_vec.x = 0.0;
            pre_vec.y = 0.0;
            pre_vec.z = 1.0; //above grasp
            release.pre_grasp_approach.direction.vector = pre_vec;

            release.post_grasp_retreat.desired_distance = post_release_dist;
            release.post_grasp_retreat.direction.header.frame_id = target.specified_point.header.frame_id;
            geometry_msgs::Vector3 post_vec;
            post_vec.x = 0.0;
            post_vec.y = 0.0;
            post_vec.z = 1.0; //above grasp
            release.post_grasp_retreat.direction.vector = post_vec;
            releases.push_back(release);

          }
        }

      }
    }     
  }
}

geometry_msgs::PoseStamped ReleaseGenerator::poseFromVec(geometry_msgs::PoseStamped pose, geometry_msgs::Vector3 vec, float dist){
  geometry_msgs::PoseStamped new_pose;
  new_pose = pose;
  new_pose.pose.position.x = pose.pose.position.x + vec.x * dist;
  new_pose.pose.position.y = pose.pose.position.y + vec.y * dist;
  new_pose.pose.position.z = pose.pose.position.z + vec.z * dist;
  return new_pose;
}

void ReleaseGenerator::publishMarkers(){
  for (int i=0; i < releases.size(); i++){
    moveit_msgs::Grasp grasp = releases[i];
    GripperMarker gripper_marker = GripperMarker(node_handle);
    std_msgs::ColorRGBA colour;
    colour.r = random(0.0, 1.0);
    colour.g = random(0.0, 1.0);
    colour.b = random(0.0, 1.0);
    colour.a = 0.5;
    std::vector<visualization_msgs::Marker> grasp_markers = gripper_marker.generateMarkerWithColour(0 + 15*i, grasp.grasp_pose, colour, "grasp_pose_frame", "main_grasp");
    geometry_msgs::PoseStamped pre_grasp_pose = poseFromVec(grasp.grasp_pose, grasp.pre_grasp_approach.direction.vector, grasp.pre_grasp_approach.desired_distance);
    std::vector<visualization_msgs::Marker> pre_grasp_markers = gripper_marker.generateMarkerWithColour(5 + 15*i, pre_grasp_pose, colour, "pre_grasp_pose_frame", "pre_grasp");
    geometry_msgs::PoseStamped post_grasp_pose = poseFromVec(grasp.grasp_pose, grasp.post_grasp_retreat.direction.vector, grasp.post_grasp_retreat.desired_distance);    
    std::vector<visualization_msgs::Marker> post_grasp_markers = gripper_marker.generateMarkerWithColour(10 + 15*i, post_grasp_pose, colour, "post_grasp_pose_frame", "post_grasp");

    for(int j=0; j < grasp_markers.size(); j++){
      marker_pub.publish(grasp_markers[j]);
      marker_pub.publish(pre_grasp_markers[j]);
      marker_pub.publish(post_grasp_markers[j]);
      ROS_INFO("publishing grasp markers");
    }
  }
}



}
