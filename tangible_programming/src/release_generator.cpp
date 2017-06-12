#include "tangible/release_generator.h"
#include "tangible/gripper_marker.h"


namespace tangible {

bool ReleaseGenerator::onSegment(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r)
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
        return true;
    return false;
}

int ReleaseGenerator::orientation(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);
 
    if (val == 0) return 0;  // colinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}

// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool ReleaseGenerator::doIntersect(geometry_msgs::Point p1, geometry_msgs::Point q1, 
                                 geometry_msgs::Point p2, geometry_msgs::Point q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
 
    // General case
    if (o1 != o2 && o3 != o4)
        return true;
 
    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
 
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
 
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
 
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
 
    return false; // Doesn't fall in any of the above cases
}

bool ReleaseGenerator::isInside(std::vector<geometry_msgs::PointStamped> corners, geometry_msgs::PoseStamped pose)
{
    /*
    // There must be at least 3 vertices in polygon[]
 
    // Create a point for line segment from p to infinite
    geometry_msgs::Point extreme;
    extreme.x = 10000.0;
    extreme.y = pose.pose.position.y;
 
    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
        int next = (i+1)%4;
 
        // Check if the line segment from 'p' to 'extreme' intersects
        // with the line segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(corners[i].point, corners[next].point, pose.pose.position, extreme))
        {
            // If the point 'p' is colinear with line segment 'i-next',
            // then check if it lies on segment. If it lies, return true,
            // otherwise false
            if (orientation(corners[i].point, pose.pose.position, corners[next].point) == 0)
               return onSegment(corners[i].point, pose.pose.position, corners[next].point);
 
            count++;
        }
        i = next;
    } while (i != 0);
 
    // Return true if count is odd, false otherwise
    return count&1;  // Same as (count%2 == 1)
    */
  //vector<Point> points = polygon.getPoints();
  int i, j; //nvert = points.size();
  bool c = false;

  for(i = 0, j = corners.size() - 1; i < corners.size(); j = i++) {
    if( ( (corners[i].point.y >= pose.pose.position.y ) != (corners[j].point.y >= pose.pose.position.y) ) &&
        (pose.pose.position.x <= (corners[j].point.x - corners[i].point.x) * (pose.pose.position.y - corners[i].point.y) / (corners[j].point.y - corners[i].point.y) + corners[i].point.x)
      )
      c = !c;
  }

  /*std::vector<geometry_msgs::PointStamped> corners_rev = corners;
  std::reverse(corners_rev.begin(),corners_rev.end());
  for(i = 0, j = corners_rev.size() - 1; i < corners_rev.size(); j = i++) {
    if( ( (corners_rev[i].point.y >= pose.pose.position.y ) != (corners_rev[j].point.y >= pose.pose.position.y) ) &&
        (pose.pose.position.x <= (corners_rev[j].point.x - corners_rev[i].point.x) * (pose.pose.position.y - corners_rev[i].point.y) / (corners_rev[j].point.y - corners_rev[i].point.y) + corners_rev[i].point.x)
      )
      c2 = !c2;
  }

  */

  return c;

}

ReleaseGenerator::ReleaseGenerator(ros::NodeHandle& n){
  node_handle = n;
  marker_pub = node_handle.advertise<visualization_msgs::Marker>("release_markers", 100);
  set_static_tf_client = node_handle.serviceClient<tangible_msgs::SetStaticTransform>("set_static_transform");

}

ReleaseGenerator::~ReleaseGenerator() {}

bool ReleaseGenerator::releaseCallback(tangible_msgs::GetReleases::Request& req,
                 tangible_msgs::GetReleases::Response& res)
{
  ROS_INFO("Got service call!");
  release_type = req.type;
  obj = req.object;
  target = req.target;
  getReleases();
	res.releases = releases;
  publishMarkers();
	return true;
}

float ReleaseGenerator::random(float low, float high){
    //srand (static_cast <unsigned> (time(0)));
    //float r = low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
    //return r;
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = high - low;
    float r = random * diff;
    return low + r;

}

void ReleaseGenerator::getReleases(){
  releases.clear();

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
  else {
    //REGION
    // randomly pick a point in region
    // pick multiple points? 
  ROS_INFO("Region type target");
    geometry_msgs::PoseStamped release_pose;
    while (true){

      geometry_msgs::PointStamped c0 = target.region_corners[0];
      geometry_msgs::PointStamped c1 = target.region_corners[1];
      geometry_msgs::PointStamped c2 = target.region_corners[2];
      geometry_msgs::PointStamped c3 = target.region_corners[3];
      float min_x = std::min(c0.point.x, std::min(c1.point.x, std::min(c2.point.x, c3.point.x)));
      float max_x = std::max(c0.point.x, std::max(c1.point.x, std::max(c2.point.x, c3.point.x)));
      float min_y = std::min(c0.point.y, std::min(c1.point.y, std::min(c2.point.y, c3.point.y)));
      float max_y = std::max(c0.point.y, std::max(c1.point.y, std::max(c2.point.y, c3.point.y)));

      float x = random(min_x, max_x);
      float y = random(min_y, max_y);
      ROS_INFO("Random: %f, %f", x,y);
      ROS_INFO("X low high: %f, %f", min_x,max_x);
      ROS_INFO("Y low high: %f, %f", min_y,max_y);

      ROS_INFO("c0: %f, %f", c0.point.x, c0.point.y);
      ROS_INFO("c1: %f, %f", c1.point.x, c1.point.y);
      ROS_INFO("c2: %f, %f", c2.point.x, c2.point.y);
      ROS_INFO("c3: %f, %f", c3.point.x, c3.point.y);

      //geometry_msgs::PoseStamped release_pose;
      release_pose.header.frame_id = target.region_corners[0].header.frame_id;
      release_pose.pose.position.x = x;
      release_pose.pose.position.y = y;

      ROS_INFO("Current release pose: %f, %f, %f", 
        release_pose.pose.position.x,
        release_pose.pose.position.y,
        release_pose.pose.position.z);

      if (isInside(target.region_corners, release_pose)){
        break;
      }
      else {
      ROS_INFO("Not inside");
      }

    }

    release_pose.pose.position.z = obj.bounding_box.pose.pose.position.z - obj.bounding_box.dimensions.z/2.0 + 
                                    obj.bounding_box.dimensions.z + palm_dist + vertical_release_offset;
  if (release_type == tangible_msgs::GetReleases::Request::DROP){
    release_pose.pose.position.z+=drop_offset;
  }

      ROS_INFO("Updates release pose: %f, %f, %f", 
        release_pose.pose.position.x,
        release_pose.pose.position.y,
        release_pose.pose.position.z);



double yaw = 0.0; // for now                                 
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
    std::vector<visualization_msgs::Marker> grasp_markers = gripper_marker.generateMarker(0 + 15*i, grasp.grasp_pose, gripper_marker.UNKNOWN, "grasp_pose_frame", "main_grasp");
    geometry_msgs::PoseStamped pre_grasp_pose = poseFromVec(grasp.grasp_pose, grasp.pre_grasp_approach.direction.vector, grasp.pre_grasp_approach.desired_distance);
    std::vector<visualization_msgs::Marker> pre_grasp_markers = gripper_marker.generateMarker(5 + 15*i, pre_grasp_pose, gripper_marker.UNKNOWN, "pre_grasp_pose_frame", "pre_grasp");
    geometry_msgs::PoseStamped post_grasp_pose = poseFromVec(grasp.grasp_pose, grasp.post_grasp_retreat.direction.vector, grasp.post_grasp_retreat.desired_distance);    
    std::vector<visualization_msgs::Marker> post_grasp_markers = gripper_marker.generateMarker(10 + 15*i, post_grasp_pose, gripper_marker.UNKNOWN, "post_grasp_pose_frame", "post_grasp");

    for(int j=0; j < grasp_markers.size(); j++){
      marker_pub.publish(grasp_markers[j]);
      marker_pub.publish(pre_grasp_markers[j]);
      marker_pub.publish(post_grasp_markers[j]);
      ROS_INFO("publishing grasp markers");
    }
  }
}



}
