#include "tangible/release_generator.h"
#include "tangible/gripper_marker.h"


namespace tangible {
bool greaterEqual(double a, double b)
{
    ROS_INFO("Comparing");
    double eps = 0.0001;
    if ((a > b) || (fabs(a - b) < eps)){
      return true;
    }
    else {
      return false;
    }
    //return (a > b) || (fabs(a - b) < EPSILON);
}
bool lessEqual(double a, double b)
{
    double eps = 0.0001;
    // return (a < b) || (fabs(a - b) < EPSILON);
    if ((a < b) || (fabs(a - b) < eps)){
      return true;
    }
    else {
      return false;
    }

}


bool ReleaseGenerator::isInside(std::vector<geometry_msgs::PointStamped> corners, geometry_msgs::PoseStamped pose)
{

  /*int i, j; 
  bool c = false;

  for(i = 0, j = corners.size() - 1; i < corners.size(); j = i++) {
    if( ((corners[i].point.y > pose.pose.position.y ) != (corners[j].point.y > pose.pose.position.y) ) &&
        (pose.pose.position.x < (corners[j].point.x - corners[i].point.x) * (pose.pose.position.y - corners[i].point.y) / (corners[j].point.y - corners[i].point.y) + corners[i].point.x)
      )
      c = !c;
  }
  return c;
*/
    //this method uses the ray tracing algorithm to determine if the point is in the polygon
    int nPoints = corners.size();
    int j=-999;
    int i=-999;
    bool locatedInPolygon=false;
    for(i=0; i< nPoints;i++){
        //repeat loop for all sets of points
        if(i==(nPoints-1)){
            //if i is the last vertex, let j be the first vertex
            j= 0;
        }else{
            //for all-else, let j=(i+1)th vertex
            j=i+1;
        }

        float vertY_i= corners[i].point.y; //(float)poly.get(i).getY();
        float vertX_i= corners[i].point.x; //(float)poly.get(i).getX();
        float vertY_j= corners[j].point.y; //(float)poly.get(j).getY();
        float vertX_j= corners[j].point.x; //(float)poly.get(j).getX();
        float testX  = pose.pose.position.x; //(float)this.getX();
        float testY  = pose.pose.position.y; //(float)this.getY();

        // following statement checks if testPoint.Y is below Y-coord of i-th vertex
        bool belowLowY=vertY_i>testY;
        // following statement checks if testPoint.Y is below Y-coord of i+1-th vertex
        bool belowHighY=vertY_j>testY;

        /* following statement is true if testPoint.Y satisfies either (only one is possible) 
        -->(i).Y < testPoint.Y < (i+1).Y        OR  
        -->(i).Y > testPoint.Y > (i+1).Y

        (Note)
        Both of the conditions indicate that a point is located within the edges of the Y-th coordinate
        of the (i)-th and the (i+1)- th vertices of the polygon. If neither of the above
        conditions is satisfied, then it is assured that a semi-infinite horizontal line draw 
        to the right from the testpoint will NOT cross the line that connects vertices i and i+1 
        of the polygon
        */
        bool withinYsEdges= belowLowY != belowHighY;

        if( withinYsEdges){
            // this is the slope of the line that connects vertices i and i+1 of the polygon
            float slopeOfLine   = ( vertX_j-vertX_i )/ (vertY_j-vertY_i) ;

            // this looks up the x-coord of a point lying on the above line, given its y-coord
            float pointOnLine   = ( slopeOfLine* (testY - vertY_i) )+vertX_i;

            //checks to see if x-coord of testPoint is smaller than the point on the line with the same y-coord
            bool isLeftToLine= testX < pointOnLine;

            if(isLeftToLine){
                //this statement changes true to false (and vice-versa)
                locatedInPolygon= !locatedInPolygon;
            }//end if (isLeftToLine)
        }//end if (withinYsEdges
    }

    return locatedInPolygon;

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
  double orientation_interval = (3.14)/((double) num_orientations);
  ROS_INFO("Orientation interval: %f", orientation_interval);
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

      yaw = ((double) i) * orientation_interval;
      ROS_INFO("Yaw: %f", yaw);
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
      yaw = ((double) i) * orientation_interval;
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

    int x_samples = (int) round( (fabs(max_x - min_x)/ region_sample_spacing));
    int y_samples = (int) round((fabs(max_y - min_y)/ region_sample_spacing));
    ROS_INFO("x samples: %d, y_samples: %d", x_samples, y_samples);
    float x = min_x;
    float y = min_y;
    for (int i=0; i < x_samples; i++) {
      for (int j=0; j < y_samples; j++) {

        x = min_x + ((float) i) * region_sample_spacing;  
        y = min_y + ((float) j) * region_sample_spacing;  

 

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
            yaw = ((double) k) * orientation_interval;
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
          ROS_INFO("Not inside!");
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
 /* for (int i=0; i < releases.size(); i++){
    moveit_msgs::Grasp grasp = releases[i];
    GripperMarker gripper_marker = GripperMarker(node_handle);
    std_msgs::ColorRGBA colour;
    colour.r = random(0.0, 1.0);
    colour.g = random(0.0, 1.0);
    colour.b = random(0.0, 1.0);
    colour.a = 0.5;
    std::ostringstream oss;
    oss << "main_grasp_" <<0 + 15*i;
    std::vector<visualization_msgs::Marker> grasp_markers = gripper_marker.generateMarkerWithColour(0 + 15*i, grasp.grasp_pose, colour, "grasp_pose_frame", oss.str());
    geometry_msgs::PoseStamped pre_grasp_pose = poseFromVec(grasp.grasp_pose, grasp.pre_grasp_approach.direction.vector, grasp.pre_grasp_approach.desired_distance);
    std::ostringstream oss1;
    oss1 << "pre_grasp_" << 5 + 15*i;
    std::vector<visualization_msgs::Marker> pre_grasp_markers = gripper_marker.generateMarkerWithColour(5 + 15*i, pre_grasp_pose, colour, "pre_grasp_pose_frame",oss1.str());
    geometry_msgs::PoseStamped post_grasp_pose = poseFromVec(grasp.grasp_pose, grasp.post_grasp_retreat.direction.vector, grasp.post_grasp_retreat.desired_distance);    
    std::ostringstream oss2;
    oss2 << "post_grasp_" << 10 + 15*i;
    std::vector<visualization_msgs::Marker> post_grasp_markers = gripper_marker.generateMarkerWithColour(10 + 15*i, post_grasp_pose, colour, "post_grasp_pose_frame",  oss2.str());
    //ROS_INFO("publishing grasp markers");
    tf::TransformListener listener;
    listener.waitForTransform("base_footprint", "grasp_pose_frame",
                              ros::Time::now(), ros::Duration(5.0)); 
    listener.waitForTransform("base_footprint", "pre_grasp_pose_frame",
                              ros::Time::now(), ros::Duration(5.0)); 
    listener.waitForTransform("base_footprint", "post_grasp_pose_frame",
                              ros::Time::now(), ros::Duration(5.0)); 
    for(int j=0; j < grasp_markers.size(); j++){
      marker_pub.publish(grasp_markers[j]);
      //ROS_INFO("%d", grasp_markers[j].id);
    }
    //ros::Duration(1.0).sleep();
    visualization_msgs::Marker marker;
    marker.action = 3;
    //marker_pub.publish(marker);
    
    for(int j=0; j < pre_grasp_markers.size(); j++){
      marker_pub.publish(pre_grasp_markers[j]);
      ROS_INFO("%d", pre_grasp_markers[j].id);
    }
    ros::Duration(1.0).sleep();
    marker_pub.publish(marker);
    for(int j=0; j < post_grasp_markers.size(); j++){
      marker_pub.publish(post_grasp_markers[j]);
      ROS_INFO("%d", post_grasp_markers[j].id);
    }
    ros::Duration(1.0).sleep();
    marker_pub.publish(marker);
   
  }*/
  ;
}



}
