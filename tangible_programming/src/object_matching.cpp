#include "tangible/object_matching.h"

namespace tangible {

ObjectMatching::ObjectMatching(ros::NodeHandle& n, double point_location_thresh, double box_dimension_thresh) {
  node_handle = n;
  box_marker_pub = node_handle.advertise<visualization_msgs::Marker>("box_markers", 20);
  cloud_marker_pub = node_handle.advertise<sensor_msgs::PointCloud2>("cloud_markers", 20);
  box_dimension_threshold = box_dimension_thresh;
  point_location_threshold = point_location_thresh;

}

ObjectMatching::~ObjectMatching() {}

bool ObjectMatching::matchCallback(tangible_msgs::GetMatchingObjects::Request& req,
                 tangible_msgs::GetMatchingObjects::Response& res)
{
    ROS_INFO("Got service call!");
    if (req.target.type == tangible_msgs::Target::POINT_LOCATION){
      ROS_INFO("Matching object for target type POINT_LOCATION");
      for (int i=0; i<req.objects.size(); i++){
        if (matchesPointLocation(req.objects[i], req.target)){
          res.objects.push_back(req.objects[i]);
        }
      }
    }
    else if (req.target.type == tangible_msgs::Target::REGION) {
      ROS_INFO("Matching object for target type REGION");
      for (int i=0; i<req.objects.size(); i++){
        if (matchesRegion(req.objects[i], req.target)){
          res.objects.push_back(req.objects[i]);
        }
      }

    }
    else if (req.target.type == tangible_msgs::Target::OBJECT_SELECTOR) {
      ROS_INFO("Matching object for target type OBJECT_SELECTOR");
      for (int i=0; i<req.objects.size(); i++){
        if (matchesObjSelector(req.objects[i], req.target)){
          res.objects.push_back(req.objects[i]);
        }
      }

    }

    publishMarkers();
  	return true;
}

bool ObjectMatching::matchesPointLocation(tangible_msgs::SceneObject obj, tangible_msgs::Target target){
  double distance  = sqrt(pow(target.specified_point.point.x - obj.bounding_box.pose.pose.position.x, 2.0) + 
                          pow(target.specified_point.point.y - obj.bounding_box.pose.pose.position.y, 2.0) + 
                          pow(target.specified_point.point.z - obj.bounding_box.pose.pose.position.z, 2.0));
  if (distance < point_location_threshold){
    return true;
  }
  else {
    return false;
  }
}

bool ObjectMatching::onSegment(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r)
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
        return true;
    return false;
}

int ObjectMatching::orientation(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);
 
    if (val == 0) return 0;  // colinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}

// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool ObjectMatching::doIntersect(geometry_msgs::Point p1, geometry_msgs::Point q1, 
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

bool ObjectMatching::isInside(std::vector<geometry_msgs::PointStamped> corners, geometry_msgs::PoseStamped pose)
{
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
}

bool ObjectMatching::matchesRegion(tangible_msgs::SceneObject obj, tangible_msgs::Target target){
  return isInside(target.region_corners, obj.bounding_box.pose);
}

bool ObjectMatching::matchesObjSelector (tangible_msgs::SceneObject obj, tangible_msgs::Target target){
  if (fabs(obj.bounding_box.dimensions.z - target.selected_object.bounding_box.dimensions.z) <  box_dimension_threshold){

    if (fabs(obj.bounding_box.dimensions.x - target.selected_object.bounding_box.dimensions.x) <  box_dimension_threshold
        && fabs(obj.bounding_box.dimensions.y - target.selected_object.bounding_box.dimensions.y) <  box_dimension_threshold){
      return true;
    }
    else if (fabs(obj.bounding_box.dimensions.x - target.selected_object.bounding_box.dimensions.y) <  box_dimension_threshold
        && fabs(obj.bounding_box.dimensions.y - target.selected_object.bounding_box.dimensions.x) <  box_dimension_threshold){
      return true;
    }
    else {
      return false;
    }


  }
  else {
    return false; 
  }
}

void ObjectMatching::publishMarkers(){
  ;
}


}
