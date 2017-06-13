#include "tangible/match_test.h"


namespace tangible {

MatchTest::MatchTest(ros::NodeHandle& n, std::string scene_service_name, std::string object_matching_service_name){
  node_handle = n;
  marker_pub = node_handle.advertise<visualization_msgs::Marker>("match_markers", 20);
  scene_client = n.serviceClient<tangible_msgs::GetScene>(scene_service_name);
  match_client = n.serviceClient<tangible_msgs::GetMatchingObjects>(object_matching_service_name);

}

MatchTest::~MatchTest() {}

void MatchTest::makeMarker(tangible_msgs::SceneObject obj, bool original, int id){

  uint32_t shape = visualization_msgs::Marker::CUBE;

visualization_msgs::Marker marker;
       // Set the frame ID and timestamp.  See the TF tutorials for information on these.
       marker.header.frame_id = obj.bounding_box.pose.header.frame_id;
       marker.header.stamp = ros::Time::now();

       // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
       marker.id = id;
       marker.ns = "matches";
       // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
       marker.type = shape;

       // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
     marker.action = visualization_msgs::Marker::ADD;
       // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
 //ros::Subscriber sub = n.subscribe("range_data", 1000,rangeCallback);
       marker.pose = obj.bounding_box.pose.pose;

       // Set the scale of the marker -- 1x1x1 here means 1m on a side
       marker.scale = obj.bounding_box.dimensions;

       // Set the color -- be sure to set alpha to something non-zero!
       if (original){
       marker.color.r = 0.50f;
       marker.color.g = 0.0f;
       marker.color.b = 0.0f;
       marker.color.a = 0.5;
     }
     else {
      marker.color.r = 0.0f;
       marker.color.g = 0.50f;
       marker.color.b = 0.0f;
       marker.color.a = 0.5;
     }

       marker.lifetime = ros::Duration();

       marker_pub.publish(marker);

}

bool MatchTest::testCallback(std_srvs::Empty::Request& req,
                 std_srvs::Empty::Response& res)
{
  visualization_msgs::Marker marker;
  marker.action = 3;
  marker_pub.publish(marker);

  ROS_INFO("Got test service call, calling scene service");
  tangible_msgs::GetScene srv;
  scene_client.call(srv);
  ROS_INFO("Got scene");

  if (srv.response.scene.objects.size() > 1){
    ROS_INFO("At least 2 objects found in scene, calling matching service");
    tangible_msgs::Target target;
    tangible_msgs::SceneObject obj = srv.response.scene.objects[0];
    target.selected_object = obj;
    target.type = tangible_msgs::Target::OBJECT_SELECTOR;
    tangible_msgs::GetMatchingObjects match_srv;
    std::vector<tangible_msgs::SceneObject> objs;
    for (int i=1; i < srv.response.scene.objects.size(); i++){
     objs.push_back(srv.response.scene.objects[i]);
    }
    match_srv.request.objects = objs;
    match_srv.request.target = target;
    match_client.call(match_srv);
    if (match_srv.response.objects.size() > 0) {
      ROS_INFO("Got matches");
      makeMarker(srv.response.scene.objects[0], true, 0);
      for (int i=0; i <match_srv.response.objects.size(); i++ ){
        makeMarker(match_srv.response.objects[i], false, i+1);

      }
    }
    else {
      ROS_WARN("No matches found");
    }

  }
  else{
    ROS_INFO("Not enough objects found in scene");

  }
}


}
