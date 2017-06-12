#include "tangible/match_test.h"


namespace tangible {

MatchTest::MatchTest(ros::NodeHandle& n, std::string scene_service_name, std::string object_matching_service_name){
  node_handle = n;
  // marker_pub = node_handle.advertise<visualization_msgs::Marker>("grasp_markers", 20);
  scene_client = n.serviceClient<tangible_msgs::GetScene>(scene_service_name);
  match_client = n.serviceClient<tangible_msgs::GetMatchingObjects>(object_matching_service_name);

}

MatchTest::~MatchTest() {}

bool MatchTest::testCallback(std_srvs::Empty::Request& req,
                 std_srvs::Empty::Response& res)
{
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
