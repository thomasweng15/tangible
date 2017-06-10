#include "tangible/grasp_test.h"


namespace tangible {

GraspTest::GraspTest(ros::NodeHandle& n, std::string scene_service_name, std::string grasp_service_name){
  node_handle = n;
  // marker_pub = node_handle.advertise<visualization_msgs::Marker>("grasp_markers", 20);
  scene_client = n.serviceClient<tangible_msgs::GetScene>(scene_service_name);
  grasp_client = n.serviceClient<tangible_msgs::GetGrasps>(grasp_service_name);

}

GraspTest::~GraspTest() {}

bool GraspTest::testCallback(std_srvs::Empty::Request& req,
                 std_srvs::Empty::Response& res)
{
  ROS_INFO("Got test service call, calling scene service");
  tangible_msgs::GetScene srv;
  scene_client.call(srv);
  ROS_INFO("Got scene");

  if (srv.response.scene.objects.size() > 0){
    ROS_INFO("Objects found in scene, calling grasp service");
    tangible_msgs::SceneObject obj = srv.response.scene.objects[0];
    tangible_msgs::GetGrasps grasp_srv;
    grasp_srv.request.object = obj;
    grasp_client.call(grasp_srv);
    if (grasp_srv.response.grasps.size() > 0) {
      ROS_INFO("Got grasps");
    }
    else {
      ROS_WARN("No grasps found");
    }

  }
  else{
    ROS_INFO("No objects found in scene");

  }
}


}
