#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "tangible/grasp_test.h"

bool testCallback(std_srvs::Empty::Request& req,
                 std_srvs::Empty::Response& res){
};

using tangible::GraspTest;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_test_node");
  ros::NodeHandle n, pn("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  // std::string output_frame;
  // std::string cloud_topic;
  std::string scene_service_name;
  std::string grasp_service_name;
  std::string grasp_test_service_name;
  pn.getParam("grasp_test_service_name", grasp_test_service_name);
  pn.getParam("scene_service_name", scene_service_name);
  pn.getParam("grasp_service_name", grasp_service_name);

  GraspTest grasp_test(n, scene_service_name, grasp_service_name);
  ros::ServiceServer grasp_test_server = n.advertiseService(grasp_test_service_name, &GraspTest::testCallback, &grasp_test);
  // ros::Subscriber cloud_sub = n.subscribe(cloud_topic, 1000, &SceneParser::cloudCallback, &scene_parser);
  //ros::spin();
  ros::waitForShutdown();
  spinner.stop();

  return 0;
};
