#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "tangible/match_test.h"

using tangible::MatchTest;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "match_test_node");
  ros::NodeHandle n, pn("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  // std::string output_frame;
  // std::string cloud_topic;
  std::string scene_service_name;
  std::string object_matching_service_name;
  std::string match_test_service_name;
  pn.getParam("match_test_service_name", match_test_service_name);
  pn.getParam("scene_service_name", scene_service_name);
  pn.getParam("object_matching_service_name", object_matching_service_name);

  MatchTest match_test(n, scene_service_name, object_matching_service_name);
  ros::ServiceServer match_test_server = n.advertiseService(match_test_service_name, &MatchTest::testCallback, &match_test);
  // ros::Subscriber cloud_sub = n.subscribe(cloud_topic, 1000, &SceneParser::cloudCallback, &scene_parser);
  //ros::spin();
  ros::waitForShutdown();
  spinner.stop();

  return 0;
};
