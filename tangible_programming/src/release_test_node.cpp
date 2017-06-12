#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "tangible/release_test.h"

using tangible::ReleaseTest;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "release_test_node");
  ros::NodeHandle n, pn("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  // std::string output_frame;
  // std::string cloud_topic;
  std::string scene_service_name;
  std::string release_service_name;
  std::string release_test_service_name;
  pn.getParam("release_test_service_name", release_test_service_name);
  pn.getParam("scene_service_name", scene_service_name);
  pn.getParam("release_service_name", release_service_name);

  ReleaseTest release_test(n, scene_service_name, release_service_name);
  ros::ServiceServer release_test_server = n.advertiseService(release_test_service_name, &ReleaseTest::testCallback, &release_test);
  // ros::Subscriber cloud_sub = n.subscribe(cloud_topic, 1000, &SceneParser::cloudCallback, &scene_parser);
  //ros::spin();
  ros::waitForShutdown();
  spinner.stop();

  return 0;
};
