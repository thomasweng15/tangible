#include "ros/ros.h"
#include "tangible/release_generator.h"

using tangible::ReleaseGenerator;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "release_generator_node");
  ros::NodeHandle n, pn("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  std::string release_service_name;
  pn.getParam("release_service_name", release_service_name);

  ReleaseGenerator release_generator(n);
  ros::ServiceServer release_server = n.advertiseService(release_service_name, &ReleaseGenerator::releaseCallback, &release_generator);
  //ros::spin();
  ros::waitForShutdown();
  spinner.stop();

  return 0;
}
