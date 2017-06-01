#include "ros/ros.h"
#include "tangible/grasp_generator.h"

using tangible::GraspGenerator;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_generator_node");
  ros::NodeHandle n, pn("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  std::string grasp_service_name;
  pn.getParam("grasp_service_name", grasp_service_name);

  GraspGenerator grasp_generator(n);
  ros::ServiceServer scene_server = n.advertiseService(grasp_service_name, &GraspGenerator::graspCallback, &grasp_generator);
  //ros::spin();
  ros::waitForShutdown();
  spinner.stop();

  return 0;
}
