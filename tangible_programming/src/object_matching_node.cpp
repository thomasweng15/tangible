#include "ros/ros.h"
#include "tangible/object_matching.h"

using tangible::ObjectMatching;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_matching_service_node");
  ros::NodeHandle n, pn("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  std::string object_matching_service_name;
  pn.getParam("object_matching_service_name", object_matching_service_name);
  double point_location_threshold;
  pn.getParam("point_location_threshold", point_location_threshold);
  double box_dimension_threshold;
  pn.getParam("box_dimension_threshold", box_dimension_threshold);

  ObjectMatching object_matching(n, point_location_threshold, box_dimension_threshold);
  ros::ServiceServer scene_server = n.advertiseService(object_matching_service_name, &ObjectMatching::matchCallback, &object_matching);
  //ros::spin();
  ros::waitForShutdown();
  spinner.stop();

  return 0;
}
