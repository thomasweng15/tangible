#include "ros/ros.h"
#include "tangible/scene_parser.h"

using tangible::SceneParser;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scene_service_node");
  ros::NodeHandle n, pn("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  std::string output_frame;
  std::string cloud_topic;
  std::string scene_service_name;
  pn.getParam("output_frame", output_frame);
  pn.getParam("cloud_topic", cloud_topic);
  pn.getParam("scene_service_name", scene_service_name);

  SceneParser scene_parser(n, output_frame);
  ros::ServiceServer scene_server = n.advertiseService(scene_service_name, &SceneParser::parseCallback, &scene_parser);
  ros::Subscriber cloud_sub = n.subscribe(cloud_topic, 1000, &SceneParser::cloudCallback, &scene_parser);
  //ros::spin();
  ros::waitForShutdown();
  spinner.stop();

  return 0;
}
