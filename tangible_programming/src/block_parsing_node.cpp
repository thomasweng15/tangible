#include "ros/ros.h"
#include "tangible/tag_extractor.h"

using tangible::TagExtractor;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "block_service_node");
  ros::NodeHandle n, pn("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  // std::string output_frame;
  // std::string cloud_topic;
  // pn.getParam("output_frame", output_frame);
  // pn.getParam("cloud_topic", cloud_topic);
  std::string tag_topic = "ar_pose_marker"; 
  TagExtractor tag_extractor(n);
  ros::ServiceServer block_server = n.advertiseService("get_blocks", &TagExtractor::parseCallback, &tag_extractor);
  ros::Subscriber tag_sub = n.subscribe(tag_topic, 1000, &TagExtractor::tagCallback, &tag_extractor);
  //ros::spin();
  ros::waitForShutdown();
  spinner.stop();

  return 0;
}
