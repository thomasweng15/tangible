#include "ros/ros.h"
#include "tangible/tag_extractor.h"

using tangible::TagExtractor;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "block_service_node");
  ros::NodeHandle n, pn("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  int idle_id;
  int run_id;
  int edit_id;
  std::string mode_change_topic;
  std::string block_service_name;
  pn.getParam("idle_id", idle_id);
  pn.getParam("run_id", run_id);
  pn.getParam("edit_id", edit_id);
  pn.getParam("mode_change_topic", mode_change_topic);
  pn.getParam("block_service_name", block_service_name);
  std::string tag_topic = "ar_pose_marker"; 
  TagExtractor tag_extractor(n, idle_id, run_id, edit_id, mode_change_topic);
  ros::ServiceServer block_server = n.advertiseService(block_service_name, &TagExtractor::parseCallback, &tag_extractor);
  ros::Subscriber tag_sub = n.subscribe(tag_topic, 1000, &TagExtractor::tagCallback, &tag_extractor);
  //ros::spin();
  ros::waitForShutdown();
  spinner.stop();

  return 0;
}
