#include "ros/ros.h"
#include "tangible/compiler.h"

using tangible::Compiler;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "program_service_node");
  ros::NodeHandle n, pn("~");

  ros::AsyncSpinner spinner(2);
  spinner.start();
  int idle_id;
  int run_id;
  int edit_id;
  pn.getParam("idle_id", idle_id);
  pn.getParam("run_id", run_id);
  pn.getParam("edit_id", edit_id);
  std::string mode_change_topic;
  std::string program_service_name;
  std::string block_service_name;
  std::string scene_service_name;
  pn.getParam("mode_change_topic", mode_change_topic);
  pn.getParam("program_service_name", program_service_name);
  pn.getParam("block_service_name", block_service_name);
  pn.getParam("scene_service_name", scene_service_name);
  Compiler compiler(n, idle_id, run_id, edit_id, block_service_name, scene_service_name);
  ros::ServiceServer program_server = n.advertiseService(program_service_name, &Compiler::programCallback, &compiler);
  ros::Subscriber mode_sub = n.subscribe(mode_change_topic, 1000, &Compiler::modeCallback, &compiler);
  compiler.compile();
  //ros::spin();
  ros::waitForShutdown();
  spinner.stop();

  return 0;
}
