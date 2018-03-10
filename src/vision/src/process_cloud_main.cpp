#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv) {
  if (argc < 3) {
    ROS_INFO("Usage: rosrun perception process_cloud_main DATA_DIR");
    return 1;
  }
  std::string data_dir(argv[1]);

  ros::init(argc, argv, "process_cloud_main");
  ros::NodeHandle nh;
  
  /* Set cloud topic */
  std::string cloud_topic = "cloud_in";
  bool saved_cloud;
  nh.param<bool>("saved_cloud", saved_cloud, false);
  if (!saved_cloud) {
    cloud_topic = "head_mount_kinect/depth_registered/points";
  }

  ros::spin();
  return 0;
}
