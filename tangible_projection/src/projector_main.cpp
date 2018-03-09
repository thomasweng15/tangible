#include <ros/ros.h>
#include <image_transport/image_transport.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "project_main");
    ros::NodeHandle nh;

    // Listen for ar tags and project onto tag. 

    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = 
    it.advertise("projector/image", 1);

    ros::spin();
    return 0;
}