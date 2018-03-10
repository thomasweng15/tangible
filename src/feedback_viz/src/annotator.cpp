#include "annotator.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>

namespace feedback_viz {
Annotator::Annotator(const image_transport::Publisher& image_pub) : image_pub_(image_pub) {}

void Annotator::Callback(const geometry_msgs::Pose& msg) {
    std::cout << "hello" << std::endl;
}
}