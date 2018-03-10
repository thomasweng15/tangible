#include "annotator.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>

using namespace feedback_viz;

int main(int argc, char** argv) {
    ros::init(argc, argv, "feedback_viz_main");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = 
    it.advertise("projector/image", 1);
    Annotator annotator(image_pub);
    ros::Subscriber annotations_sub = 
        nh.subscribe("annotations", 1, &Annotator::Callback, &annotator);

    ros::Rate r(10);
    while (nh.ok()){
        ros::spinOnce();
        annotator.project();
        r.sleep();
    }
    return 0;
}