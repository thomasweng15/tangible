#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose.h>

namespace feedback_viz {
class Annotator {
    public:
        Annotator(const image_transport::Publisher& image_pub);
        void Callback(const geometry_msgs::Pose& msg);

    private:
        image_transport::Publisher image_pub_;
};
}