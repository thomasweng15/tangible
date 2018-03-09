#include <iostream.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

void p(Mat mat, int prec)
{      
    for (int i=0; i<mat.size().height; i++) {
        std::cout << "[";
        for (int j=0; j<mat.size().width; j++) {
            std::cout << std::setprecision(prec) << mat.at<double>(i,j);
            if (j != mat.size().width-1) {
                std::cout << ", ";
            } else {
                std::cout << "]" << std::endl;
            }
        }
    }
}

int main(int argc, char** argv) {
    Mat h_mat = (Mat_<double>(3, 3) << 0.017056081696756556, 5.169707061419698, -408.13652237435826, 5.40767706711781, 0.164021971295147,
  -710.2027607113433, 2.433709738255249e-06, 0.00011007254751859909, 1.0);

    // Get intrinsic camera matrix
    Mat c_mat = (Mat_<double>(3, 3) << 530.871929, 0, 309.633482, 0, 531.987242, 257.565487, 0, 0, 1);

    std::vector<Mat> Rs, Ts, normals;
    // decomposeHomographyMat(h_mat, c_mat, Rs, Ts, normals);
		
    for (std::vector<Mat>::const_iterator i = normals.begin(); i != normals.end(); ++i) {
        p(*i, 5);
    }
}