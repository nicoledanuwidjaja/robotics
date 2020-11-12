
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "cam.hh"

using namespace cv;
using namespace std;

void
cam_init()
{
    namedWindow("camera", 1);
}

void
cam_show(Mat frame)
{
    const int NN = 500;
    if (frame.size().width > 0) {
       // store image in matrix
       Mat image(NN, NN, CV_8UC3);

       for (int ii = 0; ii < NN; ++ii) {
           for (int jj = 0; jj < NN; ++jj) {
                                          //    B         G         R
               image.at<Vec3b>(ii, jj) = Vec3b(ii % 256, jj % 256, jj / 10);
           }
       }

       // make image copy
       // Mat binCopy;
       // image.copyTo(binCopy);

       // transform image to binary form
       // cvtColor(binCopy, binCopy, cv::COLOR_BGR2GRAY);
       // threshold(binCopy, binCopy, 50, 255, cv::THRESH_BINARY_INV);


        imshow("camera", frame);
        cv::waitKey(1);
        cout << "showed a frame" << endl;
    }
}
