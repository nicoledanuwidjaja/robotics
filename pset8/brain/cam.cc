#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "cam.hh"

using namespace cv;
using namespace std;

void cam_init()
{
    namedWindow("Robot POV", 1);
    namedWindow("Maze", 1);
    namedWindow("Hough", 1);
}

/* Convert image to maze representation with Canny edge detection algorithm. */
void cam_show(Mat frame)
{
    const int NN = 500;
    Mat gray;
    Mat blurred;
    Mat canny;

    if (frame.size().width > 0)
    {
        // store image in matrix
        Mat image(NN, NN, CV_8UC3);

        for (int ii = 0; ii < NN; ++ii)
        {
            for (int jj = 0; jj < NN; ++jj)
            {
                // BGR color representation
                image.at<Vec3b>(ii, jj) = Vec3b(0, ii % 256, jj % 256);
            }
        }

        // transform image from RGB --> HSV --> grayscale
        cvtColor(image, gray, COLOR_BGR2GRAY);

        // smooth window and apply filtering
        GaussianBlur(gray, blurred, Size(5, 5), 1.5);
        Canny(blurred, canny, 100, 200);

        // Canny detection with scharr
        Mat dx, dy, final;
        int scharr = 1;
        Scharr(canny, dx, CV_16S, 1, 0);
        Scharr(canny, dy, CV_16S, 0, 1);
        Canny(dx, dy, final, scharr, scharr * 3);

        // Used hough lines logic from OpenCV code.
        vector<Vec2f> lines;
        HoughLines(final, lines, 1, CV_PI / 180, 100, 0, 0);
        
        for (size_t i = 0; i < lines.size(); i++)
        {
            float rho = lines[i][0], theta = lines[i][1];
            Point pt1, pt2;
            double a = cos(theta);
            double b = sin(theta);
            double x0 = a * rho;
            double y0 = b * rho;
            pt1.x = cvRound(x0 + 1000 * -b);
            pt1.y = cvRound(y0 + 1000 * a);
            pt2.x = cvRound(x0 - 1000 * -b);
            pt2.y = cvRound(y0 - 1000 * a);
            line(final, pt1, pt2, Scalar(0, 0, 255), 3, LINE_AA);
        }

        imshow("Robot POV", frame);
        imshow("Maze", canny);
        imshow("Hough", final);

        cv::waitKey(1);
        cout << "Frame sent" << endl;
    }
}
