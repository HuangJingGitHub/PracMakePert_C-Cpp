#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;


int main(int argc, char** argv)
{
    Mat source_image = imread("./images/color_test.jpeg"), hsv_image, dst, bgr_thr;
    Scalar red_low(0, 160, 158), red_high(10, 255, 255);
    /* Use hsv and inRange*/
    cvtColor(source_image, hsv_image, COLOR_BGR2HSV);
    inRange(hsv_image, red_low, red_high, dst);

    bgr_thr = Mat::zeros(source_image.size(), CV_8UC1);
    int Rows = bgr_thr.rows, Cols = bgr_thr.cols;
    for (int r = 0; r < Rows; r ++)
        for (int c = 0; c < Cols; c++)
        {
            Vec3b intensity = source_image.at<Vec3b>(Point(c, r));
            if (intensity[2] - intensity[0] > 100 && intensity[2] - intensity[1] > 100)
            {
                bgr_thr.at<uchar>(Point(c, r)) = 255;
            }
        }
    
    // For contours 
    vector<vector<Point>> contours;
    //vector<Vec4i> hierarchy;

    findContours(dst, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
    Mat drawing = Mat::zeros(dst.size(), CV_8UC3);
    drawContours(drawing, contours, -1, Scalar(0, 0, 255));
    imshow("Contours in the image", drawing);

    imshow("Source Image", source_image);
    imshow("HSV Detected Result", dst);
    imshow("BGR Difference Detected Result", bgr_thr);
    waitKey(0);
}