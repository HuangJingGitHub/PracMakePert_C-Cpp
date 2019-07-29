#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp> // for cv::undistort()
#include <iostream>
using namespace cv;
using namespace std;


// get_vertex is just a primary and not efficient way to get the geometry centroid.
vector<int> get_vertex(Mat& input_image)
{
    int y_min = -1, y_max = -1, x_min = -1, x_max = -1,
        x_y_min = 0, x_y_max = 0, y_x_min = 0, y_x_max = 0,
        channels = input_image.channels(),
        nRows = input_image.rows, 
        nCols = input_image.cols * channels;
        cout << "The cols x rows: " << nCols << " x " << nRows << endl;
        vector<int> center;

    if (channels != 1)
    {
        cout << "Input image channel number is limited to be 1 only. END." << endl;
        center.push_back(0);
        center.push_back(0);
        return center;
    }

    // MatIterator_<uchar> it = input_image.begin<uchar>(), end = input_image.end<uchar>();
    for (int row = 0, counter = 0;  row < nRows; row++)
        for (int col = 0; col < nCols; col++)
        {
            if (input_image.at<uchar>(row, col) > 0)
            {
                y_min = (y_min == -1) ? row : y_min;
                x_y_min = (x_y_min == 0 & y_min != -1) ? col: x_y_min;
                y_max = row;
                x_y_max = col;

                x_min = (x_min == -1) ? col : x_min;
                x_min = (x_min < col) ? x_min : col;
                y_x_min = (x_min == col) ? row : y_x_min;
                x_max = (x_max == -1) ? col : x_max;
                x_max = (x_max < col) ? col : x_max;
                y_x_max = (x_max == col) ? row : y_x_max;     
            }
            counter++;
        }
        
        center.push_back((y_min + y_max + y_x_min + y_x_max) / 4);
        center.push_back((x_min + x_max + x_y_min + x_y_max) / 4);
        return center; 
}

int main(int argc, char** argv)
{
    //Mat source_image = imread("./images/color_test.jpeg"), test_image, dst, gray, bin_thr, bgr_thr;
    Mat source_image = imread("./images/GSON_distorted.png"), test_image, dst, gray, bin_thr, bgr_thr;
    Scalar red_low(0, 160, 158), red_high(10, 255, 255);
    /* Use hsv and inRange*/
    cvtColor(source_image, test_image, COLOR_BGR2HSV);
    inRange(test_image, red_low, red_high, dst);

    cout << source_image.size << " " << test_image.size << endl;
    cout << dst.channels() << " " << source_image.channels() << " " << test_image.channels() << endl;

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
    
    // vector<int> cen = get_vertex(dst);
    // cout << "Geometry center is : " << cen[0] << " " << cen[1] << endl;
    // cv::circle(source_image, cv::Point(cen[0], cen[1]), 20, CV_RGB(255, 0, 0));
    // cv::rectangle(source_image, cv::Point(cen[0]-20, cen[1]-20), cv::Point(cen[0]+20, cen[1]+20), 0.5, 8, 0);
    /* Use gray image */

    // cvtColor(source_image, gray, COLOR_BGR2GRAY);
    // threshold(dst, bin_thr, 100, 255, THRESH_BINARY);
    // imshow("Gray image", bin_thr);
    
    Moments mc = moments(dst, true);
    Point p(mc.m10/mc.m00, mc.m01/mc.m00);
    cout << Mat(p) << endl;
    circle(source_image, p, 10, Scalar(128, 0, 0), -1);
    
    Mat tracked_image_undist;
    Mat camera_matrix = (cv::Mat_<double>(3, 3) << 683.9731, 0, 320., 0, 683.9731, 240, 0, 0, 1);
    Mat dist_coef = (cv::Mat_<double>(1, 5) << -0.7175, 2.8528, 0, 0, -5.1548);
    cv::undistort(source_image, tracked_image_undist, camera_matrix, dist_coef);
    imshow("Source Image", source_image);
    imshow("HSV Detected Result", dst);
    imshow("BGR Difference Detected Result", bgr_thr);
    imshow("test undistorted", tracked_image_undist);
    waitKey(0);
}