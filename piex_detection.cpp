#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
using namespace cv;
using namespace std;

vector<int> get_vertex(Mat& input_image)
{
    int y_min = -1, y_max = -1, x_min = -1, x_max = -1,
        x_y_min = 0, x_y_max = 0, y_x_min = 0, y_x_max = 0,
        channels = input_image.channels(),
        nRows = input_image.rows, 
        nCols = input_image.cols * channels;
        cout << "The cols x rows: " << nCols << " X " << nRows << endl;
        vector<int> center;

    if (channels != 1)
    {
        cout << "Input image channel number is limited to be 1 only. END." << endl;
        center.push_back(0);
        center.push_back(0);
        return center;
    }

    MatIterator_<uchar> it = input_image.begin<uchar>(), end = input_image.end<uchar>();
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
    Mat source_image = imread("./color_test.jpeg"), test_image, dst;
    Scalar red_low(0, 173, 152), red_high(10, 255, 255);

    cvtColor(source_image, test_image, COLOR_BGR2HSV);
    inRange(test_image, red_low, red_high, dst);

    cout << source_image.size << " " << test_image.size << endl;
    cout << dst.channels() << " " << source_image.channels() << " " << test_image.channels() << endl;

    vector<int> cen = get_vertex(dst);
    cout << "Center is : " << cen[0] << " " << cen[1] << endl;
    cv::circle(source_image, cv::Point(cen[0], cen[1]), 20, CV_RGB(255, 0, 0));
    // cv::rectangle(source_image, cv::Point(cen[0]-20, cen[1]-20), cv::Point(cen[0]+20, cen[1]+20), 0.5, 8, 0);

    imshow("Source Image", source_image);
    imshow("Detected Result", dst);
    waitKey(0);
}