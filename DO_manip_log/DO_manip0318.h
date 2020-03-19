#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

using namespace cv;
using namespace std;

Point2f point;
// vector<Point2f> points[2];
bool addRemovePt = false;

static void onMouse(int event, int x, int y, int, void*)
{
    if (event == EVENT_LBUTTONDOWN){
        point = Point2f((float)x, (float)y);
        // std::cout << "MOUSE MOTION DETECETED--->>>\n";
        addRemovePt = true;
    }
}

class LK_tracker{
public: 
    string winTrack;
    TermCriteria termcrit;
    int maxCount;
    Size subPixWinSize, winSize;
    vector<Point2f> points[2];
    Mat originImg;

    LK_tracker() {}
    LK_tracker(const string winName)
    {
        winTrack = winName;
        termcrit = TermCriteria(TermCriteria::COUNT|TermCriteria::EPS, 20, 0.03);
        maxCount = 3;
        subPixWinSize = Size(10, 10);
        winSize = Size(31, 31);   
    }

    void track(Mat& image, Mat& gray, Mat& prevGray)// Technically, no diffenence between passing cv::Mat and cv::Mat& 
    {
        originImg = image;  // Just create a matrix header to the original image.
        setMouseCallback(winTrack, onMouse, 0);
        if (!points[0].empty()){
            vector<uchar> status;
            vector<float> err;
            if (prevGray.empty())
                gray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);
            size_t i, k;
            for (i = k = 0; i < points[1].size(); i++){
                if (addRemovePt){
                    if (norm(point - points[1][i]) <= 5){
                        addRemovePt = false;
                        continue;
                    }
                }
                if (!status[i])
                    continue;
                points[1][k++] = points[1][i];
                circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
            }
            points[1].resize(k);
        }
        if (addRemovePt && points[1].size() < (size_t) maxCount){
            std::cout << "points[0].empty() returns true--->>>";
            vector<Point2f> tmp;
            tmp.push_back(point);
            cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
            points[1].push_back(tmp[0]);
            addRemovePt = false;
        }
        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);
    }

    float angle3Pts()
    {
        float res;
        if (points[1].size() < 3){
            std::cout << "No Sufficient Reference Pixels!\n";
            return 0.0;
        }
        Eigen::Vector2f ray_1(points[1][1].x - points[1][0].x, points[1][1].y - points[1][0].y),
                        ray_2(points[1][2].x - points[1][0].x, points[1][2].y - points[1][0].y);
        res = acos(ray_1.dot(ray_2) / (ray_1.norm()*ray_2.norm())) * 180 / M_PI;
        cv::line(originImg, points[1][0], points[1][1], Scalar(255, 0, 0), 2);
        cv::line(originImg, points[1][0], points[1][2], Scalar(255, 0, 0), 2);
        return res;
    }
};


class imgExtractor{
public:
    Scalar DOHSVLow;
    Scalar DOHSVHigh;
    Scalar PHSVLow;
    Scalar PHSVHigh;
    Mat originHSVImg;
    vector<vector<Point>> DOContours;
    vector<vector<Point>> PContours;
    vector<Point> PApproxPoly;
    int DOLargestCotrIdx = 0;
    int PLargestCotrIdx = 0;
    bool DOExtractSucceed = false;
    bool PExtractSucceed = false;
    bool extractSucceed = false;
    
    imgExtractor() {}
    imgExtractor(Mat& HSVImg)
    {
        DOHSVLow = Scalar(12, 46, 160);
        DOHSVHigh = Scalar(30, 255, 255);
        PHSVLow = Scalar(152, 166, 135);    // For 3D printed red plane end-effector.
        PHSVHigh = Scalar(180, 255, 255);
        originHSVImg = HSVImg;
    }

    void extract()
    {
        if (originHSVImg.empty()){
            cout << "No Valid Image Inputed!\n";
            exit(-1);
        }

        Mat DOdst, Pdst;        
        vector<Vec4i> hierarcht;
        inRange(originHSVImg, DOHSVLow, DOHSVHigh, DOdst);
        Moments m_dst = moments(DOdst, true);
        cout << m_dst.m00 <<" " << m_dst.m10 << " " << m_dst.m01 << "\n";
        if (m_dst.m00 < 5000){
            cout << "No DO Detected!\n";
            DOExtractSucceed = false;
        }
        else{
            findContours(DOdst, DOContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));  
            cout << "DOContours Size: " << DOContours.size() << " " << DOContours[0].size() << "\n";
            for (int i = 1; i < DOContours.size(); i++){
                DOLargestCotrIdx = (DOContours[i].size() > DOContours[DOLargestCotrIdx].size()) ? i : DOLargestCotrIdx;
            }
            DOExtractSucceed = true;
        }

        inRange(originHSVImg, PHSVLow, PHSVHigh, Pdst);
        m_dst = moments(Pdst, true);
        if (m_dst.m00 < 5000){
            cout << "No End-Effector Detected!\n";
            PExtractSucceed = false;
        }
        else{
            findContours(Pdst, PContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0)); 
            for (int i = 1; i < PContours.size(); i++){
                PLargestCotrIdx = (PContours[i].size() > PContours[PLargestCotrIdx].size()) ? i : PLargestCotrIdx;
            } 
            PExtractSucceed = true;

            approxPolyDP(PContours[PLargestCotrIdx], PApproxPoly, 3, true);
            cout << "PApproxPoly Info: " << PApproxPoly.size() << endl;
        } 
        extractSucceed = DOExtractSucceed && PExtractSucceed;
    }

    void effectorCharacterize()
    {
        Point xMin, xMax;
    }
};

