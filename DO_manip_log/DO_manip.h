#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>
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

struct img_p{
    Point sl;
    Point sr;
    Eigen::Vector2f ne;
};

bool yComparator(Point pt1, Point pt2){
    return (pt1.y < pt2.y);
}

vector<Point> *referencePolygon;
bool distanceComparator(Point pt1, Point pt2)
{
    if ((*referencePolygon).empty()){
        cout << "Warning: The Reference Polygon Is Empty!" << endl;
        return false;
    }
    float minDistance1 = 1e3, minDistance2 = 1e3;
    //int step = (*referencePolygon).size() / 20 + 1;
    int step = 5;
    for (auto itr = (*referencePolygon).begin(); ((*referencePolygon).end() - itr) > step; itr+=step){
        minDistance1 = (minDistance1 < norm(*itr - pt1)) ? minDistance1 : norm(*itr - pt1);
        minDistance2 = (minDistance2 < norm(*itr - pt2)) ? minDistance2 : norm(*itr - pt2);

    }
    return (minDistance1 < minDistance2);
}

Point getProjection(Point pt, vector<Point> &referContour)
{
    if (referContour.empty()){
        cout << "Warning: The Contour Is Empty!" << endl;
        return Point(0,0);
    }
    Point projectionPt = referContour[0];
    int step = 2;
    for (auto itr = referContour.begin(); (referContour.end() - itr) > step; itr+=step){
        projectionPt = (norm(*itr - pt) < norm(projectionPt - pt)) ? *itr : projectionPt;
    }
    return projectionPt;
}

Point2f LK_point;
// vector<Point2f> points[2];
bool addRemovePt = false;
Mat saveImg;
RNG cvrng(12345);
int randomInt = cvrng.uniform(0,100);
static void onMouse(int event, int x, int y, int, void*)
{
    // std::cout << "MOUSE MOTION DETECETED--->>>\n";
    static int savedImgCount = 1;
    if (event == EVENT_LBUTTONDOWN){
        LK_point = Point2f((float)x, (float)y);
        addRemovePt = true;
    }
    if (event == EVENT_LBUTTONDBLCLK){
        imwrite("/home/jing/Pictures/DO_manip_Img/DO_manip_img" + to_string(randomInt+savedImgCount) + ".jpg", saveImg);
        savedImgCount++;
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

    void track(Mat& image, Mat& gray, Mat& prevGray) // Technically, no diffenence between passing cv::Mat and cv::Mat& 
    {
        originImg = image;  // Just create a matrix header to the original image.
        saveImg = image;
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
                    if (norm(LK_point - points[1][i]) <= 5){
                        addRemovePt = false;
                        continue;
                    }
                }
                if (!status[i])
                    continue;
                points[1][k++] = points[1][i];
                circle(image, points[1][i], 3, Scalar(0, 0, 255), -1, 8);
            }
            points[1].resize(k);
        }
        if (addRemovePt && points[1].size() < (size_t) maxCount){
            cout << "points[0].empty() returns true--->>>";
            vector<Point2f> tmp;
            tmp.push_back(LK_point);
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
        cv::line(originImg, points[1][0], points[1][1], Scalar(0, 0, 255), 2);
        cv::line(originImg, points[1][0], points[1][2], Scalar(0, 0, 255), 2);
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
    vector<Point> DOApproxPoly;
    img_p endeffectorP;
    vector<int> segmentationIdx[2];
    int DOLargestCotrIdx = 0;
    int PLargestCotrIdx = 0;
    bool DOExtractSucceed = false;
    bool PExtractSucceed = false;
    bool extractSucceed = false;
    bool effectorCharacterizeSucceed = false;
    
    imgExtractor() {}
    imgExtractor(Mat& HSVImg)
    {
        DOHSVLow = Scalar(8, 97, 155);
        DOHSVHigh = Scalar(35, 255, 255);
        PHSVLow = Scalar(20, 148, 135);    // For 3D printed red plane end-effector.
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

            double DOEpisilon = 0.05 * arcLength(DOContours[DOLargestCotrIdx], true);
            // approxPolyDP(DOContours[DOLargestCotrIdx], DOApproxPoly, DOEpisilon, true);
            referencePolygon = &DOContours[DOLargestCotrIdx];
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
            double PEpisilon = 0.05 * arcLength(PContours[PLargestCotrIdx], true);
            cout << "episilon = " << PEpisilon << "\n";
            approxPolyDP(PContours[PLargestCotrIdx], PApproxPoly, PEpisilon, true);
            cout << "PApproxPoly Info: " << PApproxPoly.size() << endl;
        } 
        extractSucceed = DOExtractSucceed && PExtractSucceed;
    }

    void effectorCharacterize()
    {
        effectorCharacterizeSucceed = false;
        vector<Point> PVertices;
        vector<int> ySortedPVertices;
        Point prevPt, nextPt;
        Eigen::Vector2i prevSide, nextSide;
        float angle;
        for (int i = 0; i < PApproxPoly.size(); i++){
            if (i == 0){
                prevPt = PApproxPoly.back();
                nextPt = PApproxPoly[1];
            }
            else if (i == PApproxPoly.size() - 1){
                prevPt = PApproxPoly[i-1];
                nextPt = PApproxPoly[0];
            }
            else{
                prevPt = PApproxPoly[i-1];
                nextPt = PApproxPoly[i+1];
            }
            prevSide = Eigen::Vector2i(prevPt.x - PApproxPoly[i].x, prevPt.y - PApproxPoly[i].y);
            nextSide = Eigen::Vector2i(nextPt.x - PApproxPoly[i].x, nextPt.y - PApproxPoly[i].y);
            angle = acos(prevSide.dot(nextSide) / (prevSide.norm()*nextSide.norm())) * 180 / M_PI;
            if (angle > 40 && angle < 150)
                PVertices.push_back(PApproxPoly[i]);
        }
        sort(PVertices.begin(), PVertices.end(), distanceComparator);
        if (PVertices.size() >= 2){      // Avoid extreme cases where just less than 2 vertices exist.
            if (PVertices[0].x < PVertices[1].x){
                endeffectorP.sl = PVertices[0];
                endeffectorP.sr = PVertices[1];
            }
            else{
                endeffectorP.sl = PVertices[1];
                endeffectorP.sr = PVertices[0];
            }
            Eigen::Vector2f endeffectorDirt(endeffectorP.sr.x - endeffectorP.sl.x, 
                                            endeffectorP.sr.y - endeffectorP.sl.y);
            endeffectorDirt = endeffectorDirt / endeffectorDirt.norm();
            endeffectorP.ne(0) = endeffectorDirt(1);    // Make sure ne points upwards.
            endeffectorP.ne(1) = -endeffectorDirt(0);
            effectorCharacterizeSucceed = true;
        }
    }

    void segment(vector<Point> &contour)
    {
        if (endeffectorP.sl == Point(0,0) || endeffectorP.sr == Point(0,0)){
            cout << "No End-effector Info On The Image Available!" << endl;
            return;
        }
        float a = endeffectorP.sr.x - endeffectorP.sl.x,    // line: ax + by + c = 0
              b = endeffectorP.sr.y - endeffectorP.sl.y,
              cl = -a*endeffectorP.sl.x - b*endeffectorP.sl.y,
              cr = -a*endeffectorP.sr.x - b*endeffectorP.sr.y,
              divCof = 1 / norm(endeffectorP.sr - endeffectorP.sl), 
              distancel, distancer;  
        vector<int> idxl, idxr;
        int distanceThreshold = 1, step = 2;
        for (int i = 0; contour.size() - i > step; i+=step){
            distancel = abs(a*contour[i].x + b*contour[i].y + cl) * divCof;
            distancer = abs(a*contour[i].x + b*contour[i].y + cr) * divCof;
            if (distancel < distanceThreshold)
                idxl.push_back(i);
            if (distancer < distanceThreshold)
                idxr.push_back(i);
        }
        int minIdxl = 0, maxIdxl = 0;
        for (int i = 1; i < idxl.size(); i++){
            minIdxl = (norm(contour[idxl[i]] - endeffectorP.sl) < norm(contour[idxl[minIdxl]] - endeffectorP.sl)) ? i:minIdxl;
            maxIdxl = (norm(contour[idxl[i]] - endeffectorP.sl) > norm(contour[idxl[maxIdxl]] - endeffectorP.sl)) ? i:maxIdxl;
        }
        segmentationIdx[0].clear(); 
        segmentationIdx[0].push_back(minIdxl);
        segmentationIdx[0].push_back(maxIdxl);

        int minIdxr = 0, maxIdxr = 0;
        for (int i = 1; i < idxr.size(); i++){
            minIdxr = (norm(contour[idxr[i]] - endeffectorP.sr) < norm(contour[idxr[minIdxr]] - endeffectorP.sr)) ? i:minIdxr;
            maxIdxr = (norm(contour[idxr[i]] - endeffectorP.sr) > norm(contour[idxr[maxIdxr]] - endeffectorP.sr)) ? i:maxIdxr;
        }
        segmentationIdx[1].clear(); 
        segmentationIdx[1].push_back(minIdxr);
        segmentationIdx[1].push_back(maxIdxr);
    }
};