#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <algorithm>
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

struct img_p {
    Point sl;
    Point sr;
    Eigen::Vector2f ne;
};

struct angleFeature3Pts {
    vector<Point2f> sPt;
    Eigen::Matrix<float, 6, 1> s;
    float angle;
    Eigen::Matrix<float, 1, 6> gradient;  // i.e. Jacobian, it is in row form.
};

bool yComparator(Point pt1, Point pt2) {
    return (pt1.y < pt2.y);
}

vector<Point> *referencePolygon;
bool distanceComparator(Point pt1, Point pt2) {
    if ((*referencePolygon).empty()) {
        cout << "Warning: The Reference Polygon Is Empty!" << endl;
        return false;
    }
    float minDistance1 = 1e3, minDistance2 = 1e3;
    int step = 2;
    for (auto itr = (*referencePolygon).begin(); ((*referencePolygon).end() - itr) > step; itr += step) {
        minDistance1 = (minDistance1 < norm(*itr - pt1)) ? minDistance1 : norm(*itr - pt1);
        minDistance2 = (minDistance2 < norm(*itr - pt2)) ? minDistance2 : norm(*itr - pt2);

    }
    return (minDistance1 < minDistance2);
}

Point getProjection(Point pt, vector<Point> &referContour) {
    if (referContour.empty()) {
        cout << "Warning: The Contour Is Empty!" << endl;
        return Point(0,0);
    }
    Point projectionPt = referContour[0];
    int step = 1;
    for (auto itr = referContour.begin(); (referContour.end() - itr) > step; itr+=step) {
        projectionPt = (norm(*itr - pt) < norm(projectionPt - pt)) ? *itr : projectionPt;
    }
    return projectionPt;
}

Point2f LK_point;
bool addRemovePt = false;
Point2f pTrack_point;
bool pAddRemovePt = false;
Mat saveImg;
RNG cvrng(12345);
int randomInt = cvrng.uniform(0,100);
static void onMouse(int event, int x, int y, int, void*) {
    static int savedImgCount = 1;
    if (event == EVENT_LBUTTONDOWN) {
        LK_point = Point2f((float)x, (float)y);
        addRemovePt = true;
    }
    if (event == EVENT_LBUTTONDBLCLK) {
        pTrack_point = Point2f((float)x, (float)y);
        pAddRemovePt = true;
    }    
    if (event == EVENT_RBUTTONDBLCLK) {
        imwrite("/home/jing/Pictures/DO_manip_Img/DO_manip_img" + to_string(randomInt+savedImgCount) + ".jpg", saveImg);
        savedImgCount++;
    }
}


class LK_tracker {
public: 
    string winTrack;
    TermCriteria termcrit;
    int maxCount;
    Size subPixWinSize, winSize;
    vector<Point2f> points[2];
    Mat originImg;
    bool validFeature = false;
    bool isInitialPoint = true;
    Point2f pointFeatureTarget;
    Point2f pointFeature;

    LK_tracker() {}
    LK_tracker(const string winName) {
        winTrack = winName;
        termcrit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
        maxCount = 1;
        subPixWinSize = Size(10, 10);
        winSize = Size(11, 11);
    }
    
    // Technically, no diffenence between passing cv::Mat and cv::Mat&, OpenCV uses reference passing by default.
    void track(Mat& image, Mat& gray, Mat& prevGray) {
        // Just create a matrix header to the original image.
        originImg = image;
        saveImg = image;
        setMouseCallback(winTrack, onMouse, 0);
        if (!points[0].empty()) {
            vector<uchar> status;
            vector<float> err;
            if (prevGray.empty())
                gray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);
            size_t i, k;
            for (i = k = 0; i < points[1].size(); i++) {
                if (addRemovePt) {
                    if (norm(LK_point - points[1][i]) <= 5){
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

            if (!points[1].empty()) {
                pointFeature = points[1][0];
                validFeature = true;
            }
            else {
                validFeature = false;
            }
        }

        if (addRemovePt && points[1].size() < (size_t) maxCount) {
            vector<Point2f> tmp;
            tmp.push_back(LK_point);
            cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
            points[1].push_back(tmp[0]);
            addRemovePt = false;
            if (isInitialPoint) {
                pointFeatureTarget = tmp[0] - Point2f(60.0, 50.0);
                isInitialPoint = false;
            }
        }
        if (!isInitialPoint)
            circle(image, pointFeatureTarget, 8, Scalar(255, 0, 0), 2, 8);
        std::swap(points[1], points[0]);
        // cv::swap(prevGray, gray);  // swap finished by p extraction operation
    }
};


class imgExtractor {
public:
    Scalar DOHSVLow;
    Scalar DOHSVHigh;
    Mat originHSVImg;
    vector<vector<Point>> DOContours;
    vector<Point> DOContour; 
    img_p endeffectorP;
    Point2f FPC_endeffector;
    vector<int> segmentationIdx[2];
    int DOLargestCotrIdx = 0;
    bool DOExtractSucceed = false;
    bool PExtractSucceed = false;
    bool extractSucceed = false;
    bool effectorCharacterizeSucceed = false;

    string winTrack;
    TermCriteria termcrit;
    int maxCount;
    Size subPixWinSize, winSize;
    vector<Point2f> pPoints[2];

    vector<float> contactDistances;
    
    imgExtractor() {}
    imgExtractor(const string winName) {
        DOHSVLow = Scalar(142, 96, 72);
        DOHSVHigh = Scalar(180, 255, 255);

        winTrack = winName;
        termcrit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
        maxCount = 1;
        subPixWinSize = Size(10, 10);
        winSize = Size(31, 31);
    }

    void extract(Mat& image, Mat& HSVImg, Mat& gray, Mat& prevGray, int occlusion = 0) {
        originHSVImg = HSVImg;
        if (originHSVImg.empty()) {
            cout << "No Valid Image Inputed!\n";
            exit(-1);
        }

        Mat DOdst;
        inRange(originHSVImg, DOHSVLow, DOHSVHigh, DOdst);
        Moments m_dst = moments(DOdst, true);
        if (m_dst.m00 < 5000) {
            cout << "No DO Detected!\n";
            DOExtractSucceed = false;
        }
        else {
            findContours(DOdst, DOContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));  
            DOLargestCotrIdx = 0;
            for (int i = 1; i < DOContours.size(); i++) {
                DOLargestCotrIdx = (DOContours[i].size() > DOContours[DOLargestCotrIdx].size()) ? i : DOLargestCotrIdx;
            }
            DOContour = DOContours[DOLargestCotrIdx];
            DOExtractSucceed = true;
        }

        if (occlusion != 0) {
            int sndLargestCotrIdx = (DOLargestCotrIdx == 0) ? 1 : 0;
            for (int i = 0; i < DOContours.size(); i++) {
                sndLargestCotrIdx = (DOContours[i].size() > DOContours[sndLargestCotrIdx].size() && i != DOLargestCotrIdx)
                                    ? i : sndLargestCotrIdx;
            }
            DOContour.insert(DOContour.end(), DOContours[sndLargestCotrIdx].begin(), DOContours[sndLargestCotrIdx].end());
            DOContours[DOLargestCotrIdx] = DOContour; 
        }

        if (!pPoints[0].empty()) {
            vector<uchar> status;
            vector<float> err;
            if (prevGray.empty())
                gray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, gray, pPoints[0], pPoints[1], status, err, winSize, 3, termcrit, 0, 0.001);
            size_t i, k;
            for (i = k = 0; i < pPoints[1].size(); i++){
                if (pAddRemovePt){
                    if (norm(pTrack_point - pPoints[1][i]) <= 5){
                        pAddRemovePt = false;
                        continue;
                    } 
                }
                if (!status[i])
                    continue;
                pPoints[1][k++] = pPoints[1][i];
                circle(image, pPoints[1][i], 4, Scalar(255, 0, 0), -1, 8);
            }
            pPoints[1].resize(k);
        }
        if (pAddRemovePt && pPoints[1].size() < (size_t) maxCount) {
            vector<Point2f> tmp;
            tmp.push_back(pTrack_point);
            // cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
            pPoints[1].push_back(tmp[0]);
            pAddRemovePt = false;
        }
        std::swap(pPoints[1], pPoints[0]);
        std::swap(prevGray, gray);
        
        PExtractSucceed = (pPoints[1].size() == 1);
        extractSucceed = DOExtractSucceed && PExtractSucceed;
    }

    void effectorCharacterize() {
        if (!PExtractSucceed) {
            effectorCharacterizeSucceed = false;
            return;
        }

        FPC_endeffector = pPoints[1][0];
        effectorCharacterizeSucceed = true;
    }
};


class deformJacobianPoint {
public: 
    Eigen::MatrixXf JdPrev;
    Eigen::MatrixXf JdCurr;
    float alpha;
    Point2f pPrev;
    Point2f ptPrev;
    Eigen::Vector2f pVecPrev;
    Eigen::Vector2f pVecCurr; 
    bool initialized = false;

    deformJacobianPoint() {}
    deformJacobianPoint(Point2f pInitial, Point2f initialPt) {
        alpha = 0.15;
        JdPrev = Eigen::Matrix<float, 2, 2>::Ones();
        pPrev = pInitial;
        ptPrev = initialPt;
        pVecPrev << pInitial.x, pInitial.y;
        initialized = true;
    }
    
    void update(Point2f pCurr, Point2f ptCurr) {
        Eigen::Vector2f delta_y;
        delta_y << ptCurr.x - ptPrev.x, ptCurr.y - ptPrev.y;
        pVecCurr << pCurr.x, pCurr.y;
        Eigen::Vector2f delta_p = pVecCurr - pVecPrev;
        // Eigen::Vector2f delta_ne = pCurr.ne - pPrev.ne;
        
        cout << "Jd Update Data:\n"
             << "Previous Jd:\n" << JdPrev << "\n"
             << "delta_y:\n" << delta_y << "\n"
             << "delta_p:\n" << delta_p << "\n";
        if (delta_p.norm() < 0.05)
            JdCurr = JdPrev;
        else
            JdCurr = JdPrev + alpha * (delta_y - JdPrev*delta_p) / (delta_p.transpose()*delta_p) * delta_p.transpose();       
        JdPrev = JdCurr;
        ptPrev = ptCurr;
        pPrev = pCurr;
        pVecPrev = pVecCurr;
    } 
};