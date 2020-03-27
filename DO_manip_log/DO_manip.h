#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>
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

struct img_p{
    Point sl;
    Point sr;
    Eigen::Vector2f ne;
};

struct angleFeature3Pts{
    vector<Point2f> sPt;
    Eigen::Matrix<float, 6, 1> s;
    float angle;
    Eigen::Matrix<float, 1, 6> gradient;  // i.e. Jacobian, it is in row form.
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
    int step = 2;
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

    angleFeature3Pts angle3Pts()
    {
        angleFeature3Pts res;
        if (points[1].size() < 3){
            std::cout << "No Sufficient Reference Pixels!\n";
            return res;
        }
        for (int i = 0; i < 3; i++){
            res.sPt.push_back(points[1][i]);
            res.s(2*i, 0) = points[1][i].x;
            res.s(2*i+1, 0) = points[1][i].y;
        }
        float x_0 = points[1][0].x, y_0 = points[1][0].y,
              x_1 = points[1][1].x, y_1 = points[1][1].y,
              x_2 = points[1][2].x, y_2 = points[1][2].y;
        Eigen::Vector2f v_1(x_1 - x_0, y_1 - y_0),
                        v_2(x_2 - x_0, y_2 - y_0);
        float t = v_1.dot(v_2) / (v_1.norm()*v_2.norm()),
              const_1 = -1 / sqrt(1 - t*t),
              const_2 = v_1.dot(v_2),
              n_1 = v_1.norm(), n_2 = v_2.norm();
        res.angle = acos(t) * 180 / M_PI;
        res.gradient(0,0) =  const_1 * ((2*x_0-x_1-x_2)*pow(n_1,2)*pow(n_2,2) - const_2*
                             ((x_0-x_1)*pow(n_2,2)+(x_0-x_2)*pow(n_1,2))) / (pow(n_1,3)*pow(n_2,3));
        res.gradient(0,1) = const_1 * ((2*y_0-y_1-y_2)*pow(n_1,2)*pow(n_2,2) - const_2*
                            ((y_0-y_1)*pow(n_2,2)+(y_0-y_2)*pow(n_2,2))) / (pow(n_1,3)*pow(n_2,3));
        res.gradient(0,2) = const_1 * ((x_2-x_0)*pow(n_1,2)*n_2 - (x_1-x_0)*const_2*n_2) / (pow(n_1,3)*pow(n_2,2));
        res.gradient(0,3) = const_1 * ((y_2-y_0)*pow(n_1,2)*n_2 - (y_1-y_0)*const_2*n_2) / (pow(n_1,3)*pow(n_2,2));
        res.gradient(0,4) = const_1 * ((x_1-x_0)*pow(n_2,2)*n_1 - (x_2-x_0)*const_2*n_1) / (pow(n_1,2)*pow(n_2,3));
        res.gradient(0,5) = const_1 * ((y_1-y_0)*pow(n_2,2)*n_1 - (y_1-y_0)*const_2*n_1) / (pow(n_1,2)*pow(n_2,3));
        
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
    vector<Point> DOContour;
    vector<Point> PContour;
    vector<Point> DOApproxPoly;    
    vector<Point> PApproxPoly;
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
            // cout << "DOContours Size: " << DOContours.size() << " " << DOContours[0].size() << "\n";
            for (int i = 1; i < DOContours.size(); i++){
                DOLargestCotrIdx = (DOContours[i].size() > DOContours[DOLargestCotrIdx].size()) ? i : DOLargestCotrIdx;
            }
            DOContour = DOContours[DOLargestCotrIdx];
            DOExtractSucceed = true;
            // double DOEpisilon = 0.05 * arcLength(DOContours[DOLargestCotrIdx], true);
            // approxPolyDP(DOContours[DOLargestCotrIdx], DOApproxPoly, DOEpisilon, true);
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
            PContour = PContours[PLargestCotrIdx];
            double PEpisilon = 0.05 * arcLength(PContours[PLargestCotrIdx], true);
            // cout << "episilon = " << PEpisilon << "\n";
            approxPolyDP(PContours[PLargestCotrIdx], PApproxPoly, PEpisilon, true);
            // cout << "PApproxPoly Info: " << PApproxPoly.size() << endl;
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
        referencePolygon = &DOContours[DOLargestCotrIdx];
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

    bool segment(vector<Point> &contour)
    {
        if (endeffectorP.sl == Point(0,0) || endeffectorP.sr == Point(0,0)){
            cout << "No End-effector Info On The Image Available!" << endl;
            return false;
        }
        float a = endeffectorP.sr.x - endeffectorP.sl.x,    // line: ax + by + c = 0
              b = endeffectorP.sr.y - endeffectorP.sl.y,
              cl = -a*endeffectorP.sl.x - b*endeffectorP.sl.y,
              cr = -a*endeffectorP.sr.x - b*endeffectorP.sr.y,
              divCof = 1 / norm(endeffectorP.sr - endeffectorP.sl), 
              distancel, distancer;
        vector<int> idxl, idxr;
        // vector<float> disLog;
        int distanceThreshold = 20, step = 1;
        for (int i = 0; contour.size() - i > step; i+=step){
            distancel = abs(a*contour[i].x + b*contour[i].y + cl) * divCof;
            distancer = abs(a*contour[i].x + b*contour[i].y + cr) * divCof;
            // disLog.push_back(distancel);
            if (distancel < distanceThreshold)
                idxl.push_back(i);
            if (distancer < distanceThreshold)
                idxr.push_back(i);
        }
        if (idxl.empty() || idxr.empty()){
            cout << "Invalid Segmentation. Maybe A Samll Threshold Distance Or A Large Step Is Given." << endl;
            return false;
        }
        /*sort(disLog.begin(), disLog.end());
        for (auto itr = disLog.begin(); itr != disLog.end(); itr++)
            cout << *itr << "\n";
        cout << contour << endl;
        segmentationIdx[0] = idxl;
        segmentationIdx[1] = idxr;*/
        
        int minIdxl = 0, maxIdxl = 0;
        for (int i = 1; i < idxl.size(); i++){
            minIdxl = (norm(contour[idxl[i]] - endeffectorP.sl) < norm(contour[idxl[minIdxl]] - endeffectorP.sl)) ? i:minIdxl;
            maxIdxl = (norm(contour[idxl[i]] - endeffectorP.sl) > norm(contour[idxl[maxIdxl]] - endeffectorP.sl)) ? i:maxIdxl;
        }
        segmentationIdx[0].clear(); 
        segmentationIdx[0].push_back(idxl[minIdxl]);
        segmentationIdx[0].push_back(idxl[maxIdxl]);
        cout << norm(contour[minIdxl] - endeffectorP.sl) << " " << norm(contour[maxIdxl] - endeffectorP.sl) << "\n";

        int minIdxr = 0, maxIdxr = 0;
        for (int i = 1; i < idxr.size(); i++){
            minIdxr = (norm(contour[idxr[i]] - endeffectorP.sr) < norm(contour[idxr[minIdxr]] - endeffectorP.sr)) ? i:minIdxr;
            maxIdxr = (norm(contour[idxr[i]] - endeffectorP.sr) > norm(contour[idxr[maxIdxr]] - endeffectorP.sr)) ? i:maxIdxr;
        }
        segmentationIdx[1].clear(); 
        segmentationIdx[1].push_back(idxr[minIdxr]);
        segmentationIdx[1].push_back(idxr[maxIdxr]);
        return true;
    }
};

class optConstructor{
public:
    imgExtractor extractedImg;
    float T_CP;
    Eigen::Matrix<float, 6, 1> pT_pP;
    Point2f ElCentroid;
    Point2f ECentroid;
    Point2f ErCentroid;
    Point2f PrialDirtl;
    Point2f PrialDirtr;
    vector<float> deformAngle;
    Point2f sw;

    optConstructor() {}
    optConstructor(imgExtractor &extractor)
    {
        extractedImg = extractor;
    }

    void getLocalContact()
    {
        Point projPtl = getProjection(extractedImg.endeffectorP.sl, extractedImg.DOContour),
              projPtr = getProjection(extractedImg.endeffectorP.sr, extractedImg.DOContour),
              projPtm = getProjection((extractedImg.endeffectorP.sl + extractedImg.endeffectorP.sr)/2, extractedImg.DOContour),
              gradtl = projPtl - extractedImg.endeffectorP.sl,
              gradtr = projPtr - extractedImg.endeffectorP.sr,
              gradtm = projPtm - (extractedImg.endeffectorP.sl + extractedImg.endeffectorP.sr)/2;
        T_CP  = max(norm(projPtl - extractedImg.endeffectorP.sl), norm(projPtr - extractedImg.endeffectorP.sr));
        pT_pP << gradtl.x, gradtl.y, gradtr.x, gradtr.y, gradtm.x, gradtm.y;
    }

    void getDeformConstraint()
    {   
        vector<Point> *contourPtr = &extractedImg.DOContour;
        vector<Point> El, E, Er;
        if (extractedImg.segmentationIdx[0].empty() || extractedImg.segmentationIdx[1].empty()){
            cout << "No Valid Segmentation Info Available From The imgExtractor！" << endl;
            return;
        }
        int k1 = extractedImg.segmentationIdx[0][0], k2 = extractedImg.segmentationIdx[0][1],
            k3 = extractedImg.segmentationIdx[1][0], k4 = extractedImg.segmentationIdx[1][1];
        cout << "k1-k2-k3-k4:\n" << k1 << "-" << k2 << "-" << k3 << "-" << k4 << "\n";
        if (k2 <= k1 && k1 <= k3 && k3 <= k4){
            El = vector<Point>((*contourPtr).begin() + k2, (*contourPtr).begin() + k1 + 1);
            Er = vector<Point>((*contourPtr).begin() + k3, (*contourPtr).begin() + k4 + 1);
            E = vector<Point>((*contourPtr).begin(), (*contourPtr).begin() + k2 + 1);
            E.insert(E.end(), (*contourPtr).begin() + k1, (*contourPtr).begin() + k3 + 1);
            E.insert(E.end(), (*contourPtr).begin() + k4, (*contourPtr).end());
        }
        else if (k1 <= k3 && k3 <= k4 && k4 <= k2){
            El = vector<Point>((*contourPtr).begin() + k2, (*contourPtr).end());
            El.insert(El.end(), (*contourPtr).begin(), (*contourPtr).begin() + k1 + 1);
            Er = vector<Point>((*contourPtr).begin() + k3, (*contourPtr).begin() + k4 + 1);
            E = vector<Point>((*contourPtr).begin() + k1, (*contourPtr).begin() + k3 + 1);
            E.insert(E.end(), (*contourPtr).begin() + k4, (*contourPtr).end());
            E.insert(E.end(), (*contourPtr).begin(), (*contourPtr).begin() + k2 + 1);
        }
        else if (k3 <= k4 && k4 <= k2 && k2 <= k1){
            El = vector<Point>((*contourPtr).begin() + k2, (*contourPtr).begin() + k1 + 1);
            Er = vector<Point>((*contourPtr).begin() + k3, (*contourPtr).begin() + k4 + 1);
            E = vector<Point>((*contourPtr).begin(), (*contourPtr).begin() + k3 + 1);
            E.insert(E.end(), (*contourPtr).begin() + k4, (*contourPtr).begin() + k2 + 1);
            E.insert(E.end(), (*contourPtr).begin() + k1, (*contourPtr).end());
        }
        else if (k4 <= k2 && k2 <= k1 && k1 << k3){
            El = vector<Point>((*contourPtr).begin() + k2, (*contourPtr).begin() + k1 + 1);
            Er = vector<Point>((*contourPtr).begin(), (*contourPtr).begin() + k4 + 1);
            Er.insert(Er.end(), (*contourPtr).begin() + k3, (*contourPtr).end());
            E = vector<Point>((*contourPtr).begin() + k4, (*contourPtr).begin() + k2 + 1);
            E.insert(E.end(), (*contourPtr).begin() + k1, (*contourPtr).begin() + k3 + 1);
        }
        else{
            cout << "Bad Segmentation Of The Contour!" << endl;
            return;
        }
        Moments ElMoments = moments(El), EMoments = moments(E), ErMoments = moments(Er);
        ElCentroid = Point2f(ElMoments.m10 / ElMoments.m00, ElMoments.m01 / ElMoments.m00);
        ECentroid = Point2f(EMoments.m10 / EMoments.m00, EMoments.m01 / EMoments.m00);
        ErCentroid = Point2f(ErMoments.m10 / ErMoments.m00, ErMoments.m01 / ErMoments.m00);

        Eigen::Matrix2f Il, I, Ir;
        Il << ElMoments.mu20, ElMoments.mu11, ElMoments.mu11, ElMoments.mu02;
        I << EMoments.mu20, EMoments.mu11, EMoments.mu11, EMoments.mu02;
        Ir << ErMoments.mu20, ErMoments.mu11, ErMoments.mu11, ErMoments.mu02;
        Eigen::EigenSolver<Eigen::Matrix2f> eigSloverl(Il), eigSolverr(Ir);
        Eigen::Vector2f eigVecl1(eigSloverl.eigenvectors().col(0).real()[0], 
                                 eigSloverl.eigenvectors().col(0).real()[1]),
                        eigVecl2(eigSloverl.eigenvectors().col(1).real()[0], 
                                 eigSloverl.eigenvectors().col(1).real()[1]),
                        eigVecr1(eigSolverr.eigenvectors().col(0).real()[0],
                                 eigSolverr.eigenvectors().col(0).real()[1]),
                        eigVecr2(eigSolverr.eigenvectors().col(1).real()[0],
                                 eigSolverr.eigenvectors().col(1).real()[1]);
        PrialDirtl = Point2f(eigVecl1[0], eigVecl1[1]);
        PrialDirtr = Point2f(eigVecr1[0], eigVecr1[1]);
        Eigen::Vector2f planeDirt(extractedImg.endeffectorP.sr.x - extractedImg.endeffectorP.sl.x,
                                  extractedImg.endeffectorP.sr.y - extractedImg.endeffectorP.sl.y);
        float anglel = acos(planeDirt.dot(eigVecl1) / (planeDirt.norm()*eigVecl1.norm())) * 180 / M_PI,
              angler = acos(planeDirt.dot(eigVecr1) / (planeDirt.norm()*eigVecr1.norm())) * 180 / M_PI;
        deformAngle.clear();
        deformAngle.push_back(anglel);
        deformAngle.push_back(angler);
    }

    void getManipulability(angleFeature3Pts angFeature){
        Point2f sm = (extractedImg.endeffectorP.sl+extractedImg.endeffectorP.sr) / 2;
        float a = extractedImg.endeffectorP.sr.x - extractedImg.endeffectorP.sl.x,    // line: ax + by + c = 0
              b = extractedImg.endeffectorP.sr.y - extractedImg.endeffectorP.sl.y,
              cl = -a*extractedImg.endeffectorP.sl.x - b*extractedImg.endeffectorP.sl.y,
              cr = -a*extractedImg.endeffectorP.sr.x - b*extractedImg.endeffectorP.sr.y,
              divCof = 1 / norm(extractedImg.endeffectorP.sr - extractedImg.endeffectorP.sl),
              refl = a*sm.x + b*sm.y + cl, 
              refr = a*sm.x + b*sm.y + cr,
              episilonIn = 20,
              episilonOut = 100;  // The distance is in the range of several tens pixels.
        vector<int> EIndictor(3,0);
        vector<float> distance(3,0), Wi(3,0);
        for (int i = 0; i < 3; i++){
            float linel = a*angFeature.sPt[i].x + b*angFeature.sPt[i].y + cl,
                  liner = a*angFeature.sPt[i].x + b*angFeature.sPt[i].y + cr;
            if (linel*refl >= 0 && liner*refr >= 0){
                EIndictor[i] = 1;
                distance[i] = divCof * abs(b*angFeature.sPt[i].x - a*angFeature.sPt[i].y - 
                              b*extractedImg.endeffectorP.sl.x + a*extractedImg.endeffectorP.sl.y);
                Wi[i] = 1 / (distance[i] + episilonIn);
            }
            else if (linel*refl < 0){
                EIndictor[i] = 0;
                distance[i] = abs(linel) * divCof;
                Wi[i] = 1 / (distance[i] + episilonOut);
            }
            else{
                EIndictor[i] = 0;
                distance[i] = abs(liner) * divCof;
                Wi[i] = 1 / (distance[i] + episilonOut); 
            }
        }
        Eigen::MatrixXf Dw = Eigen::MatrixXf::Zero(6,6);
        for (int i = 0; i<3; i++){
            Dw(2*i, 2*i) = Wi[i];
            Dw(2*i+1, 2*i+1) = Wi[i];
        }
        cout << "Line Distance:\n";
        for (auto x:distance)
            cout << x << "  ";
        cout << "\nDw =\n" << Dw << "\n";
        cout << "Angle Feature Jacobian:\n" << angFeature.gradient << "\n";
        Eigen::Matrix<float, 1, 6> weightedGradient = angFeature.gradient * Dw;
        cout << "weightedGradient:\n" << weightedGradient << "\n";
        sw = Point2f(0,0);
        for (int i = 0; i < 3; i++){
            cout << "weight:\n"
                 << (weightedGradient.col(2*i).lpNorm<1>() + weightedGradient.col(2*i+1).lpNorm<1>()) / weightedGradient.lpNorm<1>()
                 << "\n";
            sw += (weightedGradient.col(2*i).lpNorm<1>() + weightedGradient.col(2*i+1).lpNorm<1>())
                  / weightedGradient.lpNorm<1>() * angFeature.sPt[i];
        }
    }
};

class deformJacobian{
public: 
    int dimRow;
    int dimCol;
    Eigen::MatrixXf JdPrev;
    Eigen::MatrixXf JdCurr;
    float alpha;
    img_p pPrev;
    angleFeature3Pts anglePrev;
    Eigen::Vector2f pVec;

    deformJacobian() {}
    deformJacobian(img_p pInitial, angleFeature3Pts angleInitial)
    {
        dimRow = 2;
        dimCol = 2;
        alpha = 0.2;
        JdPrev = Eigen::Matrix<float, 2, 2>::Identity();
        pPrev = pInitial;
        anglePrev = angleInitial;
    }
    
    void update(angleFeature3Pts angleCurr)
    {
        float delta_y = angleCurr.angle - anglePrev.angle;
    }
};