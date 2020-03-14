#include "opencv2/video/tracking.hpp"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

Point2f point;
vector<Point2f> points[2];
bool addRemovePt = false;
static void onMouse(int event, int x, int y, int, void*)
{
    if (event == EVENT_LBUTTONDOWN){
        point = Point2f((float)x, (float)y);
        std::cout << "MOUSE MOTION DETECETED--->>>>>\n";
        addRemovePt = true;
    }
}

class LK_tracker{
public: 
    string winTrack;
    TermCriteria termcrit;
    int maxCount;
    Size subPixWinSize, winSize;
    //Mat image, gray, prevGray;

    LK_tracker(const string winName) // Mat &originImg, Mat &trackGray, Mat &trackPrevGray)
    {
        //image = originImg;
        winTrack = winName;
        //gray = trackGray;
        //prevGray = trackPrevGray;
        termcrit = TermCriteria(TermCriteria::COUNT|TermCriteria::EPS, 20, 0.03);
        maxCount = 20;
        subPixWinSize = Size(10, 10);
        winSize = Size(31, 31); 
        
    }

    void track(Mat &image, Mat &gray, Mat &prevGray)
    {
        setMouseCallback(winTrack, onMouse, 0);
        if (!points[0].empty()){
        std::cout << "points[0].empty() returns false----->>>>>";
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
                if ( !status[i])
                    continue;
                points[1][k++] = points[1][i];
                circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
            }
            points[1].resize(k);
        }
        if (addRemovePt && points[1].size() < (size_t) maxCount){
        std::cout << "points[0].empty() returns true----->>>>>";
            vector<Point2f> tmp;
            tmp.push_back(point);
            cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
            points[1].push_back(tmp[0]);
            addRemovePt = false;
        }
        std::cout << "LK ALGORITHM TEST COMPLETED----->>>>>\n";
        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);
    }
};