#include <iostream>
#include <vector>
#include <string>
#include <math.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp> // Rodrigues()

using namespace cv;
using namespace std;

double PI = 3.14159265;

int main(int argc, char **argv)
{

    Mat markerImage, cameraMatrix, distCoeffs, dvrkBaseRef, cam2dvrkBase, videoFrame;
    cameraMatrix = (Mat_<double>(3,3) << 1261, 0, 924,
                                         0, 1266, 521,
                                         0, 0, 1);
    distCoeffs =  (Mat_<double>(1, 4) << 0.0664, -0.1899, 0, 0);

    double dvrkBaseAngle = 75.0 / 180.0 * PI;
    dvrkBaseRef = (Mat_<double>(3,3) << cos(dvrkBaseAngle), 0, sin(dvrkBaseAngle),
                                        0, 1, 0,
                                        -sin(dvrkBaseAngle), 0, cos(dvrkBaseAngle));

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_50);
    // aruco::drawMarker(dictionary, 0, 400, markerImage, 1);
    // imwrite(string("/home/jing/Pictures/DO_manip_Img/")+"aruco_marrker_400pxs.jpg", markerImage);
    
    VideoCapture cam((argc > 1 ? atoi(argv[1]) : 0));
    if (!cam.isOpened()){
        cerr << "Unable to open the cam!\n";
        return -1; 
    }

    while (true){
        cam >> videoFrame;

        vector<int> ids;
        vector<vector<Point2f>> corners;
        aruco::detectMarkers(videoFrame, dictionary, corners, ids);

        if (ids.size() > 0){
            aruco::drawDetectedMarkers(videoFrame, corners, ids);

            vector<Vec3d> rvecs, tvecs;
            Mat rotMat;
            aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

            for (int i = 0; i < ids.size(); i++){
                aruco::drawAxis(videoFrame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.05);
                Rodrigues(rvecs[i], rotMat);
                cam2dvrkBase = dvrkBaseRef * rotMat.t();
                cout << "\nrotation vectors (angle-axis):\n" << rvecs[i]
                     << "\nRotation matrix:\n" << rotMat
                     << "\ntranslation vectors:\n" << tvecs[i] 
                     << "\ndvrkBaseRef:\n" << dvrkBaseRef
                     << "\ncam2dvrkBase:\n" << cam2dvrkBase;
            }
        }

        imshow("out", videoFrame);
        if (waitKey(30) == 27)
            break;
    }
}
