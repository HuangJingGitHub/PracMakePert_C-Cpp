#include <iostream>
#include <vector>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include "RRTStar_DOM_debug.h"

using namespace cv;
using namespace std;
using namespace Eigen;


vector<Point2f> QuadraticBSplineSmoothing(vector<RRTStarNode*> path) {
    vector<Point2f> res;
    if (path.size() < 3) {
        cout << "No sufficient control points on the input path!\n"
             << "(At least 3 control points are needed for quadratic B-Spline smoothing.)\n";
        return res;
    }

    const float step_len = 0.01;   // For block() mathod needs constexpr as argument.
    const int step_num = 1 / step_len + 1,
              pts_num = path.size();

    Matrix3f coeff_mat_1, coeff_mat_2, coeff_mat_3;
    MatrixXf parameter_var(3, step_num), 
             control_pts(2, pts_num);
    Matrix<float, 2, Dynamic> smoothed_path;

    coeff_mat_1 << 1, -2, 1, -1.5, 2, 0, 0.5, 0, 0;
    coeff_mat_2 << 0.5, -1, 0.5, -1, 1, 0.5, 0.5, 0, 0;
    coeff_mat_3 << 0.5, -1, 0.5, -1.5, 1, 0.5, 1, 0, 0;

    
    for (int i = 0; i < step_len; i++) {
        float cur_var = step_len * i;
        parameter_var(0, i) = cur_var * cur_var;
        parameter_var(1, i) = cur_var;
        parameter_var(2, i) = 1;
    }
    for (int i = 0; i < pts_num; i++) {
        control_pts(0, i) = path[i]->pos.x;
        control_pts(1, i) = path[i]->pos.y;
    }

    MatrixXf cur_pts = control_pts.block<2, 3>(0, 0);
    MatrixXf cur_spline = cur_pts * coeff_mat_1 * parameter_var;
    smoothed_path = cur_spline;

    for (int i = 1; i < pts_num - 2; i++) {
        cur_pts = control_pts.block<2, 3>(0, i);
        cur_spline = cur_pts * coeff_mat_2 * parameter_var;
        smoothed_path.conservativeResize(2, smoothed_path.cols() + cur_spline.cols());
        smoothed_path.block<2, step_num>(0, smoothed_path.cols() - step_num) = cur_spline;
    }
    cur_pts = control_pts.block<2, 3>(0, pts_num - 3);
    cur_spline = cur_pts * coeff_mat_3 * parameter_var;
    smoothed_path.conservativeResize(2, smoothed_path.cols() + cur_spline.cols());
    smoothed_path.block<2, step_num>(0, smoothed_path.cols() - step_num) = cur_spline;

    cout << "path smoothing:\n"
        << path.size() << ' '
        << smoothed_path.rows() << " x " << smoothed_path.cols() << '\n'
        << cur_spline.rows() << " x " << cur_spline.cols() << endl;
    cout << smoothed_path;
    for (int i = 0; i < smoothed_path.cols(); i++)
        res.push_back(Point2f(smoothed_path(0, i), smoothed_path(1, i)));
    return res;
}