#ifndef VISUAL_PERCEPTION_INCLUDED
#define VISUAL_PERCEPTION_INCLUDED

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace cv;

Point2f feedback_pt_picked;
Point2f feedback_target_pt_picked;
Point2f ee_point_picked;
bool add_remove_pt = false;
bool add_remove_target_pt = false;
bool add_remove_ee_pt = false;
bool clear_all_feedback_pt = false;
bool save_image_flag = false;

static void onMouse(int event, int x, int y, int, void*) {
    if (event == EVENT_LBUTTONDOWN) {
        feedback_pt_picked = Point2f((float)x, (float)y);
        add_remove_pt = true;
    }
    if (event == EVENT_RBUTTONDOWN) {
        feedback_target_pt_picked = Point2f((float)x, (float)y);
        add_remove_target_pt = true;
    }
    if (event == EVENT_MBUTTONDOWN) {
        ee_point_picked = Point2f((float)x, (float)y);
        add_remove_ee_pt = true;
    }
    if (event == EVENT_LBUTTONDBLCLK) {
        clear_all_feedback_pt = true;
    }
    if (event == EVENT_RBUTTONDBLCLK) {
        save_image_flag = true;
    }
}


class LK_Tracker {
public: 
    string window_to_track_;
    TermCriteria termiantion_criteria_;
    static const int points_num_ = 1;  // Determination of Jd size needs a constexpr argument.
    vector<Point2f> points_[2];
    vector<Point2f> target_pts_[2];
    vector<Point2f> ee_point_[2];
    Scalar points_color_;
    Scalar ee_point_color_;
    Mat pre_gray_img_;
    Mat next_gray_img_;

    Eigen::MatrixXf pre_Jd_;
    Eigen::MatrixXf cur_Jd_;
    bool Jd_initialized_ = false;
    float Jd_update_rate_ = 0.1;


    LK_Tracker() {}
    LK_Tracker(const string win_name) {
        window_to_track_ = win_name;
        termiantion_criteria_ = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 0.03);
        points_color_ = Scalar(255, 0, 0);
        ee_point_color_ = Scalar(0, 255, 0);
    }


    void Track(Mat& image, Mat& input_gray_img) {
        next_gray_img_ = input_gray_img;
        setMouseCallback(window_to_track_, onMouse, 0);
        if (add_remove_pt && points_[0].size() < (size_t) points_num_) {
            vector<Point2f> temp;
            temp.push_back(feedback_pt_picked);
            cornerSubPix(next_gray_img_, temp, Size(3, 3), Size(-1, -1), termiantion_criteria_);
            for (auto& pt : points_[0])
                if (cv::norm(temp[0] - pt) < 5)
                    add_remove_pt = false;

            if (add_remove_pt) {
                points_[0].push_back(temp[0]);
                add_remove_pt = false;
            }
        }

        if (add_remove_target_pt && target_pts_[0].size() < points_[0].size()) {
            target_pts_[0].push_back(feedback_target_pt_picked);
            add_remove_target_pt = false;
        }

        if (add_remove_ee_pt && ee_point_[0].size() < 1) {
            vector<Point2f> temp;
            temp.push_back(ee_point_picked);
            cornerSubPix(next_gray_img_, temp, Size(3, 3), Size(-1, -1), termiantion_criteria_);
            ee_point_[0].push_back(temp[0]);
            add_remove_ee_pt = false;
        }

        if (clear_all_feedback_pt) {
            clear_all_feedback_pt = false;
            points_[0].clear();
            points_[1].clear();
            target_pts_[0].clear();
            target_pts_[1].clear();
        }

        if (!points_[0].empty()) {
            InvokeLK(points_[0], points_[1], image, points_color_); 
        }
        if (!ee_point_[0].empty()) {
            InvokeLK(ee_point_[0], ee_point_[1], image, ee_point_color_);
        }
        next_gray_img_.copyTo(pre_gray_img_);
        UpdateJd();        
        for (size_t i = 0; i < points_[0].size(); i++)
            circle(image, points_[0][i], 3, points_color_, -1, 8);
    } 


    void InvokeLK(vector<Point2f>& pre_pts, vector<Point2f>& next_pts, Mat& display_img, Scalar pt_color) {
        vector<uchar> status;
        vector<float> error;
        if (pre_gray_img_.empty())
            next_gray_img_.copyTo(pre_gray_img_);

        calcOpticalFlowPyrLK(pre_gray_img_, next_gray_img_, pre_pts, next_pts, status, error, Size(21, 21),
                            3, termiantion_criteria_, 0, 0.001);
        size_t i, k;
        for (i = k = 0; i < next_pts.size(); i++) {
            if (!status[i])
                continue;
            next_pts[k++] = next_pts[i];
            circle(display_img, next_pts[i], 3, pt_color, -1, 8);
        }
        // next_pts.resize(k);   
        if (next_pts.size() < pre_pts.size())
            next_pts = pre_pts;
        std::swap(pre_pts, next_pts);     
    }


    vector<Point2f> GetFeedbackPoints() {
        if (points_[0].size() != points_num_) {
            cout << "Less feedback points than expected.\n";
            return {};
        }
        return points_[0];
    }   

    bool ValidFeedbackAndTargetPts() {
        return points_[0].size() == target_pts_[0].size() && points_[0].size() > 0;
    }

    void UpdateJd() {
        if (!(points_[0].size() == points_num_ && points_[1].size() == points_num_)
            || !(ee_point_[0].size() == 1 && ee_point_[1].size() == 1)
            || Jd_initialized_ == false) {
            cout << "Unsuccessful tracking. No update of deformation Jacobian is performaed.\n";

            if (Jd_initialized_ == false) {
                pre_Jd_ = Eigen::Matrix<float, points_num_ * 2, 2>::Zero();
                for (int i = 0; i < points_num_; i++) 
                    pre_Jd_.block<2, 2>(2*i, 0) = Eigen::Matrix2f::Identity();
                
                Jd_initialized_ = true;
            }
            return;
        }

        Eigen::Matrix<float, points_num_ * 2, 1> delta_points;
        Eigen::Matrix<float, 2, 1> delta_ee;
        // Note poits_[0] is current value, points_[1] stores previous value due to swap() in LK algorithm.
        for (int i = 0; i < points_num_; i++) {
            delta_points(2 * i, 0) = points_[0][i].x - points_[1][i].x;
            delta_points(2 * i + 1, 0) = points_[0][i].y - points_[1][i].y; 
        }
        delta_ee(0, 0) = ee_point_[0][0].x - ee_point_[1][0].x;
        delta_ee(1, 0) = ee_point_[0][0].y - ee_point_[1][0].y;

        if (delta_ee.norm() < 0.5) {
            cur_Jd_ = pre_Jd_;
            return;
        }
        
        cur_Jd_ = pre_Jd_ + Jd_update_rate_ * (delta_points - pre_Jd_*delta_ee) / delta_ee.squaredNorm() * delta_ee.transpose();
        pre_Jd_ = cur_Jd_;
    }
};


class ImgExtractor {
public:
    Mat HSV_img_;
    Scalar DO_HSV_low_;
    Scalar DO_HSV_high_;
    vector<vector<Point>> DO_contours_;
    vector<Point> DO_contour_;
    int largest_DO_countor_idx_ = 0;
    int obs_extracted_times_ = 0;
    bool DO_extract_succeed_ = false;
    string window_to_track_;
    

    ImgExtractor() {}
    ImgExtractor(const string win_name, 
                vector<int> DO_HSV_thresholds = vector<int>(6, 0)) {
        window_to_track_ = win_name;
        DO_HSV_low_ = Scalar(DO_HSV_thresholds[0], DO_HSV_thresholds[1], DO_HSV_thresholds[2]);
        DO_HSV_high_ = Scalar(DO_HSV_thresholds[3], DO_HSV_thresholds[4], DO_HSV_thresholds[5]);
    }


    void Extract(Mat& input_image) {
        // setMouseCallback(window_to_track_, onMouse, 0);
        if (input_image.empty()) {
            cout << "Invalid image for extracting!\n";
            return;
        }

        cvtColor(input_image, HSV_img_, COLOR_BGR2HSV);
        GaussianBlur(HSV_img_, HSV_img_, Size(3, 3), 5, 5);

        Mat destination_img;
        inRange(HSV_img_, DO_HSV_low_, DO_HSV_high_, destination_img);
        Moments m_dst = moments(destination_img, true);
        if (m_dst.m00 < 500) {
            cout << "No DO detected!\n";
            DO_extract_succeed_ = false;
        }
        else {
            //findContours(destination_img, DO_contours_, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
            findContours(destination_img, DO_contours_, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));
            largest_DO_countor_idx_ = 0;
            for (int i = 1; i < DO_contours_.size(); i++) {
                largest_DO_countor_idx_ = (DO_contours_[i].size() > DO_contours_[largest_DO_countor_idx_].size())
                                            ? i : largest_DO_countor_idx_;
            }
            DO_contour_ = DO_contours_[largest_DO_countor_idx_];
            DO_extract_succeed_ = true;
        }
    }
};


class GraspPositionSelector {
    double gamma_ = 0.25;
    double k_ = 0.01;
    double r_ = 10;
    double delta_r_ = 10;
    double delta_ = 0.1;
    double alpha_0_ = M_PI / 6;
    double alpha_1_ = M_PI / 6 * 5;
    size_t integration_step_num_ = 100;

public:
    GraspPositionSelector() {};

    Point2f SelectSingleGraspPosition(vector<Point>& DO_contour, vector<double>& Q_value,
                                      vector<Point2f>& S_0, vector<Point2f>& S_d) {
        if (DO_contour.empty() || S_0.empty() || S_d.empty()) {
            cerr << "Empty vector is inputed in Function " << __FUNCTION__ << "\n";
            return Point2f(0, 0);
        }
        if (S_0.size() != S_d.size()) {
            cerr << "Different sizes of feedback points and target points in Function " << __FUNCTION__ << "\n";
            return Point2f(0, 0);
        }

        size_t contour_pt_num = DO_contour.size(), pt_num = S_0.size(); 
        Q_value = vector<double>(contour_pt_num, 0);

        for (size_t contour_idx = 0; contour_idx < contour_pt_num; contour_idx++) {
            double Q = 1;
            Eigen::Vector2d s_g(DO_contour[contour_idx].x, DO_contour[contour_idx].y);
            
            for (size_t pt_idx = 0; pt_idx < pt_num; pt_idx++) {
                Eigen::Vector2d s_0_i(S_0[pt_idx].x, S_0[pt_idx].y),
                                s_d_i(S_d[pt_idx].x, S_d[pt_idx].y),
                                e_i = s_d_i - s_0_i,
                                d_i = s_0_i - s_g;
                double e_i_norm = e_i.norm(),
                       d_i_norm = d_i.norm(), 
                       alpha_i = acos(e_i.dot(d_i) / (e_i_norm * d_i_norm)),
                       xi = 1;
                if (alpha_i <= alpha_0_ || alpha_i >= alpha_1_)
                    xi = 1 + delta_;
                Eigen::Vector2d s_d_g = s_g + xi * e_i;
                double step_len = e_i_norm / integration_step_num_;
                
                double q_i = 0;
                for (size_t step_i = 1; step_i < integration_step_num_; step_i++) {
                    Eigen::Vector2d s_g_cur = s_g + (double)step_i / integration_step_num_ * xi * e_i,
                                    s_0_cur = s_0_i + (double)step_i / integration_step_num_ * e_i;
                    d_i = s_0_cur - s_g_cur;
                    d_i_norm = d_i.norm();
                    alpha_i = acos(e_i.dot(d_i) / (e_i_norm * d_i_norm));
                    double lambda = 1;
                    if (d_i_norm <= r_)
                        lambda = 1 / (1 - gamma_ * sin(alpha_i));
                    else if (d_i_norm > r_ && d_i_norm <= r_ + delta_r_)
                        lambda = 1 / (1 - gamma_ * sin(alpha_i)) * (1 - (d_i_norm - r_) / delta_r_ 
                                * gamma_ * sin(alpha_i));
                    double M =  exp(-k_ * d_i_norm) * (1 - gamma_ * sin(alpha_i)) * lambda;
                    q_i += M * step_len;
                }
                Q *= q_i;
            }
            Q_value[contour_idx] = Q;
        }

        size_t res_idx = 0;
        double Q_max = DBL_MIN;
        for (size_t i = 0; i < Q_value.size(); i++) {
            if (Q_value[i] > Q_max) {
                res_idx = i;
                Q_max = Q_value[i];
            }
        }
        cout << "Q_max: " << Q_max << '\n';
        for (double& Q_i : Q_value)
            Q_i /= Q_max;
        return DO_contour[res_idx];
    }

    
    vector<Point2f> SelectDualGraspPositions(vector<Point>& DO_contour, vector<double>& Q_value,
                                             vector<Point2f>& S_0, vector<Point2f>& S_d) {
        if (DO_contour.empty() || S_0.empty() || S_d.empty()) {
            cerr << "Empty vector is inputed in Function " << __FUNCTION__ << "\n";
            return {};
        }
        if (S_0.size() != S_d.size()) {
            cerr << "Different sizes of feedback points and target points in Function " << __FUNCTION__ << "\n";
            return {};
        }
        if (S_0.size() < 2) {
            cerr << "Size of feedback points and target points in Function " << __FUNCTION__ << " is "
                 << S_0.size() << "\n";
            return {};
        }

        vector<double> S_displacement(S_0.size(), 0);
        for (size_t i = 0; i < S_0.size(); i++)
            S_displacement[i] = cv::norm(S_d[i] - S_0[i]);
        size_t max_dis_idx = max_element(S_displacement.begin(), S_displacement.end()) - S_displacement.begin();
        Point2f s_k_1 = S_0[max_dis_idx];

        vector<double> distance_to_s_k_1(S_0.size(), 0);
        for (size_t i = 0; i < S_0.size(); i++) {
            distance_to_s_k_1[i] = cv::norm(S_0[i] - s_k_1);
        }
        double max_distance_to_s_k_1 = *max_element(distance_to_s_k_1.begin(), distance_to_s_k_1.end());

        vector<double> evaluation_J_k(S_0.size(), 0);
        Eigen::Vector2d e_k_1(S_d[max_dis_idx].x - s_k_1.x, S_d[max_dis_idx].y - s_k_1.y);
        double e_k_1_norm = e_k_1.norm();
        for (size_t i = 0; i < S_0.size(); i++) {
            Eigen::Vector2d e_i(S_d[i].x - S_0[i].x, S_d[i].y - S_0[i].y);
            double z_i = 0, cos_val = e_i.dot(e_k_1) / (e_i.norm() * e_k_1_norm);
            if (cos_val <= -1.0)
                z_i = -M_PI;
            else if (cos_val >= 1.0)
                z_i = 0;
            else
                z_i = acos(cos_val);
            double lambda_k_i = 1 / (1 + (exp(-z_i))) * distance_to_s_k_1[i] / max_distance_to_s_k_1;
            evaluation_J_k[i] = lambda_k_i * S_displacement[i];
        }
        size_t max_J_k_idx = max_element(evaluation_J_k.begin(), evaluation_J_k.end()) - evaluation_J_k.begin();
        Point2f s_k_2 = S_0[max_J_k_idx];

        vector<Point2f> s_k_1_set{S_0[max_dis_idx]}, s_k_1_target_set{S_d[max_dis_idx]},
                        s_k_2_set{S_0[max_J_k_idx]}, s_k_2_target_set{S_d[max_J_k_idx]};
        Point2f s_g_1 = SelectSingleGraspPosition(DO_contour, Q_value, s_k_1_set, s_k_1_target_set),
                s_g_2 = SelectSingleGraspPosition(DO_contour, Q_value, s_k_2_set, s_k_2_target_set);
        return vector<Point2f>{s_g_1, s_g_2};
    }


    vector<Point2f> SelectDualGraspPositionsByCluster(vector<Point>& DO_contour, vector<double>& Q_value,
                                                    vector<Point2f>& S_0, vector<Point2f>& S_d) {
        if (S_0.size() < 2) {
            cerr << "Feedback vector size is less than 2 in " << __FUNCTION__ << '\n';
            return {};
        }

        vector<vector<int>> index_cluster = twoMeansClustering(S_0);                                        
        vector<Point2f> pt_cluster_1, pt_cluster_2, target_pt_cluster_1, target_pt_cluster_2;
        for (size_t i = 0; i < index_cluster[0].size(); i++) {
            int idx = index_cluster[0][i];
            pt_cluster_1.push_back(S_0[idx]);
            target_pt_cluster_1.push_back(S_d[idx]);
        }
        for (size_t i = 0; i < index_cluster[1].size(); i++) {
            int idx = index_cluster[1][i];
            pt_cluster_2.push_back(S_0[idx]);
            target_pt_cluster_2.push_back(S_d[idx]);
        }

        Point2f s_g_1 = SelectSingleGraspPosition(DO_contour, Q_value, pt_cluster_1, target_pt_cluster_1),
                s_g_2 = SelectSingleGraspPosition(DO_contour, Q_value, pt_cluster_2, target_pt_cluster_2);
        return vector<Point2f>{s_g_1, s_g_2};
    }


    vector<vector<int>> twoMeansClustering(vector<Point2f>& S_0) {
        if (S_0.size() < 2) {
            cerr << "Input size error in clustering!\n";
            return {};
        }
        
        vector<vector<int>> res(2, vector<int>());
        
        Point2f centroid_1 = S_0[0], centroid_2 = S_0[1], 
                previous_centroid_1 = Point2f(0, 0), previous_centroid_2 = Point2f(0, 0);
        double threshold = 1e-1;
        while (cv::norm(centroid_1 - previous_centroid_1) > threshold 
            || cv::norm(centroid_2 - previous_centroid_2) > threshold) {
                vector<int> cluster_indices_1, cluster_indices_2;            
                
                for (size_t i = 0; i < S_0.size(); i++) {
                    if (cv::norm(S_0[i] - centroid_1) < cv::norm(S_0[i] - centroid_2))
                        cluster_indices_1.push_back(i);
                    else
                        cluster_indices_2.push_back(i);
                }

                previous_centroid_1 = centroid_1;
                previous_centroid_2 = centroid_2;
                centroid_1.x = 0;
                centroid_1.y = 0;
                centroid_2.x = 0;
                centroid_2.y = 0;
                for (size_t i = 0; i <  cluster_indices_1.size(); i++)
                    centroid_1 += S_0[cluster_indices_1[i]];
                centroid_1 /= (double)(cluster_indices_1.size());
                for (size_t i = 0; i < cluster_indices_2.size(); i++)
                    centroid_2 += S_0[cluster_indices_2[i]];
                centroid_2 /= (double)(cluster_indices_2.size());

                res[0] = cluster_indices_1;
                res[1] = cluster_indices_2;
        }
        return res;
    }
};


#endif