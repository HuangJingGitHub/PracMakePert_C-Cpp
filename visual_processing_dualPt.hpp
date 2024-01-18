#ifndef VISUAL_PROCESSING_DualPt_INCLUDED
#define VISUAL_PROCESSING_DualPt_INCLUDED

#include <string>
#include <vector>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <eigen3/Eigen/Dense>
#include "obstacles.hpp"

using namespace std;
using namespace cv;

Point2f feedback_pt_picked;
Point2f feedback_target_pt_picked;
Point2f ee_pt_picked;
Point2f obs_vertex_picked;
bool add_remove_pt = false;
bool add_remove_target_pt = false;
bool ee_add_remove_pt = false;  // ee: end-effector
bool obs_add_remove_pt = false;

static void onMouse(int event, int x, int y, int, void*) {
    if (event == EVENT_LBUTTONDOWN) {
        feedback_pt_picked = Point2f((float)x, (float)y);
        add_remove_pt = true;
    }
    if (event == EVENT_LBUTTONDBLCLK) {
        ee_pt_picked = Point2f((float)x, (float)y);
        ee_add_remove_pt = true;
    }
    if (event == EVENT_RBUTTONDOWN) {
        obs_vertex_picked = Point2f((float)x, (float)y);
        obs_add_remove_pt = true;
    }
    if (event == EVENT_RBUTTONDBLCLK) {
        feedback_target_pt_picked = Point2f((float)x, (float)y);
        add_remove_target_pt = true;
    }
}


class LK_Tracker {
public: 
    string window_to_track_;
    TermCriteria termiantion_criteria_;
    static const int points_num_ = 5;  // Determination of Jd size needs a constexpr argument.
    vector<Point2f> points_[2];
    vector<Point2f> target_points_;
    vector<Point2f> ee_points_[2];
    Scalar points_color_;
    Scalar ee_points_color_;
    Mat pre_gray_img_;
    Mat next_gray_img_;

    Eigen::MatrixXf pre_Jd_;
    Eigen::MatrixXf cur_Jd_;
    float update_rate_;
    bool Jd_initialized_ = false;

    LK_Tracker() {}
    LK_Tracker(const string win_name) {
        window_to_track_ = win_name;
        termiantion_criteria_ = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 0.03);
        points_color_ = Scalar(255, 0, 0);
        ee_points_color_ = Scalar(0, 255, 0);

        update_rate_ = 0.5;
        pre_Jd_ = Eigen::Matrix<float, points_num_ * 2, 4>::Identity();
    }

    void Track(Mat& image, Mat& input_gray_img) {
        next_gray_img_ = input_gray_img;
        setMouseCallback(window_to_track_, onMouse, 0);
        if (add_remove_pt && points_[0].size() < (size_t) points_num_) {
            vector<Point2f> temp;
            temp.push_back(feedback_pt_picked);
            cornerSubPix(next_gray_img_, temp, Size(5, 5), Size(-1, -1), termiantion_criteria_);
            for (auto& pt : points_[0])
                if (cv::norm(temp[0] - pt) < 2)
                    add_remove_pt = false;
            if (add_remove_pt) {
                points_[0].push_back(temp[0]);
                add_remove_pt = false;
            }
        }
        if (ee_add_remove_pt && ee_points_[0].size() < 2) {
            vector<Point2f> temp;
            temp.push_back(ee_pt_picked);
            cornerSubPix(next_gray_img_, temp, Size(5, 5), Size(-1, -1), termiantion_criteria_);
            ee_points_[0].push_back(temp[0]);
            ee_add_remove_pt = false;
        }
        if (add_remove_target_pt && target_points_.size() < points_num_) {
            target_points_.push_back(feedback_target_pt_picked);
            add_remove_target_pt = false;
        }

/*         if (!points_[0].empty())
            InvokeLK(points_[0], points_[1], image, points_color_);      
        if (!ee_points_[0].empty())
            InvokeLK(ee_points_[0], ee_points_[1], image, ee_points_color_);

        next_gray_img_.copyTo(pre_gray_img_); */
    } 

    void InvokeLK(vector<Point2f>& pre_pts, vector<Point2f>& next_pts, Mat& display_img, Scalar& pt_color) {
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
    
    void UpdateJd() {
        if (!(points_[0].size() == points_num_ && points_[1].size() == points_num_)
            || !(ee_points_[0].size() == 2 && ee_points_[1].size() == 2)) {
            cout << "Unsuccessful tracking. No update of deformation Jacobian performaed.\n";
            pre_Jd_ = Eigen::Matrix<float, points_num_ * 2, 4>::Identity();
            return;
        }
        Eigen::Matrix<float, points_num_ * 2, 1> delta_points;
        Eigen::Matrix<float, 4, 1> delta_ee;
        // Note poits_[0] is current value, points_[1] stores previous value due to swap() in LK algorithm.
        for (int i = 0; i < points_num_; i++) {
            delta_points(2 * i, 0) = points_[0][i].x - points_[1][i].x;
            delta_points(2 * i + 1, 0) = points_[0][i].y - points_[1][i].y; 
        }
        delta_ee(0, 0) = ee_points_[0][0].x - ee_points_[1][0].x;
        delta_ee(1, 0) = ee_points_[0][0].y - ee_points_[1][0].y;
        delta_ee(2, 0) = ee_points_[0][1].x - ee_points_[1][1].x;
        delta_ee(3, 0) = ee_points_[0][1].y - ee_points_[1][1].y;        

        if (delta_ee.norm() < 1) {
            cur_Jd_ = pre_Jd_;
            return;
        }

        std::cout << "Check the current JD dismension: " << cur_Jd_.rows() << " " << cur_Jd_.cols();
        cur_Jd_ = pre_Jd_ + update_rate_ * (delta_points - pre_Jd_*delta_ee) / delta_ee.squaredNorm() * delta_ee.transpose();
        std::cout << "Check the updated JD dismension: " << cur_Jd_.rows() << " " << cur_Jd_.cols();
        pre_Jd_ = cur_Jd_;
    }

    vector<Point2f> GetFeedbackPoints() {
        if (points_[0].size() != points_num_) {
            cout << "Less feedback points than expected.\n";
            return {};
        }
        return points_[0];
    }

    vector<Point2f> GetEEPointAsVec() {
        if (ee_points_[0].size() != 1) {
            cout << "Fail to track the end-effector.\n";
            return {};
        }
        return ee_points_[0];
    }    
};


class ImgExtractor {
public:
    Mat HSV_img_;
    Scalar DO_HSV_low_;
    Scalar DO_HSV_high_;
    Scalar obs_HSV_low_;
    Scalar obs_HSV_high_;
    vector<vector<Point>> DO_contours_;
    vector<Point> DO_contour_;
    vector<PolygonObstacle> obs_polygons_;
    vector<pair<Point2f, Point2f>> DO_to_obs_projections_;
    int largest_DO_countor_idx_ = 0;
    int obs_num_preset_;
    int obs_extracted_times_ = 0;
    bool DO_extract_succeed_ = false;
    bool obs_extract_succeed_ = false;
    string window_to_track_;
    int obs_vertices_num_preset_ = 4;
    vector<Point2f> obs_vertices_picked_;
    

    ImgExtractor() {}
    ImgExtractor(int obs_num, const string win_name, 
                vector<int> DO_HSV_thresholds = vector<int>(6, 0)) {
        obs_num_preset_ = obs_num;
        window_to_track_ = win_name;
        DO_HSV_low_ = Scalar(DO_HSV_thresholds[0], DO_HSV_thresholds[1], DO_HSV_thresholds[2]);
        DO_HSV_high_ = Scalar(DO_HSV_thresholds[3], DO_HSV_thresholds[4], DO_HSV_thresholds[5]);
        obs_HSV_low_ = Scalar(0, 0, 0);
        obs_HSV_high_ = Scalar(255, 255, 255);
        DO_to_obs_projections_ = vector<pair<Point2f, Point2f>>(obs_num_preset_);
    }

    void Extract(Mat& input_image, int occlusion = 0) {
        // setMouseCallback(window_to_track_, onMouse, 0);
        cout << "Current obstacle vertices number: " << obs_vertices_picked_.size() << "\n"
             << "Obstacle number: " << obs_polygons_.size() << '\n';
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
            findContours(destination_img, DO_contours_, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
            largest_DO_countor_idx_ = 0;
            for (int i = 1; i < DO_contours_.size(); i++) {
                largest_DO_countor_idx_ = (DO_contours_[i].size() > DO_contours_[largest_DO_countor_idx_].size())
                                            ? i : largest_DO_countor_idx_;
            }
            DO_contour_ = DO_contours_[largest_DO_countor_idx_];
            DO_extract_succeed_ = true;
        }

        if (occlusion != 0) {
            int second_largetst_countor_idx = (largest_DO_countor_idx_ == 0) ? 1 : 0;
            for (int i = 0; i < DO_contours_.size(); i++){
                second_largetst_countor_idx = (DO_contours_[i].size() > DO_contours_[second_largetst_countor_idx].size() && i != largest_DO_countor_idx_)
                                                ? i : second_largetst_countor_idx;
            }
            DO_contour_.insert(DO_contour_.end(), DO_contours_[second_largetst_countor_idx].begin(), DO_contours_[second_largetst_countor_idx].end());
            DO_contours_[largest_DO_countor_idx_] = DO_contour_; 
        }

        if (obs_add_remove_pt && obs_vertices_picked_.size() < obs_vertices_num_preset_ * obs_num_preset_) {
            obs_vertices_picked_.push_back(obs_vertex_picked);
            obs_add_remove_pt = false;
            circle(input_image, obs_vertex_picked, 3, Scalar(0, 0, 255), -1, 8);
        }
        else if (obs_extract_succeed_ == false && obs_vertices_picked_.size() == obs_vertices_num_preset_ * obs_num_preset_) {
            vector<Point2f> vertices_for_single_obs;
            for (int i = 0; i < obs_vertices_picked_.size(); i++) {
                vertices_for_single_obs.push_back(obs_vertices_picked_[i]);
                if (i > 0 && (i + 1) % obs_vertices_num_preset_ == 0) {
                    obs_polygons_.push_back(PolygonObstacle(vertices_for_single_obs));
                    vertices_for_single_obs.clear();
                }
            }
            if (obs_polygons_.size() == obs_num_preset_)
                obs_extract_succeed_ = true;
        }
    }

    void ProjectDOToObstacles() {
        if (DO_contour_.empty() || obs_polygons_.empty()) {
            if (DO_contour_.empty())
                cout << "No valid DO contour available.\n";
            else
                cout << "No obstacle detected.\n";
            return;
        }
        for (int i = 0; i < obs_num_preset_; i++) {
            vector<pair<float, int>> distance_log;
            int step = (DO_contour_.size() <= 25) ? 1 : DO_contour_.size() / 25;
            Point2f cur_pt;
            float cur_distance, min_distance = FLT_MAX;
            for (int idx = 0; idx < DO_contour_.size(); idx += step) {
                cur_pt = Point2f(DO_contour_[idx].x, DO_contour_[idx].y);
                float cur_distance = MinDistanceToObstacle(obs_polygons_[i], cur_pt);
                distance_log.push_back(pair<float, int>(cur_distance, idx));
            }
            
            std::sort(distance_log.begin(), distance_log.end());
            for (int cnt = 0; cnt < distance_log.size() && cnt < 3; cnt++) {
                int range_left = distance_log[cnt].second - step + 1,
                    range_right = distance_log[cnt].second + step - 1;
                if (range_left < 0)
                    range_left = 0;
                if (range_right >= DO_contour_.size())
                    range_right = DO_contour_.size() - 1;

                for ( ; range_left <= range_right; range_left++) {
                    cur_pt = Point2f(DO_contour_[range_left].x, DO_contour_[range_left].y);
                    cur_distance = MinDistanceToObstacle(obs_polygons_[i], cur_pt);
                    if (cur_distance < min_distance) {
                        min_distance = cur_distance;
                        DO_to_obs_projections_[i].first = Point2f(DO_contour_[range_left].x, DO_contour_[range_left].y);
                        DO_to_obs_projections_[i].second = obs_polygons_[i].min_distance_pt;
                    }
                }
            }
        }
    }
};


class PathSetTracker {
public:
    vector<vector<Point2f>> path_set_;
    vector<vector<float>> path_accumulated_lengths_;
    int path_num_;
    int forward_shift_;
    vector<int> min_distance_indices_;
    vector<int> tracked_indices_log_;

    PathSetTracker() {}
    PathSetTracker(vector<vector<Point2f>>& input_path_set) {
            path_set_ = input_path_set;
            path_num_ = input_path_set.size();
            min_distance_indices_ = vector<int>(path_num_, 0);
            tracked_indices_log_ = vector<int>(path_num_, 0);
    }

    vector<Point2f> ProjectPtsToPathSet(vector<Point2f> cur_points) {
        vector<Point2f> res(path_num_, Point2f(0, 0));
        if (cur_points.size() != path_num_) {
            cout << "Dimensions of feedback points and path set do not match. Path set size: "
                 << path_set_.size() 
                 << "\nfeedback points size: "
                 << cur_points.size() << "\n";
            for (int& idx : tracked_indices_log_)
                idx = 0;
            return res;
        }

        for (int i = 0; i < path_num_; i++) {
            forward_shift_ = path_set_[i].size() / 10;
            float cur_distance_sqr, min_distance_sqr = FLT_MAX;
            int start_idx = 0;  //tracked_indices_log_[i];
            for (; start_idx < path_set_[i].size(); start_idx++) {
                cur_distance_sqr = normSqr(cur_points[i] - path_set_[i][start_idx]);
                if (cur_distance_sqr < min_distance_sqr) {
                    min_distance_sqr = cur_distance_sqr;
                    min_distance_indices_[i] = start_idx;
                }
            } 
            if (min_distance_indices_[i] + forward_shift_ >= path_set_[i].size())
                res[i] = path_set_[i].back();
            else
                res[i] = path_set_[i][min_distance_indices_[i] + forward_shift_];
            tracked_indices_log_[i] = min_distance_indices_[i];
        }
        return res;
    }

};
#endif