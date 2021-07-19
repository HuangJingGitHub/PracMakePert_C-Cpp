#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "visual_processing_angle.h"
#include "feature_geometry.h"
#include "path_processing.h"
#include "visual_module/visual_info_service_angle.h"

using namespace std;
using namespace cv;

class VisualServiceCore {
private:
    ros::NodeHandle node_handle_;
    ros::ServiceServer visual_info_service = node_handle_.advertiseService("visual_info_service_angle", 
                                                                &VisualServiceCore::GetVisualInfoService, this);
    image_transport::ImageTransport image_trans_;
    image_transport::Subscriber image_subscriber_;

    const string kWindowName = "Main Window in service node";
    Mat cur_gray_img_;
    const int feedback_pt_num_ = 3;
    bool path_set_planned_ = false;
    LK_Tracker tracker_;
    ImgExtractor extractor_;
    PathSetTracker path_set_tracker_;
    vector<vector<Point2f>> path_set_;
    vector<vector<float>> path_local_width_set_;
    vector<Point2f> projection_pts_on_path_set_;
    vector<Point2f> target_feedback_pts_;

public:
    VisualServiceCore(string ros_image_stream, 
                    vector<int> DO_HSV_thresholds = vector<int>(6, 0)): image_trans_(node_handle_) {
        tracker_ = LK_Tracker(kWindowName);
        extractor_ = ImgExtractor(1, kWindowName, DO_HSV_thresholds);
        image_subscriber_ = image_trans_.subscribe(ros_image_stream, 30, &VisualServiceCore::ProcessImg, this);
        namedWindow(kWindowName);
    }

    ~VisualServiceCore() {
        destroyWindow(kWindowName);
    }

    void ProcessImg(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& exception_type) {
            ROS_ERROR("cv_bridge exception: %s", exception_type.what());
            return;
        }
        cvtColor(cv_ptr->image, cur_gray_img_, COLOR_BGR2GRAY); 
        tracker_.Track(cv_ptr->image, cur_gray_img_);
        extractor_.Extract(cv_ptr->image);
        
        vector<Point2f> initial_feedback_pts = tracker_.GetFeedbackPoints();
        if (!path_set_planned_ 
            && initial_feedback_pts.size() == feedback_pt_num_ 
            && extractor_.obs_extract_succeed_) {
            int pivot_idx = 0;
            float feedback_pts_radius = 10;
            target_feedback_pts_ = initial_feedback_pts;
            vector<float> initial_angle_pos(6, 0), desired_angle_pos,
                          deisred_s_0_pos{initial_feedback_pts[0].x - 20, initial_feedback_pts[0].y - 100};
            for (int i = 0; i < feedback_pt_num_; i++) {
                initial_angle_pos[2*i] = initial_feedback_pts[i].x;
                initial_angle_pos[2*i + 1] = initial_feedback_pts[i].y;
            }
            Angle3pts angle_obj(initial_angle_pos);
            angle_obj.Update(initial_angle_pos);
            feedback_pts_radius = max(angle_obj.v_1_.norm(), angle_obj.v_2_.norm());
            feedback_pts_radius = min(feedback_pts_radius, float(25));
            desired_angle_pos = angle_obj.DetermineTarget(initial_angle_pos, 120, deisred_s_0_pos, 0, 0.05,
                                                            extractor_.obs_polygons_, true);
            for (int i = 0; i < feedback_pt_num_; i++) {
                target_feedback_pts_[i].x = desired_angle_pos[2*i];
                target_feedback_pts_[i].y = desired_angle_pos[2*i + 1];
            }
            angle_obj.Update(desired_angle_pos);
            pivot_idx = angle_obj.GetPivotIndex();
            cout << "Pivot index in target config: " << pivot_idx << "\n";

            path_set_ = GeneratePathSet(initial_feedback_pts, target_feedback_pts_, pivot_idx, 
                                        feedback_pts_radius, extractor_.obs_polygons_, cv_ptr->image);
            path_set_planned_ = !path_set_.empty();
            if (path_set_planned_) {
                path_set_tracker_ = PathSetTracker(path_set_);
                for (vector<Point2f>& cur_path : path_set_) {
                    vector<float> cur_path_local_width = GetLocalPathWidth2D(cur_path, extractor_.obs_polygons_);
                    path_local_width_set_.push_back(cur_path_local_width);
                }
                SavePathData();
            }
        }
        else if (path_set_planned_) {
            tracker_.UpdateJd();
            projection_pts_on_path_set_ = path_set_tracker_.ProjectPtsToPathSet(tracker_.GetFeedbackPoints());
            extractor_.ProjectDOToObstacles();
            for (auto& path : path_set_) {
                for (int i = 0; i < int(path.size() - 1); i++)
                    line(cv_ptr->image, path[i], path[i + 1], Scalar(0, 250, 250), 2);
            }
            for (int i = 0; i < feedback_pt_num_; i++) {
                arrowedLine(cv_ptr->image, tracker_.points_[0][i], projection_pts_on_path_set_[i],
                            Scalar(0, 0, 255), 2, 8, 0, 0.05);
                circle(cv_ptr->image, path_set_[i].back(), 5, Scalar(0, 255, 0), 2);
            }
            for (int i = 0; i < feedback_pt_num_; i++) {
                circle(cv_ptr->image, target_feedback_pts_[0], 3, Scalar(255, 0, 0), -1);
                circle(cv_ptr->image, tracker_.points_[0][i], 3, Scalar(0, 0, 255), -1);
                if (i != 0) {
                    line(cv_ptr->image, target_feedback_pts_[0], target_feedback_pts_[i], Scalar(255, 0, 0), 2);
                    line(cv_ptr->image, tracker_.points_[0][0], tracker_.points_[0][i], Scalar(255, 0, 0), 2);
                }
            }
        }

        // drawing
        if (extractor_.DO_extract_succeed_)
            drawContours(cv_ptr->image, extractor_.DO_contours_, extractor_.largest_DO_countor_idx_, Scalar(250, 0, 150), 1);
        if (extractor_.obs_extract_succeed_) {
            for (int i = 0; i < extractor_.obs_vertices_picked_.size() - 1; i++) 
                line(cv_ptr->image, extractor_.obs_vertices_picked_[i], extractor_.obs_vertices_picked_[i + 1],
                    Scalar(0, 0, 255), 2);
            line(cv_ptr->image, extractor_.obs_vertices_picked_.front(), extractor_.obs_vertices_picked_.back(),
                Scalar(0, 0, 255), 2);
        }
        arrowedLine(cv_ptr->image, extractor_.DO_to_obs_projections_[0].first, 
                    extractor_.DO_to_obs_projections_[0].second, Scalar(255, 255, 255), 2, 8, 0, 0.05);
        
        imshow(kWindowName, cv_ptr->image);
        waitKey(2);
    }

    bool GetVisualInfoService(  visual_module::visual_info_service_angle::Request &request,
                                visual_module::visual_info_service_angle::Response &response) {
        if (tracker_.points_[0].size() < feedback_pt_num_ || tracker_.ee_points_[0].empty() || 
            extractor_.DO_extract_succeed_ == false || projection_pts_on_path_set_.size() < feedback_pt_num_)
            return false;

        for (int i = 0; i < feedback_pt_num_; i++) {
            response.feedback_pt[2*i] = tracker_.points_[0][i].x;
            response.feedback_pt[2*i + 1] = tracker_.points_[0][i].y;
            response.target_pt[2*i] = target_feedback_pts_[i].x;
            response.target_pt[2*i + 1] = target_feedback_pts_[i].y;
        }
        response.ee_pt[0] = tracker_.ee_points_[0][0].x;
        response.ee_pt[1] = tracker_.ee_points_[0][0].y;
        response.DO_to_obs_DO_pt[0] = extractor_.DO_to_obs_projections_[0].first.x;
        response.DO_to_obs_DO_pt[1] = extractor_.DO_to_obs_projections_[0].first.y;
        response.DO_to_obs_obs_pt[0] = extractor_.DO_to_obs_projections_[0].second.x;
        response.DO_to_obs_obs_pt[1] = extractor_.DO_to_obs_projections_[0].second.y;
        for (int i = 0; i < feedback_pt_num_; i++) {
            response.projection_on_path_pt[2*i] = projection_pts_on_path_set_[i].x;
            response.projection_on_path_pt[2*i + 1] = projection_pts_on_path_set_[i].y;
        }
        for (int row = 0, cnt = 0; row < 6; row++)
            for (int col = 0; col < 2; col++) {
                response.Jd[cnt] = tracker_.cur_Jd_(row, col);
                cnt++;
            }
        return true;
    }

    void SavePathData() {
        string save_directory = "./src/visual_module/src/data/";
        string file_postfix;
        std::time_t cur_time = std::time(0);
        std::tm* cur_tm = std::localtime(&cur_time);
        file_postfix = std::to_string(cur_tm->tm_year + 1900) + "-"
                        + std::to_string(cur_tm->tm_mon + 1) + "-"
                        + std::to_string(cur_tm->tm_mday) + "_"
                        + std::to_string(cur_tm->tm_hour) + ":"
                        + std::to_string(cur_tm->tm_min) + ".txt";        
        string  path_file_name = "planned_path_" + file_postfix, 
                path_widht_file_name = "loacl_path_width_" + file_postfix,
                obs_file_name = "obstacle_vertices_" + file_postfix;
        
        ofstream path_data_os(save_directory + path_file_name, std::ios::trunc);
        if (path_data_os.is_open()) {
            for (const vector<Point2f>& cur_path : path_set_){
                for (const Point2f& cur_pt : cur_path) 
                    path_data_os << cur_pt.x << " " << cur_pt.y << " ";
                path_data_os << "\nNew path\n";
            }
            path_data_os.close();
        }
        else {
            cerr << "Fail to open the path save file.\n";
        }

        ofstream path_width_os(save_directory + path_widht_file_name, std::ios::trunc);
        if (path_width_os.is_open()) {
            for (const vector<float>& cur_path_width : path_local_width_set_) {
                for (const float& cur_width : cur_path_width)
                    path_width_os << cur_width << " ";
                path_width_os << "\nNew local path width\n";
            }
            path_width_os.close();
        }
        else {
            cerr << "Fail to open the path width save file.\n";
        }

        ofstream obs_vertices_os(save_directory + obs_file_name, std::ios::trunc);
        if (obs_vertices_os.is_open()) {
            for (const Point2f& cur_pt : extractor_.obs_vertices_picked_)
                obs_vertices_os << cur_pt.x << " " << cur_pt.y << " ";
            obs_vertices_os.close();
        }
        else {
            cerr << "Fail to open the obstacle save file.\n";
        }
    }
};

int main(int argc, char** argv) {
    vector<int> DO_HSV_thresholds{0, 0, 0, 180, 255, 255};
    ifstream HSV_file("./src/visual_module/src/data/HSV_thresholds.txt");
    if (HSV_file.is_open()) {
        string item_str;
        int cnt = 0, item;
        while (HSV_file >> item_str) {
            item = stoi(item_str);
            DO_HSV_thresholds[cnt] = item;
            cnt++;
        }
        HSV_file.close();
    }
    else {
        cout << "Fail to read DO HSV threshold file. Default value is used.\n";
    }

    string ros_image_stream = "cameras/source_camera/image";
    ros::init(argc, argv, "visual_core");
    VisualServiceCore visual_process_obj(ros_image_stream, DO_HSV_thresholds);
    ros::spin();
    return 0;
}