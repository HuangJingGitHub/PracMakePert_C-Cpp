#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "visual_processing.h"
#include "path_processing.h"
#include "visual_module/visual_info_service_singlePt.h"

using namespace std;
using namespace cv;

class VisualServiceCore {
private:
    ros::NodeHandle node_handle_;
    ros::ServiceServer visual_info_service = node_handle_.advertiseService("visual_info_service_singlePt", 
                                                            &VisualServiceCore::GetVisualInfoService, this);
    image_transport::ImageTransport image_trans_;
    image_transport::Subscriber image_subscriber_;

    const string kWindowName = "Main Window in service node";
    Mat cur_gray_img_;
    bool path_set_planned_ = false;
    LK_Tracker tracker_;
    ImgExtractor extractor_;
    PathSetTracker path_set_tracker_;
    PathSetTracker ee_path_singleton_tracker_;
    vector<vector<Point2f>> path_set_;
    vector<vector<Point2f>> ee_path_singleton_;
    bool plan_ee_path_request_ = false;
    bool ee_path_singleton_planned_ = false;
    vector<vector<float>> path_local_width_set_;
    vector<Point2f> projection_pts_on_path_set_;
    vector<Point2f> ee_projection_pt_on_path_;
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
        if (!path_set_planned_ && (!initial_feedback_pts.empty()) && extractor_.obs_extract_succeed_) {
            int pivot_idx = 0;
            float feedback_pts_radius = 10;
            target_feedback_pts_ = initial_feedback_pts;
            for (Point2f& pt : target_feedback_pts_)
                pt += Point2f(-180, 100);
            
            path_set_ = GeneratePathSet(initial_feedback_pts, target_feedback_pts_, pivot_idx, feedback_pts_radius,
                                        extractor_.obs_polygons_, cv_ptr->image);
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
            if (ee_path_singleton_planned_)
                ee_projection_pt_on_path_ = ee_path_singleton_tracker_.ProjectPtsToPathSet(tracker_.GetEEPointAsVec());
            for (auto& path : path_set_) {
                for (int i = 0; i < int(path.size() - 1); i++)
                    line(cv_ptr->image, path[i], path[i + 1], Scalar(0, 250, 250), 2);
            }
            arrowedLine(cv_ptr->image, tracker_.points_[0][0], projection_pts_on_path_set_[0],
                        Scalar(0, 0, 255), 2, 8, 0, 0.05);
            circle(cv_ptr->image, path_set_[0].back(), 5, Scalar(0, 255, 0), 2);
        }

        if (plan_ee_path_request_) 
            PlanLocalEEPath(cv_ptr->image);

        if (extractor_.DO_extract_succeed_)
            drawContours(cv_ptr->image, extractor_.DO_contours_, extractor_.largest_DO_countor_idx_, Scalar(250, 0, 150), 1);
        if (extractor_.obs_extract_succeed_) {
            for (int i = 0; i < extractor_.obs_vertices_picked_.size() - 1; i++) 
                line(cv_ptr->image, extractor_.obs_vertices_picked_[i], extractor_.obs_vertices_picked_[i + 1],
                    Scalar(0, 0, 255), 2);
            line(cv_ptr->image, extractor_.obs_vertices_picked_.front(), extractor_.obs_vertices_picked_.back(),
                Scalar(0, 0, 255), 2);
        }
        extractor_.ProjectDOToObstacles();
        arrowedLine(cv_ptr->image, extractor_.DO_to_obs_projections_[0].first, 
                    extractor_.DO_to_obs_projections_[0].second, Scalar(255, 255, 255), 2, 8, 0, 0.05);
        
        imshow(kWindowName, cv_ptr->image);
        waitKey(2);
    }

    void PlanLocalEEPath(Mat& display_image) {
        Point2f projection_to_ee_vec = tracker_.ee_points_[0][0] - projection_pts_on_path_set_[0];
        int projection_index = path_set_tracker_.tracked_indices_log_[0], target_idx = projection_index;
        
        for (; target_idx < path_set_[0].size(); target_idx++) {
            bool obs_free = true;
            for (auto& obs : extractor_.obs_polygons_) {
                if (ObstacleFree(obs, path_set_[0][target_idx], path_set_[0][target_idx] + projection_to_ee_vec))
                    continue;
                else
                    obs_free = false;
            }
            if (obs_free == false)
                break;
        }
        for (; target_idx < path_set_[0].size(); target_idx++) {
            bool obs_free = true;
            for (auto& obs: extractor_.obs_polygons_) {
                if (ObstacleFree(obs, path_set_[0][target_idx], path_set_[0][target_idx] + projection_to_ee_vec) == false) {
                    obs_free = false;
                    break;
                }  
            }
            if (obs_free == true)
                break;
        }

        vector<Point2f> ee_start_set{tracker_.ee_points_[0][0]}, ee_targart_set{path_set_[0][target_idx] + projection_to_ee_vec};
        ee_path_singleton_ = GeneratePathSet(ee_start_set, ee_targart_set, 0, 10,
                                                            extractor_.obs_polygons_, display_image);
        ee_path_singleton_planned_ = !ee_path_singleton_.empty(); 
        if (ee_path_singleton_planned_) {
            ee_path_singleton_tracker_ = PathSetTracker(ee_path_singleton_);
        }        
    }

    bool GetVisualInfoService(  visual_module::visual_info_service_singlePt::Request &request,
                                visual_module::visual_info_service_singlePt::Response &response) {
        if (tracker_.points_[0].empty() || tracker_.ee_points_[0].empty() || 
            extractor_.DO_to_obs_projections_.empty() || projection_pts_on_path_set_.empty())
            return false;
        
        plan_ee_path_request_ = request.need_ee_path;

        response.feedback_pt[0] = tracker_.points_[0][0].x;
        response.feedback_pt[1] = tracker_.points_[0][0].y;
        response.target_pt[0] = target_feedback_pts_[0].x;
        response.target_pt[1] = target_feedback_pts_[0].y;
        response.ee_pt[0] = tracker_.ee_points_[0][0].x;
        response.ee_pt[1] = tracker_.ee_points_[0][0].y;
        response.DO_to_obs_DO_pt[0] = extractor_.DO_to_obs_projections_[0].first.x;
        response.DO_to_obs_DO_pt[1] = extractor_.DO_to_obs_projections_[0].first.y;
        response.DO_to_obs_obs_pt[0] = extractor_.DO_to_obs_projections_[0].second.x;
        response.DO_to_obs_obs_pt[1] = extractor_.DO_to_obs_projections_[0].second.y;
        response.projection_on_path_pt[0] = projection_pts_on_path_set_[0].x;
        response.projection_on_path_pt[1] = projection_pts_on_path_set_[0].y;
        for (int row = 0, cnt = 0; row < 2; row++)
            for (int col = 0; col < 2; col++) {
                response.Jd[cnt] = tracker_.cur_Jd_(row, col);
                cnt++;
            }

        if (ee_path_singleton_planned_ && ee_projection_pt_on_path_.empty() == false) {
            response.projection_on_ee_path_pt[0] = ee_projection_pt_on_path_[0].x;
            response.projection_on_ee_path_pt[1] = ee_projection_pt_on_path_[0].y;
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
            for (const vector<Point2f>& cur_path : path_set_) {
                for (const Point2f& cur_pt : cur_path) 
                    path_data_os << cur_pt.x << " " << cur_pt.y << " ";
                path_data_os << "\n";
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
                path_width_os << "\n";
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