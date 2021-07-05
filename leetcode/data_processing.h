#ifndef DATA_PROCESSING_INCLUDED
#define DATA_PROCESSING_INCLUDED

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <eigen3/Eigen/Dense>
#include "visual_module/visual_info_service_singlePt.h"

const int total_data_dim = 16;
int total_adjust_time = 0;
Eigen::Matrix3f camera_to_base = Eigen::Matrix3f::Identity();
Eigen::Vector2f feedback_pt, target_pt,
                ee_pt, DO_to_obs_DO_pt, DO_to_obs_obs_pt,
                projection_on_path_pt,
                path_error_pt, total_error_pt,
                ee_velocity_image;
Eigen::Vector3f ee_velocity_image_3D = Eigen::Array3f::Zero(),
                ee_velocity_3D;
Eigen::Matrix2f Jd = Eigen::Matrix2f::Identity();
bool ee_path_planned = false;

std::string save_directory = "./src/visual_module/src/data/";
std::ofstream  data_save_os;
std::vector<float> data_vec;
std::string data_save_order_info = "Format: feedback_pt---target_pt---ee_pt---DO_to_obs_DO_pt"
                                    "---DO_to_obs_obs_pt---projection_on_path_pt---Jd";

void InitializeFiles(const int& motion_interval, 
                    const float& motion_magnitude, 
                    const float& error_threshold) {
    std::string file_name_postfix, file_name, exp_specification_info;
    std::time_t cur_time = std::time(0);
    std::tm* cur_tm = std::localtime(&cur_time);
    file_name_postfix = std::to_string(cur_tm->tm_year + 1900) + "-"
                        + std::to_string(cur_tm->tm_mon + 1) + "-"
                        + std::to_string(cur_tm->tm_mday) + "_"
                        + std::to_string(cur_tm->tm_hour) + ":"
                        + std::to_string(cur_tm->tm_min) + ".txt";
    file_name = "singlePt_data_" + file_name_postfix;

    exp_specification_info = "motion_interval: " + std::to_string(motion_interval) + " ms "
                            + "motion_magnitude: " + std::to_string(motion_magnitude) + " m "
                            + "error_threshold: " + std::to_string(error_threshold) + " px"; 
    
    data_save_os.open(save_directory + file_name, std::ios::trunc);
    if (data_save_os.is_open() == false) {
        std::cerr << "Fail to open the data storage file!\n";
    }
    else {
        data_save_os << exp_specification_info << "\n" << data_save_order_info << "\n";
    }
}

void ProcessServece(const visual_module::visual_info_service_singlePt& srv) {
    data_vec.clear();
    feedback_pt(0, 0) = srv.response.feedback_pt[0];  
    feedback_pt(1, 0) = srv.response.feedback_pt[1]; 
    target_pt(0, 0) = srv.response.target_pt[0]; 
    target_pt(1, 0) = srv.response.target_pt[1]; 
    ee_pt(0, 0) = srv.response.ee_pt[0]; 
    ee_pt(1, 0) = srv.response.ee_pt[1];     
    DO_to_obs_DO_pt(0, 0) = srv.response.DO_to_obs_DO_pt[0]; 
    DO_to_obs_DO_pt(1, 0) = srv.response.DO_to_obs_DO_pt[1]; 
    DO_to_obs_obs_pt(0, 0) = srv.response.DO_to_obs_obs_pt[0]; 
    DO_to_obs_obs_pt(1, 0) = srv.response.DO_to_obs_obs_pt[1]; 
    projection_on_path_pt(0, 0) = srv.response.projection_on_path_pt[0]; 
    projection_on_path_pt(1, 0) = srv.response.projection_on_path_pt[1]; 
    for (int row = 0, cnt = 0; row < 2; row++)
        for (int col = 0; col < 2; col++) {
            Jd(row, col) = srv.response.Jd[cnt];
            cnt++;
        }  
    std::cout << "Jd: \n" << Jd << '\n' 
            << "Jd^-1:\n" << Jd.inverse() << '\n';    

    path_error_pt = feedback_pt - projection_on_path_pt;
    total_error_pt = feedback_pt - target_pt;
    ee_velocity_image = -Jd.inverse() * path_error_pt;
    ee_velocity_image_3D(0, 0) = ee_velocity_image(0, 0);
    ee_velocity_image_3D(1, 0) = ee_velocity_image(1, 0);
    
    data_vec.push_back(feedback_pt(0, 0));
    data_vec.push_back(feedback_pt(1, 0));
    data_vec.push_back(target_pt(0, 0));
    data_vec.push_back(target_pt(1, 0));
    data_vec.push_back(ee_pt(0, 0));
    data_vec.push_back(ee_pt(1, 0));
    data_vec.push_back(DO_to_obs_DO_pt(0, 0));
    data_vec.push_back(DO_to_obs_DO_pt(1, 0));
    data_vec.push_back(DO_to_obs_obs_pt(0, 0));
    data_vec.push_back(DO_to_obs_obs_pt(1, 0));
    data_vec.push_back(projection_on_path_pt(0, 0));
    data_vec.push_back(projection_on_path_pt(1, 0));
    for (int i = 0; i < 4; i++)
        data_vec.push_back(srv.response.Jd[i]);
}

void WriteDataToFile() {
    if (data_vec.size() != total_data_dim)
        std::cout << "No valid data available. Saving data failed.\n";
    if (data_save_os.is_open() == false) 
        std::cerr << "Fail to open the file. Saving data failed.\n";
    
    for (float& data : data_vec)
        data_save_os << data << " ";
    data_save_os << "\n";
}


class VelocityControllerSinglePt {
public:
    static const int constraint_num_ = 4;
    std::vector<std::string> constraint_info_;
    std::vector<float> threshold_vec_;
    std::vector<float> threshold_upper_bound_vec_;
    std::vector<bool> active_state_;
    std::vector<Eigen::Vector2f> unit_gradients_;

    VelocityControllerSinglePt() {
        constraint_info_ = std::vector<std::string>{"End-effector to obstacle constraint",
                                                    "Joint limit constraint",
                                                    "DO to obstacle constraint",
                                                    "DO shape constraint"};
        threshold_vec_ = std::vector<float>{15, FLT_MAX, 15, FLT_MAX};
        threshold_upper_bound_vec_ = std::vector<float>{20, FLT_MAX, 20, FLT_MAX};
        active_state_ = std::vector<bool>(constraint_num_, false);
        unit_gradients_ = std::vector<Eigen::Vector2f>(constraint_num_, Eigen::Vector2f::Zero());
    }

    void DetectViolation(const visual_module::visual_info_service_singlePt& srv) {
        // End-effector to obstacle constraint
        Eigen::Vector2f obs_to_ee_vec;
        obs_to_ee_vec(0, 0) = srv.response.ee_pt[0] - srv.response.DO_to_obs_obs_pt[0];
        obs_to_ee_vec(1, 0) = srv.response.ee_pt[1] - srv.response.DO_to_obs_obs_pt[1];
        if (obs_to_ee_vec.norm() <= threshold_vec_[0])
            active_state_[0] = true;
        else if (obs_to_ee_vec.norm() >= threshold_vec_[0] + 10)
            active_state_[0] = false;
        unit_gradients_[0] = obs_to_ee_vec / obs_to_ee_vec.norm();

        //Joint limit constraint
        active_state_[1] = false;

        // DO to obstacle constraint
        Eigen::Vector2f obs_to_DO_vec;
        obs_to_DO_vec(0, 0) = srv.response.DO_to_obs_DO_pt[0] - srv.response.DO_to_obs_obs_pt[0];
        obs_to_DO_vec(1, 0) = srv.response.DO_to_obs_DO_pt[1] - srv.response.DO_to_obs_obs_pt[1];
        if (obs_to_DO_vec.norm() <= threshold_vec_[2])
            active_state_[2] = true;
        else if (obs_to_DO_vec.norm() >= threshold_vec_[2] + 10)
            active_state_[2] = false;
        unit_gradients_[2] = obs_to_DO_vec / obs_to_DO_vec.norm();
        
        // DO shape constraint
        active_state_[3] = false;
    }

    bool IsConstrained() {
        for (bool state : active_state_)
            if (state == true)
                return true;
        return false;
    }

    Eigen::Vector3f GenerateAdjustmentVelocity() {
        Eigen::Vector3f res = Eigen::Array3f::Zero();
        if (IsConstrained() == false) {
            std::cout << "No constraint violation exists.\n";
            return res;
        }

        std::vector<Eigen::Vector2f> active_unit_gradients; 
        for (int i = 0; i < constraint_num_; i++) 
            if (active_state_[i])
                active_unit_gradients.push_back(unit_gradients_[i]);
        
        int active_constraint_num = active_unit_gradients.size();
        
        Eigen::Vector2f gradient_projection = active_unit_gradients.back();
        for (int i = active_constraint_num - 2; i >= 0; i--) {
            gradient_projection = active_unit_gradients[i] + 
                (gradient_projection - gradient_projection.dot(active_unit_gradients[i]) * active_unit_gradients[i]);
        }
        gradient_projection = gradient_projection / gradient_projection.norm();
        res(0, 0) = gradient_projection(0, 0);
        res(1, 0) = gradient_projection(1, 0);
        res(2, 0) = 0;
        return res;
    }
};

#endif