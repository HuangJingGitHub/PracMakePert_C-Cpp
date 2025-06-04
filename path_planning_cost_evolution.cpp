#include <chrono>
#include <ctime>
#include <fstream>
#include "../RRTStar.hpp"
#include "../PRMStar.hpp"
#include "path_processing.hpp"
#include <opencv2/viz/types.hpp>

using namespace std::chrono;

void PlanningTimeTest(int test_num, int obs_num , int sample_num, vector<PolygonObstacle>& obs_vec) {
    Mat back_img(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255));
    int obs_side_len = 60;
    bool varying_side_len = true;
    Point2f start = Point2f(1, 1), end = Point2f(back_img.size().width - 1, back_img.size().height - 1);
    float rrt_step = 50, interior_depth = 10;
    vector<vector<float>> rrts_len_cost_log(test_num), rrts_clearance_cost_log(test_num), rrts_mpw_cost_log(test_num);
 
    for (int test_idx = 0; test_idx < test_num; test_idx++) {
        cout << "sample number: " << sample_num << " test iteration: " << test_idx << "\n";
  
        RRTStarPlanner rrts_len(start, end, obs_vec, rrt_step, back_img.size(), 0),
                    rrts_clearance(start, end, obs_vec, rrt_step, back_img.size(), 1),
                    rrts_mpw(start, end, obs_vec, rrt_step, back_img.size(), 2);

        bool planned_len = rrts_len.Plan(back_img, sample_num);
        rrts_len_cost_log[test_idx] = rrts_len.min_cost_log_;
        
        bool planned_clearance = rrts_clearance.Plan(back_img, sample_num);
        rrts_clearance_cost_log[test_idx] = rrts_clearance.min_cost_log_;
        
        bool planned_mpw = rrts_mpw.Plan(back_img, sample_num);
        rrts_mpw_cost_log[test_idx] = rrts_mpw.min_cost_log_;    
    } 
    
    string save_directory = "./src/ptopp/src/data/planning_time_2d/";
    string file_name_postfix, file_name, test_info;
    file_name = "cost_log_2d_obs_" + to_string(obs_num) + "_test_" + to_string(test_num) + "_sample_" + to_string(sample_num / 1000) + "k_";
    std::time_t cur_time = std::time(0);
    std::tm* cur_tm = std::localtime(&cur_time);
    file_name_postfix = to_string(cur_tm->tm_year + 1900) + "-"
                        + to_string(cur_tm->tm_mon + 1) + "-"
                        + to_string(cur_tm->tm_mday) + "-"
                        + to_string(cur_tm->tm_hour) + "-"
                        + to_string(cur_tm->tm_min) + ".txt";
    file_name = file_name + file_name_postfix;
    test_info = "Environment dimension: " + to_string(back_img.size().width) + " x " + to_string(back_img.size().height) + "\n"  
                + "Obstacle number: " + to_string(obs_num) + "\n"
                + "Obstacle main/max side length: " + to_string(obs_side_len) + "\n"
                + "Total test number: " + to_string(test_num) + "\n" 
                + "Continuous varying side length with resolution 1: " + (varying_side_len ? "true" : "false") + "\n"
                + "Valid sample number: " + to_string(sample_num) + "\n"
                + "RRT* step: " + to_string(rrt_step) + "\n"
                + "Planning interior depth: " + to_string(interior_depth) + "\n\n";

    ofstream  data_save_os;
    data_save_os.open(save_directory + file_name, std::ios::trunc);
    if (data_save_os.is_open() == false) {
        std::cerr << "Fail to open the data storage file!\n";
    }                  
    data_save_os << test_info;
    data_save_os << "1-RRT* length\n";
    for (int test_idx = 0; test_idx < test_num; test_idx++) {
        for (float& cost : rrts_len_cost_log[test_idx])
            data_save_os << cost << ", ";
        data_save_os << "\n";
    }
    data_save_os << "2-RRT* max clearance\n";
    for (int test_idx = 0; test_idx < test_num; test_idx++) {
        for (float& cost : rrts_clearance_cost_log[test_idx])
            data_save_os << cost << ", ";
        data_save_os << "\n";
    }
    data_save_os << "3-RRT* mpw\n";
    for (int test_idx = 0; test_idx < test_num; test_idx++) {
        for (float& cost : rrts_mpw_cost_log[test_idx])
            data_save_os << cost << ", ";
        data_save_os << "\n";
    }     
    data_save_os.close();    
}

int main(int argc, char** argv) {
    int test_num = 5;
    int obs_num = 40;
    vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, Size(1000, 600), 60, true); 
    PlanningTimeTest(test_num, obs_num, 20000, obs_vec);
}