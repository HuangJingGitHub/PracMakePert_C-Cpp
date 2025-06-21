#include <chrono>
#include <ctime>
#include <fstream>
#include "../3d/RRTStar_3d.hpp"
#include "../3d/PRMStar_3d.hpp"

using namespace std::chrono;

void PlanningCostEvolutionTest3d3d(int test_num, int obs_num, int sample_num, vector<PolygonObstacle3d>& obs_vec) {
    Mat back_img(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255));
    int obs_side_len = 60;
    float config_height = 200, interior_depth = 10, rrts_step = 100;
    Point3f start = Point3f(1, 1, 100), target = Point3f(back_img.size().width - 1, back_img.size().height - 1, 200);
    float rrt_step = 50;
    vector<vector<float>> rrts_len_cost_log(test_num), rrts_clearance_cost_log(test_num), rrts_mpw_cost_log(test_num),
                        prms_len_cost_log(test_num), prms_clearance_cost_log(test_num), prms_mpw_cost_log(test_num);

    // vector<int> test_samples{10, 50, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 2000, 3000, 4000, 5000, 
    //                        6000, 7000, 8000, 9000, 10000, 15000, 20000};
    vector<int> test_samples{10, 50, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 2000};    
    for (int test_idx = 0; test_idx < test_num; test_idx++) {
        cout << "RRT* sample number: " << sample_num << " test iteration: " << test_idx << "\n";
  
        RRTStarPlanner3d rrts_len(start, target, obs_vec, rrt_step, back_img.size(), config_height, 0),
                        rrts_clearance(start, target, obs_vec, rrt_step, back_img.size(), config_height, 1),
                        rrts_mpw(start, target, obs_vec, rrt_step, back_img.size(), config_height, 1);
        PRMStarPlanner3d prms_len(start, target, obs_vec, back_img.size(), config_height, 0),
                        prms_clearance(start, target, obs_vec, back_img.size(), config_height, 1),
                        prms_mpw(start, target, obs_vec, back_img.size(), config_height, 2);

        bool planned_len = rrts_len.Plan(sample_num);
        rrts_len_cost_log[test_idx] = rrts_len.min_cost_log_;
        
        bool planned_clearance = rrts_clearance.Plan(sample_num);
        rrts_clearance_cost_log[test_idx] = rrts_clearance.min_cost_log_;
        
        bool planned_mpw = rrts_mpw.Plan(sample_num);
        rrts_mpw_cost_log[test_idx] = rrts_mpw.min_cost_log_;    
        
        /* for (int i = 100; i < sample_num; i+= 100) {
            cout << rrts_len_cost_log[test_idx][i] << ", ";
        }
        cout << "\n";
        for (int i = 100; i < sample_num; i+= 100) {
            cout << rrts_clearance_cost_log[test_idx][i] << ", ";
        }
        cout << "\n";
        for (int i = 100; i < sample_num; i+= 100) {
            cout << rrts_mpw_cost_log[test_idx][i] << ", ";
        }
        cout << "\n"; */

        vector<float> prm_len_cost = prms_len.QueryCostLog(test_samples);
        prms_len_cost_log[test_idx] = prm_len_cost;
        
        vector<float> prm_clearance_cost = prms_clearance.QueryCostLog(test_samples);
        prms_clearance_cost_log[test_idx] = prm_clearance_cost;
        
        vector<float> prm_mpw_cost = prms_mpw.QueryCostLog(test_samples);
        prms_mpw_cost_log[test_idx] = prm_mpw_cost;

        for (float cost : prm_len_cost)
            cout << cost << ", ";
        cout << "\n";
        for (float cost : prm_clearance_cost)
            cout << cost << ", ";
        cout << "\n";        
        for (float cost : prm_mpw_cost)
            cout << cost << ", ";
        cout << "\n";                
    }   
    
    string save_directory = "./src/ptopp/src/data/planning_time_3d/";
    string file_name_postfix, file_name, test_info;
    file_name = "cost_log_3d_obs_" + to_string(obs_num) + "_test_" + to_string(test_num) + "_sample_" + to_string(sample_num / 1000) + "k_";
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
                + "Sample numbers: ";
    for (int sample_num : test_samples)
        test_info += (to_string(sample_num) + ", ");
    test_info += "\n\n";

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
    data_save_os << "4-PRM* length\n";
    for (int test_idx = 0; test_idx < test_num; test_idx++) {
        for (float& cost : prms_len_cost_log[test_idx])
            data_save_os << cost << ", ";
        data_save_os << "\n";
    }
    data_save_os << "5-PRM* max clearance\n";
    for (int test_idx = 0; test_idx < test_num; test_idx++) {
        for (float& cost : prms_clearance_cost_log[test_idx])
            data_save_os << cost << ", ";
        data_save_os << "\n";
    }
    data_save_os << "6-PRM* mpw\n";
    for (int test_idx = 0; test_idx < test_num; test_idx++) {
        for (float& cost : prms_mpw_cost_log[test_idx])
            data_save_os << cost << ", ";
        data_save_os << "\n";
    }      
    data_save_os.close();     
    
}

int main(int argc, char** argv) {
    int test_num = 1;
    int obs_num = 40;
    vector<PolygonObstacle3d> obs_vec = GenerateRandomObstacles3d(obs_num, Size(1000, 600), 400, 60, true);
    int sample_num = 10000;
    // for (int sample_num = 1000; sample_num <= 20000; sample_num += 1000)
        PlanningCostEvolutionTest3d3d(test_num, obs_num, sample_num, obs_vec);
    return 0;
}