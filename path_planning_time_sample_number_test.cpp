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
    vector<float> rrts_len_time(test_num, 0), rrts_interior_time(test_num, 0), rrts_clearance_time(test_num, 0),
                  rrts_gpw_time(test_num, 0), rrts_gpw_all_time(test_num, 0),
                  prms_len_time(test_num, 0), prms_interior_time(test_num, 0), prms_clearance_time(test_num, 0),
                  prms_gpw_time(test_num, 0), prms_gpw_all_time(test_num, 0);  
 
    for (int test_idx = 0; test_idx < test_num; test_idx++) {
        cout << "sample number: " << sample_num << " test iteration: " << test_idx << "\n";
  
        RRTStarPlanner rrts_len(start, end, obs_vec, rrt_step, back_img.size(), 0),
                    rrts_interior(start, end, obs_vec, rrt_step, back_img.size(), 0),
                    rrts_clearance(start, end, obs_vec, rrt_step, back_img.size(), 1),
                    rrts_gpw(start, end, obs_vec, rrt_step, back_img.size(), 3),
                    rrts_gpw_all(start, end, obs_vec, rrt_step, back_img.size(), 3, true);
        PRMStarPlanner prms_len(start, end, obs_vec, back_img.size(), 0, false, false, 1, sample_num),
                    prms_interior(start, end, obs_vec, back_img.size(), 0, false, true, interior_depth, sample_num),
                    prms_clearance(start, end, obs_vec, back_img.size(), 1, false, false, 1, sample_num),
                    prms_gpw(start, end, obs_vec, back_img.size(), 3, false, false, 1, sample_num),
                    prms_gpw_all(start, end, obs_vec, back_img.size(), 3, true, false, 1, sample_num);

        auto start_time = high_resolution_clock::now();
        bool planned_len = rrts_len.Plan(back_img, sample_num);
        auto end_time = high_resolution_clock::now();
        auto duration_time = duration_cast<milliseconds>(end_time - start_time); 
        rrts_len_time[test_idx] = (float) duration_time.count();
        
        start_time = high_resolution_clock::now();
        bool planned_interior = rrts_interior.Plan(back_img, sample_num, interior_depth);
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        rrts_interior_time[test_idx] = (float) duration_time.count();
        
        start_time = high_resolution_clock::now();
        bool planned_clearance = rrts_clearance.Plan(back_img, sample_num);
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        rrts_clearance_time[test_idx] = (float)duration_time.count();
        
        start_time = high_resolution_clock::now();
        bool planned_gpw = rrts_gpw.Plan(back_img, sample_num);
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);   
        rrts_gpw_time[test_idx] = (float)duration_time.count();
        
        start_time = high_resolution_clock::now();
        bool planned_gpw_all = rrts_gpw_all.Plan(back_img, sample_num);
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time); 
        rrts_gpw_all_time[test_idx] = (float)duration_time.count();
    
        // data_save_os << "RRT* " << rrts_len_time[test_idx] << ", " << rrts_interior_time[test_idx] << ", " << rrts_clearance_time[test_idx] << ", "
        // << rrts_gpw_time[test_idx] << ", " << rrts_gpw_all_time[test_idx] << "\n";
        
        // PRM*
        start_time = high_resolution_clock::now();
        prms_len.QueryPath(back_img);
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        prms_len_time[test_idx] = (float) duration_time.count();
        
        start_time = high_resolution_clock::now();
        prms_interior.QueryPath(back_img);
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        prms_interior_time[test_idx] = (float) duration_time.count();

        start_time = high_resolution_clock::now();
        prms_clearance.QueryPath(back_img);
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        prms_clearance_time[test_idx] = (float)duration_time.count();

        start_time = high_resolution_clock::now();
        prms_gpw.QueryPath(back_img);
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        prms_gpw_time[test_idx] = (float)duration_time.count();   

        start_time = high_resolution_clock::now();
        prms_gpw_all.QueryPath(back_img);
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        prms_gpw_all_time[test_idx] = (float)duration_time.count();
        
        // data_save_os << "PRM* " << prms_len_time[test_idx] << ", " << prms_interior_time[test_idx] << ", " << prms_clearance_time[test_idx] << ", " 
        // << prms_gpw_time[test_idx] << ", " << prms_gpw_all_time[test_idx] << "\n";    
        // data_save_os.close();  
    } 
    
    string save_directory = "./src/ptopp/src/data/planning_time_2d/";
    string file_name_postfix, file_name, test_info;
    file_name = "planning_time_2d_obs_" + to_string(obs_num) + "_test_" + to_string(test_num) + "_sample_" + to_string(sample_num / 1000) + "k_";
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
    data_save_os << "1-RRT* length, 2-RRT* interior, 3-RRT* clearance, 4-RRT* gpw, 5-RRT* gpw all (check all passages)\n";
    for (int test_idx = 0; test_idx < test_num; test_idx++)
        data_save_os << rrts_len_time[test_idx] << ", " << rrts_interior_time[test_idx] << ", " << rrts_clearance_time[test_idx] << ", "
        << rrts_gpw_time[test_idx] << ", " << rrts_gpw_all_time[test_idx] << "\n";

    data_save_os << "1-PRM* length, 2-PRM* interior, 3-PRM* clearance, 4-PRM* gpw, 5-PRM* gpw all (check all passages)\n"; 
    for (int test_idx = 0; test_idx < test_num; test_idx++)
        data_save_os << prms_len_time[test_idx] << ", " << prms_interior_time[test_idx] << ", " << prms_clearance_time[test_idx] << ", " 
                    << prms_gpw_time[test_idx] << ", " << prms_gpw_all_time[test_idx] << "\n";  
    data_save_os.close();    
}

int main(int argc, char** argv) {
    int test_num = 5;
    int obs_num = 40;
    vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, Size(1000, 600), 60, true); 
    for (int sample_num = 1000; sample_num <= 20000; sample_num += 1000) {
        PlanningTimeTest(test_num, obs_num, sample_num, obs_vec);
    }
}