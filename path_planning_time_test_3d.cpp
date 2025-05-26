#include <chrono>
#include <ctime>
#include <fstream>
#include "../3d/RRTStar_3d.hpp"
#include "../3d/PRMStar_3d.hpp"
#include "../vis/obs_path_vis.hpp"
#include "../decomposition.hpp"

using namespace std::chrono;

void PlanningTimeTest3d(int test_num = 1, int obs_num = 10) {
    Mat back_img(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255));
    int obs_side_len = 60, sample_num = 0;
    float config_height = 400, interior_depth = 10, rrts_step = 50;
    Point3f start = Point3f(1, 1, 100), target = Point3f(back_img.size().width - 1, back_img.size().height - 1, 200);
    vector<float> rrts_len_time(test_num, 0), rrts_interior_time(test_num, 0), rrts_clearance_time(test_num, 0),
                    rrts_gpw_time(test_num, 0), rrts_gpw_all_time(test_num, 0),
                    prms_len_time(test_num, 0), prms_interior_time(test_num, 0), prms_clearance_time(test_num, 0),
                    prms_gpw_time(test_num, 0), prms_gpw_all_time(test_num, 0);  

    for (int test_idx = 0; test_idx < test_num; test_idx++) {
        cout << "\n\nobstacle number: " << obs_num << " test iteration: " << test_idx << "\n";

        vector<PolygonObstacle3d> obs_vec = GenerateRandomObstacles3d(obs_num, back_img.size(), config_height, obs_side_len, true);
        RRTStarPlanner3d rrts_test(start, target, obs_vec, 30, back_img.size(), 400, 0);
        while (rrts_test.cells_.empty()) {
            obs_vec = GenerateRandomObstacles3d(obs_num, back_img.size(), config_height, obs_side_len, true);
            RRTStarPlanner3d rrts_temp(start, target, obs_vec, 30, back_img.size(), 400, 0);
            rrts_test.cells_ = rrts_temp.cells_;
            cout << "Regenerating obstacles\n";
        }

        RRTStarPlanner3d rrts_len(start, target, obs_vec, rrts_step, back_img.size(), config_height, 0),
                        rrts_interior(start, target, obs_vec, rrts_step, back_img.size(), config_height, 0),
                        rrts_clearance(start, target, obs_vec, rrts_step, back_img.size(), config_height, 1),
                        rrts_gpw(start, target, obs_vec, rrts_step, back_img.size(), config_height, 3),
                        rrts_gpw_all(start, target, obs_vec, rrts_step, back_img.size(), config_height, 3, true);
        PRMStarPlanner3d prms_len(start, target, obs_vec, back_img.size(), config_height, 0),
                        prms_interior(start, target, obs_vec, back_img.size(), config_height, 0, false, true, interior_depth),
                        prms_clearance(start, target, obs_vec, back_img.size(), config_height, 1),
                        prms_gpw(start, target, obs_vec, back_img.size(), config_height, 3),
                        prms_gpw_all(start, target, obs_vec, back_img.size(), config_height, 3, true);
        sample_num = rrts_len.MAX_GRAPH_SIZE;

        auto start_time = high_resolution_clock::now();
        bool planned_len = rrts_len.Plan();
        auto end_time = high_resolution_clock::now();
        auto duration_time = duration_cast<milliseconds>(end_time - start_time); 
        rrts_len_time[test_idx] = (float) duration_time.count();

        start_time = high_resolution_clock::now();
        bool planned_interior = rrts_interior.Plan(true, interior_depth);
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        rrts_interior_time[test_idx] = (float) duration_time.count();

        start_time = high_resolution_clock::now();
        bool planned_clearance = rrts_clearance.Plan();
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        rrts_clearance_time[test_idx] = (float)duration_time.count();

        start_time = high_resolution_clock::now();
        bool planned_gpw = rrts_gpw.Plan();
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);   
        rrts_gpw_time[test_idx] = (float)duration_time.count();

        start_time = high_resolution_clock::now();
        bool planned_gpw_all = rrts_gpw_all.Plan();
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time); 
        rrts_gpw_all_time[test_idx] = (float)duration_time.count();
        
        // PRM*
        start_time = high_resolution_clock::now();
        prms_len.QueryPath();
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        prms_len_time[test_idx] = (float) duration_time.count();
        if (!prms_len.plan_success_)
            prms_len_time[test_idx] = 1e6;

        start_time = high_resolution_clock::now();
        prms_interior.QueryPath();
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        prms_interior_time[test_idx] = (float) duration_time.count();
        if (!prms_interior.plan_success_)
            prms_interior_time[test_idx] = 1e6;

        start_time = high_resolution_clock::now();
        prms_clearance.QueryPath();
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        prms_clearance_time[test_idx] = (float)duration_time.count();
        if (!prms_clearance.plan_success_)
            prms_clearance_time[test_idx] = 1e6;

        start_time = high_resolution_clock::now();
        prms_gpw.QueryPath();
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        prms_gpw_time[test_idx] = (float)duration_time.count();   
        if (!prms_gpw.plan_success_)
            prms_gpw_time[test_idx] = 1e6;

        start_time = high_resolution_clock::now();
        prms_gpw_all.QueryPath();
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        prms_gpw_all_time[test_idx] = (float)duration_time.count();
        if (!prms_gpw_all.plan_success_)
            prms_gpw_all_time[test_idx] = 1e6; 
    }

    string save_directory = "./src/ptopp/src/data/planning_time_3d/";
    string file_name_postfix, file_name, test_info;
    file_name = "planning_time_3d_obs_" + to_string(obs_num) + "_test_" + to_string(test_num) + "_";
    std::time_t cur_time = std::time(0);
    std::tm* cur_tm = std::localtime(&cur_time);
    file_name_postfix = to_string(cur_tm->tm_year + 1900) + "-"
                        + to_string(cur_tm->tm_mon + 1) + "-"
                        + to_string(cur_tm->tm_mday) + "-"
                        + to_string(cur_tm->tm_hour) + "-"
                        + to_string(cur_tm->tm_min) + ".txt";
    file_name = file_name + file_name_postfix;
    test_info = "Environment dimension: " + to_string(back_img.size().width) + " x " + to_string(back_img.size().height) +
                " x " + to_string(config_height) + "\n"  
                + "Obstacle number: " + to_string(obs_num) + "\n"
                + "Obstacle main/max side length: " + to_string(obs_side_len) + "\n"
                + "Total test number: " + to_string(test_num) + "\n" 
                + "Continuous varying side length with resolution 1: " + "true" + "\n"
                + "Valid sample number: " + to_string(sample_num) + "\n"
                + "RRT* step: " + to_string(rrts_step) + "\n"
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
    int test_num = 30;
    for (int obs_num = 180; obs_num <= 200; obs_num += 20)
        PlanningTimeTest3d(test_num, obs_num);
    return 0;
}