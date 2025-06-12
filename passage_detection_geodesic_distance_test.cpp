#include <chrono>
#include <ctime>
#include <fstream>
#include "../decomposition.hpp"

using namespace std::chrono;

void PassageDetectionTest(const int test_num = 50, 
                          const int obs_num = 50, 
                          const int side_len = 30,
                          Mat back_img = Mat(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255))) {
    vector<float> EV_check_time(test_num, 0), DG_check_time(test_num, 0),
                  EV_cell_check_time(test_num, 0);
    vector<int> EV_psg_num(test_num, 0), DG_psg_num(test_num, 0), DG_psg_gd_one_num(test_num, 0);
    bool varying_side_len = true;
    // bool varying_side_len_test = false; // For fixed obstacle nubmer, varying side lengths.

    vector<float> psg_lengths;
    for (int test_idx = 0; test_idx < test_num; test_idx++) {
        vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, back_img.size(), side_len, varying_side_len);

        auto start_time = high_resolution_clock::now();
        // EV: extended visibility = Gabriel condition, check all obstacle pairs.
        Passages EV_check_res = ExtendedVisibilityPassageCheck(obs_vec, false, false);
        auto end_time = high_resolution_clock::now();
        auto duration_time = duration_cast<milliseconds>(end_time - start_time);
        EV_check_time[test_idx] = (float) duration_time.count();
        EV_psg_num[test_idx] = EV_check_res.pairs.size();
  

        // Using Delaunary graph
        start_time = high_resolution_clock::now();
        Passages DG_check_res = PassageCheckInDelaunayGraph(obs_vec, false);        
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        DG_check_time[test_idx] = (float) duration_time.count();
        DG_psg_num[test_idx] = DG_check_res.pairs.size();

        start_time = high_resolution_clock::now();
        Passages DG_check_gd_one_res = PassageCheckInDelaunayGraphWithGeodesicDistance(obs_vec, 1, false);        
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        DG_psg_gd_one_num[test_idx] = DG_check_gd_one_res.pairs.size();

        if (DG_check_res.pairs.size() != EV_check_res.pairs.size()) {
            set<vector<int>> psg_pair_set(DG_check_res.pairs.begin(), DG_check_res.pairs.end());
            cout << "Passages not found in DG: \n";
            for (vector<int>& psg_pair : EV_check_res.pairs) {
                if (psg_pair_set.count(psg_pair) == 0) {
                    cout << psg_pair[0] << "-" << psg_pair[1] << "\n";
                }
            }
        }

        /*psg_lengths = vector<float>(DG_check_res.pts.size(), 0);
        for (int i = 0; i < psg_lengths.size(); i++)
            psg_lengths[i] = cv::norm(DG_check_res.pts[i][0] - DG_check_res.pts[i][1]);
        std::sort(psg_lengths.begin(), psg_lengths.end());
        for (float& len : psg_lengths)
            cout << len << ", ";
        cout << "\n";*/
    }
    return;

    string save_directory = "./src/ptopp/src/data/passage_detection_2d/";
    string file_name_postfix, file_name, test_info;
    file_name = "detection_data_obs_" + to_string(obs_num) + "_side_" + to_string(side_len) + "_geodesic_distance_";
    std::time_t cur_time = std::time(0);
    std::tm* cur_tm = std::localtime(&cur_time);
    file_name_postfix = to_string(cur_tm->tm_year + 1900) + "-"
                        + to_string(cur_tm->tm_mon + 1) + "-"
                        + to_string(cur_tm->tm_mday) + "_"
                        + to_string(cur_tm->tm_hour) + "-"
                        + to_string(cur_tm->tm_min) + ".txt";
    file_name = file_name + (varying_side_len ? "varying_side_len_" : "") + file_name_postfix;
    test_info = "Environment dimension: " + to_string(back_img.size().width) + " x " + to_string(back_img.size().height) + "\n"  
                + "Obstacle num: " + to_string(obs_num) + "\n"
                + "Obstacle side length range: 20 - " + to_string(side_len) + "\n"
                + "Total test number: " + to_string(test_num) + "\n" 
                + "Continuous varying side length with resolution 1: " + (varying_side_len ? "true" : "false") + "\n"
                + "No environment boundaries are considered\n\n";    

    ofstream  data_save_os;
    data_save_os.open(save_directory + file_name, std::ios::trunc);
    if (data_save_os.is_open() == false) {
        std::cerr << "Fail to open the data storage file!\n";
    }
    else {
        data_save_os << test_info;
        data_save_os << "1-Passage number using Gabriel condition in brute-force traversal, " 
                         "2-Passage number using Delaunay graph (geodesic distance one), "
                         "3-Passage number using Delaunay graph (geodesic distance two)\n";
        for (int i = 0; i < test_num; i++)
            data_save_os << EV_psg_num[i] << ", " << DG_psg_gd_one_num[i] << ", " << DG_psg_num[i] << "\n";   
        data_save_os.close();                          
    }           
}

int main(int argc, char** argv) {
    Mat back_img(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255));
    for (int obs_num = 20; obs_num <= 200; obs_num += 20)
       PassageDetectionTest(30, obs_num, 60, back_img);
    // for (int side_len = 10; side_len <= 100; side_len += 10)
    //    PassageDetectionTest(50, 40, side_len, back_img);
    return 0;
}