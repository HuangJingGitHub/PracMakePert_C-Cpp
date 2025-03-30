#include <chrono>
#include <ctime>
#include <fstream>
#include "../3d/RRTStar_3d.hpp"
#include "../decomposition.hpp"

using namespace std::chrono;

void PassageDetectionTest3d(const int test_num = 50, 
                          const int obs_num = 50, 
                          const int side_len = 30,
                          Mat back_img = Mat(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255))) {
    vector<int> psg_num(test_num, 0), psg_num_2d(test_num, 0), cell_num_2d(test_num, 0);
    vector<float> check_time(test_num, 0), check_time_2d(test_num, 0), cell_check_time_2d(test_num, 0);
    float config_height = 400;
    bool varying_side_len = true;

    for (int test_idx = 0; test_idx < test_num; test_idx++) {
        vector<PolygonObstacle3d> obs_vec = GenerateRandomObstacles3d(obs_num, back_img.size(), config_height, side_len, varying_side_len);
        vector<PolygonObstacle> obs_vec_2d = ConvertTo2dObstacles(obs_vec);

        auto start_time = high_resolution_clock::now();
        Passages3d passages = PassageCheckInDelaunayGraph3d(obs_vec, false);
        auto end_time = high_resolution_clock::now();
        auto duration_time = duration_cast<milliseconds>(end_time - start_time);
        check_time[test_idx] = (float)duration_time.count();
        psg_num[test_idx] = passages.pairs.size();
        
        start_time = high_resolution_clock::now();
        Passages passages_2d = PassageCheckInDelaunayGraph(obs_vec_2d, false);
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        check_time_2d[test_idx] = (float)duration_time.count();
        psg_num_2d[test_idx] = passages_2d.pairs.size();

        start_time = high_resolution_clock::now();
        vector<vector<int>> cells = ReportGabrielCells(obs_vec_2d, passages_2d.pairs);
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        cell_check_time_2d[test_idx] = (float)duration_time.count();
        cell_num_2d[test_idx] = cells.size();
    }

    string save_directory = "./src/ptopp/src/data/passage_detection_3d/";
    string file_name_postfix, file_name, test_info;
    file_name = "detection_data_obs_" + to_string(obs_num) + "_side_" + to_string(side_len) + "_";
    std::time_t cur_time = std::time(0);
    std::tm* cur_tm = std::localtime(&cur_time);
    file_name_postfix = to_string(cur_tm->tm_year + 1900) + "-"
                        + to_string(cur_tm->tm_mon + 1) + "-"
                        + to_string(cur_tm->tm_mday) + "_"
                        + to_string(cur_tm->tm_hour) + "-"
                        + to_string(cur_tm->tm_min) + ".txt";
    file_name = file_name + file_name_postfix;
    test_info = "Environment dimension: " + to_string(back_img.size().width) + " x " + to_string(back_img.size().height) 
                + " x " + to_string((int)config_height) + "\n"  
                + "Obstacle number: " + to_string(obs_num) + "\n"
                + "Obstacle main/max side length: " + to_string(side_len) + "\n"
                + "Total test number: " + to_string(test_num) + "\n" 
                + "Continuous varying side length: " + (varying_side_len ? "true" : "false") + "\n"
                + "Detection is built on Delaunay graphs. No environment boundaries are considered\n\n";    

    ofstream  data_save_os;
    data_save_os.open(save_directory + file_name, std::ios::trunc);
    if (data_save_os.is_open() == false) {
        std::cerr << "Fail to open the data storage file!\n";
    }
    else {
        data_save_os << test_info;
        data_save_os << "1 -3D passage number - 2 -Base passage number - 3 -Base cell number\n";
        for (int i = 0; i < test_num; i++)
            data_save_os << psg_num[i] << ", " << psg_num_2d[i] << ", " << cell_num_2d[i] << "\n";
        data_save_os << "1 -3D passage detection time - 2 -Base passage detection time - 3 -Base cell detection time (ms)\n";;  
        for (int i = 0; i < test_num; i++)
            data_save_os << check_time[i] << ", " << check_time_2d[i] << ", " << cell_check_time_2d[i] << "\n";    
        data_save_os.close();                          
    }  
          
}

int main(int argc, char** argv) {
    int obs_num = 200;
    float side_len = 60;
    Mat back_img(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255));
    PassageDetectionTest3d(10, obs_num, side_len, back_img);
    return 0;

    /* Mat back_img(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255));
    int obs_num = 200;
    float config_height = 400, side_len = 60;
    bool varying_side_len = true;

    vector<PolygonObstacle3d> obs_vec = GenerateRandomObstacles3d(obs_num, back_img.size(), config_height, side_len, varying_side_len);
    vector<PolygonObstacle> obs_vec_2d = ConvertTo2dObstacles(obs_vec);

    Passages3d passages = PassageCheckInDelaunayGraph3d(obs_vec);
    Passages passages_2d = PassageCheckInDelaunayGraph(obs_vec_2d, false);
    int cnt = 0;
    for (int i = 0; i < passages.pairs.size(); i++) 
        if (passages.heights[i][0] < 1e-2) 
            cnt++;
        
    // cout << "2d results: \n";
    // for (int i = 0; i < passages_2d.pairs.size(); i++)
    //     cout << passages_2d.pairs[i][0] << "-" << passages_2d.pairs[i][1] << "\n";
    cout << cnt << " vs " << passages_2d.pairs.size() << "\n";
    
    for (int i = 4; i < obs_num + 4; i++) {
        Point2f cur_centroid = GetObstaclesCentroids({obs_vec_2d[i]}).front();
        vector<vector<Point>> input_array_cv(1, vector<Point>(obs_vec_2d[i].vertices.begin(), obs_vec_2d[i].vertices.end()));
        fillPoly(back_img, input_array_cv, Scalar(0.827, 0.827, 0.827));
        int obs_vertex_num = obs_vec_2d[i].vertices.size();
        for (int j = 0; j < obs_vertex_num; j++)
            line(back_img, obs_vec_2d[i].vertices[j], obs_vec_2d[i].vertices[(j + 1) % obs_vertex_num], Scalar(0, 0, 0), 2);   
        putText(back_img, std::to_string(i), cur_centroid - Point2f(10, -5), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);         
    }
    imshow("Obstacles on Base Ground", back_img);
    waitKey(0);
    return 0;    */
}