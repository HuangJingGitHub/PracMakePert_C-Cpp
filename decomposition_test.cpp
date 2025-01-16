#include <chrono>
#include <ctime>
#include "../decomposition.hpp"

using namespace std::chrono;

void SVITest(const int obs_num = 50, const int side_len = 30, 
            Mat back_img = Mat(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255)), bool varying_side_len = false) {
    vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, back_img.size(), side_len, varying_side_len);
    auto EV_check_res = ExtendedVisibilityPassageCheck(obs_vec);
    for (int i = 4; i < obs_num + 4; i++) {
        Point2f cur_centroid = GetObstaclesCentroids({obs_vec[i]}).front();
        vector<vector<Point>> input_array_cv(1, vector<Point>(obs_vec[i].vertices.begin(), obs_vec[i].vertices.end()));
        fillPoly(back_img, input_array_cv, Scalar(0.827, 0.827, 0.827));
        int cur_obs_vertex_num = obs_vec[i].vertices.size();
        for (int j = 0; j < cur_obs_vertex_num; j++)
            line(back_img, obs_vec[i].vertices[j], obs_vec[i].vertices[(j + 1) % cur_obs_vertex_num], Scalar(0, 0, 0), 2);   
        // putText(back_img, std::to_string(i), cur_centroid - Point2f(10, -5), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);         
    }

    for (int i = 0; i < EV_check_res.first.size(); i++) {
        vector<int> pair = EV_check_res.first[i];
        vector<vector<Point2f>> SVI_key_pts = SVIntersection(obs_vec[pair[0]], obs_vec[pair[1]]);
        /* cout << SVI_key_pts[0][0] << " " << SVI_key_pts[0][1] << "\n";
        cout << SVI_key_pts[1][0] << " " << SVI_key_pts[1][1] << "\n";
        cout << SVI_key_pts[2][0] << " " << SVI_key_pts[2][1] << "\n"; */
        DrawDshedLine(back_img, SVI_key_pts[0][0], SVI_key_pts[0][1], Scalar(0.2, 0.67, 1), 2);
        DrawDshedLine(back_img, SVI_key_pts[1][0], SVI_key_pts[1][1], Scalar(0.2, 0.67, 1), 2);
        DrawDshedLine(back_img, SVI_key_pts[2][0], SVI_key_pts[2][1], Scalar(0, 0, 0), 2);
    }
    
    string save_directory = "./src/ptopp/src/img/";
    string file_name_postfix, file_name_prefix, file_name;
    file_name_prefix = "Obs_" + to_string(obs_num) + "_Side_" + to_string(side_len) + (varying_side_len ? "_varying_len_" : "_");
    std::time_t cur_time = std::time(0);
    std::tm* cur_tm = std::localtime(&cur_time);
    file_name_postfix = to_string(cur_tm->tm_year + 1900) + "-"
                        + to_string(cur_tm->tm_mon + 1) + "-"
                        + to_string(cur_tm->tm_mday) + "_"
                        + to_string(cur_tm->tm_hour) + "-"
                        + to_string(cur_tm->tm_min) + ".png";
    file_name = file_name_prefix + file_name_postfix;

    imshow("Shadow Volume for Passage Representation", back_img);
    if (true)
        imwrite(save_directory + file_name, 255 * back_img);
    waitKey(0); 
}

/// Test consistency between extended visibility condition and Delaunay garph-based methods in passage detection.
/// Environment walls are processed using traversal check.
void EVDGConsistencyTest(const int test_num = 100, const int obs_num = 50, const int side_len = 30, 
                    Mat back_img = Mat(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255))) {
    float EV_check_time = 0, DG_check_time = 0;
    int inconsistency_num = 0;
    for (int test_idx = 0; test_idx < test_num; test_idx++) {
        vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, back_img.size(), side_len);
        vector<Point2f> obs_centroids = GetObstaclesCentroids(obs_vec);

        auto start_time = high_resolution_clock::now();
        // EV: extended visibility
        auto EV_check_res = ExtendedVisibilityPassageCheck(obs_vec);
        auto end_time = high_resolution_clock::now();
        auto duration_time = duration_cast<milliseconds>(end_time - start_time);
        EV_check_time += (float) duration_time.count();

        start_time = high_resolution_clock::now();
        auto DG_check_res = PassageCheckDelaunayGraphWithWalls(obs_vec);        
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        DG_check_time += (float) duration_time.count();

        if (EV_check_res.first.size() != DG_check_res.first.size()) {
            inconsistency_num++;
            cout << "Passage number in extended visibility check vs passage number in Delaunay Graph check:\n"
                 << EV_check_res.first.size() << " vs " << DG_check_res.first.size() << "\n";
            set<vector<int>> EV_psg_set(EV_check_res.first.begin(), EV_check_res.first.end()),
                             DG_psg_set(DG_check_res.first.begin(), DG_check_res.first.end());
            vector<vector<int>> EV_exclusive(50), DG_exclusive(50);
            auto extended_itr = set_difference(EV_psg_set.begin(), EV_psg_set.end(), 
                                               DG_psg_set.begin(), DG_psg_set.end(), EV_exclusive.begin());
            EV_exclusive.resize(extended_itr - EV_exclusive.begin());
            auto DG_itr = set_difference(DG_psg_set.begin(), DG_psg_set.end(), EV_psg_set.begin(), 
                                         EV_psg_set.end(), DG_exclusive.begin());
            DG_exclusive.resize(DG_itr - DG_exclusive.begin());
            
            cout << "passages only in extended visibility check condition:\n";
            for (auto psg : EV_exclusive)
                cout << psg[0] << "-" << psg[1] << "\n";
            
            cout << "\npassages only in DG check:\n";
            for (auto psg : DG_exclusive) {
                cout << psg[0] << "-" << psg[1] << "\n";
                for (int i = 0; i < DG_check_res.first.size(); i++)
                    if (DG_check_res.first[i][0] == psg[0] && DG_check_res.first[i][1] == psg[1]) {
                        cout << DG_check_res.second[i][0] << ", " << DG_check_res.second[i][1] << "\n";
                        DrawDshedLine(back_img, DG_check_res.second[i][0], DG_check_res.second[i][1], Scalar(0, 0, 255), 2);
                        circle(back_img, (DG_check_res.second[i][0] + DG_check_res.second[i][1]) / 2, 
                                cv::norm(DG_check_res.second[i][0] - DG_check_res.second[i][1]) / 2, Scalar(0, 0, 255), 2);
                        break;
                    }
            }
            
            Mat DG_img = back_img.clone();
            DrawObstacles(back_img, obs_vec);
            
            for (int i = 0; i < DG_check_res.second.size(); i++)
                DrawDshedLine(back_img, DG_check_res.second[i][0], DG_check_res.second[i][1], Scalar(0, 0, 0), 2);

            vector<vector<int>> DG_adj = DelaunayTriangulationObstables(obs_vec);
            for (int i = 0; i < DG_adj.size(); i++) 
                for (int j : DG_adj[i]) 
                    if (i < j)
                        DrawDshedLine(DG_img, obs_centroids[i], obs_centroids[j], Scalar(0, 0, 255), 2);
            imshow("Delaunay Trigulation", DG_img);
            imshow("Gabriel Cells", back_img);
            waitKey(0);                                       
        }
    }

    cout << "TEST RESULT:"
         << "\nobstacle number: " << obs_num
         << "\nobstacle side primary length: " << side_len
         << "\nIn " << test_num << " tests, " << inconsistency_num << " inconsistent test(s) happen."
         << "\nAverage check time using extended visibility condition: " << EV_check_time / test_num << " ms"
         << "\nAverage check time based on Delaunay graph: " << DG_check_time / test_num << " ms\n";
}

void GabrielCellTest(const int test_num = 100, const int obs_num = 50, const int side_len = 30, 
                    Mat back_img = Mat(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255))) {
    vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, back_img.size(), side_len);
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obs_vec);

    auto DG_check_res = PassageCheckDelaunayGraphWithWalls(obs_vec);        
    vector<vector<int>> cells = ReportGabrielCells(obs_vec, DG_check_res.first);
    auto cell_info = GetGabrielCellsInfo(cells, DG_check_res);
    for (int i = 0; i < cell_info.size(); i++) {
        vector<int> cell_obs = cell_info[i].obs_indices;
        int obs_cnt = 0;
        Point2f cell_centroid(0, 0);
        for (int obs_idx : cell_obs) {
            cout << obs_idx << "-";
            if (obs_idx > 3) {
                cell_centroid += obs_centroids[obs_idx];
                obs_cnt += 1;
            }
        }
        cout << "\n";
        cell_centroid /= (float)obs_cnt;
        putText(back_img, std::to_string(i), cell_centroid, cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);

        for (Point2f pt : cell_info[i].vertices) 
                cout << pt << "-";
        cout << "\n";
    }

    Point2f test_pt_1(1, 1), test_pt_2(980, 580);
    cout << "Point "<< test_pt_1 << " is in cell: " << LocatePtInCells(test_pt_1, cell_info) << "\n";
    cout << "Point "<< test_pt_2 << " is in cell: " << LocatePtInCells(test_pt_2, cell_info) << "\n";

    DrawObstacles(back_img, obs_vec);
    for (int i = 0; i < DG_check_res.second.size(); i++)
        DrawDshedLine(back_img, DG_check_res.second[i][0], DG_check_res.second[i][1], Scalar(0, 0, 0), 2);
    imshow("Gabriel Cells", back_img);
    waitKey(0);                                        
}

int main(int argc, char** argv) {
    Mat back_img(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255));
    // EVDGConsistencyTest(10, 30, 30, back_img);
    SVITest(20, 80, back_img, true);
    // GabrielCellTest(10, 30, 30, back_img);
    return 0;
}