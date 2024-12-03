#include <chrono>
#include <ctime>
#include "../decomposition.hpp"

using namespace std::chrono;

// Test consistency between extended visibility condition and Gabriel garph-based methods
void ConsistencyTest(const int test_num = 100, const int obs_num = 50, const int side_len = 30, 
                    Mat back_img = Mat(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255))) {
    float EV_check_time = 0, DG_check_time = 0;
    int inconsistency_num = 0;
    for (int test_idx = 0; test_idx < test_num; test_idx++) {
        vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, back_img.size(), side_len);

        auto start_time = high_resolution_clock::now();
        // EV: extended visibility
        auto EV_check_res = ExtendedVisibilityPassageCheck(obs_vec);
        auto end_time = high_resolution_clock::now();
        auto duration_time = duration_cast<milliseconds>(end_time - start_time);
        EV_check_time += (float) duration_time.count();

        start_time = high_resolution_clock::now();
        auto DG_check_res = PassageCheckInDelaunayGraph(obs_vec);        
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        DG_check_time += (float) duration_time.count();

        if (EV_check_res.first.size() != DG_check_res.first.size()) {
            inconsistency_num++;
            cout << "Passage number in extended visibility check vs passage number in Delaunay Graph check\n"
                 << EV_check_res.first.size() << " vs " << DG_check_res.first.size() << "\n";
            set<vector<int>> EV_psg_set(EV_check_res.first.begin(), EV_check_res.first.end()),
                             DG_psg_set(DG_check_res.first.begin(), DG_check_res.first.end());
            vector<vector<int>> EV_exclusive(10), DG_exclusive(10);
            auto extended_itr = set_difference(EV_psg_set.begin(), EV_psg_set.end(), DG_psg_set.begin(), DG_psg_set.end(), EV_exclusive.begin());
            EV_exclusive.resize(extended_itr - EV_exclusive.begin());
            auto DG_itr = set_difference(DG_psg_set.begin(), DG_psg_set.end(), EV_psg_set.begin(), EV_psg_set.end(), DG_exclusive.begin());
            DG_exclusive.resize(DG_itr - DG_exclusive.begin());
            cout << "passages only in extended visibility check condition:\n";
            for (auto psg : EV_exclusive)
                cout << psg[0] << "-" << psg[1] << "\n";
            cout << "\npassages only in DG check: ";
            for (auto psg : DG_exclusive) {
                cout << psg[0] << "-" << psg[1] << "\n";
                for (int i = 0; i < DG_check_res.first.size(); i++)
                    if (DG_check_res.first[i][0] == psg[0] && DG_check_res.first[i][1] == psg[1]) {
                        DrawDshedLine(back_img, DG_check_res.second[i][0], DG_check_res.second[i][1], Scalar(0, 0, 255), 2);
                        circle(back_img, (DG_check_res.second[i][0] + DG_check_res.second[i][1]) / 2, 
                                cv::norm(DG_check_res.second[i][0] - DG_check_res.second[i][1]) / 2, Scalar(0, 0, 255), 2);
                        break;
                    }
            }
            
            Mat DG_img = back_img.clone();
            for (int i = 4; i < obs_num + 4; i++) {
                PolygonObstacle obs = obs_vec[i];
                int cur_vertex_num = obs.vertices.size();
                for (int j = 0; j < cur_vertex_num; j++) {
                    line(back_img, obs.vertices[j], obs.vertices[(j + 1) % cur_vertex_num], Scalar(0, 0, 0), 2);
                    // line(DG_img, obs.vertices[j], obs.vertices[(j + 1) % cur_vertex_num], Scalar(0, 0, 0), 2);
                }
                Point2f obs_centroid = GetObstaclesCentroids({obs}).front();
                putText(back_img, std::to_string(i), obs_centroid - Point2f(10, -5), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
            }  
            
            for (int i = 0; i < DG_check_res.second.size(); i++)
                DrawDshedLine(back_img, DG_check_res.second[i][0], DG_check_res.second[i][1], Scalar(0, 0, 0), 2);

            vector<vector<int>> DG_adj = DelaunayTriangulationObstables(obs_vec);
            for (int i = 0; i < DG_adj.size(); i++) 
                for (int j : DG_adj[i]) 
                    if (i < j)
                        DrawDshedLine(DG_img, GetObstaclesCentroids({obs_vec[i]})[0], GetObstaclesCentroids({obs_vec[j]})[0], Scalar(0, 0, 255), 2);
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

void SVITest(const int obs_num = 50, const int side_len = 30, 
            Mat back_img = Mat(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255))) {
    vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, back_img.size(), side_len);
    auto EV_check_res = ExtendedVisibilityPassageCheck(obs_vec);
    for (int i = 4; i < obs_num + 4; i++) {
        Point2f cur_centroid = GetObstaclesCentroids({obs_vec[i]}).front();
        putText(back_img, std::to_string(i), cur_centroid - Point2f(10, -5), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2); 
        int cur_obs_vertex_num = obs_vec[i].vertices.size();
        for (int j = 0; j < cur_obs_vertex_num; j++)
            line(back_img, obs_vec[i].vertices[j], obs_vec[i].vertices[(j + 1) % cur_obs_vertex_num], Scalar(0, 0, 0), 2);           
    }

    for (int i = 0; i < EV_check_res.first.size(); i++) {
        vector<int> pair = EV_check_res.first[i];
        vector<vector<Point2f>> SVI_key_pts = SVIntersection(obs_vec[pair[0]], obs_vec[pair[1]]);
        DrawDshedLine(back_img, SVI_key_pts[0][0], SVI_key_pts[0][1], Scalar(0, 0, 255), 2);
        DrawDshedLine(back_img, SVI_key_pts[1][0], SVI_key_pts[1][1], Scalar(0, 0, 255), 2);
        DrawDshedLine(back_img, SVI_key_pts[2][0], SVI_key_pts[2][1], Scalar(0, 0, 0), 2);
        /* line(back_img, SVI_key_pts[3][0], SVI_key_pts[3][1], Scalar(0, 0, 0), 2);
        line(back_img, SVI_key_pts[4][0], SVI_key_pts[4][1], Scalar(0, 0, 0), 2); */
    }
    imshow("Shadow Volume for Passage Representation", back_img);
    waitKey(0); 
}

int main(int argc, char** argv) {
    Mat back_img(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255));
    ConsistencyTest(10, 100, 30, back_img);
    // SVITest(50, 50, back_img);
    return 0;
}