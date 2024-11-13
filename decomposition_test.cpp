#include <chrono>
#include <ctime>
#include "decomposition.hpp"
#include <opencv2/viz/types.hpp>

using namespace std::chrono;

void DrawDshedLine(Mat img, const Point2f& initial_pt, const Point2f& end_pt, Scalar color = Scalar(0, 0, 0), int thickness = 2, float dash_len = 5) {
    float line_len = cv::norm(end_pt - initial_pt);
    Point2f line_direction = (end_pt -initial_pt) / line_len;
    int dash_num = line_len / dash_len;
    if (line_len < 2 * dash_len) 
        line(img, initial_pt, initial_pt + dash_len * line_direction, color, thickness);
    for (int i = 0; i <= dash_num; i += 2) 
        if (i == dash_num)
            line(img, initial_pt + i * dash_len * line_direction, end_pt, color, thickness);
        else
            line(img, initial_pt + i * dash_len * line_direction, initial_pt + (i + 1) * dash_len * line_direction, color, thickness);
}

// Test consistency between extended visibility condition and Gabriel garph-based methods
void ConsistencyTest(const int test_num = 100, const int obs_num = 50, const int side_len = 30, 
                    Mat back_img = Mat(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255))) {
    float extended_check_time = 0, DG_check_time = 0;
    int inconsistency_num = 0;
    for (int test_idx = 0; test_idx < test_num; test_idx++) {
        vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, back_img.size(), side_len);

        auto start_time = high_resolution_clock::now();
        auto extended_visibility_check_res = ExtendedVisibilityPassageCheck(obs_vec);
        auto end_time = high_resolution_clock::now();
        auto duration_time = duration_cast<milliseconds>(end_time - start_time);
        extended_check_time += (float) duration_time.count();

        start_time = high_resolution_clock::now();
        auto DG_check_res = PassageCheckInDelaunayGraph(obs_vec);        
        end_time = high_resolution_clock::now();
        duration_time = duration_cast<milliseconds>(end_time - start_time);
        DG_check_time += (float) duration_time.count();

        if (extended_visibility_check_res.first.size() != DG_check_res.first.size()) {
            inconsistency_num++;
            cout << extended_visibility_check_res.first.size() << " vs " << DG_check_res.first.size() << "\n";
            set<vector<int>> extended_psg_set(extended_visibility_check_res.first.begin(), extended_visibility_check_res.first.end()),
                             DG_psg_set(DG_check_res.first.begin(), DG_check_res.first.end());
            vector<vector<int>> extended_exclusive(10), DG_exclusive(10);
            auto extended_itr = set_difference(extended_psg_set.begin(), extended_psg_set.end(), DG_psg_set.begin(), DG_psg_set.end(), extended_exclusive.begin());
            extended_exclusive.resize(extended_itr - extended_exclusive.begin());
            auto DG_itr = set_difference(DG_psg_set.begin(), DG_psg_set.end(), extended_psg_set.begin(), extended_psg_set.end(), DG_exclusive.begin());
            DG_exclusive.resize(DG_itr - DG_exclusive.begin());
            cout << "passages only in extended check condition: ";
            for (auto psg : extended_exclusive)
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
                PolygonObstacle cur_obs = obs_vec[i];
                int cur_vertex_num = cur_obs.vertices.size();
                for (int j = 0; j < cur_vertex_num; j++) {
                    line(back_img, cur_obs.vertices[j], cur_obs.vertices[(j + 1) % cur_vertex_num], Scalar(0, 0, 0), 2);
                    line(DG_img, cur_obs.vertices[j], cur_obs.vertices[(j + 1) % cur_vertex_num], Scalar(0, 0, 0), 2);
                }
                Point2f cur_centroid = GetObstaclesCentroids({cur_obs}).front();
                putText(back_img, std::to_string(i), cur_centroid - Point2f(10, -5), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
            }  
            
            for (int i = 0; i < extended_visibility_check_res.first.size(); i++)
                DrawDshedLine(back_img, extended_visibility_check_res.second[i][0], extended_visibility_check_res.second[i][1], Scalar(0, 0, 0), 2);

            vector<vector<int>> DG_adj = DelaunayTriangulationObstables(obs_vec);
            for (int i = 0; i < DG_adj.size(); i++) 
                for (int j : DG_adj[i]) 
                    if (i < j)
                        DrawDshedLine(DG_img, GetObstaclesCentroids({obs_vec[i]})[0], GetObstaclesCentroids({obs_vec[j]})[0], Scalar(0, 0, 255), 2);
            cout << "here\n";
            imshow("Delaunay Trigulation", DG_img);

            imshow("Gabriel Cells", back_img);
            waitKey(0);                                       
        }
    }

    cout << "TEST RESULT:"
         << "\nobstacle number: " << obs_num
         << "\nobstacle side primary length: " << side_len
         << "\nIn " << test_num << " tests, " << inconsistency_num << " inconsistent test(s) happen."
         << "\nAverage check time using extended visibility condition: " << extended_check_time / test_num << " ms"
         << "\nAverage check time based on Delaunay graph: " << DG_check_time / test_num << " ms\n";
}

void SVITest(const int obs_num = 50, const int side_len = 30, 
            Mat back_img = Mat(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255))) {
    vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, back_img.size(), side_len);
    auto extended_visibility_check_res = ExtendedVisibilityPassageCheck(obs_vec);
    for (int i = 4; i < obs_num + 4; i++) {
        Point2f cur_centroid = GetObstaclesCentroids({obs_vec[i]}).front();
        putText(back_img, std::to_string(i), cur_centroid - Point2f(10, -5), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2); 
        int cur_obs_vertex_num = obs_vec[i].vertices.size();
        for (int j = 0; j < cur_obs_vertex_num; j++)
            line(back_img, obs_vec[i].vertices[j], obs_vec[i].vertices[(j + 1) % cur_obs_vertex_num], Scalar(0, 0, 0), 2);           
    }

    for (int i = 0; i < extended_visibility_check_res.first.size(); i++) {
        vector<int> pair = extended_visibility_check_res.first[i];
        vector<vector<Point2f>> SVI_key_pts = SVIntersection(obs_vec[pair[0]], obs_vec[pair[1]]);
        line(back_img, SVI_key_pts[0][0], SVI_key_pts[0][1], Scalar(0, 0, 255), 2);
        line(back_img, SVI_key_pts[1][0], SVI_key_pts[1][1], Scalar(0, 0, 255), 2);
        DrawDshedLine(back_img, SVI_key_pts[2][0], SVI_key_pts[2][1], Scalar(0, 0, 0), 2);
/*         line(back_img, SVI_key_pts[3][0], SVI_key_pts[3][1], Scalar(0, 0, 0), 2);
        line(back_img, SVI_key_pts[4][0], SVI_key_pts[4][1], Scalar(0, 0, 0), 2); */
    }
    imshow("Shadow Volume for Passage Representation", back_img);
    waitKey(0); 
}

int main(int argc, char** argv) {
    Mat back_img(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255));
    //ConsistencyTest(1000, 100, 30, back_img);
    SVITest(30, 50, back_img);
/*     vector<vector<int>> faces = FindPlannarFaces(obs_vec, extended_visibility_check_res.first);
    cout << "Detected face number among sparse passages: " << faces.size() << "\n";
    for (vector<int>& face : faces) {
        for (int& obs_idx : face)
            cout << obs_idx << "<-->";
        cout << "\n";
    }
 */
    return 0;
}