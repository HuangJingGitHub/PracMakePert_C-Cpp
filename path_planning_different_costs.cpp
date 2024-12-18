#include <chrono>
#include <ctime>
#include "../RRTStar.hpp"
#include "path_processing.hpp"
#include <opencv2/viz/types.hpp>

using namespace std::chrono;

int main(int argc, char** argv) {
    Mat back_img(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255));
    int obs_num = 50;
    vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, back_img.size());
    for (int i = 4; i < obs_num + 4; i++) {
        PolygonObstacle obs = obs_vec[i];
        int vertex_num = obs.vertices.size();
        for (int j = 0; j < vertex_num; j++)
            line(back_img, obs.vertices[j], obs.vertices[(j + 1) % vertex_num], Scalar(0, 0, 0), 2);
    }
    auto visibility_check_res = PureVisibilityPassageCheck(obs_vec);
    auto ev_check_res = ExtendedVisibilityPassageCheck(obs_vec);
    cout << "Visibility check passage res: " << visibility_check_res.first.size() 
         << "\nExtended visibility check passage res: " << ev_check_res.first.size() << "\n\n";
    
    Point2f start = Point2f(1, 1), end = Point2f(back_img.size().width - 1, back_img.size().height - 1);
    RRTStarPlanner planner_mpw_cost(start, end, obs_vec, 50, 20, back_img.size(), 2, 100),
                   planner_len_cost(start, end, obs_vec, 15, 10, back_img.size(), 0);
    auto start_time = high_resolution_clock::now();
    bool planned_mpw_cost = planner_mpw_cost.Plan(back_img);
    auto end_time = high_resolution_clock::now();
    auto duration_time = duration_cast<milliseconds>(end_time - start_time);
    float ptopp_planning_time = (float)duration_time.count();

    start_time = high_resolution_clock::now();
    bool planned_len_cost = planner_len_cost.Plan(back_img);
    end_time = high_resolution_clock::now();
    duration_time = duration_cast<milliseconds>(end_time - start_time);
    float len_planning_time = (float) duration_time.count();

    cout << "Planning time in brute-force ptopp: " << ptopp_planning_time << " ms"
         << "\nPlanning time in shorest length planning: " << len_planning_time << " ms\n";

    vector<PathNode*> planned_path_ratio_cost, planned_path_weight_1, planned_path_weight_2, planned_path_weight_3;
    if (planned_mpw_cost) {
        planned_path_weight_1 = planner_mpw_cost.GetPath();
        auto // smooth_path_ratio_cost = QuadraticBSplineSmoothing(planned_path_ratio_cost),
             smooth_path_weight_1 = QuadraticBSplineSmoothing(planned_path_weight_1);
        // cout << "path node num: " << planned_path_ratio_cost.size() << " smooth node num: " << smooth_path_ratio_cost.size() << "\n";
        vector<int> passage_indices = RetrievePassedPassages(planned_path_weight_1, planner_mpw_cost.ev_passage_pts_).first;
        // DrawPath(back_img, smooth_path_ratio_cost, cv::viz::Color::blue());
        DrawPath(back_img, smooth_path_weight_1, cv::viz::Color::red());
    }
    
    for (int i = 0; i < ev_check_res.second.size(); i++)
        DrawDshedLine(back_img, ev_check_res.second[i][0], ev_check_res.second[i][1]);
    imshow("RRT* PTOPP", back_img);
    waitKey(0); 
    return 0;    
}