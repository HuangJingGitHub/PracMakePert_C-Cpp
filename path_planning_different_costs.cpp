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
    DrawObstacles(back_img, obs_vec, false);
    auto visibility_check_res = PureVisibilityPassageCheck(obs_vec);
    auto ev_check_res = ExtendedVisibilityPassageCheck(obs_vec);
    for (int i = 0; i < ev_check_res.second.size(); i++)
        DrawDshedLine(back_img, ev_check_res.second[i][0], ev_check_res.second[i][1]);
    
    Point2f start = Point2f(1, 1), end = Point2f(back_img.size().width - 1, back_img.size().height - 1);
    RRTStarPlanner planner_mpw(start, end, obs_vec, 50, back_img.size(), 2),
                   planner_gpw(start, end, obs_vec, 50, back_img.size(), 3),
                   planner_len(start, end, obs_vec, 30, back_img.size(), 0);
                  
    auto start_time = high_resolution_clock::now();
    bool planned_mpw = planner_mpw.Plan(back_img);
    bool planned_gpw = planner_gpw.Plan(back_img);
    auto end_time = high_resolution_clock::now();
    auto duration_time = duration_cast<milliseconds>(end_time - start_time);
    float ptopp_planning_time = (float)duration_time.count();

    start_time = high_resolution_clock::now();
    bool planned_len_cost = false; //planner_len.Plan(back_img);
    end_time = high_resolution_clock::now();
    duration_time = duration_cast<milliseconds>(end_time - start_time);
    float len_planning_time = (float) duration_time.count();

    cout << "Planning time in brute-force ptopp: " << ptopp_planning_time << " ms"
         << "\nPlanning time in shorest length planning: " << len_planning_time << " ms\n";

    vector<PathNode*> planned_path_ratio_cost, planned_path_mpw, planned_path_gpw, planned_path_weight_3;
    if (planned_mpw && planned_gpw) {
        planned_path_mpw = planner_mpw.GetPath(),
        planned_path_gpw = planner_gpw.GetPath();
        auto smooth_path_mpw = QuadraticBSplineSmoothing(planned_path_mpw);
        auto smooth_path_gpw = QuadraticBSplineSmoothing(planned_path_gpw);
        DrawPath(back_img, smooth_path_mpw, cv::viz::Color::red());
        DrawPath(back_img, smooth_path_gpw, cv::viz::Color::blue());

        auto passed_psgs_mpw = RetrievePassedPassages(smooth_path_mpw, planner_mpw.ev_passage_pts_),
             passed_psgs_gpw = RetrievePassedPassages(smooth_path_gpw, planner_gpw.ev_passage_pts_);

        cout << "optimal sorted passage width list in mpw:\n";
            for(float width : planner_mpw.target_node_->sorted_passage_list)
                cout << width << "\n";
        cout << "optimal sorted passage width list in gpw:\n";
            for(float width : planner_gpw.target_node_->sorted_passage_list)
                cout << width << "\n";

        cout << "passage widths in mpw:\n";
        for (int psg_idx : passed_psgs_mpw.first) {
            auto psg_pts = planner_mpw.ev_passage_pts_[psg_idx];
            cout << cv::norm(psg_pts[0] - psg_pts[1]) << "\n";
        }
        cout << "passage widths in gpw:\n";        
        for (int psg_idx : passed_psgs_gpw.first) {
            auto psg_pts = planner_gpw.ev_passage_pts_[psg_idx];
            cout << cv::norm(psg_pts[0] - psg_pts[1]) << "\n";
        }        
    }
    
    imshow("RRT* PTOPP", back_img);
    waitKey(0); 
    return 0;    
}