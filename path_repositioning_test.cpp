#include "path_processing.hpp"
#include <opencv2/viz/types.hpp>

void DrawPath(Mat img, const vector<Point2f>& path, 
            Scalar color = Scalar(255, 0, 0), 
            int thickness = 2,
            Point2f shift = Point2f(0, 0)) {
    for (int i = 0; i < path.size() - 1; i++)
        line(img, path[i] + shift, path[i + 1] + shift, color, thickness);
}

void DrawPath(Mat img, const vector<RRTStarNode*>& path, 
            Scalar color = Scalar(255, 0, 0), 
            int thickness = 2,
            Point2f shift = Point2f(0, 0)) {
    for (int i = 0; i < path.size() - 1; i++)
        line(img, path[i]->pos + shift, path[i + 1]->pos + shift, color, thickness);
}


int main(int argc, char** argv) {
    Mat back_img(Size(500, 300), CV_64FC3, Scalar(255, 255, 255));
    int obs_num = 20;
    vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, back_img.size());
    for (int i = 4; i < obs_num + 4; i++) {
        PolygonObstacle cur_obs = obs_vec[i];
        int cur_vertex_num = cur_obs.vertices.size();
        for (int j = 0; j < cur_vertex_num; j++)
            line(back_img, cur_obs.vertices[j], cur_obs.vertices[(j + 1) % cur_vertex_num], Scalar(0, 0, 0), 2);
    }
    auto visibility_check_res = PureVisibilityPassageCheck(obs_vec);
    auto extended_visibility_check_res = ExtendedVisibilityPassageCheck(obs_vec);
    for (int i = 0; i < visibility_check_res.first.size(); i++) 
        line(back_img, visibility_check_res.second[i][0], visibility_check_res.second[i][1], Scalar(0, 255, 0), 2);
    for (int i = 0; i < extended_visibility_check_res.first.size(); i++)
        line(back_img, extended_visibility_check_res.second[i][0], extended_visibility_check_res.second[i][1], Scalar(0, 0, 0), 2);
    cout << "Visibility check passage res: " << visibility_check_res.first.size() 
         << "\nExtended visibility check passage res: " << extended_visibility_check_res.first.size() << '\n';
    
    Point2f start = Point2f(1, 1), end = Point2f(back_img.size().width - 1, back_img.size().height - 1);
    vector<Point2f> init_pts{start, Point2f(1, 50)}, target_pts{end, end + Point2f(0, 0)};
    RRTStarPlanner planner_weight_cost(start, end, obs_vec, 15, 20, 10, back_img.size(), 0, 10);
    bool path_planned = planner_weight_cost.Plan(back_img);
    vector<RRTStarNode*> planned_path_node;
    vector<Point2f> planned_path_pts;
    if (path_planned) {
        planned_path_node = planner_weight_cost.GetPath();
        planned_path_pts = planner_weight_cost.GetPathInPts();
        auto smooth_path_weight_2 = QuadraticBSplineSmoothing(planned_path_node);
        // cout << "path node num: " << planned_path_ratio_cost.size() << " smooth node num: " << smooth_path_ratio_cost.size() << "\n";
        vector<int> passage_indices = RetrievePassedPassages(planned_path_node, planner_weight_cost.extended_visibility_passage_pts_);
        for (int passage_idx : passage_indices) {
            cout << planner_weight_cost.extended_visibility_passage_pair_[passage_idx][0] << "---" << planner_weight_cost.extended_visibility_passage_pair_[passage_idx][1] << '\n';
            auto intersection_pt = GetPathSetIntersectionsOnPassageLine(planned_path_pts, {start}, {end}, 0,  
                                                                        planner_weight_cost.extended_visibility_passage_pts_[passage_idx]);
            circle(back_img, intersection_pt[0], 4, Scalar(0, 0, 255), -1);
        }

        vector<Point2f> reposition_pts = RepositionPivotPath(planned_path_pts, init_pts, target_pts, 0, planner_weight_cost.extended_visibility_passage_pts_);
        for (Point2f& pt : reposition_pts)
            if (pt.x > 0)
                circle(back_img, pt, 4, Scalar(0, 0, 255), -1);
        DrawPath(back_img, smooth_path_weight_2, cv::viz::Color::blue());
    }

    imshow("RRT* path planning", back_img);
    waitKey(0); 
    return 0;    
}