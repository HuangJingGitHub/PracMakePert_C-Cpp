#include "path_processing.hpp"
#include "decomposition.hpp"
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

int main(int argc, char** argv) {
    Mat back_img(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255));
    int obs_num = 30;
    vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, back_img.size(), 60);

    for (int i = 4; i < obs_num + 4; i++) {
        PolygonObstacle cur_obs = obs_vec[i];
        int cur_vertex_num = cur_obs.vertices.size();
        for (int j = 0; j < cur_vertex_num; j++)
            line(back_img, cur_obs.vertices[j], cur_obs.vertices[(j + 1) % cur_vertex_num], Scalar(0, 0, 0), 2);
        
        Point2f cur_centroid = GetObstaclesCentroids({cur_obs}).front();
        putText(back_img, std::to_string(i), cur_centroid - Point2f(10, -5), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
    }
    auto visibility_check_res = PureVisibilityPassageCheck(obs_vec);
    auto extended_visibility_check_res = ExtendedVisibilityPassageCheck(obs_vec);
    // for (int i = 0; i < visibility_check_res.first.size(); i++) 
    //     DrawDshedLine(back_img, visibility_check_res.second[i][0], visibility_check_res.second[i][1], Scalar(0.741, 0.447, 0), 1);
    for (int i = 0; i < extended_visibility_check_res.first.size(); i++)
        DrawDshedLine(back_img, extended_visibility_check_res.second[i][0], extended_visibility_check_res.second[i][1], Scalar(0, 0, 0), 2);
    cout << "Visibility check passage res: " << visibility_check_res.first.size() 
         << "\nExtended visibility check passage res: " << extended_visibility_check_res.first.size() << "\n";
    
    vector<vector<int>> faces = FindPlannarFaces(obs_vec, extended_visibility_check_res.first);
    cout << "Detected face number among sparse passages: " << faces.size() << "\n";
    for (vector<int>& face : faces) {
        for (int& obs_idx : face)
            cout << obs_idx << "<-->";
        cout << "\n";
    }

    vector<PolygonObstacle> obs_vec_no_box(obs_vec.size() - 4);
    for (int i = 0; i < obs_vec_no_box.size(); i++)
        obs_vec_no_box[i] = obs_vec[i + 4];
    
    vector<vector<int>> DT_result = DelaunayTriangulationOfObstable(obs_vec_no_box);
    for (int i = 0; i < DT_result.size(); i++) {
        cout << i + 4 << "---";
        for (int idx : DT_result[i])
            cout << idx + 4 << ", ";
        cout << "\n";
    }


    imshow("Gabriel Cells", back_img);
    waitKey(0); 
    return 0;
}