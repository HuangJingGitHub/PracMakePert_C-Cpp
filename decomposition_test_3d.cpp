#include <chrono>
#include <ctime>
#include <fstream>
#include "../3d/RRTStar_3d.hpp"
#include "../decomposition.hpp"

int main(int argc, char** argv) {
    Mat back_img(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255));
    int obs_num = 40;
    float config_height = 400, side_len = 60;
    bool varying_side_len = true;

    vector<PolygonObstacle3d> obs_vec = GenerateRandomObstacles3d(obs_num, back_img.size(), config_height, side_len, varying_side_len);
    vector<PolygonObstacle> obs_vec_2d = ConvertTo2dObstacles(obs_vec);

    Passages3d passages = PassageCheckInDelaunayGraph3d(obs_vec);
    FilterPassagesWithWalls3d(passages);
    Passages3d passages_wall = PassageCheckForWalls3d(obs_vec);
    Passages passages_2d = PassageCheckDelaunayGraphWithWalls(obs_vec_2d);

    vector<vector<int>> cells = ReportGabrielCells(obs_vec_2d, passages_2d.pairs, true);
    cout << "\n";
    for (vector<int>& cell : cells) {
        for (int obs_idx : cell)    
            cout << obs_idx << ", ";
        cout << "\n";
    }
    
    for (int i = 4; i < obs_num + 4; i++) {
        Point2f cur_centroid = GetObstaclesCentroids({obs_vec_2d[i]}).front();
        vector<vector<Point>> input_array_cv(1, vector<Point>(obs_vec_2d[i].vertices.begin(), obs_vec_2d[i].vertices.end()));
        fillPoly(back_img, input_array_cv, Scalar(0.827, 0.827, 0.827));
        int obs_vertex_num = obs_vec_2d[i].vertices.size();
        for (int j = 0; j < obs_vertex_num; j++)
            line(back_img, obs_vec_2d[i].vertices[j], obs_vec_2d[i].vertices[(j + 1) % obs_vertex_num], Scalar(0, 0, 0), 2);   
        putText(back_img, std::to_string(i), cur_centroid - Point2f(10, -5), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);         
    }
    for (int i = 0; i < passages.pts.size(); i++) {
        if (passages.heights[i][0] < 0.1)
            DrawDashedLine(back_img, passages.pts[i][0], passages.pts[i][1]);
        else
            DrawDashedLine(back_img, passages.pts[i][0], passages.pts[i][1], Scalar(0, 0, 255));
    }
    for (int i = 0; i < passages_wall.pts.size(); i++) {
        if (passages_wall.heights[i][0] < 0.1)
            DrawDashedLine(back_img, passages_wall.pts[i][0], passages_wall.pts[i][1]);
        else    
            DrawDashedLine(back_img, passages_wall.pts[i][0], passages_wall.pts[i][1], Scalar(0, 0, 255));        
    }

    imshow("Obstacles on Base Ground", back_img);
    waitKey(0);
    return 0;    
}