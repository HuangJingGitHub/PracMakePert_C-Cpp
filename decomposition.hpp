#ifndef DECOMPOSITION_INCLUDED
#define DECOMPOSITION_INCLUDED

#include "obstacles.hpp"

int InNegativeHalfPlane(Point2f& pt) {
    return int(pt.y < 0 || (abs(pt.y) < 1e-6 && pt.x < 0));
}

float TriplePtCross(Point2f& origin_pt, Point2f& p1, Point2f& p2) {
    return (p1 - origin_pt).cross(p2 - origin_pt);
}

vector<std::vector<int>> FindPlannarFaces(const vector<PolygonObstacle>& obstacles, const vector<vector<int>>& passage_pairs) {
    int obs_num = obstacles.size();
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obstacles);
    vector<vector<int>> adjacency_list(obs_num);
    for (const vector<int>& passage_pair : passage_pairs) {
        adjacency_list[passage_pair[0]].push_back(passage_pair[1]);
        adjacency_list[passage_pair[1]].push_back(passage_pair[0]);
    }
    
    std::vector<std::vector<char>> used(obs_num);
    for (int i = 0; i < obs_num; i++) {
        used[i].resize(adjacency_list[i].size());
        used[i].assign(adjacency_list[i].size(), 0);
        auto compare = [&](int l, int r) {
            Point2f pl = obs_centroids[l] - obs_centroids[i];
            Point2f pr = obs_centroids[r] - obs_centroids[i];
            if (InNegativeHalfPlane(pl) != InNegativeHalfPlane(pr))
                return InNegativeHalfPlane(pl) < InNegativeHalfPlane(pr);
            return pl.cross(pr) > 0;
        };
        std::sort(adjacency_list[i].begin(), adjacency_list[i].end(), compare);
    }

    vector<vector<int>> faces;
    for (int i = 0; i < obs_num; i++) {
        for (int edge_id = 0; edge_id < adjacency_list[i].size(); edge_id++) {
            if (used[i][edge_id]) {
                continue;
            }
            vector<int> face;
            int v = i;
            int e = edge_id;
            while (used[v][e] == false) {
                used[v][e] = true;
                face.push_back(v);
                int u = adjacency_list[v][e];
                int e1 = std::lower_bound(adjacency_list[u].begin(), adjacency_list[u].end(), v, [&](int l, int r) {
                    Point2f pl = obs_centroids[l] - obs_centroids[u];
                    Point2f pr = obs_centroids[r] - obs_centroids[u];
                    if (InNegativeHalfPlane(pl) != InNegativeHalfPlane(pr))
                        return InNegativeHalfPlane(pl) < InNegativeHalfPlane(pr);
                    return pl.cross(pr) > 0;
                }) - adjacency_list[u].begin() + 1;
                if (e1 == adjacency_list[u].size()) {
                    e1 = 0;
                }
                v = u;
                e = e1;
            }

            std::reverse(face.begin(), face.end());
            int sign = 0;
            for (int j = 0; j < face.size(); j++) {
                int j1 = (j + 1) % face.size();
                int j2 = (j + 2) % face.size();
                float val = TriplePtCross(obs_centroids[face[j]], obs_centroids[face[j1]], obs_centroids[face[j2]]);
                if (val > 0) {
                    sign = 1;
                    break;
                } else if (val < 0) {
                    sign = -1;
                    break;
                }
            }
            if (sign <= 0) {
                faces.insert(faces.begin(), face);
            } else {
                faces.emplace_back(face);
            }
        }
    }
    return faces;
}

#endif