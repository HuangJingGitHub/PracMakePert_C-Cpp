#ifndef OBSTACLES3D_INCLUDED
#define OBSTACLES3D_INCLUDED

#include "../obstacles.hpp"

struct PolygonObstacle3d {
    vector<Point2f> vertices;
    float height;
    PolygonObstacle3d() {}
    PolygonObstacle3d(vector<Point2f> polygon_vertices, float obs_height = 0): vertices(polygon_vertices) , height(obs_height) {}
    PolygonObstacle3d(vector<Point> polygon_vertices, float obs_height) {
        vertices = vector<Point2f>(polygon_vertices.size(), Point2f(0, 0));
        height = obs_height;
        for (int i = 0; i < polygon_vertices.size(); i++) {
            vertices[i].x = (float) polygon_vertices[i].x;
            vertices[i].y = (float) polygon_vertices[i].y;
        }
    }

    PolygonObstacle Get2dObstacle() const {
        return PolygonObstacle(vertices);
    }                                                  
};

class ObstacleHeightComparator {
public: 
    bool operator() (const PolygonObstacle3d& lhs, const PolygonObstacle3d& rhs) {
        return lhs.height > rhs.height ? true : false;
    }    
};

struct Passages3d {
    vector<vector<int>> pairs;
    vector<vector<Point2f>> pts;
    vector<vector<float>> heights;
    Passages3d() {}
    Passages3d(const vector<vector<int>>& input_pairs, const vector<vector<Point2f>>& input_pts, const vector<vector<float>>& input_heights): 
                pairs(input_pairs), pts(input_pts), heights(input_heights) {}
};

vector<PolygonObstacle> ConvertTo2dObstacles(const vector<PolygonObstacle3d>& obstacles) {
    vector<PolygonObstacle> res(obstacles.size());
    for (int i = 0; i < obstacles.size(); i++) {
        res[i] = obstacles[i].Get2dObstacle();
    }
    return res;
}

vector<Point2f> GetObstaclesCentroids(const vector<PolygonObstacle3d>& obstacles) {
    vector<Point2f> obs_centroids(obstacles.size());
    for (int i = 0; i < obstacles.size(); i++) {
        obs_centroids[i] = Point2f(0, 0);
        for (const Point2f& vertex : obstacles[i].vertices) 
            obs_centroids[i] += vertex;
        obs_centroids[i] /= (float) obstacles[i].vertices.size();
    } 
    return obs_centroids;
}

bool PointObstacleFree(const Point3f& pt, const vector<PolygonObstacle3d>& obs_vec) {
    Point2f pt_2d(pt.x, pt.y);
    for (const PolygonObstacle3d& obs : obs_vec) {
        if (InsidePolygon(pt_2d, obs.vertices) == true && pt.z <= obs.height)
            return false;
    }
    return true;
}

bool PlaneIntersection(const Point2f& plane_pt1, const Point2f& plane_pt2,
                        const float low_height, const float high_height,
                        const Point3f& p1, const Point3f& p2) {
    Point2f p1_2d(p1.x, p1.y), p2_2d(p2.x, p2.y);
    if (SegmentIntersection(plane_pt1, plane_pt2, p1_2d, p2_2d)) {
        Point2f intersection_pt_2d = GetSegmentsIntersectionPt(plane_pt1, plane_pt2, p1_2d, p2_2d);
        Point3f plane_intersection_pt = p1 + (p2 - p1) * cv::norm(intersection_pt_2d - p1_2d) / cv::norm(p2_2d - p1_2d);
        return low_height <= plane_intersection_pt.z && plane_intersection_pt.z <= high_height;
    }
    return false;
}

bool ObstacleFree(const PolygonObstacle3d& obs, const Point3f& p1, const Point3f& p2) {
    if (obs.vertices.size() <= 1)
        return true;

    Point2f p1_2d(p1.x, p1.y), p2_2d(p2.x, p2.y); 
    int vertex_num = obs.vertices.size();
    vector<Point2f> vertices_2d(vertex_num);
    for (int i = 0; i < vertex_num; i++)
        vertices_2d[i] = Point2f(obs.vertices[i].x, obs.vertices[i].y);

    for (int i = 0; i < vertex_num; i++) {
        if (SegmentIntersection(vertices_2d[i], vertices_2d[(i + 1) % vertex_num], p1_2d, p2_2d)) {
            Point2f intersection_pt_2d = GetSegmentsIntersectionPt(vertices_2d[i], vertices_2d[(i + 1) % vertex_num], p1_2d, p2_2d);
            Point3f plane_intersection_pt = p1 + (p2 - p1) * cv::norm(intersection_pt_2d - p1_2d) / cv::norm(p2_2d - p1_2d);
            if (plane_intersection_pt.z <= obs.height)
                return false;
        }
    }
    return true;
}

float ComputeObstacleArea(const PolygonObstacle3d& obs) {
    if (obs.vertices.size() <= 2) {
        throw std::invalid_argument("An obstacle with vertex number less than two is input in " + string(__func__));
    }
    float res = 0;
    for (int i = 0; i < obs.vertices.size(); i++)
        res += CrossProductVal(obs.vertices[i], obs.vertices[(i + 1) % obs.vertices.size()]);
    return abs(res) / 2;
}

float FreespaceVolume(const vector<PolygonObstacle3d>& obstacles, const Size2f config_size, const float config_height, bool has_env_walls = true) {
    float total_obs_vol = 0, total_vol = config_size.width * config_size.height * config_height;
    int idx = has_env_walls ? 4 : 0;
    for ( ; idx < obstacles.size(); idx++)
        total_obs_vol += ComputeObstacleArea(obstacles[idx]) * obstacles[idx].height;
    return total_vol - total_obs_vol;
} 

vector<PolygonObstacle3d> GenerateRandomObstacles3d(int obstacle_num, Size2f config_size = Size2f(640, 480), float config_height = 100, 
                                                    float size_len = 30, bool varying_side_len = false) {
    random_device rd_height;
    mt19937 rd_engine(rd_height());
    uniform_real_distribution<> distribution_heihgt(0, config_height);
    // uniform_int_distribution<> distribution_heihgt(0, config_height);

    vector<PolygonObstacle> obs_2d = GenerateRandomObstacles(obstacle_num, config_size, size_len, varying_side_len);
    vector<PolygonObstacle3d> res(obstacle_num + 4);
    for (int i = 0; i < 4; i++) 
        res[i] = PolygonObstacle3d(obs_2d[i].vertices, config_height);

    for (int i = 4; i < obstacle_num + 4; i++) {
        float height = distribution_heihgt(rd_height);
        res[i] = PolygonObstacle3d(obs_2d[i].vertices, height);
    }
    return res;
}

#endif