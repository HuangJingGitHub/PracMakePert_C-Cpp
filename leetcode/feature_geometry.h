#include <eigen3/Eigen/Dense>
#include "obstacles.h"

using namespace std;
using namespace Eigen;

class Angle3pts {
    /*Points storage order: s_0, s_1, s_2.*/
public:    
    Matrix<float, 6, 1> points_pos_;
    Matrix<float, 6, 1> gradient_;
    Vector2f v_1_;
    Vector2f v_2_;
    float angle_val_;   // deg
    
    Angle3pts(): points_pos_(ArrayXXf::Zero(6, 1)), gradient_(ArrayXXf::Zero(6, 1)), angle_val_(0) {}
    Angle3pts(vector<float> initial_pos) {
        if (initial_pos.size() != 6) {
            cout << "Invalid initial points coordinates. Custom initialization failed.\n"
                 << "The input vector should have a length of 6, but " << initial_pos.size() 
                 << " is given.\n";
            points_pos_ = ArrayXXf::Zero(6, 1);
        }
        else {
            for (int i = 0; i < 6; i++)
                points_pos_(i, 0) = initial_pos[i];
        }
        angle_val_ = 0;
    }


    void Update(vector<float> new_point_pos) {
        if (new_point_pos.size() != 6) {
            cout << "Fail to update the featuer information since input vector has an invalid dimension of "
                 << new_point_pos.size() << ". The length of the input vector should be 6.\n";
            return;
        }
        for (int i = 0; i < 6; i++)
            points_pos_(i, 0) = new_point_pos[i];
        
        float x_0 = points_pos_(0, 0), y_0 = points_pos_(1, 0),
              x_1 = points_pos_(2, 0), y_1 = points_pos_(3, 0),
              x_2 = points_pos_(4, 0), y_2 = points_pos_(5, 0);
        v_1_(0, 0) = x_1 - x_0; 
        v_1_(1, 0) = y_1 - y_0;
        v_2_(0, 0) = x_2 - x_0; 
        v_2_(1, 0) = y_2 - y_0;
        float t = v_1_.dot(v_2_) / (v_1_.norm()*v_2_.norm()),
              const_1 = -1 / sqrt(1 - t*t),
              const_2 = v_1_.dot(v_2_),
              n_1 = v_1_.norm(), n_2 = v_2_.norm();

        angle_val_ = acos(t) * 180 / M_PI;
        gradient_(0,0) = const_1 * ((2*x_0-x_1-x_2)*pow(n_1,2)*pow(n_2,2) - const_2*
                        ((x_0-x_1)*pow(n_2,2)+(x_0-x_2)*pow(n_1,2))) / (pow(n_1,3)*pow(n_2,3));
        gradient_(1,0) = const_1 * ((2*y_0-y_1-y_2)*pow(n_1,2)*pow(n_2,2) - const_2*
                        ((y_0-y_1)*pow(n_2,2)+(y_0-y_2)*pow(n_2,2))) / (pow(n_1,3)*pow(n_2,3));
        gradient_(2,0) = const_1 * ((x_2-x_0)*pow(n_1,2)*n_2 - (x_1-x_0)*const_2*n_2) / (pow(n_1,3)*pow(n_2,2));
        gradient_(3,0) = const_1 * ((y_2-y_0)*pow(n_1,2)*n_2 - (y_1-y_0)*const_2*n_2) / (pow(n_1,3)*pow(n_2,2));
        gradient_(4,0) = const_1 * ((x_1-x_0)*pow(n_2,2)*n_1 - (x_2-x_0)*const_2*n_1) / (pow(n_1,2)*pow(n_2,3));
        gradient_(5,0) = const_1 * ((y_1-y_0)*pow(n_2,2)*n_1 - (y_2-y_0)*const_2*n_1) / (pow(n_1,2)*pow(n_2,3));
    }


    int GetPivotIndex() {
        vector<pair<float, int>> gradient_square_to_index;
        for (int i = 0; i < 3; i++) {
            float gradient_square = pow(gradient_(2*i), 2) + pow(gradient_(2*i + 1), 2);
            gradient_square_to_index.push_back(pair<float, int>(gradient_square, i));
        }
        pair<float, int> max_square_pair = *max_element(gradient_square_to_index.begin(), gradient_square_to_index.end());
        return max_square_pair.second;
    }


    vector<float> DetermineTarget(vector<float> initial_pos, float desired_angle_val, vector<float> complete_pt_pos, 
                                  int complete_pt_idx = 0, float shape_ratio_constraint = 0.05,
                                  vector<PolygonObstacle> obstacles = vector<PolygonObstacle>(0)) {
        vector<float> target_pos(6, 0);
        if (desired_angle_val < 0 || complete_pt_idx < 0 || complete_pt_idx > 2 || complete_pt_pos.size() != 2 ||
            initial_pos.size() != 6) {
                cout << "Invalid input.\n";
                return target_pos;
            }
        vector<float> ref_pos(6, 0), dif_pos(2, 0);
        dif_pos[0] = complete_pt_pos[0] - initial_pos[2 * complete_pt_idx];
        dif_pos[1] = complete_pt_pos[1] - initial_pos[2 * complete_pt_idx + 1];
        target_pos[0] = complete_pt_pos[0];
        target_pos[1] = complete_pt_pos[1];
        for (int i = 0; i < 3; i++) {
            ref_pos[2 * i] = initial_pos[2 * i] + dif_pos[0];
            ref_pos[2 * i + 1] = initial_pos[2 * i + 1] + dif_pos[1];
        }

        float ref_length_v_1 = sqrt(pow(ref_pos[2] - ref_pos[0], 2) + pow(ref_pos[3] - ref_pos[1], 2)),
              ref_length_v_2 = sqrt(pow(ref_pos[4] - ref_pos[0], 2) + pow(ref_pos[5] - ref_pos[1], 2)),
              cur_angle_parameter, cur_angle_accumulated,
              cur_x_1, cur_y_1, cur_x_2, cur_y_2,
              distance_pt_1, distance_pt_2,
              lambda = 0.5, evaluation_H = 0,
              min_evaluation_H = FLT_MAX;
        Point2f cv_complete_pt(complete_pt_pos[0], complete_pt_pos[1]), 
                cv_ref_pt_1(ref_pos[2], ref_pos[3]),
                cv_ref_pt_2(ref_pos[4], ref_pos[5]),
                cv_pt_1, cv_pt_2;
        
        for (int i = 0; i <= 100; i++) {
            cur_angle_parameter = i / 100 * 2 * M_PI;
            cur_angle_accumulated = cur_angle_parameter + desired_angle_val;
            for (float ratio_1 = 1 - shape_ratio_constraint; ratio_1 <= 1 + shape_ratio_constraint; 
                ratio_1 += shape_ratio_constraint/5) {
                cur_x_1 = complete_pt_pos[0] + ref_length_v_1 * ratio_1 * cos(cur_angle_parameter);
                cur_y_1 = complete_pt_pos[1] + ref_length_v_1 * ratio_1 * sin(cur_angle_parameter);
                
                cv_pt_1.x = cur_x_1; 
                cv_pt_1.y = cur_y_1;
                for (auto obs : obstacles)
                    if (!ObstacleFree(obs, cv_complete_pt, cv_pt_1))
                        continue;
                distance_pt_1 = MinDistanceToObstaclesVec(obstacles, cv_pt_1);

                for (float ratio_2 = 1 - shape_ratio_constraint; ratio_2 <= 1 + shape_ratio_constraint; 
                ratio_2 += shape_ratio_constraint/5) {
                    cur_x_2 = complete_pt_pos[0] + ref_length_v_2 * ratio_2 * cos(cur_angle_accumulated);
                    cur_y_2 = complete_pt_pos[1] + ref_length_v_2 * ratio_2 * sin(cur_angle_accumulated);

                    cv_pt_2.x = cur_x_2;
                    cv_pt_2.y = cur_y_2;
                    for (auto obs : obstacles)
                        if (!ObstacleFree(obs, cv_complete_pt, cv_pt_2))
                           continue;
                    distance_pt_2 = MinDistanceToObstaclesVec(obstacles, cv_pt_2);
                    evaluation_H = lambda * (distance_pt_1 + distance_pt_2) + 
                                (1 - lambda) * (norm(cv_pt_1 - cv_ref_pt_1) + norm(cv_pt_2 - cv_ref_pt_2));
                    if (evaluation_H < min_evaluation_H) {
                        target_pos[2] = cur_x_1;
                        target_pos[3] = cur_y_1;
                        target_pos[4] = cur_x_2;
                        target_pos[5] = cur_y_2;
                    }
                }
            }
        }
        return target_pos;
    }
};
