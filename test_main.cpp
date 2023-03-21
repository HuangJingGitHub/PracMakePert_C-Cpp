#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "robot_config.hpp"

const int motion_interval = 45;   // in millisecond

int main(int argc, char** argv) {

    initRobotMain();
    return 0;
}