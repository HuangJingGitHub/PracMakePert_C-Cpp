#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "robot_config.hpp"

void printQueryResult(const std::vector<double>& value) {
    if (value.size() == 0)
        return;

    std::cout << "[";
    for (int i = 0; i < value.size() - 1; i++)
        std::cout << value[i] << ", ";
    std::cout << value.back() << "]\n";
}


int main(int argc, char** argv) {
    flexiv::Log log;
    try {
        flexiv::Robot_CRTK robot(robotIP, localIP);

        if (robot.isFault()) {
            log.warn("Fault occurred on robot server, trying to clear ...");
            robot.clearFault();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            if (robot.isFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.info("Fault on robot server is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        int secondsWaited = 0;
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (++secondsWaited == 10) {
                log.warn(
                    "Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode");
            }
        }
        log.info("Robot is now operational");

        std::cout << "Calling robot.measured_jp()...\n";
        printQueryResult(robot.measured_jp());

        std::cout << "Calling robot.measured_cp()...\n";
        printQueryResult(robot.measured_cp());

        std::cout << "Calling robot.measured_cv()...\n";
        printQueryResult(robot.measured_cv());

        std::cout << "Calling robot.measured_cf()...\n";
        printQueryResult(robot.measured_cf());

        std::cout << "Calling robot.goal_cp()...\n";
        printQueryResult(robot.goal_cp());
       
       int cnt = 0, time_step_millisecond = 1;
       double amplitude = 0.05, frequency = 0.5;
       std::vector<double> init_pose = robot.measured_cp();
       while (true) {
            double y_variance = amplitude * sin(2 * M_PI * frequency * cnt * time_step_millisecond * 0.001);
            auto target_pose = init_pose;
            target_pose[1] += y_variance;
            robot.interpolate_cp(target_pose);
            cnt++;
            std::this_thread::sleep_for(std::chrono::milliseconds(time_step_millisecond));
       }
    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    }
    return 0;
}