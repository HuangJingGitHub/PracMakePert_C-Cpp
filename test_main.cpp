#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "robot_config.hpp"

int main(int argc, char** argv) {
    flexiv::Log log;
    try {
        flexiv::Robot robot(robotIP, localIP);

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
        std::vector<double> curJointPosition = robot.measured_jp();
        std::cout << "[";
        for (double singleJointPosition : curJointPosition)
            std::cout << singleJointPosition << " ";
        std::cout << "]";
    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    }
    return 0;
}