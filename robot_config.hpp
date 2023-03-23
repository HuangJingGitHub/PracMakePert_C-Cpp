/**
 * @brief Flexiv robot
 */
#ifndef ROBOT_INCLUDED
#define ROBOT_INCLUDED

#include "flexiv_include/flexiv/Robot.hpp"
#include "flexiv_include/flexiv/Exception.hpp"
#include "flexiv_include/flexiv/Log.hpp"
#include "flexiv_include/flexiv/Scheduler.hpp"
#include "flexiv_include/flexiv/Utility.hpp"

#include <iostream>
#include <cmath>
#include <thread>


// IP of the robot server
std::string robotIP = "192.168.2.100";
// IP of the workstation PC running this program
std::string localIP = "192.168.2.109";
bool robotInitialized = false;
flexiv::Robot* robotPtr = nullptr;
const unsigned int k_cartPoseSize = 7;
static std::vector<double> targetTcpPose;



/** Callback function for realtime periodic task */
bool moveRobot(flexiv::Robot* robotPtr, double delta_x, double delta_y) {
    // Flag whether initial Cartesian position is set
    if (robotPtr == nullptr) {
        std::cout << "Invalid robot pointer\n"; 
        return false;
    }

    try {
        // Monitor fault on robot server
        if (robotPtr->isFault()) {
            throw flexiv::ServerException(
                "Fault occurred on robot server, exiting ...");
        }

        flexiv::RobotStates robotStates;
        robotPtr->getRobotStates(robotStates);
        targetTcpPose = robotStates.tcpPose;
        targetTcpPose[0] += delta_x;
        targetTcpPose[1] += delta_y;
        robotPtr->sendTcpPose(targetTcpPose);
        return true;
    } 
    catch (const flexiv::Exception& e) {
        std::cerr << e.what();
        std::cout << "Robot Error\n";
        return false;
    }
}


int initRobotMain() {
    flexiv::Log log;

    static flexiv::Robot robot(robotIP, localIP);
    if (robot.isFault()) {
        robot.clearFault();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        // Check again
        if (robot.isFault()) {
            log.error("Fault cannot be cleared, exiting ...");
            return 1;
        }
        log.info("Fault on robot server is cleared");
    }

    log.info("Enabling robot ...");
    robot.enable();

    // Wait for the robot to become operational
    int secondsWaited = 0;
    while (robot.isOperational() == false) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (++secondsWaited == 10) {
            log.warn(
                "Still waiting for robot to become operational, please "
                "check that the robot 1) has no fault, 2) is booted "
                "into Auto mode");
        }
    }
    log.info("Robot is now operational");

    // Set mode after robot is operational
    robot.setMode(flexiv::MODE_CARTESIAN_IMPEDANCE_NRT);
    // Wait for the mode to be switched
    while (robot.getMode() != flexiv::MODE_CARTESIAN_IMPEDANCE_NRT) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    robot.switchTcp(0);
    robotPtr = &robot;

    return 1;
}

#endif