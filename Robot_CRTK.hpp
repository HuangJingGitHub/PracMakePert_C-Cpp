/**
 * @file Robot_CRTK.hpp
 * @copyright Copyright (C) Jing. All Rights Reserved.
 */

#ifndef FLEXIVRDK_ROBOT_CRTK_HPP_
#define FLEXIVRDK_ROBOT_CRTK_HPP_

#include "Robot.hpp"

namespace flexiv {

/**
 * @class Robot
 * @brief Main interface with the robot, including system, control, motion, and
 * IO methods. Also responsible for communication.
 */
class Robot_CRTK : public Robot
{
public:
    Robot_CRTK(const std::string& serverIP, const std::string& localIP): Robot(serverIP, localIP) {}
    virtual ~Robot_CRTK() {}

    //<<<<<<<<<<---------->>>>>>>>>>
    /**
     * @brief CRTK tests
     */
    std::vector<double> measured_jp() {
        RobotStates curRobotState;
        this->getRobotStates(curRobotState);
        return curRobotState.q;
    }

    std::vector<double> measured_cp() {
        RobotStates curRobotState;
        this->getRobotStates(curRobotState);
        return curRobotState.tcpPose;
    }

    std::vector<double> measured_cv() {
        RobotStates curRobotState;
        this->getRobotStates(curRobotState);
        return curRobotState.tcpVel;
    }

    std::vector<double> measured_cf() {
        RobotStates curRobotState;
        this->getRobotStates(curRobotState);
        return curRobotState.extForceInBaseFrame;
    }    

    std::vector<double> goal_cp() {
        RobotStates curRobotState;
        this->getRobotStates(curRobotState);
        return curRobotState.tcpPoseDes;
    }

    void servo_jp(const std::vector<double>& positions, 
                const std::vector<double>& velocities,
                const std::vector<double>& accelerations) {
        streamJointPosition(positions, velocities, accelerations);
    }
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_ROBOT_CRTK_HPP_ */
