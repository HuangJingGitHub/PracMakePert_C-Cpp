/**
 * @file Robot_CRTK.hpp
 * @copyright Copyright (C) Jing. All Rights Reserved.
 */

#ifndef FLEXIVRDK_ROBOT_CRTK_HPP_
#define FLEXIVRDK_ROBOT_CRTK_HPP_

#include "Robot.hpp"
#include <thread>

namespace flexiv {
/**
 * @class Robot
 * @brief Main interface with the robot, including system, control, motion, and
 * IO methods. Also responsible for communication.
 */
class Robot_CRTK : public Robot {
public:
    Robot_CRTK(const std::string& serverIP, const std::string& localIP): Robot(serverIP, localIP) {}
    virtual ~Robot_CRTK() {}

    //<<<<<<<<<<---------->>>>>>>>>>
    /**
     * @brief CRTK methods
     */
    std::vector<double> measured_jp() {
        RobotStates curRobotState;
        getRobotStates(curRobotState);
        return curRobotState.q;
    }

    std::vector<double> measured_cp() {
        RobotStates curRobotState;
        getRobotStates(curRobotState);
        return curRobotState.tcpPose;
    }

    std::vector<double> measured_cv() {
        RobotStates curRobotState;
        getRobotStates(curRobotState);
        return curRobotState.tcpVel;
    }

    std::vector<double> measured_cf() {
        RobotStates curRobotState;
        getRobotStates(curRobotState);
        return curRobotState.extForceInBaseFrame;
    }    

    std::vector<double> goal_cp() {
        RobotStates curRobotState;
        getRobotStates(curRobotState);
        return curRobotState.tcpPoseDes;
    }

    void servo_jp(const std::vector<double>& positions, 
                const std::vector<double>& velocities,
                const std::vector<double>& accelerations) {
        streamJointPosition(positions, velocities, accelerations);
    }

    void servo_jf(const std::vector<double>& torques,
                 bool enableGravityComp = true, 
                 bool enableSoftLimits = true) {
        streamJointTorque(torques, enableGravityComp, enableSoftLimits);
    }

    void servo_cp(const std::vector<double>& pose,
                const std::vector<double>& maxWrench
                = {100.0, 100.0, 100.0, 30.0, 30.0, 30.0}) {                    
        streamTcpPose(pose, maxWrench);        
    }

    void interpolate_jp(const std::vector<double>& positions,
                        const std::vector<double>& velocities,
                        const std::vector<double>& accelerations,
                        const std::vector<double>& maxVel, 
                        const std::vector<double>& maxAcc,
                        const std::vector<double>& maxJerk) {
        sendJointPosition(positions, velocities, accelerations, maxVel, maxAcc, maxJerk);
    }

    void interpolate_cp(const std::vector<double>& pose,
                        const std::vector<double>& maxWrench
                        = {100.0, 100.0, 100.0, 30.0, 30.0, 30.0}) {
        if (getMode() != flexiv::MODE_CARTESIAN_IMPEDANCE_NRT) {
            setMode(flexiv::MODE_CARTESIAN_IMPEDANCE_NRT);
            while (getMode() != flexiv::MODE_CARTESIAN_IMPEDANCE_NRT) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }                            
        }
        sendTcpPose(pose, maxWrench);
    }
};
} /* namespace flexiv */

#endif /* FLEXIVRDK_ROBOT_CRTK_HPP_ */
