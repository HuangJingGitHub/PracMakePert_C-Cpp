/**
 * @file Robot_CRTK.hpp
 * @copyright Copyright (C) Jing. All Rights Reserved.
 */

#ifndef FLEXIVRDK_ROBOT_CRTK_HPP_
#define FLEXIVRDK_ROBOT_CRTK_HPP_

#include "Robot.hpp"
#include "Utility.hpp"
#include <thread>

namespace flexiv {
/**
 * @class The base class is Robot
 * @brief Robot is the main interface with the robot, including system, control, motion, and
 * IO methods. Also responsible for communication.
 */
class Robot_CRTK : public Robot {
public:
    const int robot_dof_num = 7;
    Robot_CRTK(const std::string& serverIP, const std::string& localIP): Robot(serverIP, localIP) {}
    virtual ~Robot_CRTK() {}

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
         if (getMode() != flexiv::MODE_JOINT_POSITION) {
            setMode(flexiv::MODE_JOINT_POSITION);
            while (getMode() != flexiv::MODE_JOINT_POSITION) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }                            
        }                      
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
        if (getMode() != flexiv::MODE_CARTESIAN_IMPEDANCE) {
            setMode(flexiv::MODE_CARTESIAN_IMPEDANCE);
            while (getMode() != flexiv::MODE_CARTESIAN_IMPEDANCE) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }                            
        }                                   
        streamTcpPose(pose, maxWrench);        
    }

    void interpolate_jp(const std::vector<double>& positions,
                        const std::vector<double>& velocities,
                        const std::vector<double>& accelerations,
                        const std::vector<double>& maxVel, 
                        const std::vector<double>& maxAcc,
                        const std::vector<double>& maxJerk) {
         if (getMode() != flexiv::MODE_JOINT_POSITION_NRT) {
            setMode(flexiv::MODE_JOINT_POSITION_NRT);
            while (getMode() != flexiv::MODE_JOINT_POSITION_NRT) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }                            
        }             
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

    void move_jp(const std::vector<double> target_joint_position) {
        if (getMode() != flexiv::MODE_PRIMITIVE_EXECUTION) {
            setMode(flexiv::MODE_PRIMITIVE_EXECUTION);
            while (getMode() != flexiv::MODE_PRIMITIVE_EXECUTION) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }                            
        }
        std::string target_joint_position_str = "MoveJ(target=";
        for (int i = 0; i < robot_dof_num - 1; i++)
            target_joint_position_str += (std::to_string(target_joint_position[i]) + " ");
        target_joint_position_str += (std::to_string(target_joint_position.back()) + ")");
        executePrimitive(target_joint_position_str);
        
        while (flexiv::utility::parsePtStates(getPrimitiveStates(), "reachedTarget") != "1") {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }        
    }

    void move_cp(const std::vector<double> target_position) {
        if (getMode() != flexiv::MODE_PRIMITIVE_EXECUTION) {
            setMode(flexiv::MODE_PRIMITIVE_EXECUTION);
            while (getMode() != flexiv::MODE_PRIMITIVE_EXECUTION) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }                            
        }
        std::string target_position_str = "MovePTP(target=";
        for (int i = 0; i < 5; i++)
            target_position_str += (std::to_string(target_position[i]) + " ");
        target_position_str += (std::to_string(target_position.back()) + " WORLD)");
        executePrimitive(target_position_str);
        
        while (flexiv::utility::parsePtStates(getPrimitiveStates(), "reachedTarget") != "1") {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } 
    }
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_ROBOT_CRTK_HPP_ */
