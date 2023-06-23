#include "flexiv_include/flexiv/Robot_CRTK.hpp"
#include "flexiv_include/flexiv/Exception.hpp"
#include "flexiv_include/flexiv/Log.hpp"
#include "flexiv_include/flexiv/Scheduler.hpp"
#include "flexiv_include/flexiv/Utility.hpp"

#include <iostream>
#include <thread>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "flexiv_crtk/RobotStates_CRTK.h"

// IP of the robot server
std::string robotIP = "192.168.2.100";
// IP of the workstation PC running this program
std::string localIP = "192.168.2.109";
std::string master_measured_cp_topic = "/MTML/measured_cp";
Eigen::Vector3d master_translation, robot_translation, initial_master_translation, initial_robot_translation, master_translation_variation;
Eigen::Quaterniond master_quaterion, robot_quaterion, initial_master_quaterion, initial_robot_quaterion;
Eigen::Matrix4d master_rotation_mat, robot_rotation_mat, initial_master_rotation_mat, initial_robot_rotation_mat;
double translation_scale = 1;
Eigen::Matrix3d master_to_robot_base_transform = Eigen::MatrixXd::Identity(3, 3); // happy to be consistent
bool master_pose_initialized = false;
bool robot_pose_initialized = false;

void masterMsgCallback(const geometry_msgs::TransformStamped& msg) {
    std::cout << "Successfully receive MTML cp: " << msg.header.seq << '\n';
    master_translation(0) = msg.transform.translation.x;
    master_translation(1) = msg.transform.translation.y;
    master_translation(2) = msg.transform.translation.z;

    master_quaterion.w() = msg.transform.rotation.w;
    master_quaterion.x() = msg.transform.rotation.x;
    master_quaterion.y() = msg.transform.rotation.y;
    master_quaterion.z() = msg.transform.rotation.z;

    if (master_pose_initialized == false) {
        master_pose_initialized = true;
        initial_master_translation = master_translation;
        initial_master_quaterion = master_quaterion;
    }
}

void robotMsgCallback(const flexiv_crtk::RobotStates_CRTK& msg) {
        robot_translation(0) = msg.measured_cp.position.x;
        robot_translation(1) = msg.measured_cp.position.y;
        robot_translation(2) = msg.measured_cp.position.z;

        robot_quaterion.w() = msg.measured_cp.orientation.w;
        robot_quaterion.x() = msg.measured_cp.orientation.x;
        robot_quaterion.y() = msg.measured_cp.orientation.y;
        robot_quaterion.z() = msg.measured_cp.orientation.z;
    if (robot_pose_initialized == false) {
        master_pose_initialized = true;
        initial_robot_translation = robot_translation;
        initial_robot_quaterion = robot_quaterion;
    }
}

void transformPoseIntoVec(std::vector<double>& target_pose, Eigen::Vector3d& translation, Eigen::Quaterniond& quaterion) {
    target_pose[0] = translation(0);
    target_pose[1] = translation(1);
    target_pose[2] = translation(2);

    target_pose[3] = quaterion.w();
    target_pose[4] = quaterion.x();
    target_pose[5] = quaterion.y();
    target_pose[6] = quaterion.z();
}

int main(int argc, char** argv) {
/*     flexiv::Log log;
    flexiv::Robot_CRTK robot(robotIP, localIP, true);
    try {
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
    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    } */

    std::string test_flag;
    std::cout << "Press 1 to start teleoperation.\n";
    getline(std::cin, test_flag);
    
    if (test_flag != "1") {
        std::cout << "Exitting teleoperation program...\n";
        return 0;
    }
    
    ros::init(argc, argv, "flexiv_teleoperation_node");
    ros::NodeHandle node_handle;
    ros::Subscriber master_cp_sub = node_handle.subscribe(master_measured_cp_topic, 100, masterMsgCallback);
    ros::Subscriber robot_cp_sub = node_handle.subscribe("flexiv/robot_states/", 100, robotMsgCallback);
    
    // TO-DO: alignment of master

    //while (master_pose_initialized == false || robot_pose_initialized == false) {
    //    std::cout << "Wait for completing master and robot pose initialization\n";
    //}
    std::cout << "Ready to start teleoperation\n";

    while (true) {
        master_translation_variation = translation_scale * (master_translation - initial_master_translation);
        std::vector<double> robot_target_pose(7, 0);
        transformPoseIntoVec(robot_target_pose, master_translation_variation, master_quaterion);
        ros::spin();
        // robot.servo_cp(robot_target_pose);
    }
    return 0;
}