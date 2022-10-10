#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "arm_include/FvrRobotClient.hpp"
#include "arm_include/ConnectionManager.hpp"
#include "arm_include/ClientTestFunction.hpp"
#include "data_processing.h"

using namespace fvr;

const int ONLINE_MOVE_MODE = 8;
const int motion_interval = 45;   // in millisecond
const float motion_magnitude = 0.001; // in meter
const float error_threshold = 5; // in pixel
bool relax_flag = false;

int main(int argc, char** argv) {
    std::ifstream camera_extrinsic_file("./src/visual_module/src/data/camera_extrinsic_matrix.txt");
    if (camera_extrinsic_file.is_open()) {
        std::string item_str;
        float item;
        int row = 0, col = 0, cnt = 0;
        while (camera_extrinsic_file >> item_str) {
            item  = std::stof(item_str);
            row = cnt / 3;
            col = cnt % 3;
            camera_to_base(row, col) = item;
            cnt++;
        }
        camera_extrinsic_file.close();
    }
    else {
        std::cout << "Fail to initialize the camera to base matrix, Default identity matix is used.\n";
    }
    std::cout << "Extrinsic camera matrix:\n" << camera_to_base << '\n';

    std::shared_ptr<FvrRobotClient> robot = std::make_shared<FvrRobotClient>();   
    const std::string server_address = "192.168.2.100";
    const std::string client_address = "192.168.2.109";
    ConnectionManager robot_connection(robot, server_address, client_address);
    std::thread connection([&]() {
        while (true) {
            if (robot_connection.run() != true)
                return;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
    ClientTestFunction client_tester(robot);

    ros::init(argc, argv, "manipulate_singelePt_main");
    ros::NodeHandle node_handle;
    ros::ServiceClient service_client = node_handle.serviceClient<visual_module::visual_info_service_singlePt>
                                        ("visual_info_service_singlePt");
    visual_module::visual_info_service_singlePt srv;
    srv.request.need_ee_path = false;

    std::string start_flag;
    std::cout << "Press 1 to start the manipulation experiment.\n";
    getline(std::cin, start_flag);
    if (start_flag != "1") {
        std::cout << "Exiting program.\n";
        return -1;
    }

    InitializeFiles(motion_interval, motion_magnitude, error_threshold);
    VelocityControllerSinglePt velocity_controller;
    while (true) {
        if (!service_client.call(srv)) {
            std::cout << "Failed to call service.\n";
            return 1;
        }

        ProcessServece(srv);
        WriteDataToFile();
        velocity_controller.DetectViolation(srv);

        if (ee_path_planned && track_ee_path_mode) {
            srv.request.need_ee_path = false;
            ee_velocity_image_3D(0, 0) = ee_path_error_pt(0, 0);
            ee_velocity_image_3D(1, 0) = ee_path_error_pt(1, 0);
            ee_velocity_image_3D(2, 0) = 0;
            ee_velocity_3D = camera_to_base * (-ee_velocity_image_3D);
            ee_velocity_3D = motion_magnitude / ee_velocity_3D.norm() * ee_velocity_3D; 
        }

        if (robot_connection.robotConnected() == false) {
            std::cout << "Fial to connect to robot server!\n";
            continue;
        }
        if (client_tester(ONLINE_MOVE_MODE, ee_velocity_3D) != true) {
            std::cout << "Fail to execute the command!\n";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(motion_interval));
        
        if (ee_path_planned && ee_total_error_pt.norm() < 15) { 
            track_ee_path_mode = false;
            relax_flag = true;
        }

        if (total_error_pt.norm() < error_threshold) {
            if (data_save_os.is_open())
                data_save_os.close();
            std::cout << "Manipulation completed. Exiting\n";
            break;
        }
    }
    return 0;
}