#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import rospy
import dvrk
from do_manip.srv import *
import math
import PyKDL
import sys
from time import sleep

deg2rad = np.pi / 180
camera2base = np.array([[-0.03647274692194044, -0.951725741046879, -0.304775085203395],
                        [-0.6460496670349185, 0.2551327847558498, -0.7193935570095104],
                        [0.7624234823613703, 0.1706615831741098, -0.6241674916017874]])

psm = dvrk.psm('PSM1')

def get_visual_info():
    rospy.wait_for_service('visual_info_service')
    try:
        srv_client = rospy.ServiceProxy('visual_info_service', visual_info_srv)
        visual_res = srv_client()
        return visual_res
    except rospy.ServiceException, e:
        print('Error in getting response from /do_manip/visual_info_service.')

def rotz(ang_rad):
    rotz = np.array([[math.cos(ang_rad), -math.sin(ang_rad), 0],
                    [math.sin(ang_rad), math.cos(ang_rad), 0],
                    [0, 0, 1]])
    return rotz

def PyKDL_Rotation2np_array(PyKDL_Rotation):
    rotation_np = np.eye(3)
    for i in range(3):
        for j in range(3):
            rotation_np[i, j] = PyKDL_Rotation[i, j]
    return rotation_np
 

def np_array2PyKDL_Rotation(np_array):
    rot_kdl = PyKDL.Rotation()
    for i in range(3):
        for j in range(3):
            rot_kdl[i, j] = np_array[i, j]
    return rot_kdl

def check_local_contact(visual_info):
    p_C_distance_threshold = 15
    p_C_distance = max(visual_info.contactDistancelr)
    # print(p_C_distance)
    return p_C_distance <= p_C_distance_threshold

def check_manipulability(visual_info):
    area_indicator = visual_info.indicatorw
    pt_area_distance = visual_info.distancew
    if area_indicator == 1 or pt_area_distance <= 5:
        return True
    else:
        return False
    

def check_safety_constraint(visual_info):
    allowed_angle_El = 180
    allowed_angle_Er = 180
    print(visual_info.deformAngles)
    return (visual_info.deformAngles[0] <= allowed_angle_El and 
            visual_info.deformAngles[1] <= allowed_angle_Er)

def adjust_local_contact(adjust_direction = 0):
    # x_t = psm.get_current_position()
    # x_t_ori = x_t.M
    # x_t_ori_np = PyKDL_Rotation2np_array(x_t_ori)
    step = 0.5 * deg2rad
    if adjust_direction == 0: # default direction
        # delta_ori_np = rotz(step)
        # delta_x_ori_np = np.matmul(camera2base, delta_ori_np)
        # delta_x_ori_np = np.matmul(delta_x_ori_np, camera2base.T)
        psm.dmove_joint_one(step, 3)
    elif adjust_direction == 1:
        # delta_ori_np = rotz(-step)
        # delta_x_ori_np = np.matmul(camera2base, delta_ori_np)
        # delta_x_ori_np = np.matmul(delta_x_ori_np, camera2base.T)
        psm.dmove_joint_one(-step, 3)
    else:
        print('Invalid Argument Given In Function: adjust_local_contact()')
        return
    # delta_x_ori_kdl = np_array2PyKDL_Rotation(delta_x_ori_np)
    # psm.dmove(delta_x_ori_kdl)
    sleep(0.05)

def adjust_manipulatbility(visual_info):
    step = 0.0005
    sl_list = [[i for i in visual_info.sl]]
    sr_list = [[i for i in visual_info.sr]]
    diff_lr = np.array(sl_list).T - np.array(sr_list).T
    diff_lr = diff_lr / np.linalg.norm(diff_lr)
    ne = np.array([[i for i in visual_info.ne]]).T
    gama = 0.25
    area_indicator = visual_info.indicatorw
    pt_area_distance = visual_info.distancew

    if area_indicator == 2:
        motion_direction = diff_lr - gama * ne 
    elif area_indicator == 3:
        motion_direction = -diff_lr - gama * ne
    
    x_dot_image = np.append(motion_direction, [[0]], axis=0)
    x_dot = np.matmul(camera2base, x_dot_image)
    x_dot_scale = x_dot / np.linalg.norm(x_dot) * step
    x_dot_kdl = PyKDL.Vector(x_dot_scale[0][0], x_dot_scale[1][0], x_dot_scale[2][0])
    psm.dmove(x_dot_kdl)
    sleep(0.05)

def adjust_safety_constraint(visual_info, adjust_direction = 0):
    step = 0.0005
    sl_list = [i for i in visual_info.sl]
    sr_list = [i for i in visual_info.sr]
    diff_lr = np.array(sl_list) - np.array(sr_list)
    diff_lr = diff_lr / np.linalg.norm(diff_lr)
    ne = np.array([i for i in visual_info.ne])
    gama = 0.3
    motion_direction = diff_lr - gama * ne
    motion_direction = motion_direction / np.linalg.norm(motion_direction)
    if adjust_direction != 0:
        motion_direction = -motion_direction
    motion_image = np.array([[motion_direction[0]], [motion_direction[1]], [0]])
    motion_base = np.matmul(camera2base, motion_image) * step
    motion_kdl = PyKDL.Vector(motion_base[0][0], motion_base[1][0], motion_base[2][0])
    psm.dmove(motion_kdl)


if __name__ == '__main__':
    psm.home()
    #init_joint_config = np.array([ 0.01037935, -0.01804803,  0.11688517, -0.00062907,  0.00054655, 0.00012608])
    #init_joint_config = np.array([-0.12344788,  0.30524102,  0.10091488, -0.01530737, -0.35935399, 0.44307293])
    init_joint_config = np.array([-0.06231473,  0.16237818,  0.09966655, -0.58776091, -0.04399695, 0.04330933]) # normal initial
    init_joint_config = np.array([ 0.09045637,  0.22986714,  0.09129122,  0.79787023, -0.13540677, 0.10878157]) # local contact adjust

    psm.move_joint(init_joint_config)

    button = input('Press 1 to start the manipulation\n')
    if button != 1:
        print('Exiting')
        sys.exit()
    
    visual_info = get_visual_info()
    current_pt_pos = np.array([i for i in visual_info.featurePoint]).T
    target_pt_pos = np.array([i for i in visual_info.featurePointTarget]).T
    pt_pos_error = current_pt_pos - target_pt_pos

    log_feature = np.array([])
    log_error = np.array([])
    log_contact_distance = np.array([])
    log_manipulability = np.array([])

    while (np.linalg.norm(pt_pos_error) > 5):
        visual_info = get_visual_info()
        current_pt_pos = np.array([[i for i in visual_info.featurePoint]]).T
        target_pt_pos = np.array([[i for i in visual_info.featurePointTarget]]).T
        pt_pos_error = current_pt_pos - target_pt_pos
        
        log_feature = np.append(log_feature, current_pt_pos)
        log_error = np.append(log_error, pt_pos_error)
        log_contact_distance = np.append(log_contact_distance, max(visual_info.contactDistancelr))
        log_manipulability = np.append(log_manipulability, visual_info.distancew)
                
        ### when deformation control can be performed
        if check_local_contact(visual_info):
            print('Local Contact Valid \nSafety Constraint Satisfied')
            K = 100
            motion_step = 0.0005
            Jd_np = np.array([[visual_info.deformJacobian[0], visual_info.deformJacobian[1]],
                              [visual_info.deformJacobian[2], visual_info.deformJacobian[3]]])
            # x_dot_image = -K * np.matmul(np.linalg.pinv(Jd_np), pt_pos_error)
            x_dot_image = -K * np.matmul(Jd_np.T, pt_pos_error)
            #x_dot_image = -K * np.matmul(np.linalg.inv(Jd_np), pt_pos_error)

            print('Jd_np')
            print(Jd_np)
            print('pt_pos_error')
            print(pt_pos_error)
            print('x_dot_image')
            print(x_dot_image)

            x_dot_image = np.append(x_dot_image, [[0]], axis=0)
            x_dot = np.matmul(camera2base, x_dot_image)
            x_dot_scale = x_dot / np.linalg.norm(x_dot) * motion_step

            print('x_dot_scale:')
            print(x_dot_scale)
            print('')

            x_dot_kdl = PyKDL.Vector(x_dot_scale[0][0], x_dot_scale[1][0], x_dot_scale[2][0])
            psm.dmove(x_dot_kdl)
            sleep(0.1)

        while not check_local_contact(visual_info):
            print('Local Contact Adjustment')
            if visual_info.contactDistancelr[0] > visual_info.contactDistancelr[1]:
                adjust_local_contact(0)
            else:
                adjust_local_contact(1)

            current_pt_pos = np.array([[i for i in visual_info.featurePoint]]).T
            target_pt_pos = np.array([[i for i in visual_info.featurePointTarget]]).T
            pt_pos_error = current_pt_pos - target_pt_pos
            log_feature = np.append(log_feature, current_pt_pos)
            log_error = np.append(log_error, pt_pos_error)
            log_contact_distance = np.append(log_contact_distance, max(visual_info.contactDistancelr))
            log_manipulability = np.append(log_manipulability, visual_info.distancew)               
            
            visual_info = get_visual_info()
        
        while not check_manipulability(visual_info):
            print('Manipulability Adjustment')
            adjust_manipulatbility(visual_info)
          
            current_pt_pos = np.array([[i for i in visual_info.featurePoint]]).T
            target_pt_pos = np.array([[i for i in visual_info.featurePointTarget]]).T
            pt_pos_error = current_pt_pos - target_pt_pos     
            
            log_feature = np.append(log_feature, current_pt_pos)
            log_error = np.append(log_error, pt_pos_error)
            log_contact_distance = np.append(log_contact_distance, max(visual_info.contactDistancelr))
            log_manipulability = np.append(log_manipulability, visual_info.distancew)

            visual_info = get_visual_info()
            

        while not check_local_contact(visual_info):
            print('Local Contact Adjustment')
            if visual_info.contactDistancelr[0] > visual_info.contactDistancelr[1]:
                adjust_local_contact(0)
            else:
                adjust_local_contact(1)
                
            current_pt_pos = np.array([[i for i in visual_info.featurePoint]]).T
            target_pt_pos = np.array([[i for i in visual_info.featurePointTarget]]).T
            pt_pos_error = current_pt_pos - target_pt_pos                      
            log_feature = np.append(log_feature, current_pt_pos)
            
            log_error = np.append(log_error, pt_pos_error)
            log_contact_distance = np.append(log_contact_distance, max(visual_info.contactDistancelr))
            log_manipulability = np.append(log_manipulability, visual_info.distancew)               
            
            visual_info = get_visual_info()

    randIdxStr = str(np.random.randint(0,500))
    np.save('log_featurePt' + randIdxStr, log_feature)
    np.save('log_error' + randIdxStr, log_error)
    np.save('log_contact_distance' + randIdxStr, log_contact_distance)
    np.save('log_manipulability' + randIdxStr, log_manipulability)
