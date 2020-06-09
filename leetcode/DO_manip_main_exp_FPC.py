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
camera2base = np.array([[-0.05022831108137371, -0.9300347774640589, -0.3640225672582554],
                        [-0.5243094571568366, 0.3347728323530255, -0.7829602441083688],
                        [0.850045122273998, 0.1515337039265905, -0.5044411032741803]])

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


if __name__ == '__main__':
    psm.home()
    init_joint_config = np.array([ 0.04084553,  0.40258606,  0.10347765,  1.34180595, -0.10493683, 0.16986141])

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

    while (np.linalg.norm(pt_pos_error) > 5):
        visual_info = get_visual_info()
        current_pt_pos = np.array([[i for i in visual_info.featurePoint]]).T
        target_pt_pos = np.array([[i for i in visual_info.featurePointTarget]]).T
        pt_pos_error = current_pt_pos - target_pt_pos

        log_feature = np.append(log_feature, current_pt_pos)
        log_error = np.append(log_error, pt_pos_error)
                
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

    randIdxStr = str(np.random.randint(0,500))
    np.save('./data/0609/log_featurePt' + randIdxStr, log_feature)
    np.save('./data/0609/log_error' + randIdxStr, log_error)
