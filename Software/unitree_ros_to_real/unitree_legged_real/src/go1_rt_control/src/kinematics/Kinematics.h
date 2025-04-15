/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
Edited by Jiatao Ding, email: jtdingx@gmail.com
************************************************************************/
#pragma once

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include "body/body.h"
#include <Eigen/Dense>



using namespace std;




class Kinematicclass{
    public: 
        Kinematicclass();
        ~Kinematicclass();
        
        double FR_leg_offset_x,FR_leg_offset_y,FR_thigh_y,FR_thigh_length,FR_calf_length;
        double FL_leg_offset_x,FL_leg_offset_y,FL_thigh_y,FL_thigh_length,FL_calf_length;
        double RR_leg_offset_x,RR_leg_offset_y,RR_thigh_y,RR_thigh_length,RR_calf_length;
        double RL_leg_offset_x,RL_leg_offset_y,RL_thigh_y,RL_thigh_length,RL_calf_length;
        
        
        double lamda;
        double leg_offset_x,leg_offset_y,thigh_y,thigh_length,calf_length;

        Eigen::Matrix<double, 3,1> FR_feet, FL_feet,RR_feet, RL_feet;

        // local kinematics:
        Eigen::Matrix<double, 3,1> Forward_kinematics(Eigen::Matrix<double, 3,1> q_joint, int feet_flag);
        Eigen::Matrix<double, 3,1> Inverse_kinematics(Eigen::Matrix<double, 3,1> pos_des, Eigen::Matrix<double, 3,1> q_ini,int feet_flag); 

        Eigen::Matrix<double, 3,1> Forward_kinematics_go1(Eigen::Matrix<double, 3,1> q_joint, int feet_flag);

        // global kinematics:
        Eigen::Matrix<double, 3,1> Forward_kinematics_g(Eigen::Matrix<double, 3,1> body_P, Eigen::Matrix<double, 3,1> body_R, Eigen::Matrix<double, 3,1> q_joint, int feet_flag);
        Eigen::Matrix<double, 3,1> Inverse_kinematics_g(Eigen::Matrix<double, 3,1> body_P, Eigen::Matrix<double, 3,1> body_R, Eigen::Matrix<double, 3,1> pos_des, Eigen::Matrix<double, 3,1> q_ini,int feet_flag);

        Eigen::Matrix<double, 3,1> pos_cal, det_pos, det_angle, q_des;

        Eigen::Matrix<double, 3,1> pos_hip,pos_thigh,pos_calf,pos_feet;

        Eigen::Matrix3d Jacobian_kin;
};

