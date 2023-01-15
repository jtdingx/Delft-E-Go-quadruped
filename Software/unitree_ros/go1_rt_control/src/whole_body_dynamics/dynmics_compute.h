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
#include "body.h"
#include <Eigen/Dense>
#include "QP/QPBaseClass.h"


using namespace std;




class Dynamiccclass : public QPBaseClass
{
    public: 
        Dynamiccclass();
        ~Dynamiccclass();
        
        double mass;
        double initial_matrix;
        Eigen::Matrix<double, 12,1> joint_torque;

        double stance_kp, stance_kd;
        double swing_kp, swing_kd;
        Eigen::Matrix<double, 3, 4> gravity_compensate;

        ////// torque_control;
        Eigen::Matrix<double, 3,1> compute_joint_torques(Eigen::Matrix<double, 3,3> Jaco,
                                                         bool support_flag, 
                                                         Eigen::Matrix<double, 3,1> p_des,
                                                         Eigen::Matrix<double, 3,1> p_est,
                                                         Eigen::Matrix<double, 3,1> pv_des,
                                                         Eigen::Matrix<double, 3,1> pv_est,
                                                         int leg_number);


        //// force distribution
        Eigen::Matrix<double, 3,4>  F_leg_ref;
        Eigen::Matrix<double, 12,1> F_leg_guess;
        void force_distribution(Eigen::Matrix<double, 3,1> com_des,
                                Eigen::Matrix<double, 12,1> leg_des, 
                                Eigen::Matrix<double, 6,1> F_force_des, 
                                int mode, double y_coefficient, double rfoot_des[3],double lfoot_des[3]);      

        void force_opt(Eigen::Matrix<double, 3,1> base_p,
                       Eigen::Matrix<double, 3,1> FR_p, 
                       Eigen::Matrix<double, 3,1> FL_p, 
                       Eigen::Matrix<double, 3,1> RR_p, 
                       Eigen::Matrix<double, 3,1> RL_p, 
                       Eigen::Matrix<double, 6,1> FT_total_des, 
                       int mode, int right_support, double y_coefficient); 
        Eigen::Matrix3d skew_hat(Eigen::Matrix<double, 3,1> vec_w);         
        void Solve();
        void solve_grf_opt();
        Eigen::Matrix<double, 12,1> grf_opt;

        bool qp_solution;  
        Eigen::Matrix<double,12,12> Q_goal, Q_goal1;
        Eigen::Matrix<double,12,1> q_goal;

        Eigen::Matrix3d A_unit;
        Eigen::Matrix<double,12,12> B_unit;


        double qp_alpha;
        double qp_beta;
        double qp_gama;
        double fz_max;
        double mu;
        double x_offset;

        Eigen::Matrix<double,12,12> AA;
        Eigen::Matrix<double,12,1> bb;
        Eigen::Matrix<double,24,12> qp_H;
        Eigen::Matrix<double,24,1> qp_h;



        // Eigen::Matrix<double,12,12> _Q_goal;
        // Eigen::Matrix<double,12,1> _Q_goal;
 
};

