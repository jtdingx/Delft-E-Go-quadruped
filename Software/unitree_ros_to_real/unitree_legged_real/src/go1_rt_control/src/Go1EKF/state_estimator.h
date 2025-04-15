/*****************************************************************************
state_estimator.h

Description:	Header file of kinematic-based state-estimation

@Version:	1.0
@Author:	Jiatao Ding
@Release:	Tue 27 Jun 2017 09:31:28 AM CEST
@Update:	Tue 27 Jun 2017 09:31:24 AM CEST
*****************************************************************************/
#pragma once

#include "sensor_msgs/JointState.h"
#include "Filter/butterworth_filter.h"
#include "Filter/butterworthLPF.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>
#include "/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/src/go1_rt_control/src/Robotpara/robot_const_para_config.h"
#include "utils/Utils.h"
#include "utils/filter.hpp"

using namespace Eigen;
using namespace std;

// namespace gait{
    class StateestimatorClass
    {
    public:

        StateestimatorClass();
        ~StateestimatorClass();
        
        void config_set();
        ///// butterworth low-pass filer for comv, comacc
        butterworthLPF  butterworthLPFx, butterworthLPFy, butterworthLPFz;

        butterworthLPF  butterworthLPFcomax, butterworthLPFcomay, butterworthLPFcomaz;

        butterworthLPF  butterworthLPFthetaax,butterworthLPFthetaay,butterworthLPFthetaaz;
	
        butterworthLPF  butterworthLPFcomx, butterworthLPFcomy, butterworthLPFcomz;	


        Eigen::Matrix<double,23,1> state_estimator(int gait_mode, int& support_flag, Eigen::Vector3d FR_foot_relative_mea, Eigen::Vector3d FL_foot_relative_mea,Eigen::Vector3d RR_foot_relative_mea, Eigen::Vector3d RL_foot_relative_mea,
                                                   Eigen::Matrix<double,5,1> footforce_FR, Eigen::Matrix<double,5,1> footforce_FL,Eigen::Matrix<double,5,1> footforce_RR,Eigen::Matrix<double,5,1> footforce_RL,
                                                   double *FRfoot_pose, double *FLfoot_pose, double *RRfoot_pose, double *RLfoot_pose, double *support_pos_sensor, double *com_sensor, double *dcm_sensor, double omega_sensor, Eigen::Vector3d estimated_root_pos_offset);
        

        Eigen::Matrix<double, 3, 1> compute_ground_inclination(Eigen::Vector3d root_euler_d, Eigen::Vector3d root_euler, Eigen::Vector3d root_pos,
                                                Eigen::Matrix<double,3,4> foot_pos_recent_contact,double right_support);         
        void groud_inclination(Eigen::Matrix<double,3,4> foot_pos_recent_contact);
        
        double com_butterworth[3];
        double com_sensor_old[3];
        double comv_sensor[3];
        double comv_sensor_old[3];
        double coma_sensor[3];
        double comv_butterworth[3];
        double coma_butterworth[3];
        double com_sensor_hip[3];
	
     	double theta_sensor[3];
        double theta_sensor_old[3];	
    	double thetav_sensor[3];
        double thetav_sensor_old[3];
        double thetaa_sensor[3];
        double thetaa_butterworth[3];


        double rfz[5];
        double lfz[5];        
    private:  
        double _dt;
        double fz_double; 
        double fz_limit;   ////vertical height threshhold

        ////// butterworth filter
        double fsampling, fcutoff,fcutoff1,fcutoff2;

        double Fz_ratio_l,Fz_ratio_r;

        double FR_leg_offset_x,FR_leg_offset_y;
        double FL_leg_offset_x,FL_leg_offset_y;
        double RR_leg_offset_x,RR_leg_offset_y;
        double RL_leg_offset_x,RL_leg_offset_y;  

        double ratio_FR, ratio_FL,ratio_RR, ratio_RL;      

        /////terrain inclination
        double terrain_angle = 0;
        Eigen::Matrix<double, 4, 3> W;
        Eigen::Vector4d foot_pos_z;
        Eigen::Vector3d a;
        Eigen::Vector3d surf_coef;
        
        // filters
        MovingWindowFilter terrain_angle_filter;
        double use_terrain_adapt;   
        Eigen::Vector3d ground_incli_d;    

        double roll_right_leg, roll_left_leg,pitch_right_leg, pitch_left_leg;

    };
// }