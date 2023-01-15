/*****************************************************************************
state_estimator.h

Description:	Header file of MpcRTControlClass

@Version:	1.0
@Author:	Jiatao Ding
@Release:	Tue 27 Jun 2017 09:31:28 AM CEST
@Update:	Tue 27 Jun 2017 09:31:24 AM CEST
*****************************************************************************/
#pragma once
#include "KinematicsApi/kinematics.h"
#include "geometry_msgs/WrenchStamped.h"
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


using namespace Eigen;
using namespace std;

namespace gait{
    class StateestimatorClass
    {
    public:

        StateestimatorClass();
        ~StateestimatorClass();
        

        ///// butterworth low-pass filer for comv, comacc
        butterworthLPF  butterworthLPFx;
        butterworthLPF  butterworthLPFy;
        butterworthLPF  butterworthLPFz;

        butterworthLPF  butterworthLPFcomax;
        butterworthLPF  butterworthLPFcomay;
        butterworthLPF  butterworthLPFcomaz;

        butterworthLPF  butterworthLPFthetaax;
        butterworthLPF  butterworthLPFthetaay;
        butterworthLPF  butterworthLPFthetaaz;
	
        butterworthLPF  butterworthLPFcomx;
        butterworthLPF  butterworthLPFcomy;
        butterworthLPF  butterworthLPFcomz;	


        Eigen::Matrix<double,23,1> state_estimator(const int count_mpc, int& support_flag, const Walker_Leg waist_pL, const Walker_Leg waist_pR,
                                                  const geometry_msgs::Wrench left_ft_, const geometry_msgs::Wrench right_ft_, Eigen::Vector2d theta_default, const sensor_msgs::Imu body_imu, const geometry_msgs::Point zmp_ankle2waist_,
                                                  double *lfoot_pose_sensor, double *rfoot_pose_sensor, double *support_pos_sensor, double *com_sensor, 
					                         	  double *zmp_sensor,double *com_sensor_hip, double *dcm_sensor, double omega_sensor);
        
	    double com_butterworth[3];
        double com_sensor_old[3];
        double comv_sensor[3];
        double comv_sensor_old[3];
        double coma_sensor[3];
        double comv_butterworth[3];
        double coma_butterworth[3];
	
	    double theta_sensor[3];
        double theta_sensor_old[3];	
	    double thetav_sensor[3];
        double thetav_sensor_old[3];
        double thetaa_sensor[3];
        double thetaa_butterworth[3];

        
    private:  
        double _z_c;
        double _dt;
        double fz_double; 
        double fz_limit;   ////vertical height threshhold
        double rfz[5];
        double lfz[5];



        ////// butterworth filter
        double fsampling, fcutoff,fcutoff1,fcutoff2;

        
        
    };
}

