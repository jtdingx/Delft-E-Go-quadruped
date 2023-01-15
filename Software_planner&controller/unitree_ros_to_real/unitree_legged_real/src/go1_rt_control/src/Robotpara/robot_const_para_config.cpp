//
// Created by chunyuchen on 18-8-23.
//
#include "robot_const_para_config.h"

namespace gait {
  
    // const double  dt_mpc_slow = 0.01;   /////first layer MPC-time interval
    const double  dt_mpc_fast = 0.01; /////second layer MPC-time interval
    const double  dt_grf = 0.002; /////grf update interval
    
    
    const double  RobotPara_G = 9.8; 
    const double  RobotPara_FOOT_LENGTH = 0.01; 
    const double  RobotPara_FOOT_WIDTH = 0.01;
    const double  RobotPara_HIP_TO_ANKLE_X_OFFSET = 0.0; 
    const double  RobotParaClass_HALF_HIP_WIDTH = 0.12675;     

    // robot parameters
    const double mass = 13;   //total mass      
    const double force_z_limt = 200; //// for real test

    const double  J_ini = mass * 0.1*0.1;  /////inertial tensor

    

    const double com_pos_max = 0.01;
    const double com_pos_min = -0.01;
    const double com_rpy_max = 0.02;
    const double com_rpy_min = -0.02;    
  
  
//math constant;
    const double PI = 3.141526;  //pi
    const double pi = 3.141526;
    const double Rad2Deg = 180 / pi;
    const double Deg2Rad = pi / 180;

//physical constant
    const double g = 9.8;  //gravity acceleration
    const double _g = 9.8;

    /// gait parameters
    const double time_set = 0.75;


    const double Z_c = 0.29;
    //ros system constant
    const double t_program_cyclic = 0.0025;  //program running time period
}


