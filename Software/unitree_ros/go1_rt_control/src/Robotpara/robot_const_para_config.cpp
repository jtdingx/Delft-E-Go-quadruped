//
// Created by chunyuchen on 18-8-23.
//
#include "robot_const_para_config.h"

namespace gait {
    const double  dt_mpc_fast = 0.01;    /////second layer MPC-time interval

    const double  J_ini = 12 * 0.1*0.1;  /////inertial tensor

    const double  RobotPara_G = 9.8; 
    const double  RobotPara_FOOT_LENGTH = 0.01; 
    const double  RobotPara_FOOT_WIDTH = 0.01;
    const double  RobotPara_HIP_TO_ANKLE_X_OFFSET = 0; 
    const double  RobotParaClass_HALF_HIP_WIDTH = 0.12675;  
    
    const double height_offset_time = 0.0;
    const double height_offsetx = 0.000; /// walking on the flat ground
    //const double height_offsetx = 0.01; //climbing and resting on the stone
    //const double height_offsetx = 0.015; //climbing multi stairs
  
    /// gait parameters
    const double time_set = 0.0; //stair climbing
    const double footstepsnumber = 30;
    //const double footstepsnumber = 15; // ball hit
  
    // robot parameters
    const double mass = 12;   //total mass      
    const double force_z_limt = 50;
    
//math constant;
    const double PI = 3.141526;  //pi
    const double pi = 3.141526;
    const double Rad2Deg = 180 / pi;
    const double Deg2Rad = pi / 180;

//physical constant
    const double g = 9.8;   //gravity accelerate
    const double _g = 9.8;
}


