/*************************************************************
 * Copyright: Walker Team, UBTech
 * Auther: chunyu.chen
 * Build Date: 31/07/2018
 * Modify Date:
 *************************************************************/

#ifndef ROBOT_CONST_PARA_CONFIG_H
#define ROBOT_CONST_PARA_CONFIG_H

#include <ros/ros.h>

/**
 * @namespace gait
 */
namespace gait{
    
  
    //// MPC gait 
    extern const double  dt_mpc_fast;  /////second layer MPC-time interval
    extern const double  J_ini;  /////robot mass
    extern const double height_offset_time;
    extern const double height_offsetx; 

    extern const double  RobotPara_G; 
    extern const double  RobotPara_FOOT_LENGTH; 
    extern const double  RobotPara_FOOT_WIDTH;
    extern const double  RobotPara_HIP_TO_ANKLE_X_OFFSET; 
    extern const double  RobotParaClass_HALF_HIP_WIDTH;  
  
    /// gait parameters
    extern const double time_set;  
    extern const double footstepsnumber;  //// step number

    // robot parameters
    extern const double mass;   //total mass    
    extern const double force_z_limt; // for support leg judgement

    
    ///math constant
    extern const double PI;  //pi
    extern const double pi;
    extern const double Rad2Deg;
    extern const double Deg2Rad;
    //physical constants
    extern const double g;   //gravity accelerate
    extern const double _g;

}//namespace gait


#endif //ROBOT_CONST_PARA_CONFIG_H
