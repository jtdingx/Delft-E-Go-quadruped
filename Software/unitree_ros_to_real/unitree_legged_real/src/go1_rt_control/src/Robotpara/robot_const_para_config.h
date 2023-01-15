
#ifndef ROBOT_CONST_PARA_CONFIG_H
#define ROBOT_CONST_PARA_CONFIG_H

#include <ros/ros.h>

/**
 * @namespace gait
 */
namespace gait{
    
  
    //// MPC gait 
    // extern const double  dt_mpc_slow;   /////first layer MPC-time interval
    extern const double  dt_mpc_fast;  /////second layer MPC-time interval
    extern const double  dt_grf;
    extern const double  J_ini;  /////robot mass

    extern const double  RobotPara_G; 
    extern const double  RobotPara_FOOT_LENGTH; 
    extern const double  RobotPara_FOOT_WIDTH;
    extern const double  RobotPara_HIP_TO_ANKLE_X_OFFSET; 
    extern const double  RobotParaClass_HALF_HIP_WIDTH;  
  
    /// gait parameters
    extern const double time_set;    ///squat down time
  
    // robot parameters
    extern const double mass;   //total mass    
    extern const double force_z_limt; // for support leg judgement
    extern const double Z_c; // normal height
    
    
    /// controller parameters
    extern const double com_pos_max;
    extern const double com_pos_min;
    extern const double com_rpy_max;
    extern const double com_rpy_min;    
    
    ///math constant
    extern const double PI;  //pi
    extern const double pi;
    extern const double Rad2Deg;
    extern const double Deg2Rad;

    //physical constants
    extern const double g;   //gravity accelerate
    extern const double _g;

    //ros system constant
    extern const double t_program_cyclic;  //program running time period, 

}//namespace gait


#endif //ROBOT_CONST_PARA_CONFIG_H
