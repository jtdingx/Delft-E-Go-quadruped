#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <sys/types.h>
#include <iostream>
#include <time.h>
#include <fstream>   
#include <string>  
#include <cassert>
#include <vector>
#include "eigen3/Eigen/Dense" //for hardware experiments
#include "math.h"
#include "whole_body_dynamics/dynmics_compute.h"
#include "/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/src/go1_rt_control/src/Robotpara/robot_const_para_config.h"
#include "yaml.h"
#include <ros/package.h> 
#include "Filter/butterworth_filter.h"
#include "Filter/butterworthLPF.h"


using namespace Eigen;
using namespace std;

sensor_msgs::JointState joint2simulation;

Dynamiccclass Dynam;

bool grf_start;

Eigen::Vector3d Rfoot_location_feedback;

Eigen::Vector3d Lfoot_location_feedback;

Eigen::Matrix<double, 3,1> body_p_des;
Eigen::Matrix<double, 3,1> body_r_des;
Eigen::Matrix<double, 3,1> FR_foot_des; 
Eigen::Matrix<double, 3,1> FL_foot_des; 
Eigen::Matrix<double, 3,1> RR_foot_des; 
Eigen::Matrix<double, 3,1> RL_foot_des; 
Eigen::Matrix<double, 6,1> F_sum; 
Eigen::Matrix<double, 3,1> comv_des; 
Eigen::Matrix<double, 3,1> thetav_des; 
Eigen::Matrix<double, 3,3> Momentum_sum;
Vector3d coma_des;
Vector3d theta_acc_des;

int gait_mode; 
int right_support; 
double y_offset;


Eigen::Matrix<double,100,1> state_feedback_receieved;

Matrix<double,100,1> wbc_gait_result;

double FR_foot_desx,FR_foot_desy, FR_foot_desz, FL_foot_desx,FL_foot_desy, FL_foot_desz;


int count_old;   
double mpc_stop;
int n_point;
int dynamic_count;
int dynamic_count_period;
void base_acc_ff_controler();
double kpp[3];
double kdp[3];
double kpw[3];
double kdw[3];
double clear_height;
Vector3d body_p_Homing_Retarget;
Eigen::Matrix<double, 3,1> FR_foot_Homing; 
Eigen::Matrix<double, 3,1> FL_foot_Homing; 
Eigen::Matrix<double, 3,1> RR_foot_Homing; 
Eigen::Matrix<double, 3,1> RL_foot_Homing;

//////state-feedback////
double com_sensor[3];
double comv_sensor[3];
double theta_sensor[3];
double thetav_sensor[3];
double FR_sensor[3];
double FL_sensor[3];
double RR_sensor[3];
double RL_sensor[3];
double support_pos_sensor[3];
double support_pos_des[3];
double body_relative_supportv_sensor[3];
double body_relative_support_sensor_old[3];

double w_pos_m_kp[3];
double w_pos_m_kd[3];
double w_rpy_m_kp[3];
double w_rpy_m_kd[3];
double com_pos_max,com_pos_min,com_rpy_max,com_rpy_min;
double global_com_feedback;
double w_pos_m[3];
double w_rpy_m[3];
double w_pos_m_filter[3];
double w_rpy_m_filter[3];
double dtx;
bool FR_swing, FL_swing, RR_swing, RL_swing;

butterworthLPF  butterworthLPF31,butterworthLPF32,butterworthLPF33,butterworthLPF34,butterworthLPF35,butterworthLPF36,butterworthLPF37;
double com_des_pre[3];
double theta_des_pre[3];
double gait_start;