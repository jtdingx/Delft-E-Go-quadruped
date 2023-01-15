/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
#pragma once

#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "convert.h"
#include "sensor_msgs/JointState.h"
#include <Eigen/Dense>
#include "kinematics/Kinematics.h"
#include "Filter/butterworth_filter.h"
#include "Filter/butterworthLPF.h"
#include "whole_body_dynamics/dynmics_compute.h"


#include "sensor_msgs/Imu.h"


#include "utils/Utils.h"
#include "utils/filter.hpp"
#include "go1_const.h"
///// state estimation
#include "Robotpara/A1Params.h"
#include "Robotpara/A1CtrlStates.h"
#include "A1EKF/A1BasicEKF.h"
#include "A1EKF/state_estimator.h"



using namespace UNITREE_LEGGED_SDK;


typedef enum {
    STAND_INIT = 100,
    STAND = 101,
    STAND_UP  = 102,
    DYNAMIC = 103,
    UP_STAIR = 104,
    DOWN_STAIR = 105,
    PLAY_FOOTBALL = 106,
    DANCE = 107,
    JUMPING = 108,
    MetaFcnCompliance = 109,
    MetaFcnStaticBalance = 110
}CmdGait;

typedef enum {
    STAND_INIT_STATUS = 200,
    STAND_STATUS = 201,
    STAND_UP_STATUS = 202,
    DYNAMIC_STATUS = 203,
    UP_STAIR_STATUS = 204,
    DOWN_STAIR_STATUS = 205,
    PLAY_FOOTBALL_STATUS = 206,
    DANCE_STATUS = 207,
    JUMPING_STATUS = 208,
    MetaFcnComplianceStatus = 209,
    MetaFcnStaticBalanceStatus = 210
}GaitStatus;



//gait switch
CmdGait cmd_gait;
GaitStatus gait_status;



double Kp_joint[12];
double Kd_joint[12];

const int torque_err_row = 500;
////// initial parameters for joint-tracking
double torq_kp_calf, torq_kd_calf, torq_ki_calf;
double torq_kp_thigh, torq_kd_thigh, torq_ki_thigh;
double torq_kp_hip, torq_kd_hip, torq_ki_hip;
Eigen::Matrix<double, torque_err_row,12> torque_err;
Eigen::Matrix<double, 12,1>  torque_err_intergration;
Eigen::Matrix<double, 12,1> Torque_ff_spring,Torque_ff_GRF,Torque_ff_GRF_opt; 

/////// spring constant here
double k_spring_calf,k_spring_thigh,k_spring_hip;
double k_p_rest_calf,k_p_rest_thigh,k_p_rest_hip;
double hip_kp_scale, thigh_kp_scale, calf_kp_scale;
double hip_kd_scale, thigh_kd_scale, calf_kd_scale;

/////impedance control here
Eigen::Matrix<double, 3,3>  swing_kp, swing_kd;
Eigen::Matrix<double, 3,3>  RLswing_kp, RLswing_kd;

Eigen::Matrix<double, 3,3> global_swing;
//// torque control mode
Eigen::Matrix<double, 12,1>  torque, torque_ff;



int n_count;

double lastPos[12], percent, pos_cmd[12];
double stand_duration;
double time_programming;
int rt_frequency;

/////// kinematics
Kinematicclass Kine;

Dynamiccclass Dynam;


/////////////*******************8 robot state ******************************************///////////////////////
Eigen::Matrix<double,3,1> body_p_Homing, body_p_Homing_Retarget,body_r_Homing_Retarget,body_p_Homing_dynamic, body_p_des, body_r_des,body_r_Homing,body_r_Homing_dynamic;

Eigen::Matrix<double,3,1> FR_foot_des, FL_foot_des,RR_foot_des, RL_foot_des;
Eigen::Matrix<double,3,1> FR_foot_des_old, FL_foot_des_old,RR_foot_des_old, RL_foot_des_old;
Eigen::Matrix<double,3,1> FR_foot_Homing, FL_foot_Homing,RR_foot_Homing, RL_foot_Homing;
Eigen::Matrix<double,3,1> body_p_est, body_r_est;
Eigen::Matrix<double,12,1> leg_position;
Eigen::Matrix<double,3,1> FR_footv_des, FL_footv_des,RR_footv_des, RL_footv_des;

Eigen::Matrix<double,3,1> FR_angle_des, FL_angle_des,RR_angle_des, RL_angle_des;
Eigen::Matrix<double,3,1> FR_angle_mea, FL_angle_mea,RR_angle_mea, RL_angle_mea;
Eigen::Matrix<double,3,1> FR_dq_mea, FL_dq_mea,RR_dq_mea, RL_dq_mea;
Eigen::Matrix<double,12,1> angle_des;

Eigen::Matrix<double,3,1> FR_foot_mea, FL_foot_mea,RR_foot_mea, RL_foot_mea;
Eigen::Matrix<double,3,1> FR_foot_relative_des, FL_foot_relative_des,RR_foot_relative_des, RL_foot_relative_des;
Eigen::Matrix<double,3,1> FR_foot_relative_des_old, FL_foot_relative_des_old,RR_foot_relative_des_old, RL_foot_relative_des_old;
Eigen::Matrix<double,3,1> FR_foot_relative_mea, FL_foot_relative_mea,RR_foot_relative_mea, RL_foot_relative_mea;
Eigen::Matrix<double,3,1> FR_foot_relative_mea_old, FL_foot_relative_mea_old,RR_foot_relative_mea_old, RL_foot_relative_mea_old;

Eigen::Matrix<double,3,3> FR_Jaco, FL_Jaco,RR_Jaco, RL_Jaco;
Eigen::Matrix<double,3,3> FR_Jaco_est, FL_Jaco_est,RR_Jaco_est, RL_Jaco_est;

Eigen::Matrix<double,3,1> FR_v_relative, FL_v_relative,RR_v_relative, RL_v_relative;
Eigen::Matrix<double,3,1> FR_v_est_relative, FL_v_est_relative,RR_v_est_relative, RL_v_est_relative;

Eigen::Matrix<double,3,1> FR_foot_mea_old, FL_foot_mea_old,RR_foot_mea_old, RL_foot_mea_old;
Eigen::Matrix<double,3,1> FR_v_est, FL_v_est,RR_v_est, RL_v_est;

////state variables
Eigen::Vector3d root_pos;
Eigen::Quaterniond root_quat;
Eigen::Vector3d root_euler;
Eigen::Vector3d root_euler_offset;
Eigen::Vector3d root_euler_angular_velocity_offset;
Eigen::Matrix3d root_rot_mat;
Eigen::Matrix3d root_rot_mat_z;
Eigen::Vector3d root_lin_vel;
Eigen::Vector3d root_ang_vel;
Eigen::Vector3d root_acc_offset;
double yaw_angle;
Eigen::Vector3d estimated_root_vel_rotz;
Eigen::Vector3d linear_vel_rotz;
Eigen::Vector3d estimated_root_pos_offset,estimation_offset;

Eigen::Vector3d root_euler_sum;
Eigen::Vector3d root_euler_angular_velocity_sum;
Eigen::Vector3d root_acc_sum;



double ratex,rate,rate_stand_up;
Eigen::Matrix<double,3,1>  q_ini;


int stand_count,stand_up_count,dynamic_count;
double nt_slow_mpc;

/////////////////////////////////////////************* Force distribution **********************/////////////////////////////////////////
Eigen::Vector3d  kpp, kdp, kpw, kdw;

Eigen::Matrix<double, 100,1> Grf_sub;
Eigen::Matrix<double, 30,1> Grf_sub_filter;
////// Force distribution 
Eigen::Matrix<double,6,1> F_sum;
Eigen::Matrix3d Momentum_sum;
Eigen::Matrix3d Momentum_global;


Eigen::Vector3d vec_foot_rl,vec_com_rfoot;
double rlleg_dis, com_rleg_dis, rleg_com_raw, rleg_com_raw1;

double rleg_com, lleg_com;
Eigen::Matrix<double,6,1> F_lr_predict;
bool FR_swing,FL_swing,RR_swing,RL_swing;

int bjx1;
int right_support;
Eigen::Matrix<double, 3,1>  FR_torque_impedance,FL_torque_impedance,RR_torque_impedance,RL_torque_impedance;
Eigen::Matrix<double, 12,1> Legs_torque;

Eigen::Matrix<double, 3,1> FR_GRF, FL_GRF, RR_GRF, RL_GRF;
Eigen::Matrix<double, 3,1> FR_GRF_opt, FL_GRF_opt, RR_GRF_opt, RL_GRF_opt;






//// gait mode test:
///  bipedal walking: 101; troting: 102; gallop: 103; bounding: 104; run:105; jump:106;
int gait_mode;

double x_offset;
double y_offset;

////global state estimation:feet trajectory and COM trajectory//////////
StateestimatorClass state_est_kine;
Eigen::Matrix<double,23,1> state_kine;

int support_flag; /////left: 0; right: 1; double: 2
double omega_sensor;
double zmp_ref[3], dcm_ref[3], dcm_sensor[3];
Eigen::Matrix<double, 6,1> Force_L_R;

double support_pos_sensor[3]; ///left support by default
double support_pos_des[3];
double body_relative_support_des_old[3];
double body_relative_support_sensor_old[3];
double body_relative_supportv_des[3];
double body_relative_supportv_sensor[3];

double com_sensor[3];
double com_sensor_hip[3];
double com_sensor_pre[3];
double com_des[3];
double comv_des[3];
double coma_des[3];
double com_des_pre[3];
double rfoot_des[3];
double lfoot_des[3];
double rfoot_theta_des[3];
double lfoot_theta_des[3];
double theta_des[3];
double theta_des_pre[3];	
double thetav_des[3];
double theta_acc_des[3];
double frfoot_pose_sensor[3];
double flfoot_pose_sensor[3];
double rrfoot_pose_sensor[3];
double rlfoot_pose_sensor[3];
double zmp_sensor[3];
bool using_ft_sensor;
Eigen::Matrix<double,23,1> comav_butterworth;
Eigen::Vector2d theta_default;
double comv_sensor[3];
double coma_sensor[3];
double w_pos_m[3];
double w_rpy_m[3];
double w_pos_m_filter[3];
double w_rpy_m_filter[3];

Eigen::Vector3d L_com;
Eigen::Vector3d com_estkine;
Eigen::Vector3d cop_estkine;
Eigen::Vector3d theta_estkine;
Eigen::Vector3d theta_estkine_pre;
Eigen::Vector3d thetav_estkine;
Eigen::Vector3d thetaa_estkine;
double Fr_estkine;
double Fl_estkine;
Eigen::Vector3d comv_estkine;
Eigen::Matrix<double, 15,1> dob_result;
double J_ini_xx_est,J_ini_yy_est;





////===============================================================/////////////////
/////////////////// for fast_mpc: body inclination optimization //////////////


int count_in_mpc_max;

sensor_msgs::JointState joint2simulationx;
sensor_msgs::JointState state_to_MPC; /// 1 flag + 18state+3right_leg+3left_foot;

bool mpc_start = false;

Eigen::Matrix<double,25,1> state_feedback;
Eigen::Matrix<double,100,1> slow_mpc_gait;
int mpc_gait_flag,  mpc_gait_flag_old;
Eigen::Matrix<double,45,1> slow_mpc_gait_inte;

int fast_mpc_gait_flag,  fast_mpc_gait_flag_old;
Eigen::Matrix<double,51,1> fast_mpc_gait;	


Eigen::Matrix<double, 21,1> rpy_mpc_body, rfoot_inter, lfoot_inter, bodytheta_inter, rftheta_inter, lftheta_inter, zmp_inter, dcm_inter, comacc_inter;

Eigen::Vector3d COM_in1, COM_in2, COMxyz_ref, COMv_ref, COM_ref2;
Eigen::Vector3d FootL_in1, FootL_in2, FootL_ref, FootLv_ref, FootL_ref2;
Eigen::Vector3d FootR_in1, FootR_in2, FootR_ref, FootRv_ref, FootR_ref2;
Eigen::Vector3d body_in1, body_in2, body_ref, bodyv_ref, body_ref2;
Eigen::Vector3d rfootrpy_in1, rfootrpy_in2, rfootrpy_ref, rfootrpyv_ref, rfootrpy_ref2;
Eigen::Vector3d lfootrpy_in1, lfootrpy_in2, lfootrpy_ref, lfootrpyv_ref, lfootrpy_ref2;
Eigen::Vector3d COMacc_in1, COMacc_in2, COMacc_ref, COMaccv_ref, COMacc_ref2;

Eigen::Vector3d zmp_in1, zmp_in2, zmpxyz_ref, zmpv_ref, zmp_ref2;
Eigen::Vector3d dcm_in1, dcm_in2, dcmxyz_ref, dcmv_ref, dcm_ref2;

Eigen::Vector3d PelvisPos, body_thetax, LeftFootPosx,RightFootPosx;
Eigen::Vector3d F_L, F_R, M_L, M_R;
Eigen::Vector3d LeftFootRPY, RightFootRPY;

Eigen::Matrix<double, 2, 5 > zmp_mpc_ref, rfoot_mpc_ref, lfoot_mpc_ref, bodyangle_mpc_ref;
Eigen::Matrix<double, 3, 5 > comacc_mpc_ref;
Eigen::Matrix<double, 4, 1 > bodyangle_state;
Eigen::Matrix<double, 6, 1 > bodyangle_mpc;


int count_in_rt_loop;
int count_in_rt_ros;
int count_inteplotation;
int count_inteplotation_fast;
int t_int;
int n_t_int;
int n_t_int_fast;
double dtx;
double _mass;
double _j_ini;
double _Zsc;
double _ggg;
    
void nrt_gait_sub_operation(const sensor_msgs::JointState::ConstPtr &msg);
void rt_gait_sub_operation(const sensor_msgs::JointState::ConstPtr &msg);


ros::Subscriber nrt_mpc_gait_subscribe_;
ros::Subscriber gait_des_sub_;

ros::Publisher control_to_rtmpc_pub_;

//// lower-pass-filter///butterworthLPF1/////
butterworthLPF  butterworthLPF1,butterworthLPF2,butterworthLPF3,butterworthLPF4,butterworthLPF5,butterworthLPF6,
butterworthLPF7,butterworthLPF8,butterworthLPF9,butterworthLPF10,butterworthLPF11,butterworthLPF12,
butterworthLPF13,butterworthLPF14,butterworthLPF15,butterworthLPF16,butterworthLPF17,butterworthLPF18,
butterworthLPF19,butterworthLPF20,butterworthLPF21,butterworthLPF22,butterworthLPF23,
butterworthLPF24,butterworthLPF25,butterworthLPF26,butterworthLPF27,butterworthLPF28,butterworthLPF29,butterworthLPF30,
butterworthLPF31,butterworthLPF32,butterworthLPF33,butterworthLPF34,butterworthLPF35,butterworthLPF36,butterworthLPF37;
double f_sample_comx1;
double f_sample_comx2;
double fcutoff_comx1;
double fcutoff_comx2;
double fcutoff_comx3;
double fcutoff_comx4;
double fcutoff_comx5;
double fcutoff_comx51;
double dt_grf;

butterworthLPF  butterworthLPF41,butterworthLPF42,butterworthLPF43,butterworthLPF44,butterworthLPF45,butterworthLPF46,
butterworthLPF47,butterworthLPF48,butterworthLPF49,butterworthLPF50,butterworthLPF51,butterworthLPF52,
butterworthLPF53,butterworthLPF54,butterworthLPF55,butterworthLPF56,butterworthLPF57,butterworthLPF58,
butterworthLPF59,butterworthLPF60,butterworthLPF61,butterworthLPF62,butterworthLPF63,
butterworthLPF64,butterworthLPF65,butterworthLPF66,butterworthLPF67,butterworthLPF68,butterworthLPF69,butterworthLPF70,
butterworthLPF71,butterworthLPF72,butterworthLPF73,butterworthLPF74,butterworthLPF75,butterworthLPF76,butterworthLPF77,
butterworthLPF78,butterworthLPF79;

double pitch_angle_W;
int n_period;




//// state-estimate
////////////////////////////// for state estimation = learning from open source controller///////////////

ros::Publisher state_estimate_ekf_pub_;
Eigen::Matrix<double, 100,1> state_gait_ekf;    


// the leg kinematics is relative to body frame, which is the center of the robot
// following are some parameters that defines the transformation between IMU frame(b) and robot body frame(r)
Eigen::Vector3d p_br;
Eigen::Matrix3d R_br;
// for each leg, there is an offset between the body frame and the hip motor (fx, fy)
double leg_offset_x[4] = {};
double leg_offset_y[4] = {};
// for each leg, there is an offset between the body frame and the hip motor (fx, fy)
double motor_offset[4] = {};
double upper_leg_length[4] = {};
double lower_leg_length[4] = {};
std::vector<Eigen::VectorXd> rho_fix_list;
std::vector<Eigen::VectorXd> rho_opt_list;

// variables related to control
A1CtrlStates a1_ctrl_states;
// A1RobotControl _root_control;
A1BasicEKF go1_estimate;

// filters
MovingWindowFilter acc_x;
MovingWindowFilter acc_y;
MovingWindowFilter acc_z;
MovingWindowFilter gyro_x;
MovingWindowFilter gyro_y;
MovingWindowFilter gyro_z;
MovingWindowFilter quat_w;
MovingWindowFilter quat_x;
MovingWindowFilter quat_y;
MovingWindowFilter quat_z;

///Euler_integration;;;;
Eigen::Vector3d imu_acc_global;
Eigen::Vector3d imu_vel_global;
Eigen::Vector3d imu_pos_global;


void gait_pose_callback(unitree_legged_msgs::LowState RecvLowROS);
Eigen::Matrix<double, 100,1>  state_est_main_update(double dt);

void base_pos_fb_controler();
void base_acc_ff_controler();
void leg_kinematic();

double  FR_foot_desx, FR_foot_desy,FR_foot_desz;
double  FL_foot_desx, FL_foot_desy,FL_foot_desz;
int dynamic_count_period;

void config_set();
double com_rpy_max,com_rpy_min,com_pos_max,com_pos_min;
double uisng_current_jaco;
double using_ff;
double global_com_feedback;
double start_grf;
double using_grf_node;
int n_rt;
int debug_mode;
int fz_limit;
double fz_load_ratio;
double swing_leg_test;
double switch_support;
double using_rotz;
double using_ros_time;
double judge_early_contact;
double judge_later_contact;
double using_ekf;
double tracking_global_foot;
double enable_spring;
double FR_k_spring_hip, FL_k_spring_hip, RR_k_spring_hip, RL_k_spring_hip;
double FR_k_spring_thigh, FL_k_spring_thigh, RR_k_spring_thigh, RL_k_spring_thigh;
double FR_k_spring_calf, FL_k_spring_calf, RR_k_spring_calf, RL_k_spring_calf;
double test_stepping_in_place;
double dt_mpc_slow,dt_mpc_fast;
double tstep;
double fsr_contact;
Eigen::Vector4d foot_contact_flag;
Eigen::Vector4d foot_earlier_contact_flag,foot_earlier_contact_flag_old;
Eigen::Matrix<double,3,4>  foot_contact_position;
Eigen::Vector4d foot_later_contact_flag,foot_later_contact_flag_old;

MovingWindowFilter recent_contact_x_filter[4];
MovingWindowFilter recent_contact_y_filter[4];
MovingWindowFilter recent_contact_z_filter[4]; 
Eigen::Vector3d ground_angle;
double use_terrain_adapt;

Eigen::Vector4d contact_planning, contact_planning_pre;  ///desired foot contact status: FR,FL,RR,RL
Eigen::Matrix<double, 5, 4> contact_detect;   /////lasting 5 detected constact status: FR,FL,RR,RL