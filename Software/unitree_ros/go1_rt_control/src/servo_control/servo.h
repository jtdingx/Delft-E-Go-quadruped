/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
Edited by Jiatao Ding, email: jtdingx@gmail.com
************************************************************************/

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
#include "kinematics/Kinematics.h"
#include "Filter/butterworth_filter.h"
#include "Filter/butterworthLPF.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "whole_body_dynamics/dynmics_compute.h"




////////// for state estimation==learning from the open source///////////////
//// control parameters
#include "Robotpara/A1Params.h"
#include "Robotpara/A1CtrlStates.h"
// #include "A1Robotcontrol/A1RobotControl.h"
#include "A1EKF/A1BasicEKF.h"
// #include "A1legKinematics/A1Kinematics.h"
#include "utils/Utils.h"
#include "utils/filter.hpp"
#include "A1EKF/state_estimator.h"






using namespace std;
using namespace unitree_model;



int n_count;

double lastPos[12], percent, pos_cmd[12];
double stand_duration;
double time_programming;
int rt_frequency;

Kinematicclass Kine;

Dynamiccclass Dynam;


/////////////*******************8 robot state ******************************************///////////////////////
Eigen::Matrix<double,3,1> body_p_Homing, body_p_Homing_Retarget, body_r_homing, body_p_des, body_r_des;
Eigen::Matrix<double,3,1> FR_foot_des, FL_foot_des,RR_foot_des, RL_foot_des;
Eigen::Matrix<double,3,1> FR_foot_Homing, FL_foot_Homing,RR_foot_Homing, RL_foot_Homing;
Eigen::Matrix<double,3,1> FR_foot_Homing_retarget, FL_foot_Homing_retarget,RR_foot_Homing_retarget, RL_foot_Homing_retarget;
Eigen::Matrix<double,3,1> body_p_est, body_r_est;


Eigen::Matrix<double,3,1> FR_angle_des, FL_angle_des,RR_angle_des, RL_angle_des;
Eigen::Matrix<double,3,1> FR_angle_mea, FL_angle_mea,RR_angle_mea, RL_angle_mea;
Eigen::Matrix<double,3,1> FR_dq_mea, FL_dq_mea,RR_dq_mea, RL_dq_mea;

Eigen::Matrix<double,3,1> FR_foot_relative_des, FL_foot_relative_des,RR_foot_relative_des, RL_foot_relative_des;
Eigen::Matrix<double,3,1> FR_foot_relative_des_old, FL_foot_relative_des_old,RR_foot_relative_des_old, RL_foot_relative_des_old;
Eigen::Matrix<double,3,1> FR_foot_relative_mea, FL_foot_relative_mea,RR_foot_relative_mea, RL_foot_relative_mea;
Eigen::Matrix<double,3,1> FR_foot_relative_mea_old, FL_foot_relative_mea_old,RR_foot_relative_mea_old, RL_foot_relative_mea_old;

Eigen::Matrix<double,3,3> FR_Jaco, FL_Jaco,RR_Jaco, RL_Jaco;
Eigen::Matrix<double,3,3> FR_Jaco_est, FL_Jaco_est,RR_Jaco_est, RL_Jaco_est;

Eigen::Matrix<double,3,1> FR_v_relative, FL_v_relative,RR_v_relative, RL_v_relative;
Eigen::Matrix<double,3,1> FR_v_est_relative, FL_v_est_relative,RR_v_est_relative, RL_v_est_relative;

////state variables
Eigen::Vector3d root_pos;
Eigen::Quaterniond root_quat;
Eigen::Vector3d root_euler;
Eigen::Matrix3d root_rot_mat;
Eigen::Matrix3d root_rot_mat_z;
Eigen::Vector3d root_lin_vel;
Eigen::Vector3d root_ang_vel;
Eigen::Vector3d root_acc;
double rate, ratex, rate_stand_up;
Eigen::Vector3d q_ini;

/////////////////////////////////////////************* Force distribution **********************/////////////////////////////////////////
////// Force distribution 
Eigen::Matrix<double,6,1> F_sum;
Eigen::Matrix3d Momentum_sum;
Eigen::Vector3d kpp, kdp, kpw, kdw;

double rleg_com, lleg_com;
Eigen::Matrix<double,6,1> F_lr_predict;
bool FR_swing,FL_swing,RR_swing,RL_swing;

int bjx1;
int right_support;
Eigen::Matrix<double, 3,1>  FR_torque,FL_torque,RR_torque,RL_torque;
Eigen::Matrix<double, 12,1> Legs_torque;



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
double thetav_des[3];
double thetaacc_des[3];
double theta_des_pre[3];	
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
double com_des_ini[3];
double rfoot_des_ini[3];
double lfoot_des_ini[3];
double theta_des_ini[3];




Eigen::Vector3d L_com;
Eigen::Vector3d com_estkine;
Eigen::Vector3d cop_estkine;
Eigen::Vector3d theta_estkine;
Eigen::Vector3d theta_estkine_pre;
Eigen::Vector3d thetaa_estkine;
double Fr_estkine;
double Fl_estkine;
Eigen::Vector3d comv_estkine;
Eigen::Matrix<double, 15,1> dob_result;
double J_ini_xx_est,J_ini_yy_est;
Eigen::Vector3d thetav_estkine;



int count_in_mpc_max;

////===============================================================/////////////////
/////////////////// for fast_mpc: body inclination optimization //////////////

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
butterworthLPF24,butterworthLPF25,butterworthLPF26,butterworthLPF27,butterworthLPF28,butterworthLPF29,butterworthLPF30;
double f_sample_comx1;
double fcutoff_comx1;
double fcutoff_comx2;
double fcutoff_comx3;

double pitch_angle_W;
int n_period;


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
// A1Kinematics a1_kin;
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



void gait_pose_callback();
Eigen::Matrix<double, 100,1>  state_est_main_update(double dt);




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
int stand_count,stand_up_count,dynamic_count;
double rlleg_dis,com_rleg_dis, rleg_com_raw, rleg_com_raw1;
Eigen::Vector3d vec_foot_rl;
Eigen::Vector3d vec_com_rfoot;
Eigen::Matrix<double, 12,1> leg_position;
double nt_slow_mpc;
double body_r_Homing_dynamic[3];
double body_p_Homing_dynamic[3];
double dt_slow_mpc;
double z_c;
double tstep;
double using_hie_using;
Eigen::Vector3d rfoot_sensor, lfoot_sensor;