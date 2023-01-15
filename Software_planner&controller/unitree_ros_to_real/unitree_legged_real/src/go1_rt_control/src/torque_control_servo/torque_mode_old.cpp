/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "convert.h"
#include "go1_const.h"
#include "sensor_msgs/JointState.h"
#include <Eigen/Dense>
#include "torque_mode.h"
#include "/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/src/go1_rt_control/src/Robotpara/robot_const_para_config.h"
#include "geometry_msgs/Twist.h"

using namespace UNITREE_LEGGED_SDK;
float pi=3.1415926;

float qDes[12]={0};
float dqDes[12] = {0}; 
float dqDes_old[12] = {0}; 
int ctrl_estimation = 1000;

double base_offset_x = 0;
double base_offset_y = 0; 
double base_offset_z = 0;
double base_offset_roll = 0;
double base_offset_pitch =0;
double base_offset_yaw = 0;

double base_offset_x_old = 0;
double base_offset_y_old = 0; 
double base_offset_z_old = 0;
double base_offset_roll_old = 0;
double base_offset_pitch_old =0;
double base_offset_yaw_old = 0;



template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

double jointLinearInterpolation(double initPos, double targetPos, double rate, int j)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    // dqDes[j] = 0.8*dqDes_old[j] + 0.2*(p - qDes[j])/(1.0/ctrl_estimation);
    // dqDes_old[j] = dqDes[j];
    return p;
}


void keyboard_model_callback(const geometry_msgs::Twist::ConstPtr &msgIn) 
{
    // if((msgIn->linear.x == 1)&&(gait_status == STAND_STATUS))
    // {
    //     cmd_gait = STAND_UP;
    //     printf("===========Switch to STAND_UP state==========");
    // }
    // if((msgIn->linear.x ==2)&&(gait_status == STAND_UP_STATUS))
    // {
    //     cmd_gait = DYNAMIC;
    //     printf("===========Switch to DYNAMIC WALKING state==========");
    // } 
    printf("111");  
}


// void Base_offset_callback(const geometry_msgs::Twist::ConstPtr &msgIn) {
//         base_offset_x = msgIn->linear.x;
//         base_offset_y = msgIn->linear.y;
//         base_offset_z = msgIn->linear.z;
//         base_offset_roll = msgIn->angular.x;
//         base_offset_pitch = msgIn->angular.y;
//         base_offset_yaw = msgIn->angular.z;

// }

double clamp_func(double new_cmd, double old_cmd, double thresh)
{
    double ref_cmd = new_cmd;
    if(new_cmd - old_cmd > abs(thresh))
    {
       ref_cmd = old_cmd + abs(thresh);
    }
    else
    {
        if(new_cmd - old_cmd < -abs(thresh))
        {
            ref_cmd = old_cmd - abs(thresh);
        }
    }
    return ref_cmd;

}


template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(ctrl_estimation);

    ros::Publisher gait_data_pub; // for data_analysis
    ros::Publisher gait_data_pubx;  

    long motiontime=0;
    torque.setZero();

    TCmd SendLowLCM = {0};
    TState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;

    sensor_msgs::JointState joint2simulation, joint2simulationx;
    joint2simulation.position.resize(100);
    joint2simulationx.position.resize(100);    


    cmd_gait = STAND_INIT;
     
    bool initiated_flag = false;  // initiate need time
    
    
    //////// for trajectory generation
    int count = 0;   

    int rate_count = 0;
    float qInit[12]={0};

    ////first configure
    // float sin_mid_q[3] = {0.0, 1.2, -2.0};
    ////second configure
    float sin_mid_q[12] = {0.0, pi/4, -pi/2,0.0, pi/4, -pi/2,0.0, pi/4, -pi/2,0.0, pi/4, -pi/2};
    double sin_joint[12] = {0};

    stand_count = 0;
    stand_up_count = 0;
    dynamic_count = 0;

    //=========spring force forward compensation=============
    k_spring_calf = 3;
    k_p_rest_calf = -1.3;
    k_spring_thigh = 21; 
    k_p_rest_thigh = 0.75;
    k_spring_hip = 0;
    k_p_rest_hip = 0;

    torque_err.setZero();
    torque_err_intergration.setZero();
    Torque_ff_spring.setZero();
    Torque_ff_GRF.setZero();

    ///////let impedance control
    swing_kp.setZero();
    swing_kd.setZero();

    swing_kp(0,0) = 40;    swing_kp(1,1) = 40;    swing_kp(2,2) = 40;
    swing_kd(0,0) = 0.75;     swing_kd(1,1) = 0.75;     swing_kd(2,2) = 0.75;

    FF_enable = false; ///// Before setting True, make sure the spring is engaged!!!!!!!!!!!!
    


    /////////===== feedback PID joint controller===========
    torq_kp_calf = 9;
    torq_kd_calf = 0.31;
    torq_ki_calf = 0.01;

    torq_kp_thigh = 9; 
    torq_kd_thigh = 0.3; 
    torq_ki_thigh = 0.01;

    torq_kp_hip= 8;  
    torq_kd_hip= 0.3;  
    torq_ki_hip= 0.01; 
    
    /////// PID gain scaling: for support phase ////////////
    hip_kp_scale = 1.2;
    hip_kd_scale = 1.1;   
    thigh_kp_scale = 1.75;
    thigh_kd_scale = 1.5;     
    calf_kp_scale = 1.5;
    calf_kd_scale = 1.2;

    ////=========Gait mode generation=======///////////
    gait_mode = 102;   
    x_offset = 0.01;

    if (gait_mode ==101) ///pacing
    {
        y_offset = 0.75;
    }
    else
    {
        if (gait_mode ==102) ///troting
        {
            y_offset = 0;
        }
        else
        { 
            y_offset = 0.11;
        }        
    }


    //===desired joint angles
    FR_angle_des.setZero(); FL_angle_des.setZero(); RR_angle_des.setZero(); RL_angle_des.setZero(); 

    angle_des.setZero();

    //===measure angles
    FR_angle_mea.setZero(); FL_angle_mea.setZero(); RR_angle_mea.setZero(); RL_angle_mea.setZero();
    FR_dq_mea.setZero();    FL_dq_mea.setZero();    RR_dq_mea.setZero();    RL_dq_mea.setZero();

    /// Jacobian:
    FR_Jaco.setZero();
    FL_Jaco.setZero();
    RR_Jaco.setZero();
    RL_Jaco.setZero();

    /// Jacobian measured:
    FR_Jaco_est.setZero();
    FL_Jaco_est.setZero();
    RR_Jaco_est.setZero();
    RL_Jaco_est.setZero();  

    FR_v_relative.setZero(); 
    FL_v_relative.setZero();
    RR_v_relative.setZero(); 
    RL_v_relative.setZero();
    FR_v_est_relative.setZero(); 
    FL_v_est_relative.setZero();
    RR_v_est_relative.setZero(); 
    RL_v_est_relative.setZero();




    //=== desired foot location reatlive to body center;
    FR_foot_relative_des.setZero(); FL_foot_relative_des.setZero();
    RR_foot_relative_des.setZero(); RL_foot_relative_des.setZero();

    FR_foot_relative_des_old.setZero(); FL_foot_relative_des_old.setZero();
    RR_foot_relative_des_old.setZero(); RL_foot_relative_des_old.setZero();    

    //=== measured foot location reatlive to body center;

    FR_foot_relative_mea.setZero(); FL_foot_relative_mea.setZero();
    RR_foot_relative_mea.setZero(); RL_foot_relative_mea.setZero(); 
    FR_foot_relative_mea_old.setZero(); FL_foot_relative_mea_old.setZero();
    RR_foot_relative_mea_old.setZero(); RL_foot_relative_mea_old.setZero(); 


    //=== desired body posioin and rotation
    body_p_Homing.setZero();
    //=== body_p_Homing_Retarget.setZero();
    body_p_Homing_Retarget << 0.02,
                              0,
                           0.28;
    FR_foot_Homing.setZero();
    FL_foot_Homing.setZero();
    RR_foot_Homing.setZero();
    RL_foot_Homing.setZero();

    body_p_Homing_dynamic.setZero();

    body_r_homing.setZero();
    body_r_Homing_dynamic.setZero();
    body_p_des.setZero();
    body_r_des.setZero();
    FR_foot_des.setZero(); 
    FL_foot_des.setZero(); 
    RR_foot_des.setZero(); 
    RL_foot_des.setZero();
    
    //// measure global position /////
    FR_foot_mea.setZero(); FL_foot_mea.setZero();
    RR_foot_mea.setZero(); RL_foot_mea.setZero(); 
    body_p_est.setZero();
    body_r_est.setZero();

    leg_position.setZero();

    root_pos.setZero();
    root_quat.setIdentity();
    root_euler.setZero();
    root_euler_offset.setZero();
    root_rot_mat.setZero();
    root_rot_mat_z.setZero();
    root_lin_vel.setZero();
    root_ang_vel.setZero();
    root_acc.setZero();

    rate = 0;
    ratex = 0;
    rate_stand_up = 0;
    q_ini.setZero();
    
    nt_slow_mpc = 0.0;

    //=== Force distribution 
    F_sum.setZero();
    Momentum_sum << 2*0.0168352186, 2*0.0004636141, 2*0.0002367952,
                    2*0.0004636141, 2*0.0656071082, 2*3.6671e-05, 
                    2*0.0002367952, 2*3.6671e-05,   2*0.0742720659;

    vec_foot_rl.setZero();
    vec_com_rfoot.setZero();

    rlleg_dis = 0;
    com_rleg_dis = 0;
    rleg_com_raw = 0; 
    rleg_com_raw1 = 0;    

    rleg_com = 0; 
    lleg_com= 0;
    F_lr_predict.setZero();   
    Force_L_R.setZero();   
    bjx1 = 1;
    right_support =2;

    FR_torque_impedance.setZero(); 
    FL_torque_impedance.setZero(); 
    RR_torque_impedance.setZero(); 
    RL_torque_impedance.setZero();  
    Legs_torque.setZero();   
    FR_GRF.setZero();
    FL_GRF.setZero();
    RR_GRF.setZero();
    RL_GRF.setZero();

    FR_swing = false; 
    FL_swing = false;
    RR_swing = false;
    RL_swing = false;              

//     ////////////////////////// state estimation///////////////////////
//     support_flag = 0; /////left: 0; right: 1; double: 2

//     fz_double = gait::mass * gait::_g /2; 
//     fz_limit = gait::force_z_limt;
//     omega_sensor = sqrt(gait::_g/gait::Z_c);

        
//     support_pos_sensor[0] = 0; ///left support by default
//     support_pos_sensor[1] = gait::RobotParaClass_HALF_HIP_WIDTH; ///left support by default
//     support_pos_sensor[2] = 0; ///left support by default
//     com_sensor[0] = 0; /////assuming com is 10cm above the pelvis
//     com_sensor[1] = 0;
//     com_sensor[2] = (gait::Z_c);
//     com_sensor_hip[0] = 0; /////assuming com is 10cm above the pelvis
//     com_sensor_hip[1] = 0;
//     com_sensor_hip[2] = (gait::Z_c);	

//     rfoot_pose_sensor[0] = 0;
//     rfoot_pose_sensor[1] = -gait::RobotParaClass_HALF_HIP_WIDTH;
//     rfoot_pose_sensor[2] = 0;
//     lfoot_pose_sensor[0] = 0;
//     lfoot_pose_sensor[1] = gait::RobotParaClass_HALF_HIP_WIDTH;
//     lfoot_pose_sensor[2] = 0;


//     using_ft_sensor = false;       

//     for(int i = 0; i < 3; i++){

//         com_sensor_pre[i] = 0;
//         com_des[i] = 0;
//         com_des_pre[i] = 0;
//         comv_des[i] = 0;
//         coma_des[i] = 0;
//         rfoot_des[i] = 0;
//         lfoot_des[i] = 0;
//         theta_des[i] = 0;
//         theta_acc_des[i] = 0;
//         theta_des_pre[i] = 0;
//         rfoot_theta_des[i] = 0;
//         lfoot_theta_des[i] = 0;

//         comv_sensor[i] = 0;
//         coma_sensor[i] = 0;
//         zmp_sensor[i] = 0;
//         zmp_ref[i] = 0;
//         dcm_ref[i]= 0;
//         dcm_sensor[i] = 0;        

//     }

//     com_des[2] = gait::Z_c;

//     rfoot_des[1] = -gait::RobotParaClass_HALF_HIP_WIDTH;
//     lfoot_des[1] = gait::RobotParaClass_HALF_HIP_WIDTH;

//     comav_butterworth.setZero();
//     theta_default.setZero();


//     L_com.setZero();
//     com_estkine.setZero();
//     cop_estkine.setZero();
//     theta_estkine.setZero();
//     thetaa_estkine.setZero();
//     Fr_estkine = 0;
//     Fl_estkine = 0;
//     comv_estkine.setZero();
//     dob_result.setZero();
//     J_ini_xx_est = 0;
//     J_ini_yy_est = 0;
//     thetav_estkine.setZero();




//     ////////////////////// real-time mpc loop /////////////////////////	
//     count_in_mpc_max = 1000;

// 	joint2simulationx.position.resize(100);
// 	state_to_MPC.position.resize(25);
// 	state_feedback.setZero();
// 	slow_mpc_gait.setZero();   
// 	mpc_gait_flag = 0;
// 	mpc_gait_flag_old = 0;
// 	slow_mpc_gait_inte.setZero(); 
	
// 	fast_mpc_gait_flag = 0;
// 	fast_mpc_gait_flag_old = 0;
// 	fast_mpc_gait.setZero();	
	
	
// 	COM_in1.setZero(); 
// 	COM_in2.setZero(); 
// 	COMxyz_ref.setZero(); 
// 	COM_ref2.setZero(); 
// 	COM_in1(2) = COM_in2(2) = COMxyz_ref(2) = COM_ref2(2) = gait::RobotPara_Z_C; 

// 	COMv_ref.setZero(); 

// 	COMacc_in1.setZero();  
// 	COMacc_in2.setZero();  
// 	COMacc_ref.setZero();  
// 	COMaccv_ref.setZero();  
// 	COMacc_ref2.setZero();  

// 	FootL_in1.setZero();  
// 	FootL_in2.setZero();  
// 	FootL_ref.setZero();  
// 	FootL_ref2.setZero();  
// 	FootL_in1(1) = FootL_in2(1)= FootL_ref(1) = FootL_ref2(1) = gait::RobotParaClass_HALF_HIP_WIDTH; 
// 	FootLv_ref.setZero(); 

// 	FootR_in1.setZero();  
// 	FootR_in2.setZero();  
// 	FootR_ref.setZero();  
// 	FootR_in1(1) = FootR_in2(1)= FootR_ref(1) = FootR_ref2(1) = -gait::RobotParaClass_HALF_HIP_WIDTH; 
// 	FootRv_ref.setZero(); 

// 	zmp_in1.setZero(); zmp_in2.setZero(); zmpxyz_ref.setZero(); zmpv_ref.setZero(); zmp_ref2.setZero();
// 	dcm_in1.setZero(); dcm_in2.setZero(); dcmxyz_ref.setZero(); dcmv_ref.setZero(); dcm_ref2.setZero();

// 	body_in1.setZero(); body_in2.setZero(); body_ref.setZero(); bodyv_ref.setZero(); body_ref2;
// 	rfootrpy_in1.setZero(); rfootrpy_in2.setZero(); rfootrpy_ref.setZero(); rfootrpyv_ref.setZero(); rfootrpy_ref2.setZero();
// 	lfootrpy_in1.setZero(); lfootrpy_in2.setZero(); lfootrpy_ref.setZero(); lfootrpyv_ref.setZero(); lfootrpy_ref2.setZero();



// 	PelvisPos.setZero();  
// 	body_thetax.setZero();  
// 	LeftFootPosx.setZero(); 
// 	RightFootPosx.setZero();  
// 	dcmxyz_ref.setZero();  
// 	F_L.setZero();  
// 	F_R.setZero(); 
// 	M_L.setZero();  
// 	M_R.setZero(); 
// 	LeftFootRPY.setZero();  
// 	RightFootRPY.setZero(); 
	
// 	rpy_mpc_body.setZero();
// 	rpy_mpc_body(2) = COM_ref2(2);

// 	rfoot_inter.setZero();
// 	rfoot_inter(1) = FootR_ref2(1);

// 	lfoot_inter.setZero();
// 	lfoot_inter(1) = FootL_ref2(1);

// 	bodytheta_inter.setZero();

// 	rftheta_inter.setZero(); 
// 	lftheta_inter.setZero();

// 	zmp_inter.setZero(); 
// 	dcm_inter.setZero();

// 	zmp_mpc_ref.setZero(); rfoot_mpc_ref.setZero(); lfoot_mpc_ref.setZero(); bodyangle_mpc_ref.setZero(); comacc_mpc_ref.setZero();
// 	bodyangle_mpc.setZero();
// 	bodyangle_state.setZero();

// 	count_in_rt_loop = 0;
// 	count_in_rt_ros = 0;

// 	count_inteplotation = 0;
// 	count_inteplotation_fast = 0;
// 	t_int = 0;
// 	dtx = gait::t_program_cyclic;   
// 	n_t_int = (int) round(gait::dt_mpc_slow /dtx); 
// 	n_t_int_fast = (int) round(gait::dt_mpc_fast /dtx); 


// 	_mass = gait::mass;
// 	_j_ini = gait::J_ini;

// 	_Zsc = 0;
// 	_ggg = gait::g;	
	
// 	//// gait filter
//     f_sample_comx1 = 1/gait::t_program_cyclic;
// 	fcutoff_comx1 = 3;
// 	fcutoff_comx2 = 10;
// 	butterworthLPF1.init(f_sample_comx1,fcutoff_comx1);	
// 	butterworthLPF2.init(f_sample_comx1,fcutoff_comx1);	
// 	butterworthLPF3.init(f_sample_comx1,fcutoff_comx1);	
// 	butterworthLPF4.init(f_sample_comx1,fcutoff_comx1);	
// 	butterworthLPF5.init(f_sample_comx1,fcutoff_comx1);	
// 	butterworthLPF6.init(f_sample_comx1,fcutoff_comx1);	
// 	butterworthLPF7.init(f_sample_comx1,fcutoff_comx1);	
// 	butterworthLPF8.init(f_sample_comx1,fcutoff_comx1);	
// 	butterworthLPF9.init(f_sample_comx1,fcutoff_comx1);	
// 	butterworthLPF10.init(f_sample_comx1,fcutoff_comx1);	
// 	butterworthLPF11.init(f_sample_comx1,fcutoff_comx1);	
// 	butterworthLPF12.init(f_sample_comx1,fcutoff_comx2);	
// 	butterworthLPF13.init(f_sample_comx1,fcutoff_comx2);	
// 	butterworthLPF14.init(f_sample_comx1,fcutoff_comx2);	

	
// 	fcutoff_comx3 = 20;
// 	butterworthLPF15.init(f_sample_comx1,fcutoff_comx3);	
// 	butterworthLPF16.init(f_sample_comx1,fcutoff_comx3);	
// 	butterworthLPF17.init(f_sample_comx1,fcutoff_comx3);	
// 	butterworthLPF18.init(f_sample_comx1,fcutoff_comx3);	
// 	butterworthLPF19.init(f_sample_comx1,fcutoff_comx3);	
// 	butterworthLPF20.init(f_sample_comx1,fcutoff_comx3);
// 	butterworthLPF21.init(f_sample_comx1,fcutoff_comx3);	
// 	butterworthLPF22.init(f_sample_comx1,fcutoff_comx3);
//     butterworthLPF23.init(f_sample_comx1,fcutoff_comx3);    
// 	butterworthLPF24.init(f_sample_comx1,fcutoff_comx3);	
// 	butterworthLPF25.init(f_sample_comx1,fcutoff_comx3);	
// 	butterworthLPF26.init(f_sample_comx1,fcutoff_comx3);	
// 	butterworthLPF27.init(f_sample_comx1,fcutoff_comx3);	
// 	butterworthLPF28.init(f_sample_comx1,fcutoff_comx3);	
// /*	butterworthLPF29.init(f_sample_comx1,fcutoff_comx1);	
// 	butterworthLPF30.init(f_sample_comx1,fcutoff_comx1);*/	
    
    pitch_angle_W = 0;

    n_period = round(gait::t_period / gait::t_program_cyclic); 

    roslcm.SubscribeState();
    gait_data_pub = n.advertise<sensor_msgs::JointState>("go1_gait_data",10);
    gait_data_pubx = n.advertise<sensor_msgs::JointState>("go1_gait_datax",10);   

    // ros::Subscriber robot_mode_cmd = n.subscribe<geometry_msgs::Twist>("/Robot_mode", 10, keyboard_model_callback); //// for robot mode selection
    // ros::Subscriber base_offset_cmd = n.subscribe<geometry_msgs::Twist>("/Base_offset", 10, Base_offset_callback); //// for base offset modulation

    ros::Subscriber robot_mode_cmd = n.subscribe("/Robot_mode", 1, keyboard_model_callback); //// for robot mode selection
    // ros::Subscriber base_offset_cmd = n.subscribe("/Base_offset", 10, Base_offset_callback); //// for base offset modulation
    





    rt_frequency = ctrl_estimation; /// frequency of lower_level control
    time_programming = 1.0/rt_frequency;

    n_count = 0;
    stand_duration = 5; /// stand up: 2s
    


    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
    }

    while (ros::ok()){
        // printf("rt loop");


        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);
        // printf("FR_1 position: %f\n",  RecvLowROS.motorState[FR_1].q);
        // printf("FR force0: %f\n",  RecvLowROS.footForceEst[0]);
        // printf("FR force0: %f\n",  RecvLowROS.footForce[0]);
        // printf("FR force1: %f\n",  RecvLowROS.footForceEst[1]);
        // printf("FR force1: %f\n",  RecvLowROS.footForce[1]);

        count_in_rt_ros += 1;

        ////////////////////////// kinematic-based state estimation ///////////////////////////////////////////////
        // Forward kinematics: for kinematic-based state estimation //////////////////////////////////////////////
        FR_angle_mea(0,0) = RecvLowROS.motorState[FR_0].q;
        FR_angle_mea(1,0) = RecvLowROS.motorState[FR_1].q;
        FR_angle_mea(2,0) = RecvLowROS.motorState[FR_2].q;
        FL_angle_mea(0,0) = RecvLowROS.motorState[FL_0].q;
        FL_angle_mea(1,0) = RecvLowROS.motorState[FL_1].q;
        FL_angle_mea(2,0) = RecvLowROS.motorState[FL_2].q;
        RR_angle_mea(0,0) = RecvLowROS.motorState[RR_0].q;
        RR_angle_mea(1,0) = RecvLowROS.motorState[RR_1].q;
        RR_angle_mea(2,0) = RecvLowROS.motorState[RR_2].q;
        RL_angle_mea(0,0) = RecvLowROS.motorState[RL_0].q;
        RL_angle_mea(1,0) = RecvLowROS.motorState[RL_1].q;
        RL_angle_mea(2,0) = RecvLowROS.motorState[RL_2].q;

        FR_dq_mea(0,0) = RecvLowROS.motorState[0].dq;
        FR_dq_mea(1,0) = RecvLowROS.motorState[1].dq;
        FR_dq_mea(2,0) = RecvLowROS.motorState[2].dq;
        FL_dq_mea(0,0) = RecvLowROS.motorState[3].dq;
        FL_dq_mea(1,0) = RecvLowROS.motorState[4].dq;
        FL_dq_mea(2,0) = RecvLowROS.motorState[5].dq;
        RR_dq_mea(0,0) = RecvLowROS.motorState[6].dq;
        RR_dq_mea(1,0) = RecvLowROS.motorState[7].dq;
        RR_dq_mea(2,0) = RecvLowROS.motorState[8].dq;
        RL_dq_mea(0,0) = RecvLowROS.motorState[9].dq;
        RL_dq_mea(1,0) = RecvLowROS.motorState[10].dq;
        RL_dq_mea(2,0) = RecvLowROS.motorState[11].dq;


        // //// IMU quaternion
        // root_quat = Eigen::Quaterniond(RecvLowROS.imu.quaternion[0],
        //                                RecvLowROS.imu.quaternion[1],
        //                                RecvLowROS.imu.quaternion[2],
        //                                RecvLowROS.imu.quaternion[3]); 

        // // euler angle: roll pitch yaw
        // root_rot_mat = root_quat.toRotationMatrix();
        // root_euler = Utils::quat_to_euler(root_quat);

        // if(n_count>1001)
        // {
        //     body_r_est(0,0) = root_euler(0) - root_euler_offset(0);
        //     body_r_est(1,0) = root_euler(1) - root_euler_offset(1);
        //     body_r_est(2,0) = root_euler(2) - root_euler_offset(2);
        // }


        // double yaw_angle = body_r_est(2,0);
        // root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

        // /// global framework;
        // body_p_est.setZero();
        // body_r_est.setZero();
        // FR_foot_mea = Kine.Forward_kinematics_g(body_p_est, body_r_est,FR_angle_mea, 0);
        // FR_Jaco_est = Kine.Jacobian_kin;
        // FL_foot_mea = Kine.Forward_kinematics_g(body_p_est, body_r_est,FL_angle_mea, 1);
        // FL_Jaco_est = Kine.Jacobian_kin;
        // RR_foot_mea = Kine.Forward_kinematics_g(body_p_est, body_r_est,RR_angle_mea, 2);
        // RR_Jaco_est = Kine.Jacobian_kin;
        // RL_foot_mea = Kine.Forward_kinematics_g(body_p_est, body_r_est,RL_angle_mea, 3);
        // RL_Jaco_est = Kine.Jacobian_kin;
        // FR_foot_relative_mea = FR_foot_mea - body_p_est;
        // FL_foot_relative_mea = FL_foot_mea - body_p_est;
        // RR_foot_relative_mea = RR_foot_mea - body_p_est;
        // RL_foot_relative_mea = RL_foot_mea - body_p_est;
        
        // //// relative velocity in global framework: another way is to use differential of measured retlative position;
        // FR_v_est_relative = FR_Jaco_est * FR_dq_mea;
        // FL_v_est_relative = FL_Jaco_est * FL_dq_mea; 
        // RR_v_est_relative = RR_Jaco_est * RR_dq_mea;
        // RL_v_est_relative = RL_Jaco_est * RL_dq_mea; 
        
        // // if (count_in_rt_ros > 1)
        // // {
        // //     FR_v_est_relative = (FR_foot_relative_mea - FR_foot_relative_mea_old)/dtx;
        // //     FL_v_est_relative = (FL_foot_relative_mea - FL_foot_relative_mea_old)/dtx;
        // //     RR_v_est_relative = (RR_foot_relative_mea - RR_foot_relative_mea_old)/dtx;
        // //     RL_v_est_relative = (RL_foot_relative_mea - RL_foot_relative_mea_old)/dtx;            
        // // }

        // ============== Key board controller ======================
        base_offset_x = clamp_func(base_offset_x,base_offset_x_old, 0.001);
        base_offset_y = clamp_func(base_offset_y,base_offset_y_old, 0.001);
        base_offset_z = clamp_func(base_offset_z,base_offset_z_old, 0.001);        
        base_offset_pitch = clamp_func(base_offset_pitch,base_offset_pitch_old, 0.001);
        base_offset_roll = clamp_func(base_offset_roll,base_offset_roll_old, 0.001);
        base_offset_yaw = clamp_func(base_offset_yaw,base_offset_yaw_old, 0.001);



        // //////////////////// gait control loop/////////////////////////////////////////////////////
        
        if(initiated_flag == true){
            motiontime++;            
            if( motiontime >= 0){
                
                //gait status switch begin
                switch (cmd_gait){
                    case STAND_INIT:
                        gait_status = STAND_INIT_STATUS;
                        ////////// joint reference///////////////////////
                        // recording initial position
                        if( motiontime >= 0 && motiontime < 500){
                            for(int j=0; j<12;j++)
                            {
                                qInit[j] = RecvLowROS.motorState[j].q;
                                qDes[j] = qInit[j];
                            }
                        }
                        //// move to the homing pose
                        if( motiontime >= 500 && motiontime <= 1500){
                            // printf("%f %f %f\n", );
                            rate_count++;
                            rate = pow(rate_count/1000.0,2); 
                            for(int j=0; j<12;j++)
                            {
                                qDes[j] = jointLinearInterpolation(qInit[j], sin_mid_q[j], rate, 0);
                            }
                        }
                        /// *****************homing pose computing*******************////
                        FR_angle_des(0,0) = qDes[0];
                        FR_angle_des(1,0) = qDes[1];
                        FR_angle_des(2,0) = qDes[2];
                        FL_angle_des(0,0) = qDes[3];
                        FL_angle_des(1,0) = qDes[4];
                        FL_angle_des(2,0) = qDes[5];
                        RR_angle_des(0,0) = qDes[6];
                        RR_angle_des(1,0) = qDes[7];
                        RR_angle_des(2,0) = qDes[8];
                        RL_angle_des(0,0) = qDes[9];
                        RL_angle_des(1,0) = qDes[10];
                        RL_angle_des(2,0) = qDes[11];   

                        FR_foot_des = FR_foot_relative_des = Kine.Forward_kinematics(FR_angle_des, 0);
                        FL_foot_des = FL_foot_relative_des = Kine.Forward_kinematics(FL_angle_des, 1);
                        RR_foot_des = RR_foot_relative_des = Kine.Forward_kinematics(RR_angle_des, 2);
                        RL_foot_des = RL_foot_relative_des = Kine.Forward_kinematics(RL_angle_des, 3);  
                        
                        /// computing the homing body position:
                        body_p_Homing(2,0) = body_p_des(2,0) = - (FR_foot_relative_des(2,0) + FL_foot_relative_des(2,0) + RR_foot_relative_des(2,0) + RL_foot_relative_des(2,0))/4;                   
                        FR_foot_des(2) = 0;
                        FL_foot_des(2) = 0;
                        RR_foot_des(2) = 0;
                        RL_foot_des(2) = 0;

                        FR_foot_Homing = FR_foot_des;
                        FL_foot_Homing = FL_foot_des;
                        RR_foot_Homing = RR_foot_des;
                        RL_foot_Homing = RL_foot_des;
                        //cout<< "xxxx!!!!!!!!!!!!!!!!!!!!xxxxxxxx"<<endl;      


                        if (motiontime==1500)
                        {
                            cmd_gait = STAND;
                            printf("===== switch to STAND HOMING POSE =======");
                        }
                        break;
                    case STAND:
                        gait_status = STAND_STATUS;
                        stand_count++;
                        
                        // // // ////////// for parameter tuning 
                        // // sin_joint[0] = sin_joint[6] = 0.1 * sin(3*M_PI*stand_count/1000.0);
                        // // sin_joint[3] = sin_joint[9] = -0.1 * sin(3*M_PI*stand_count/1000.0);
                        
                        // // sin_joint[1] = sin_joint[4] = 0.4 * sin(3*M_PI*stand_count/1000.0);
                        // // sin_joint[7] = sin_joint[10] = 0.4 * sin(3*M_PI*stand_count/1000.0);
                        
                        // // sin_joint[2] = sin_joint[5] = sin_joint[8] = sin_joint[11] = -0.4 * sin(3*M_PI*stand_count/1000.0);
                        
                        // // // //// FR leg 
                        // // // sin_joint[0] = 0.1 * sin(3*M_PI*stand_count/1000.0);
                        // // // sin_joint[1] = 0.4 * sin(3*M_PI*stand_count/1000.0);
                        // // // sin_joint[2] = -0.4 * sin(3*M_PI*stand_count/1000.0);                       
                        
                        // // for(int j=0; j<12;j++)
                        // // {
                        // //     qDes[j] = sin_mid_q[j] + sin_joint[j];
                        // // }
                        // // FR_angle_des(0,0) = qDes[0];
                        // // FR_angle_des(1,0) = qDes[1];
                        // // FR_angle_des(2,0) = qDes[2];
                        // // FL_angle_des(0,0) = qDes[3];
                        // // FL_angle_des(1,0) = qDes[4];
                        // // FL_angle_des(2,0) = qDes[5];
                        // // RR_angle_des(0,0) = qDes[6];
                        // // RR_angle_des(1,0) = qDes[7];
                        // // RR_angle_des(2,0) = qDes[8];
                        // // RL_angle_des(0,0) = qDes[9];
                        // // RL_angle_des(1,0) = qDes[10];
                        // // RL_angle_des(2,0) = qDes[11];                         
                        // // FR_foot_des = Kine.Forward_kinematics_g(body_p_des, body_r_des,FR_angle_des, 0);
                        // // FR_Jaco = Kine.Jacobian_kin;
                        // // FL_foot_des = Kine.Forward_kinematics_g(body_p_des, body_r_des,FL_angle_des, 1);
                        // // FL_Jaco = Kine.Jacobian_kin;
                        // // RR_foot_des = Kine.Forward_kinematics_g(body_p_des, body_r_des,RR_angle_des, 2);
                        // // RR_Jaco = Kine.Jacobian_kin;
                        // // RL_foot_des = Kine.Forward_kinematics_g(body_p_des, body_r_des,RL_angle_des, 3);
                        // // RL_Jaco = Kine.Jacobian_kin;

                        
                        // /////// for normal walking: preparing for stand up
                        // ratex = pow(stand_count/100.0,2); 
                        // if(ratex<=1000)
                        // {
                        //     for(int j=0; j<3;j++)
                        //     {
                        //         body_p_des[j] = jointLinearInterpolation(body_p_Homing(j,0), body_p_Homing_Retarget(j,0), ratex, 0);
                                
    

                        //     }
                        //     body_p_Homing = body_p_des;
                        //     body_r_homing = body_r_des;
                        // }
                        // else
                        // {   

                        //     body_p_des[0] = body_p_Homing[0] + base_offset_x;
                        //     body_p_des[1] = body_p_Homing[1] + base_offset_y;
                        //     body_p_des[2] = body_p_Homing[2] + base_offset_z;
                        //     body_r_des[0] = body_r_homing[0] + base_offset_roll;
                        //     body_r_des[1] = body_r_homing[1] + base_offset_pitch;
                        //     body_r_des[2] = body_r_homing[2] + base_offset_yaw;                            
                        // }

                        // ///cout << "xyyyy"<<endl;
                        // q_ini = FR_angle_des;
                        // FR_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,FR_foot_des,q_ini,0);
                        // FR_Jaco = Kine.Jacobian_kin;

                        // q_ini = FL_angle_des;
                        // FL_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,FL_foot_des,q_ini,1);
                        // FL_Jaco = Kine.Jacobian_kin;

                        // q_ini = RR_angle_des;
                        // RR_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,RR_foot_des,q_ini,2);
                        // RR_Jaco = Kine.Jacobian_kin;

                        // q_ini = RL_angle_des;
                        // RL_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,RL_foot_des,q_ini,3);  
                        // RL_Jaco = Kine.Jacobian_kin;                          

                        // qDes[0] = FR_angle_des(0,0);
                        // qDes[1] = FR_angle_des(1,0);
                        // qDes[2] = FR_angle_des(2,0);
                        // qDes[3] = FL_angle_des(0,0);
                        // qDes[4] = FL_angle_des(1,0);
                        // qDes[5] = FL_angle_des(2,0);
                        // qDes[6] = RR_angle_des(0,0);
                        // qDes[7] = RR_angle_des(1,0);
                        // qDes[8] = RR_angle_des(2,0);
                        // qDes[9] = RL_angle_des(0,0);
                        // qDes[10] = RL_angle_des(1,0);
                        // qDes[11] = RL_angle_des(2,0); 


                        break;
                    case STAND_UP:
                        gait_status = STAND_UP_STATUS;
                        stand_up_count++;
                        // if(stand_up_count==1)
                        // {
                        //     printf(" The robot is standing up by GRF compensation");
                        // }
                        // // ////========= key board control for homing posing modulation ========= ///////////////////
                        // body_p_Homing_dynamic[0] = body_p_des[0] = body_p_Homing[0] + base_offset_x;
                        // body_p_Homing_dynamic[1] = body_p_des[1] = body_p_Homing[1] + base_offset_y;
                        // body_p_Homing_dynamic[2] = body_p_des[2] = body_p_Homing[2] + base_offset_z;
                        // body_r_Homing_dynamic[0] = body_r_des[0] = body_r_homing[0] + base_offset_roll;
                        // body_r_Homing_dynamic[1] = body_r_des[1] = body_r_homing[1] + base_offset_pitch;
                        // body_r_Homing_dynamic[2] = body_r_des[2] = body_r_homing[2] + base_offset_yaw; 

                        // q_ini = FR_angle_des;
                        // FR_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,FR_foot_des,q_ini,0);
                        // FR_Jaco = Kine.Jacobian_kin;

                        // q_ini = FL_angle_des;
                        // FL_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,FL_foot_des,q_ini,1);
                        // FL_Jaco = Kine.Jacobian_kin;

                        // q_ini = RR_angle_des;
                        // RR_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,RR_foot_des,q_ini,2);
                        // RR_Jaco = Kine.Jacobian_kin;

                        // q_ini = RL_angle_des;
                        // RL_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,RL_foot_des,q_ini,3);  
                        // RL_Jaco = Kine.Jacobian_kin;                          

                        // qDes[0] = FR_angle_des(0,0);
                        // qDes[1] = FR_angle_des(1,0);
                        // qDes[2] = FR_angle_des(2,0);
                        // qDes[3] = FL_angle_des(0,0);
                        // qDes[4] = FL_angle_des(1,0);
                        // qDes[5] = FL_angle_des(2,0);
                        // qDes[6] = RR_angle_des(0,0);
                        // qDes[7] = RR_angle_des(1,0);
                        // qDes[8] = RR_angle_des(2,0);
                        // qDes[9] = RL_angle_des(0,0);
                        // qDes[10] = RL_angle_des(1,0);
                        // qDes[11] = RL_angle_des(2,0); 


                        // ///////// add GRF compensation=========///////
                        // F_sum(3) = gait::mass * gait::_g;
                        // FR_GRF = FL_GRF = F_sum.block<3,1>(0,0) / 2.0 * (body_p_des[0] - (RR_foot_des[0] + RL_foot_des[0])/2)/((FR_foot_des[0] + FL_foot_des[0])/2 - (RR_foot_des[0] + RL_foot_des[0])/2);
                        // RR_GRF = RL_GRF = F_sum.block<3,1>(0,0) / 2.0  - FR_GRF;

                        // Torque_ff_GRF.block<3,1>(0,0) = - FR_Jaco.transpose() *  FR_GRF;
                        // Torque_ff_GRF.block<3,1>(3,0) = - FL_Jaco.transpose() *  FL_GRF;
                        // Torque_ff_GRF.block<3,1>(6,0) = - RR_Jaco.transpose() *  RR_GRF;
                        // Torque_ff_GRF.block<3,1>(9,0) = - RL_Jaco.transpose() *  RL_GRF;                        


                        // rate_stand_up = pow(stand_up_count/1000.0,2); 
                        // for(int j=0; j<12;j++)
                        // {
                        //     Torque_ff_GRF[j] = jointLinearInterpolation(0, Torque_ff_GRF(j,0), rate_stand_up, 0);
                        // }

                        // FR_swing = false; 
                        // FL_swing = false;
                        // RR_swing = false;
                        // RL_swing = false; 


                        break;                        
                    case DYNAMIC:
                        gait_status = DYNAMIC_STATUS;
                        dynamic_count++;
                        if(dynamic_count==1)
                        {
                            printf("The robot is performing dynamic walking");
                        }                        
                        // ////////dyanmic walking test/////////////////////////
                        // // body_p_des[0] = body_p_Homing_dynamic[0] + 0.0 * sin(2*M_PI*stand_count/1000.0);
                        // // body_p_des[1] = body_p_Homing_dynamic[1] + 0.0 * sin(2*M_PI*stand_count/1000.0);
                        // // body_p_des[2] = body_p_Homing_dynamic[2] + 0.1 * sin(2*M_PI*stand_count/1000.0);
                        // // body_r_des[0] = body_r_Homing_dynamic[0] + 0.0 * sin(2*M_PI*stand_count/1000.0);
                        // // body_r_des[1] = body_r_Homing_dynamic[1] + 0.0 * sin(2*M_PI*stand_count/1000.0);
                        // // body_r_des[2] = body_r_Homing_dynamic[2] + 0.0 * sin(2*M_PI*stand_count/1000.0); 

                        // q_ini = FR_angle_des;
                        // FR_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,FR_foot_des,q_ini,0);
                        // FR_Jaco = Kine.Jacobian_kin;

                        // q_ini = FL_angle_des;
                        // FL_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,FL_foot_des,q_ini,1);
                        // FL_Jaco = Kine.Jacobian_kin;

                        // q_ini = RR_angle_des;
                        // RR_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,RR_foot_des,q_ini,2);
                        // RR_Jaco = Kine.Jacobian_kin;

                        // q_ini = RL_angle_des;
                        // RL_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,RL_foot_des,q_ini,3);  
                        // RL_Jaco = Kine.Jacobian_kin;                          

                        // qDes[0] = FR_angle_des(0,0);
                        // qDes[1] = FR_angle_des(1,0);
                        // qDes[2] = FR_angle_des(2,0);
                        // qDes[3] = FL_angle_des(0,0);
                        // qDes[4] = FL_angle_des(1,0);
                        // qDes[5] = FL_angle_des(2,0);
                        // qDes[6] = RR_angle_des(0,0);
                        // qDes[7] = RR_angle_des(1,0);
                        // qDes[8] = RR_angle_des(2,0);
                        // qDes[9] = RL_angle_des(0,0);
                        // qDes[10] = RL_angle_des(1,0);
                        // qDes[11] = RL_angle_des(2,0); 

                        


                        // nt_slow_mpc = (( dynamic_count % n_t_int )* dtx) / (gait::dt_mpc_slow);

                        // F_sum(0) = gait::mass * coma_des[0];
                        // F_sum(1) = gait::mass * coma_des[1];
                        // F_sum(2) = gait::mass * gait::_g + gait::mass * coma_des[2];
                        
                        // ///////==== momentum check !!!!!!!!!!!!!!!!!!!!!!!! =================
                        // F_sum(3,0) = Momentum_sum(0,0) * theta_acc_des[0] + Momentum_sum(0,1) * theta_acc_des[1] + Momentum_sum(0,2) * theta_acc_des[2];
                        // F_sum(4,0) = Momentum_sum(1,0) * theta_acc_des[0] + Momentum_sum(1,1) * theta_acc_des[1] + Momentum_sum(1,2) * theta_acc_des[2];
                        // F_sum(5,0) = Momentum_sum(2,0) * theta_acc_des[0] + Momentum_sum(2,1) * theta_acc_des[1] + Momentum_sum(2,2) * theta_acc_des[2];

                        // vec_foot_rl << lfoot_des[0] - rfoot_des[0],
                        //             lfoot_des[1] - rfoot_des[1],
                        //             lfoot_des[2] - rfoot_des[2];

                        // vec_com_rfoot << com_des[0] - rfoot_des[0],
                        //             com_des[1] - rfoot_des[1],
                        //             com_des[2] - rfoot_des[2];                          

                        // rlleg_dis = sqrt(pow(vec_foot_rl[0], 2) + pow(vec_foot_rl[1], 2) + pow(vec_foot_rl[2], 2));
                        // com_rleg_dis = vec_foot_rl[0]*vec_com_rfoot[0] + vec_foot_rl[1]*vec_com_rfoot[1] + vec_foot_rl[2]*vec_com_rfoot[2];
                        // rleg_com_raw = com_rleg_dis /rlleg_dis;
                        // rleg_com_raw1 = std::min(rleg_com_raw,1.0);
                        // rleg_com = std::max(rleg_com_raw1,0.0);

                        // lleg_com = 1 - rleg_com;  

                        // if(right_support == 0) ////left support
                        // {
                        //     F_lr_predict(0) = F_sum(0);
                        //     F_lr_predict(1) = F_sum(1);
                        //     F_lr_predict(2) = F_sum(2);
                        //     F_lr_predict(3) = 0;
                        //     F_lr_predict(4) = 0;
                        //     F_lr_predict(5) = 0;
                        //     switch (gait_mode)
                        //         {
                        //         case 101:  ////biped walking
                        //             FR_swing = true;
                        //             RR_swing = true;
                        //             FL_swing = false;
                        //             RL_swing = false;
                        //             break;
                        //         case 102:  ///troting
                        //             FR_swing = false;
                        //             RL_swing = false;
                        //             FL_swing = true;
                        //             RR_swing = true;

                        //             break;
                        //         case 103:  ///gallop: alter the  x-y direction
                        //             FR_swing = true;
                        //             FL_swing = true;
                        //             RR_swing = false;
                        //             RL_swing = false;                    

                        //             break;            
                        //         default:
                        //             break;
                        //         } 
                        // }
                        // else
                        // {
                        //     if (right_support == 1)
                        //     {
                        //         F_lr_predict(0) = 0;
                        //         F_lr_predict(1) = 0;
                        //         F_lr_predict(2) = 0;
                        //         F_lr_predict(3) = F_sum(0);
                        //         F_lr_predict(4) = F_sum(1);
                        //         F_lr_predict(5) = F_sum(2);
                        //         switch (gait_mode)
                        //             {
                        //             case 101:  ////biped walking
                        //                 FR_swing = false;
                        //                 RR_swing = false;
                        //                 FL_swing = true;
                        //                 RL_swing = true;
                        //                 break;
                        //             case 102:  ///troting
                        //                 FR_swing = true;
                        //                 RL_swing = true;
                        //                 FL_swing = false;
                        //                 RR_swing = false;                            

                        //                 break;
                        //             case 103:  ///gallop: alter the  x-y direction
                        //                 FR_swing = false;
                        //                 FL_swing = false;
                        //                 RR_swing = true;
                        //                 RL_swing = true;                    

                        //                 break;            
                        //             default:
                        //                 break;
                        //             }
                        //     }
                        //     else
                        //     {
                        //         F_lr_predict(0) = F_sum(0) * rleg_com;
                        //         F_lr_predict(3) = F_sum(0) - F_lr_predict(0);
                        //         F_lr_predict(1) = F_sum(1) * rleg_com;
                        //         F_lr_predict(4) = F_sum(1) - F_lr_predict(1);
                        //         F_lr_predict(2) = F_sum(2) * rleg_com;
                        //         F_lr_predict(5) = F_sum(2) - F_lr_predict(2);

                        //         FR_swing = false;
                        //         FL_swing = false;
                        //         RR_swing = false;
                        //         RL_swing = false;  
                        //     }                
                            
                        // }  


                        // ////force:  left-right-leg   
                        // for(int j=0; j<6; j++)
                        // {
                        //     //Force_L_R(j)= slow_mpc_gait(15+j) + nt_slow_mpc * (F_lr_predict(j) - slow_mpc_gait(15+j));  //continuous force profile
                        //     //Force_L_R(j)= slow_mpc_gait(15+j);  //discontinuous force profile
                        //     Force_L_R(j) = F_lr_predict(j);
                        // }     

                        // leg_position.block<3,1>(0,0) = FR_foot_des;
                        // leg_position.block<3,1>(3,0) = FL_foot_des; 
                        // leg_position.block<3,1>(6,0) = RR_foot_des;
                        // leg_position.block<3,1>(9,0) = RL_foot_des; 
                        // Dynam.force_distribution(body_p_des,leg_position, Force_L_R, gait_mode, y_offset,rfoot_des,lfoot_des);

                        // // cout<<"right_support:"<<right_support<<endl;
                        // Dynam.force_opt(body_p_des,FR_foot_des, FL_foot_des, RR_foot_des, RL_foot_des,
                        //                 F_sum, gait_mode, right_support, y_offset);
           
                        // Torque_ff_GRF.block<3,1>(0,0) = - FR_Jaco.transpose() *  Dynam.F_leg_ref.col(0);
                        // Torque_ff_GRF.block<3,1>(3,0) = - FL_Jaco.transpose() *  Dynam.F_leg_ref.col(1);
                        // Torque_ff_GRF.block<3,1>(6,0) = - RR_Jaco.transpose() *  Dynam.F_leg_ref.col(2);
                        // Torque_ff_GRF.block<3,1>(9,0) = - RL_Jaco.transpose() *  Dynam.F_leg_ref.col(3);  

                        break;
                    default:
                         break;
                }


                

                ////======= torque controller: Joint space + Catesian Space ========////////
                //===== joint space: pid feedback + softplus feedfoward (when enable spring)====////////
                for(int j=0; j<12;j++)
                {
                    if(j % 3 == 0)
                    {
                        if(qDes[j]<=go1_Hip_min)
                        {
                            qDes[j]=go1_Hip_min;
                        }
                        else if (qDes[j]>=go1_Hip_max)
                        {
                            qDes[j]=go1_Hip_max;
                        }

                        // if(j /3 ==0)  //FR
                        // {
                        //     //// support leg
                        //     if(!FR_swing)
                        //     {
                        //         torq_kp_hip = 8 * hip_kp_scale;
                        //         torq_kd_hip = 0.3 * hip_kd_scale;
                        //         torq_ki_hip = 0.01; 
                        //     }  
                        // }
                        // if(j /3 ==1)  //FL
                        // {
                        //     //// support leg
                        //     if(!FL_swing)
                        //     {
                        //         torq_kp_hip = 8 * hip_kp_scale;
                        //         torq_kd_hip = 0.3 * hip_kd_scale;
                        //         torq_ki_hip = 0.01; 
                        //     }  
                        // }                                                 
                         
                        // if(j /3 ==2)  //RR
                        // {
                        //     //// support leg
                        //     if(!RR_swing)
                        //     {
                        //         torq_kp_hip = 8 * hip_kp_scale;
                        //         torq_kd_hip = 0.3 * hip_kd_scale;
                        //         torq_ki_hip = 0.01; 
                        //     }  
                        // }
                        // if(j /3 ==3)  //RL
                        // {
                        //     //// support leg
                        //     if(!RL_swing)
                        //     {
                        //         torq_kp_hip = 8 * hip_kp_scale;
                        //         torq_kd_hip = 0.3 * hip_kd_scale;
                        //         torq_ki_hip = 0.01; 
                        //     }  
                        // }                                                  


                        //// hip joint tracking
                        torque_err.block<torque_err_row-1,1>(0,j) = torque_err.block<torque_err_row-1,1>(1,j);

                        torque_err(torque_err_row-1,j) = qDes[j] - RecvLowROS.motorState[j].q;

                        torque_err_intergration.setZero();
                        for(int ij=0; ij<torque_err_row; ij++)
                        {
                        torque_err_intergration(j,0) += torque_err(ij,j);
                        } 
                        
                        torque(j,0) = (qDes[j] - RecvLowROS.motorState[j].q)*torq_kp_hip + (0 - RecvLowROS.motorState[j].dq)*torq_kd_hip + torque_err_intergration(j,0)*torq_ki_hip;
                        
                        //// ff control
                        ///// tuning the parameters carefully
                        if(qDes[j]<=k_p_rest_hip)
                        {
                            Torque_ff_spring(j,0) = k_spring_hip * (qDes[j] - (k_p_rest_hip));
                        }
                        else
                        {
                            Torque_ff_spring(j,0) = 0;
                        }
        
                        if(FF_enable)
                        {
                            torque(j,0) += Torque_ff_spring(j,0);
                        }  
                        
                        if((j / 3 ==0) ||(j / 3 ==2))
                        {
                            torque(j,0) += (-0.65f);
                        }
                        else
                        {
                            torque(j,0) += (0.65f);
                        }

                    }
                    else
                    {
                        if(j % 3 ==1)
                        {
                            if(qDes[j]<=go1_Thigh_min)
                            {
                                qDes[j]=go1_Thigh_min;
                            }
                            else if (qDes[j]>=go1_Thigh_max)
                            {
                                qDes[j]=go1_Thigh_max;
                            }


                            //// thigh joint tracking
                            if(j /3 ==0)
                            {
                                //// too soft: only for swing leg
                                // torq_kp_thigh = 7;
                                // torq_kd_thigh = 0.3;
                                // torq_ki_thigh = 0.01;  
                                // k_spring_thigh = 11;                              
                                // k_p_rest_thigh = 0.49;
                                
                                // if(!FR_swing)
                                // {
                                // //// support leg
                                // torq_kp_thigh = 7 * thigh_kp_scale;
                                // torq_kd_thigh = 0.3 * thigh_kd_scale;
                                // }
                                torq_ki_thigh = 0.01;  
                                k_spring_thigh = 11;                              
                                // k_p_rest_thigh = 0.49;
                                k_p_rest_thigh = 0.52;
                                
                                
                            }
                            
                            if(j /3 ==1)
                            {
                                // //// too soft: only for swing leg
                                // torq_kp_thigh = 8;
                                // torq_kd_thigh = 0.3;
                                // torq_ki_thigh = 0.01;                                  
                                // k_spring_thigh = 15;
                                // k_p_rest_thigh = 0.41;
                                // if(!FL_swing)
                                // {
                                // //// support leg
                                // torq_kp_thigh = 8* thigh_kp_scale;
                                // torq_kd_thigh = 0.3* thigh_kd_scale;
                                // } 
                                torq_ki_thigh = 0.01;                                 
                                k_spring_thigh = 15;
                                // k_p_rest_thigh = 0.41;
                                k_p_rest_thigh = 0.37;
                                                             
                            }

                            if(j /3 ==2)
                            {
                                // //// too soft: only for swing leg
                                // torq_kp_thigh = 8;
                                // torq_kd_thigh = 0.3;
                                // torq_ki_thigh = 0.01;                                 
                                // k_spring_thigh = 19;
                                // k_p_rest_thigh = 0.45;
                                // if(!RR_swing)
                                // {
                                // //// support leg
                                // thigh_kp_scale = 2;
                                // torq_kp_thigh = 8* thigh_kp_scale;
                                // torq_kd_thigh = 0.3* thigh_kd_scale;
                                // }
                                torq_ki_thigh = 0.01;                                 
                                k_spring_thigh = 19;
                                //k_p_rest_thigh = 0.45;
                                k_p_rest_thigh = 0.33;

                            } 
                            if(j /3 ==3)
                            {
                                // //// too soft: only for swing leg
                                // torq_kp_thigh = 9;
                                // torq_kd_thigh = 0.3;
                                // torq_ki_thigh = 0.005;                                                               
                                // k_spring_thigh = 16;
                                // // k_p_rest_thigh = 0.37;
                                // k_p_rest_thigh = 0.82;
                                // if(!RL_swing)
                                // {
                                // //// support leg
                                // thigh_kp_scale = 2;
                                // torq_kp_thigh = 9* thigh_kp_scale;
                                // torq_kd_thigh = 0.3* thigh_kd_scale;
                                // }
                                torq_ki_thigh = 0.01;                                 
                                k_spring_thigh = 16;
                                //k_p_rest_thigh = 0.82;  
                                k_p_rest_thigh = 0.83;
                            }  

                            //// fb control
                            torque_err.block<torque_err_row-1,1>(0,j) = torque_err.block<torque_err_row-1,1>(1,j);

                            torque_err(torque_err_row-1,j) = qDes[j] - RecvLowROS.motorState[j].q;

                            torque_err_intergration.setZero();
                            for(int ij=0; ij<torque_err_row; ij++)
                            {
                            torque_err_intergration(j,0) += torque_err(ij,j);
                            } 
                            
                            torque(j,0) = (qDes[j] - RecvLowROS.motorState[j].q)*torq_kp_thigh + (0 - RecvLowROS.motorState[j].dq)*torq_kd_thigh + torque_err_intergration(j,0)*torq_ki_thigh;
                            
                            //// ff control
                            // if(qDes[j]>=k_p_rest_thigh)
                            // {
                            //     Torque_ff_spring(j,0) = k_spring_thigh * (qDes[j] - (k_p_rest_thigh));
                            // }
                            // else
                            // {
                            //     Torque_ff_spring(j,0) = 0;
                            // }
                            //////// softplus funtion ///////////
                            Torque_ff_spring(j,0) = log(1+exp(k_spring_thigh*(qDes[j] - (k_p_rest_thigh))));
                
                            if(FF_enable)
                            {
                                torque(j,0) += Torque_ff_spring(j,0);
                            }                             


                        }  
                        else
                        {
                            if(qDes[j]<=go1_Calf_min)
                            {
                                qDes[j]=go1_Calf_min;
                            }
                            else if (qDes[j]>=go1_Calf_max)
                            {
                                qDes[j]=go1_Calf_max;
                            }

                            //// calf joint tracking
                            if(j /3 ==0)
                            {
                                // if(!FR_swing)
                                // {
                                //     //// basic value is for swing leg
                                //     torq_kp_calf = 9 * calf_kp_scale;
                                //     torq_kd_calf = 0.31 * calf_kd_scale;
                                //     torq_ki_calf = 0.01;  
                                // }
                                k_spring_calf = 6;                              
                                // k_p_rest_calf = -1.3;
                                k_p_rest_calf = -1.38;
                                
                            }
                            
                            if(j /3 ==1)
                            {
                                // if(!FL_swing)
                                // {
                                //     //// basic value is for swing leg
                                //     torq_kp_calf = 9 * calf_kp_scale;
                                //     torq_kd_calf = 0.31 * calf_kd_scale;
                                //     torq_ki_calf = 0.01;   
                                // }                               
                                k_spring_calf = 6;
                                // k_p_rest_calf = -1.26;
                                k_p_rest_calf = -1.33;
                            }

                            if(j /3 ==2)
                            {
                                //// basic value is for swing leg
                                // if(!RR_swing)
                                // {
                                //     calf_kp_scale = 1.75;
                                //     torq_kp_calf = 9 * calf_kp_scale;
                                //     torq_kd_calf = 0.31 * calf_kd_scale;
                                // }
                                torq_ki_calf = 0.01;                                  
                                k_spring_calf = 6;
                                // k_p_rest_calf = -1.2;
                                k_p_rest_calf = -1.28;
                            } 
                            if(j /3 ==3)
                            {
                                //// basic value is for swing leg
                                // if(!RL_swing)
                                // {
                                //     calf_kp_scale = 1.75;
                                //     torq_kp_calf = 9 * calf_kp_scale;
                                //     torq_kd_calf = 0.31 * calf_kd_scale;
                                // }
                                torq_ki_calf = 0.01;                                  
                                k_spring_calf = 6;
                                // k_p_rest_calf = -1.45;
                                k_p_rest_calf = -1.47;
                            } 

                            //// calf joint tracking
                            torque_err.block<torque_err_row-1,1>(0,j) = torque_err.block<torque_err_row-1,1>(1,j);

                            torque_err(torque_err_row-1,j) = qDes[j] - RecvLowROS.motorState[j].q;

                            torque_err_intergration.setZero();
                            for(int ij=0; ij<torque_err_row; ij++)
                            {
                            torque_err_intergration(j,0) += torque_err(ij,j);
                            } 
                            
                            torque(j,0) = (qDes[j] - RecvLowROS.motorState[j].q)*torq_kp_calf + (0 - RecvLowROS.motorState[j].dq)*torq_kd_calf + torque_err_intergration(j,0)*torq_ki_calf;
                            
                            //// ff control
                            // if(qDes[j]<=k_p_rest_calf)
                            // {
                            //     Torque_ff_spring(j,0) = k_spring_calf * (qDes[j] - (k_p_rest_calf));
                            // }
                            // else
                            // {
                            //     Torque_ff_spring(j,0) = 0;
                            // }

                            //////// softplus funtion ///////////
                            Torque_ff_spring(j,0) = -log(1+exp(k_spring_calf*(-(qDes[j] - (k_p_rest_calf)))));


                            if(FF_enable)
                            {
                                torque(j,0) += Torque_ff_spring(j,0);
                            }


                        }                      
                    }
                }
                
                
                //=== Catesian space: PD type impedance control + Grf feedforward (when touching the ground)====////////
                if(gait_status != STAND_INIT_STATUS)
                {
                     //=== 111 leg impedance control ===============////////////////////
                    //// desired foot position and foot velocity: relative to  body framework
                    FR_foot_relative_des = (FR_foot_des - body_p_des);
                    FL_foot_relative_des = (FL_foot_des - body_p_des);
                    RR_foot_relative_des = (RR_foot_des - body_p_des);
                    RL_foot_relative_des = (RL_foot_des - body_p_des);            
                    
                    if(stand_count>1)
                    {
                        FR_v_relative = (FR_foot_relative_des - FR_foot_relative_des_old) /dtx;
                        FL_v_relative = (FL_foot_relative_des - FL_foot_relative_des_old) /dtx;
                        RR_v_relative = (RR_foot_relative_des - RR_foot_relative_des_old) /dtx;
                        RL_v_relative = (RL_foot_relative_des - RL_foot_relative_des_old) /dtx;
                    }

                    FR_torque_impedance = (FR_Jaco.transpose() * (swing_kp*(FR_foot_relative_des - FR_foot_relative_mea) 
                                                                        + swing_kd*(FR_v_relative - FR_v_est_relative)));
                    
                    FL_torque_impedance = (FL_Jaco.transpose() * (swing_kp*(FL_foot_relative_des - FL_foot_relative_mea) 
                                                                        + swing_kd*(FL_v_relative - FL_v_est_relative)));

                    RR_torque_impedance = (RR_Jaco.transpose() * (swing_kp*(RR_foot_relative_des - RR_foot_relative_mea) 
                                                                        + swing_kd*(RR_v_relative - RR_v_est_relative))); 

                    RL_torque_impedance = (RL_Jaco.transpose() * (swing_kp*(RL_foot_relative_des - RL_foot_relative_mea) 
                                                                        + swing_kd*(RL_v_relative - RL_v_est_relative)));   

                                                                                                                                        

                    Legs_torque.block<3,1>(0,0) = FR_torque_impedance;
                    Legs_torque.block<3,1>(3,0) = FL_torque_impedance;
                    Legs_torque.block<3,1>(6,0) = RR_torque_impedance;
                    Legs_torque.block<3,1>(9,0) = RL_torque_impedance;   

                    torque.block<3,1>(0,0) += FR_torque_impedance;

                    torque.block<3,1>(3,0) += FL_torque_impedance;

                    torque.block<3,1>(6,0) += RR_torque_impedance;

                    torque.block<3,1>(9,0) += RL_torque_impedance;

                    /////// 222 Grf feedforward control///////////
                    if(!FR_swing)
                    {
                        torque.block<3,1>(0,0) += Torque_ff_GRF.block<3,1>(0,0);
                    }
                    if(!FL_swing)
                    {
                        torque.block<3,1>(3,0) += Torque_ff_GRF.block<3,1>(3,0);
                    }
                    if(!RR_swing)
                    {
                        torque.block<3,1>(6,0) += Torque_ff_GRF.block<3,1>(6,0);
                    }
                    if(!RL_swing)
                    {
                        torque.block<3,1>(9,0) += Torque_ff_GRF.block<3,1>(9,0);
                    }

                }

                for(int j=0; j<12;j++)
                {
                    if(j % 3 == 0)
                    {
                        if(torque(j,0) > 5.0f) torque(j,0) = 5.0f;
                        if(torque(j,0) < -5.0f) torque(j,0) = -5.0f;
                    }
                    if(j % 3 == 1)
                    {
                        if(torque(j,0) > 15.0f) torque(j,0) = 15.0f;
                        if(torque(j,0) < -15.0f) torque(j,0) = -15.0f;
                    }
                    if(j % 3 == 2)
                    {
                        if(torque(j,0) > 10.0f) torque(j,0) = 10.0f;
                        if(torque(j,0) < -10.0f) torque(j,0) = -10.0f;
                    }


                }

                ///============ torque controller finishing =================//////////////



                /////////////// send commanded torque to LCM ///////////////////////
                for(int j=0; j<12;j++)
                {
                    SendLowROS.motorCmd[j].q = PosStopF;
                    SendLowROS.motorCmd[j].dq = VelStopF;
                    SendLowROS.motorCmd[j].Kp = 0;
                    SendLowROS.motorCmd[j].Kd = 0;
                    SendLowROS.motorCmd[j].tau = torque(j,0); ///for length generation
                   // SendLowROS.motorCmd[j].tau = 0;     ///for rest length measurement                 
                }


            }

        }

        // FR_foot_relative_des_old = FR_foot_relative_des;
        // FL_foot_relative_des_old = FL_foot_relative_des;
        // RR_foot_relative_des_old = RR_foot_relative_des;
        // RL_foot_relative_des_old = RL_foot_relative_des;

        // FR_foot_relative_mea_old = FR_foot_relative_mea;
        // FL_foot_relative_mea_old = FL_foot_relative_mea;
        // RR_foot_relative_mea_old = RR_foot_relative_mea;
        // RL_foot_relative_mea_old = RL_foot_relative_mea;

        // com_des_pre[0] = com_des[0];
        // com_des_pre[1] = com_des[1];
        // com_des_pre[2] = com_des[2];


        // base_offset_x_old = base_offset_x;
        // base_offset_y_old = base_offset_y; 
        // base_offset_z_old = base_offset_z;
        // base_offset_roll_old = base_offset_roll;
        // base_offset_pitch_old = base_offset_pitch;
        // base_offset_yaw_old = base_offset_yaw;

        // ///********************* data saving & publisher ************************************///////
        // joint2simulation.header.stamp = ros::Time::now();
        // joint2simulationx.header.stamp = ros::Time::now();
        // ////
        // for(int j=0; j<12; j++){
        //         joint2simulation.position[j] = qDes[j]; // desired joint angles; 
        // }
        // for(int j=0; j<12; j++)
        // {
        //     joint2simulation.position[12+j] = RecvLowROS.motorState[j].q;   // measured joint angles;
        // } 

        // for(int j=0; j<3; j++)
        // {
        //     joint2simulation.position[24+j] = FR_foot_des(j,0);   // desired FR Leg position;
        // }         
        // for(int j=0; j<3; j++)
        // {
        //     joint2simulation.position[27+j] = FL_foot_des(j,0);   // desired position;
        // }
        // for(int j=0; j<3; j++)
        // {
        //     joint2simulation.position[30+j] = RR_foot_des(j,0);   // desired position;
        // }
        // for(int j=0; j<3; j++)
        // {
        //     joint2simulation.position[33+j] = RL_foot_des(j,0);   // desired position;
        // } 

        // for(int j=0; j<3; j++)
        // {
        //     joint2simulation.position[36+j] = FR_foot_relative_mea(j,0) + body_p_des(j,0);   // measured position;
        // }         
        // for(int j=0; j<3; j++)
        // {
        //     joint2simulation.position[39+j] = FL_foot_relative_mea(j,0) + body_p_des(j,0);   // measured position;
        // }
        // for(int j=0; j<3; j++)
        // {
        //     joint2simulation.position[42+j] = RR_foot_relative_mea(j,0) + body_p_des(j,0);   // measured position;
        // }
        // for(int j=0; j<3; j++)
        // {
        //     joint2simulation.position[45+j] = RL_foot_relative_mea(j,0) + body_p_des(j,0);   // measured position;
        // } 
        
        // /// body_pos_des
        // for(int j=0; j<3; j++)
        // {
        //     joint2simulation.position[48+j] = body_p_des(j,0);   // desired body position;
        // }
        // /// body_R_des
        // for(int j=0; j<3; j++)
        // {
        //     joint2simulation.position[51+j] = body_r_des(j,0);   // desired body ori;
        // }
        // /// body_R_mea
        // for(int j=0; j<3; j++)
        // {
        //     joint2simulation.position[90+j] = body_r_est(j,0);   // measure body ori;
        // }
        
        // ////////
        // for(int j=0; j<6; j++)
        // {
        //     joint2simulation.position[54+j] = Force_L_R(j);  // desired force on left and right leg;
        // }


        // // FR force
        // for(int j=0; j<3; j++)
        // {
        //     joint2simulation.position[66+j] = Dynam.F_leg_ref(j,0); 
        // }

        // // FL force
        // for(int j=0; j<3; j++)
        // {
        //     joint2simulation.position[69+j] = Dynam.F_leg_ref(j,1);
        // } 

        // // RR force
        // for(int j=0; j<3; j++)
        // {
        //     joint2simulation.position[72+j] = Dynam.F_leg_ref(j,2); 
        // }  

        // // RL force
        // for(int j=0; j<3; j++)
        // {
        //     joint2simulation.position[75+j] = Dynam.F_leg_ref(j,3);
        // } 


        // ////// ============= //////////////////// 
        // // joint torque desired: fb + ff //////// 
        // for(int j=0; j<12; j++)
        // {
        //     joint2simulation.position[78+j] = SendLowROS.motorCmd[j].tau;
        // }

        // //////////////////////torque display
        // //// torque measured //////////////////////
        // for(int j=0; j<12; j++)
        // {
        //     joint2simulationx.position[j] = RecvLowROS.motorState[j].tauEst;
        // }
        
        // ////// optimal grf compensation/////////
        // for(int j=0; j<12; j++)
        // {
        //     joint2simulationx.position[12+j] = Dynam.grf_opt(j,0);
        // }

        // /////// spring torque compensation//////
        // for(int j=0; j<12; j++)
        // {
        //     joint2simulationx.position[24+j] = Torque_ff_spring(j,0);
        // } 

        // /////// leg impedance-control torque compensation//////
        // for(int j=0; j<12; j++)
        // {
        //     joint2simulationx.position[36+j] = Legs_torque(j,0);
        // }         
        


        gait_data_pub.publish(joint2simulation);
        gait_data_pubx.publish(joint2simulationx);

        // // // // // /////sending command //////////// 
        // SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
        // roslcm.Send(SendLowLCM);



        ros::spinOnce();
        loop_rate.sleep();

        count++;
        if(count > 1000){
            count = 1000;
            initiated_flag = true;
        }
        
        ////// IMU drift calibration///////
        n_count++;  
        if(n_count==1001)
        {
            root_euler_offset(0) *= (1.0/1000);
            root_euler_offset(1) *= (1.0/1000);
            root_euler_offset(2) *= (1.0/1000);         
               
        }
        else
        {
            if(n_count>1001)
            {
                n_count =1002;
            }
            else
            {
                root_euler_offset(0) += root_euler(0);
                root_euler_offset(1) += root_euler(1);
                root_euler_offset(2) += root_euler(2);                
            }
             
        }

    }
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "torque_ros_mode");

    UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
}