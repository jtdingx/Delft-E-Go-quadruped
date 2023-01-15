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
#include "sensor_msgs/JointState.h"
#include <Eigen/Dense>
#include "go1_const.h"
#include "torque_mode_udp.h"
#include "/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/src/go1_rt_control/src/Robotpara/robot_const_para_config.h"
#include "geometry_msgs/Twist.h"
// #include "/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/include/yaml.h"
#include "yaml.h"
#include <ros/package.h> 



using namespace UNITREE_LEGGED_SDK;
using namespace std;

// global varable generation

float qDes[12]={0};
float qDes_pre[12]={0};
float dqDes[12] = {0}; 
float dqDes_old[12] = {0}; 
float Kp_joint_ini[12] = {50/2,60/2,60/2,50/2,60/2,60/2,60/2,60/2,60/2,60/2,60/2,60/2};
float Kd_joint_ini[12] = {3/2,4/2,4/2,3/2,4/2,4/2,2/2,4/2,4/2,2/2,4/2,4/2};
// float Kp_joint_retarget[12] = {50,150,150,50,150,150,80,150,150,80,150,150};
// float Kd_joint_retarget[12] = {3,5,5,3,5,5,2,5,5,2,5,5};
float Kp_joint_retarget[12] = {60,60,40,60,60,40,60,75,70,60,75,70};
float Kd_joint_retarget[12] = {1.25,1.25,1.25,1.25,1.25,1.25,1.25,1.5,1.5,1.25,1.5,1.5};  ///smaller kd causes quiet motion

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

double w_pos_m_kp[3] = {0.05,0.1,0.2};
double w_pos_m_kd[3] = {0.0001,0.0001,0.0001};

double w_rpy_m_kp[3] = {0.25,0.25,0.25};
double w_rpy_m_kd[3] = {0.00005,0.00005,0};

///// gains variation
double  kpp0_det;
double  kdp0_det;
double  kpp1_det;
double  kdp1_det;          
double  kpp2_det;
double  kdp2_det;

double clear_height = 0.05;

Eigen::Matrix<double,5,1> footforce_fr, footforce_fl, footforce_rr,footforce_rl;



/// @brief //
/// @param initPos 
/// @param targetPos 
/// @param rate 
/// @param j 
/// @return 
double Quadruped::jointLinearInterpolation(double initPos, double targetPos, double rate, int j)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    // dqDes[j] = 0.8*dqDes_old[j] + 0.2*(p - qDes[j])/(1.0/ctrl_estimation);
    // dqDes_old[j] = dqDes[j];
    return p;
}


void Quadruped::keyboard_model_callback(const geometry_msgs::Twist::ConstPtr &msgIn) 
{
    if((msgIn->linear.x == 1)&&(gait_status == STAND_STATUS))
    {
        cmd_gait = STAND_UP;
        printf("===========Switch to STAND_UP state==========\n");
    }
    if((msgIn->linear.x ==2)&&(gait_status == STAND_UP_STATUS))
    {
        cmd_gait = DYNAMIC;
        printf("===========Switch to DYNAMIC WALKING state==========\n");
    } 
    // std::cout<<"msgIn->linear.x:"<<msgIn->linear.x<<std::endl;  
}



void Quadruped::Base_offset_callback(const geometry_msgs::Twist::ConstPtr &msgIn) {
        // base_offset_x = msgIn->linear.x;
        // base_offset_y = msgIn->linear.y;
        // base_offset_z = msgIn->linear.z;
        // base_offset_roll = msgIn->angular.x;
        // base_offset_pitch = msgIn->angular.y;
        // base_offset_yaw = msgIn->angular.z;

    if((cmd_gait == DYNAMIC)||cmd_gait == STAND_UP)
    {
        kpp0_det = msgIn->linear.x;
        kdp0_det = msgIn->linear.y;
        kpp1_det = msgIn->linear.z;
        kdp1_det = msgIn->angular.x;
        kpp2_det = msgIn->angular.y;
        kdp2_det = msgIn->angular.z;

        // kpp[0] +=  kpp0_det;
        // kdp[0] +=  kdp0_det;

        // kpp[1] +=  kpp1_det;
        // kdp[1] +=  kdp1_det;          

        // kpp[2] +=  kpp2_det;
        // kdp[2] +=  kdp2_det; 

        // kpw[0] +=  2*kpp0_det;
        // kdw[0] +=  2*kdp0_det;

        // kpw[1] +=  2*kpp1_det;
        // kdw[1] +=  2*kdp1_det;          

        // kpw[2] +=  2*kpp2_det;
        // kdw[2] +=  2*kdp2_det;             

        // std::cout<<"kpw[0]:"<<kpw[0]<<endl; 
        // std::cout<<"kdw[0]:"<<kdw[0]<<endl; 
        // std::cout<<"kpw[1]:"<<kpw[1]<<endl; 
        // std::cout<<"kdw[1]:"<<kdw[1]<<endl;           
        // std::cout<<"kpw[2]:"<<kpw[2]<<endl; 
        // std::cout<<"kdw[2]:"<<kdw[2]<<endl; 

        // swing_kp(0,0) +=  kpp0_det;
        // swing_kd(0,0) +=  kdp0_det;

        // swing_kp(1,1) +=  kpp1_det;
        // swing_kd(1,1) +=  kdp1_det;          

        // swing_kp(2,2) +=  kpp2_det;
        // swing_kd(2,2) +=  kdp2_det;

        // std::cout<<"swing_kp(0,0):"<<swing_kp(0,0)<<endl; 
        // std::cout<<"swing_kd(0,0):"<<swing_kd(0,0)<<endl;  
        // std::cout<<"swing_kp(1,1):"<<swing_kp(1,1)<<endl; 
        // std::cout<<"swing_kd(1,1):"<<swing_kd(1,1)<<endl;           
        // std::cout<<"swing_kp(2,2):"<<swing_kp(2,2)<<endl; 
        // std::cout<<"swing_kd(2,2):"<<swing_kd(2,2)<<endl; 

        // //////// leg PD parameters tuning ////////// 
        // Kp_joint[0] +=  kpp0_det;
        // Kd_joint[0] +=  0.1*kdp0_det;

        // Kp_joint[1] +=  kpp1_det;
        // Kd_joint[1] +=  0.1*kdp1_det;          

        // Kp_joint[2] +=  kpp2_det;
        // Kd_joint[2] +=  0.1*kdp2_det;

        // std::cout<<"Kp_joint[0]:"<<Kp_joint[0]<<endl; 
        // std::cout<<"Kd_joint[0]:"<<Kd_joint[0]<<endl;  
        // std::cout<<"Kp_joint[1]:"<<Kp_joint[1]<<endl; 
        // std::cout<<"Kd_joint[1]:"<<Kd_joint[1]<<endl;           
        // std::cout<<"Kp_joint[2]:"<<Kp_joint[2]<<endl; 
        // std::cout<<"Kd_joint[2]:"<<Kd_joint[2]<<endl; 

    }

}

double Quadruped::clamp_func(double new_cmd, double old_cmd, double thresh)
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

// // void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &msg)
// // {
// //     printf("yaw = %f\n", msg->imu.rpy[2]);
// //     printf("yaw = %f\n", msg->imu.rpy[2]);
// // }

// // void lowStateCallback(const unitree_legged_msgs::LowState::ConstPtr &msg)
// // {
// //     printf("FR_2_pos = %f\n", msg->motorState[FR_2].q);
// // }

void Quadruped::gait_pose_callback(LowState RecvLowROS)
{
    /// FL, FR, RL, RR;
    a1_ctrl_states.joint_pos[0] = RecvLowROS.motorState[3].q;
    a1_ctrl_states.joint_vel[0] = RecvLowROS.motorState[3].dq;
    a1_ctrl_states.joint_pos[1] = RecvLowROS.motorState[4].q;
    a1_ctrl_states.joint_vel[1] = RecvLowROS.motorState[4].dq;
    a1_ctrl_states.joint_pos[2] = RecvLowROS.motorState[5].q;
    a1_ctrl_states.joint_vel[2] = RecvLowROS.motorState[5].dq;

    a1_ctrl_states.joint_pos[3] = RecvLowROS.motorState[0].q;
    a1_ctrl_states.joint_vel[3] = RecvLowROS.motorState[0].dq;
    a1_ctrl_states.joint_pos[4] = RecvLowROS.motorState[1].q;
    a1_ctrl_states.joint_vel[4] = RecvLowROS.motorState[1].dq;
    a1_ctrl_states.joint_pos[5] = RecvLowROS.motorState[2].q;
    a1_ctrl_states.joint_vel[5] = RecvLowROS.motorState[2].dq;

    a1_ctrl_states.joint_pos[6] = RecvLowROS.motorState[9].q;
    a1_ctrl_states.joint_vel[6] = RecvLowROS.motorState[9].dq;   
    a1_ctrl_states.joint_pos[7] = RecvLowROS.motorState[10].q;
    a1_ctrl_states.joint_vel[7] = RecvLowROS.motorState[10].dq;
    a1_ctrl_states.joint_pos[8] = RecvLowROS.motorState[11].q;
    a1_ctrl_states.joint_vel[8] = RecvLowROS.motorState[11].dq;

    a1_ctrl_states.joint_pos[9] = RecvLowROS.motorState[6].q;
    a1_ctrl_states.joint_vel[9] = RecvLowROS.motorState[6].dq;
    a1_ctrl_states.joint_pos[10] = RecvLowROS.motorState[7].q;
    a1_ctrl_states.joint_vel[10] = RecvLowROS.motorState[7].dq;
    a1_ctrl_states.joint_pos[11] = RecvLowROS.motorState[8].q;
    a1_ctrl_states.joint_vel[11] = RecvLowROS.motorState[8].dq;

    // a1_ctrl_states.foot_force[0] = (footforce_fl.mean()); 
    // a1_ctrl_states.foot_force[1] = (footforce_fr.mean()); 
    // a1_ctrl_states.foot_force[2] = (footforce_rl.mean()); 
    // a1_ctrl_states.foot_force[3] = (footforce_rr.mean()); 

    a1_ctrl_states.foot_force[0] = (footforce_fl(4,0)); 
    a1_ctrl_states.foot_force[1] = (footforce_fr(4,0)); 
    a1_ctrl_states.foot_force[2] = (footforce_rl(4,0)); 
    a1_ctrl_states.foot_force[3] = (footforce_rr(4,0));     


    a1_ctrl_states.imu_acc = Eigen::Vector3d(
            acc_x.CalculateAverage(RecvLowROS.imu.accelerometer[0]),
            acc_y.CalculateAverage(RecvLowROS.imu.accelerometer[1]),
            acc_z.CalculateAverage(RecvLowROS.imu.accelerometer[2])
    );

    a1_ctrl_states.imu_ang_vel = Eigen::Vector3d(
            gyro_x.CalculateAverage(RecvLowROS.imu.gyroscope[0]),
            gyro_y.CalculateAverage(RecvLowROS.imu.gyroscope[1]),
            gyro_z.CalculateAverage(RecvLowROS.imu.gyroscope[2])
    );

    // calculate several useful variables
    // euler should be roll pitch yaw
    // a1_ctrl_states.root_quat = root_quat;
    // a1_ctrl_states.root_rot_mat = a1_ctrl_states.root_quat.toRotationMatrix();
    // a1_ctrl_states.root_euler = Utils::quat_to_euler(a1_ctrl_states.root_quat);

    a1_ctrl_states.root_quat = root_quat;
    a1_ctrl_states.root_rot_mat = root_rot_mat;
    a1_ctrl_states.root_euler = body_r_est;


    a1_ctrl_states.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

    a1_ctrl_states.root_ang_vel = a1_ctrl_states.root_rot_mat * a1_ctrl_states.imu_ang_vel;

    // FL, FR, RL, RR
    for (int i = 0; i < NUM_LEG; ++i) {
        // a1_ctrl_states.foot_pos_rel.block<3, 1>(0, i) = a1_kin.fk(
        //         a1_ctrl_states.joint_pos.segment<3>(3 * i),
        //         rho_opt_list[i], rho_fix_list[i]);
        // a1_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i) = a1_kin.jac(
        //         a1_ctrl_states.joint_pos.segment<3>(3 * i),
        //         rho_opt_list[i], rho_fix_list[i]);
        a1_ctrl_states.foot_pos_rel.block<3, 1>(0, i) = Kine.Forward_kinematics_go1(a1_ctrl_states.joint_pos.segment<3>(3 * i), i);
        a1_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i) = Kine.Jacobian_kin;

        Eigen::Matrix3d tmp_mtx = a1_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i);
        Eigen::Vector3d tmp_vec = a1_ctrl_states.joint_vel.segment<3>(3 * i);

        //////// foot_velocity to base: Jaco * dq
        a1_ctrl_states.foot_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

        ///////

        a1_ctrl_states.foot_pos_abs.block<3, 1>(0, i) = a1_ctrl_states.root_rot_mat * a1_ctrl_states.foot_pos_rel.block<3, 1>(0, i);
        a1_ctrl_states.foot_vel_abs.block<3, 1>(0, i) = a1_ctrl_states.root_rot_mat * a1_ctrl_states.foot_vel_rel.block<3, 1>(0, i);

        a1_ctrl_states.foot_pos_world.block<3, 1>(0, i) = a1_ctrl_states.foot_pos_abs.block<3, 1>(0, i) + a1_ctrl_states.root_pos;
        a1_ctrl_states.foot_vel_world.block<3, 1>(0, i) = a1_ctrl_states.foot_vel_abs.block<3, 1>(0, i) + a1_ctrl_states.root_lin_vel;
    }

}


Eigen::Matrix<double, 100,1>  Quadruped::state_est_main_update(double dt) {

    Eigen::Matrix<double,100,1> state;
    state.setZero();
    // state estimation
    if (!go1_estimate.is_inited()) {
        go1_estimate.init_state(a1_ctrl_states);
    } else {
        go1_estimate.update_estimation(a1_ctrl_states, dt, estimated_root_pos_offset);
    }

    state.block<3,1>(0,0) = a1_ctrl_states.estimated_root_pos;
    state.block<3,1>(3,0) = a1_ctrl_states.estimated_root_vel;
    state.block<3,1>(6,0) = a1_ctrl_states.root_euler;
    state.block<3,1>(9,0) = a1_ctrl_states.imu_ang_vel;
    state.block<3,1>(12,0) = a1_ctrl_states.foot_pos_world.col(1);
    state.block<3,1>(15,0) = a1_ctrl_states.foot_pos_world.col(0);
    state.block<3,1>(18,0) = a1_ctrl_states.foot_pos_world.col(3);
    state.block<3,1>(21,0) = a1_ctrl_states.foot_pos_world.col(2);
    state.block<3,1>(24,0) = a1_ctrl_states.foot_vel_world.col(1);
    state.block<3,1>(27,0) = a1_ctrl_states.foot_vel_world.col(0);
    state.block<3,1>(30,0) = a1_ctrl_states.foot_vel_world.col(3);
    state.block<3,1>(33,0) = a1_ctrl_states.foot_vel_world.col(2);

    return state;
}


void Quadruped::nrt_gait_sub_operation(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int jx = 0; jx<100; jx++)
    {
        slow_mpc_gait(jx) = msg->position[jx]; 
    }
        // mpc_gait_flag = slow_mpc_gait(99);
}


void Quadruped::Grf_sub_operation(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int jx = 0; jx<100; jx++)
    {
        Grf_sub(jx) = msg->position[jx]; 
    }
}



void Quadruped::leg_kinematic()
{
    q_ini = FR_angle_des;
    FR_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,FR_foot_des,q_ini,0);
    FR_Jaco = Kine.Jacobian_kin;

    q_ini = FL_angle_des;
    FL_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,FL_foot_des,q_ini,1);
    FL_Jaco = Kine.Jacobian_kin;

    q_ini = RR_angle_des;
    RR_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,RR_foot_des,q_ini,2);
    RR_Jaco = Kine.Jacobian_kin;

    q_ini = RL_angle_des;
    RL_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,RL_foot_des,q_ini,3);  
    RL_Jaco = Kine.Jacobian_kin;                          

    qDes[0] = FR_angle_des(0,0);
    qDes[1] = FR_angle_des(1,0);
    qDes[2] = FR_angle_des(2,0);
    qDes[3] = FL_angle_des(0,0);
    qDes[4] = FL_angle_des(1,0);
    qDes[5] = FL_angle_des(2,0);
    qDes[6] = RR_angle_des(0,0);
    qDes[7] = RR_angle_des(1,0);
    qDes[8] = RR_angle_des(2,0);
    qDes[9] = RL_angle_des(0,0);
    qDes[10] = RL_angle_des(1,0);
    qDes[11] = RL_angle_des(2,0);     
}

void Quadruped::base_pos_fb_controler()
{
    ////// PD-type CoM position control//////
    if(right_support == 0) ////left support
    {
        switch (gait_mode)
            {
            case 101:  ////biped walking
                FR_swing = true;
                RR_swing = true;
                FL_swing = false;
                RL_swing = false;
                if(using_ekf>0.5)
                {
                    support_pos_sensor[0] = (a1_ctrl_states.foot_pos_world(0,0) + a1_ctrl_states.foot_pos_world(0,2))/2;
                    support_pos_sensor[1] = (a1_ctrl_states.foot_pos_world(1,0) + a1_ctrl_states.foot_pos_world(1,2))/2;
                    support_pos_sensor[2] = (a1_ctrl_states.foot_pos_world(2,0) + a1_ctrl_states.foot_pos_world(2,2))/2;
                }
                else
                {
                    support_pos_sensor[2] = (a1_ctrl_states.foot_pos_world(2,0) + a1_ctrl_states.foot_pos_world(2,2))/2;
                }
                support_pos_des[0] = (FL_foot_des[0] + RL_foot_des[0])/2;
                support_pos_des[1] = (FL_foot_des[1] + RL_foot_des[1])/2;
                support_pos_des[2] = (FL_foot_des[2] + RL_foot_des[2])/2; 

                break;
            case 102:  ///troting
                FR_swing = false;
                RL_swing = false;
                FL_swing = true;
                RR_swing = true;
                if(using_ekf>0.5)
                {
                    support_pos_sensor[0] = (a1_ctrl_states.foot_pos_world(0,1) + a1_ctrl_states.foot_pos_world(0,2))/2;
                    support_pos_sensor[1] = (a1_ctrl_states.foot_pos_world(1,1) + a1_ctrl_states.foot_pos_world(1,2))/2;
                    support_pos_sensor[2] = (a1_ctrl_states.foot_pos_world(2,1) + a1_ctrl_states.foot_pos_world(2,2))/2;
                }
                else
                {
                    support_pos_sensor[2] = (a1_ctrl_states.foot_pos_world(2,1) + a1_ctrl_states.foot_pos_world(2,2))/2;
                }
                support_pos_des[0] = (FR_foot_des[0] + RL_foot_des[0])/2;
                support_pos_des[1] = (FR_foot_des[1] + RL_foot_des[1])/2;
                support_pos_des[2] = (FR_foot_des[2] + RL_foot_des[2])/2;


                break;
            case 103:  ///gallop: alter the  x-y direction
                FR_swing = true;
                FL_swing = true;
                RR_swing = false;
                RL_swing = false;                    

                break;            
            default:


                break;
            } 
    }
    else
    {
        if (right_support == 1)
        {
            switch (gait_mode)
                {
                case 101:  ////biped walking
                    FR_swing = false;
                    RR_swing = false;
                    FL_swing = true;
                    RL_swing = true;
                    if(using_ekf>0.5)
                    {
                        support_pos_sensor[0] = (a1_ctrl_states.foot_pos_world(0,1) + a1_ctrl_states.foot_pos_world(0,3))/2;
                        support_pos_sensor[1] = (a1_ctrl_states.foot_pos_world(1,1) + a1_ctrl_states.foot_pos_world(1,3))/2;
                        support_pos_sensor[2] = (a1_ctrl_states.foot_pos_world(2,1) + a1_ctrl_states.foot_pos_world(2,3))/2;
                    }
                    else
                    {
                        support_pos_sensor[2] = (a1_ctrl_states.foot_pos_world(2,1) + a1_ctrl_states.foot_pos_world(2,3))/2;
                    }

                    support_pos_des[0] = (FR_foot_des[0] + RR_foot_des[0])/2;
                    support_pos_des[1] = (FR_foot_des[1] + RR_foot_des[1])/2;
                    support_pos_des[2] = (FR_foot_des[2] + RR_foot_des[2])/2; 

                    break;
                case 102:  ///troting
                    FR_swing = true;
                    RL_swing = true;
                    FL_swing = false;
                    RR_swing = false;      
                    if(using_ekf>0.5)
                    {                    
                        support_pos_sensor[0] = (a1_ctrl_states.foot_pos_world(0,0) + a1_ctrl_states.foot_pos_world(0,3))/2;
                        support_pos_sensor[1] = (a1_ctrl_states.foot_pos_world(1,0) + a1_ctrl_states.foot_pos_world(1,3))/2;
                        support_pos_sensor[2] = (a1_ctrl_states.foot_pos_world(2,0) + a1_ctrl_states.foot_pos_world(2,3))/2;
                    }
                    else
                    {
                        support_pos_sensor[2] = (a1_ctrl_states.foot_pos_world(2,0) + a1_ctrl_states.foot_pos_world(2,3))/2;
                    }
                    support_pos_des[0] = (FL_foot_des[0] + RR_foot_des[0])/2;
                    support_pos_des[1] = (FL_foot_des[1] + RR_foot_des[1])/2;
                    support_pos_des[2] = (FL_foot_des[2] + RR_foot_des[2])/2;                                                              

                    break;
                case 103:  ///gallop: alter the  x-y direction
                    FR_swing = false;
                    FL_swing = false;
                    RR_swing = true;
                    RL_swing = true;                    

                    break;            
                default:
                    break;
                }
        }
        else
        {
            FR_swing = false;
            FL_swing = false;
            RR_swing = false;
            RL_swing = false;  

            //// assuming all support in trotting mode; 
            if(using_ekf>0.5)
            {
                support_pos_sensor[0] = (a1_ctrl_states.foot_pos_world(0,0) + a1_ctrl_states.foot_pos_world(0,3) 
                                    + a1_ctrl_states.foot_pos_world(0,1) + a1_ctrl_states.foot_pos_world(0,2))/4;
                support_pos_sensor[1] = (a1_ctrl_states.foot_pos_world(1,0) + a1_ctrl_states.foot_pos_world(1,3)
                                    + a1_ctrl_states.foot_pos_world(1,1) + a1_ctrl_states.foot_pos_world(1,2))/4;
                support_pos_sensor[2] = (a1_ctrl_states.foot_pos_world(2,0) + a1_ctrl_states.foot_pos_world(2,3)
                                    + a1_ctrl_states.foot_pos_world(2,1) + a1_ctrl_states.foot_pos_world(2,2))/4;
                support_pos_des[0] = (FL_foot_des[0] + RR_foot_des[0] + FR_foot_des[0] + RL_foot_des[0])/4;
                support_pos_des[1] = (FL_foot_des[1] + RR_foot_des[1] + FR_foot_des[1] + RL_foot_des[1])/4;
                support_pos_des[2] = (FL_foot_des[2] + RR_foot_des[2] + FR_foot_des[2] + RL_foot_des[2])/4;                                    
            }
            else
            {
                support_pos_sensor[2] = (a1_ctrl_states.foot_pos_world(2,0) + a1_ctrl_states.foot_pos_world(2,3)
                                    + a1_ctrl_states.foot_pos_world(2,1) + a1_ctrl_states.foot_pos_world(2,2))/4;                
            
                support_pos_des[0] = support_pos_sensor[0];
                support_pos_des[1] = support_pos_sensor[1];
                support_pos_des[2] = support_pos_sensor[2];             
            }
            
               
        }                
        
    }

    if(using_ekf<0.5)
    {
        switch_support =1;
    }
    if(switch_support<0.5) //// assuming not switch of the support pos
    {
        if(using_ekf>0.5)
        {
            support_pos_sensor[0] = (a1_ctrl_states.foot_pos_world(0,0) + a1_ctrl_states.foot_pos_world(0,3) 
                                    + a1_ctrl_states.foot_pos_world(0,1) + a1_ctrl_states.foot_pos_world(0,2))/4;
            support_pos_sensor[1] = (a1_ctrl_states.foot_pos_world(1,0) + a1_ctrl_states.foot_pos_world(1,3)
                                    + a1_ctrl_states.foot_pos_world(1,1) + a1_ctrl_states.foot_pos_world(1,2))/4;
            support_pos_sensor[2] = (a1_ctrl_states.foot_pos_world(2,0) + a1_ctrl_states.foot_pos_world(2,3)
                                    + a1_ctrl_states.foot_pos_world(2,1) + a1_ctrl_states.foot_pos_world(2,2))/4;  
        } 
        else
        {
            support_pos_sensor[0] = (frfoot_pose_sensor[0] + flfoot_pose_sensor[0] + rrfoot_pose_sensor[0] + rlfoot_pose_sensor[0])/4;
            support_pos_sensor[1] = (frfoot_pose_sensor[1] + flfoot_pose_sensor[1] + rrfoot_pose_sensor[1] + rlfoot_pose_sensor[1])/4;
            support_pos_sensor[2] = (frfoot_pose_sensor[2] + flfoot_pose_sensor[2] + rrfoot_pose_sensor[2] + rlfoot_pose_sensor[2])/4;             
        }       

          
        support_pos_des[0] = (FL_foot_des[0] + RR_foot_des[0] + FR_foot_des[0] + RL_foot_des[0])/4;
        support_pos_des[1] = (FL_foot_des[1] + RR_foot_des[1] + FR_foot_des[1] + RL_foot_des[1])/4;
        support_pos_des[2] = (FL_foot_des[2] + RR_foot_des[2] + FR_foot_des[2] + RL_foot_des[2])/4;         
    }
    


    body_relative_supportv_sensor[0] = butterworthLPF71.filter((com_sensor[0] - support_pos_sensor[0] - body_relative_support_sensor_old[0])/dtx);
    body_relative_supportv_sensor[1] = butterworthLPF72.filter((com_sensor[1] - support_pos_sensor[1] - body_relative_support_sensor_old[1])/dtx);
    body_relative_supportv_sensor[2] = butterworthLPF73.filter((com_sensor[2] - support_pos_sensor[2] - body_relative_support_sensor_old[2])/dtx);
    if(global_com_feedback >0.5)
    {
        w_pos_m[0] = butterworthLPF74.filter(w_pos_m_kp[0] * (body_p_des[0] - (com_sensor[0])) + w_pos_m_kd[0] * (0 - comv_sensor[0])); /// x;
        w_pos_m[1] = butterworthLPF75.filter(w_pos_m_kp[1] * (body_p_des[1] - (com_sensor[1])) + w_pos_m_kd[1] * (0-comv_sensor[1]));  /// y;
        w_pos_m[2] = butterworthLPF76.filter(w_pos_m_kp[2] * (body_p_des[2] - (com_sensor[2])) + w_pos_m_kd[2] * (0-comv_sensor[2])); /// z:
    } 
    else
    {
        w_pos_m[0] = butterworthLPF74.filter(w_pos_m_kp[0] * (body_p_des[0] - support_pos_des[0] - (com_sensor[0] - support_pos_sensor[0])) + w_pos_m_kd[0] * (0-body_relative_supportv_sensor[0])); /// x;
        w_pos_m[1] = butterworthLPF75.filter(w_pos_m_kp[1] * (body_p_des[1] - support_pos_des[1] - (com_sensor[1] - support_pos_sensor[1])) + w_pos_m_kd[1] * (0-body_relative_supportv_sensor[1]));  /// y;
        w_pos_m[2] = butterworthLPF76.filter(w_pos_m_kp[2] * (body_p_des[2] - support_pos_des[2] - (com_sensor[2] - support_pos_sensor[2])) + w_pos_m_kd[2] * (0-body_relative_supportv_sensor[2])); /// z:   
    }   
    w_pos_m_filter[0] = butterworthLPF31.filter(w_pos_m[0]);
    w_pos_m_filter[1] = butterworthLPF32.filter(w_pos_m[1]);
    w_pos_m_filter[2] = butterworthLPF33.filter(w_pos_m[2]);
    if ((w_pos_m_filter[0] >= com_pos_max))
    {
        w_pos_m_filter[0] = com_pos_max;
    }
    else
    {
        if ((w_pos_m_filter[0] <= com_pos_min))
        {
            w_pos_m_filter[0] = com_pos_min;
        }
    }

    if ((w_pos_m_filter[1] >= com_pos_max))
    {
        w_pos_m_filter[1] = com_pos_max;
    }
    else
    {
        if ((w_pos_m_filter[1] <= com_pos_min))
        {
            w_pos_m_filter[1] = com_pos_min;
        }
    } 
    if ((w_pos_m_filter[2] >= com_pos_max))
    {
        w_pos_m_filter[2] = com_pos_max;
    }
    else
    {
        if ((w_pos_m_filter[2] <= com_pos_min))
        {
            w_pos_m_filter[2] = com_pos_min;
        }
    } 

    w_rpy_m[0] = w_rpy_m_kp[0] * (body_r_des[0]- theta_estkine[0]) + w_rpy_m_kp[0] * (0-thetav_estkine[0]); /// x;
    w_rpy_m[1] = w_rpy_m_kp[1] * (body_r_des[1]- theta_estkine[1]) + w_rpy_m_kp[1] * (0-thetav_estkine[1]);  /// y;
    w_rpy_m[2] = w_rpy_m_kp[2] * (body_p_des[2]- theta_estkine[2]) + w_rpy_m_kp[2] * (0-thetav_estkine[2]); /// z:
    w_rpy_m_filter[0] = butterworthLPF34.filter(w_rpy_m[0]);
    w_rpy_m_filter[1] = butterworthLPF35.filter(w_rpy_m[1]);
    w_rpy_m_filter[2] = butterworthLPF36.filter(w_rpy_m[2]);
    if ((w_rpy_m_filter[0] >= com_rpy_max))
    {
        w_rpy_m_filter[0] = com_rpy_max;
    }
    else
    {
        if ((w_rpy_m_filter[0] <= com_rpy_min))
        {
            w_rpy_m_filter[0] = com_rpy_min;
        }
    }
    if ((w_rpy_m_filter[1] >= com_rpy_max))
    {
        w_rpy_m_filter[1] = com_rpy_max;
    }
    else
    {
        if ((w_rpy_m_filter[1] <= com_rpy_min))
        {
            w_rpy_m_filter[1] = com_rpy_min;
        }
    } 
    if ((w_rpy_m_filter[2] >= com_rpy_max))
    {
        w_rpy_m_filter[2] = com_rpy_max;
    }
    else
    {
        if ((w_rpy_m_filter[2] <= com_rpy_min))
        {
            w_rpy_m_filter[2] = com_rpy_min;
        }
    }                        
                  
}

void Quadruped::base_acc_ff_controler()
{
    /// update planning loop: reinitiliazation of global CoM position;
    if(init_count>init_count_old)
    {
       std::cout<<"re_initialization:"<<init_count <<std::endl;
       body_p_des_old_ini = body_p_des;
       com_sensor_old_ini[0] = com_sensor[0];
       com_sensor_old_ini[1] = com_sensor[1];
       com_sensor_old_ini[2] = com_sensor[2];
    }
    ////// PD-type CoM acceleration control: for Grf compensation//////
    ///// note state-estimation error would caused undesired acceleration behaviour
    // coma_des[0] = kpp[0] * (body_p_des[0] + com_sensor[0]) 
    //               + kdp[0]*(comv_des[0] + comv_sensor[0]);
    if(global_com_feedback>0.5)
    {
        coma_des[0] = butterworthLPF77.filter(kpp[0] * (body_p_des[0] - body_p_des_old_ini[0] - (com_sensor[0] - com_sensor_old_ini[0])) 
                      + kdp[0]*(0 - comv_sensor[0]));        
        coma_des[1] = butterworthLPF78.filter(kpp[1] * (body_p_des[1] - body_p_des_old_ini[1] - (com_sensor[1] - com_sensor_old_ini[1])) 
                    + kdp[1]*(0 - comv_sensor[1]));
        coma_des[2] = butterworthLPF79.filter(kpp[2] * (body_p_des[2] - com_sensor[2]) 
                    + kdp[2]*(0 - comv_sensor[2])); 
    }
    else
    {
        coma_des[0] = butterworthLPF77.filter(kpp[0] * (body_p_des[0] - support_pos_des[0] - (com_sensor[0] - support_pos_sensor[0])) 
                      + kdp[0]*(0 - comv_sensor[0]));        
        coma_des[1] = butterworthLPF78.filter(kpp[1] * (body_p_des[1] - support_pos_des[1] - (com_sensor[1] - support_pos_sensor[1])) 
                      + kdp[1]*(0 - comv_sensor[1]));
        coma_des[2] = butterworthLPF79.filter(kpp[2] * (body_p_des[2] - support_pos_des[2] - (com_sensor[2] - support_pos_sensor[2])) 
                      + kdp[2]*(0 - comv_sensor[2]));                
    }
    ////// Rotation matrix could be a better choice/////
    theta_acc_des[0] = kpw[0] * (body_r_des(0,0) - theta_estkine[0]) + kdw[0]*(0 - thetav_estkine[0]);
    theta_acc_des[1] = kpw[1] * (body_r_des(1,0) - theta_estkine[1]) + kdw[1]*(0 - thetav_estkine[1]);
    theta_acc_des[2] = kpw[2] * (body_r_des(2,0) - theta_estkine[2]) + kdw[2]*(0 - thetav_estkine[2]); 

}



void Quadruped::config_set()
{   
    /////load default parameter from the yaml.file
    ///////////////////  yaml code . ///////// 
    YAML::Node config = YAML::LoadFile("/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/config/config.yaml");

    Kp_joint_retarget[0] = config["Kp_joint_retarget0"].as<double>();
    Kp_joint_retarget[1] = config["Kp_joint_retarget1"].as<double>();
    Kp_joint_retarget[2] = config["Kp_joint_retarget2"].as<double>();
    Kp_joint_retarget[3] = config["Kp_joint_retarget3"].as<double>();
    Kp_joint_retarget[4] = config["Kp_joint_retarget4"].as<double>();
    Kp_joint_retarget[5] = config["Kp_joint_retarget5"].as<double>();
    Kp_joint_retarget[6] = config["Kp_joint_retarget6"].as<double>();
    Kp_joint_retarget[7] = config["Kp_joint_retarget7"].as<double>();
    Kp_joint_retarget[8] = config["Kp_joint_retarget8"].as<double>();
    Kp_joint_retarget[9] = config["Kp_joint_retarget9"].as<double>();
    Kp_joint_retarget[10] = config["Kp_joint_retarget10"].as<double>();
    Kp_joint_retarget[11] = config["Kp_joint_retarget11"].as<double>();


    Kd_joint_retarget[0] = config["Kd_joint_retarget0"].as<double>();
    Kd_joint_retarget[1] = config["Kd_joint_retarget1"].as<double>();
    Kd_joint_retarget[2] = config["Kd_joint_retarget2"].as<double>();
    Kd_joint_retarget[3] = config["Kd_joint_retarget3"].as<double>();
    Kd_joint_retarget[4] = config["Kd_joint_retarget4"].as<double>();
    Kd_joint_retarget[5] = config["Kd_joint_retarget5"].as<double>();
    Kd_joint_retarget[6] = config["Kd_joint_retarget6"].as<double>();
    Kd_joint_retarget[7] = config["Kd_joint_retarget7"].as<double>();
    Kd_joint_retarget[8] = config["Kd_joint_retarget8"].as<double>();
    Kd_joint_retarget[9] = config["Kd_joint_retarget9"].as<double>();
    Kd_joint_retarget[10] = config["Kd_joint_retarget10"].as<double>();
    Kd_joint_retarget[11] = config["Kd_joint_retarget11"].as<double>();

    kpp[0] = config["kpp0"].as<double>();
    kpp[1] = config["kpp1"].as<double>();
    kpp[2] = config["kpp2"].as<double>();
    kdp[0] = config["kdp0"].as<double>();
    kdp[1] = config["kdp1"].as<double>();
    kdp[2] = config["kdp2"].as<double>();
    kpw[0] = config["kpw0"].as<double>();
    kpw[1] = config["kpw1"].as<double>();
    kpw[2] = config["kpw2"].as<double>();
    kdw[0] = config["kdw0"].as<double>();
    kdw[1] = config["kdw1"].as<double>();
    kdw[2] = config["kdw2"].as<double>();

    swing_kp(0,0) = config["swingkp0"].as<double>();
    swing_kp(1,1) = config["swingkp1"].as<double>();
    swing_kp(2,2) = config["swingkp2"].as<double>();
    swing_kd(0,0) = config["swingkd0"].as<double>();
    swing_kd(1,1) = config["swingkd1"].as<double>();
    swing_kd(2,2) = config["swingkd2"].as<double>();

    body_p_Homing_Retarget(0,0)  = config["body_p_Homing_Retarget0"].as<double>();
    body_p_Homing_Retarget(1,0)  = config["body_p_Homing_Retarget1"].as<double>();
    body_p_Homing_Retarget(2,0)  = config["body_p_Homing_Retarget2"].as<double>();

    body_r_Homing_Retarget(0,0)  = config["body_r_Homing_Retarget0"].as<double>();
    body_r_Homing_Retarget(1,0)  = config["body_r_Homing_Retarget1"].as<double>();
    body_r_Homing_Retarget(2,0)  = config["body_r_Homing_Retarget2"].as<double>();    

    clear_height = config["foot_clear_height"].as<double>();

    w_pos_m_kp[0] = config["w_pos_m_kp0"].as<double>();
    w_pos_m_kp[1] = config["w_pos_m_kp1"].as<double>();
    w_pos_m_kp[2] = config["w_pos_m_kp2"].as<double>();
    w_pos_m_kd[0] = config["w_pos_m_kd0"].as<double>();
    w_pos_m_kd[1] = config["w_pos_m_kd1"].as<double>();
    w_pos_m_kd[2] = config["w_pos_m_kd2"].as<double>();
    w_rpy_m_kp[0] = config["w_rpy_m_kp0"].as<double>();
    w_rpy_m_kp[1] = config["w_rpy_m_kp1"].as<double>();
    w_rpy_m_kp[2] = config["w_rpy_m_kp2"].as<double>();
    w_rpy_m_kd[0] = config["w_rpy_m_kd0"].as<double>();
    w_rpy_m_kd[1] = config["w_rpy_m_kd1"].as<double>();
    w_rpy_m_kd[2] = config["w_rpy_m_kd2"].as<double>();

    com_rpy_max = config["com_rpy_max2"].as<double>();
    com_rpy_min = config["com_rpy_min2"].as<double>();
    com_pos_max = config["com_pos_max2"].as<double>();
    com_pos_min = config["com_pos_min2"].as<double>();

    using_ff = config["using_ff1"].as<double>();
    uisng_current_jaco = config["uisng_current_jaco1"].as<double>();
    global_com_feedback = config["global_com_feedback1"].as<double>();
    using_grf_node = config["using_grf_node1"].as<double>();

    dtx = config["dt_rt_loop1"].as<double>();
    fcutoff_comx51 = config["fcut_off51"].as<double>();
    fcutoff_comx4 = config["fcutoff_comx41"].as<double>();
    fcutoff_comx1 = config["fcut_off11"].as<double>();
    fcutoff_comx2 = config["fcut_off21"].as<double>();
    fcutoff_comx3 = config["fcut_off31"].as<double>();

    fz_limit = config["fz_limit1"].as<double>();
    debug_mode= config["debug_mode1"].as<double>();
    swing_leg_test= config["test_leg"].as<double>();  
    switch_support= config["switch_support1"].as<double>();   
    using_rotz =   config["using_rotz"].as<double>();    
    using_ros_time = config["using_ros_time"].as<double>();  
    judge_early_contact = config["judge_early_contact"].as<double>();
    judge_later_contact = config["judge_later_contact"].as<double>();   
    using_ekf = config["using_ekf"].as<double>(); 
    tracking_global_foot = config["tracking_global_foot"].as<double>();  
    enable_spring = config["enable_spring"].as<double>(); 

    FR_k_spring_hip = config["FR_k_spring_hip"].as<double>(); 
    FL_k_spring_hip = config["FL_k_spring_hip"].as<double>(); 
    RR_k_spring_hip = config["RR_k_spring_hip"].as<double>(); 
    RL_k_spring_hip = config["RL_k_spring_hip"].as<double>(); 
    FR_k_spring_thigh = config["FR_k_spring_thigh"].as<double>(); 
    FL_k_spring_thigh = config["FL_k_spring_thigh"].as<double>(); 
    RR_k_spring_thigh = config["RR_k_spring_thigh"].as<double>(); 
    RL_k_spring_thigh = config["RL_k_spring_thigh"].as<double>(); 
    FR_k_spring_calf = config["FR_k_spring_calf"].as<double>(); 
    FL_k_spring_calf = config["FL_k_spring_calf"].as<double>(); 
    RR_k_spring_calf = config["RR_k_spring_calf"].as<double>(); 
    RL_k_spring_calf = config["RL_k_spring_calf"].as<double>(); 
    test_stepping_in_place = config["test_stepping_in_place"].as<double>();

    dt_mpc_slow  = config["dt_slow_mpc"].as<double>();
    tstep = config["t_period"].as<double>();
    fsr_contact = config["fsr_contact"].as<double>();

    use_terrain_adapt = config["use_terrain_adapt"].as<double>(); 

    cout<<"debug_mode:"<<debug_mode<<endl;


}





///////// class object
Quadruped::Quadruped(uint8_t level): safe(LeggedType::Go1), udp(level){
    udp.InitCmdData(cmd);

    //////////////////////////======================
    std::cout << "===========================" << std::endl;
    std::cout << "Variable initialization" << std::endl;
    std::cout << "==========================="  << std::endl;
    //////////////////////////////=====================
    //////
    motiontime=0;

    torque.setZero();
    torque_ff.setZero();



    
    joint2simulation.position.resize(100);
    joint2simulationx.position.resize(100);  
    leg2sim.position.resize(100); 
    date2wbc.position.resize(100); 

    initiated_flag = false;  // initiate need time

    cmd_gait = STAND_INIT;
    gait_status = STAND_INIT_STATUS;




    stand_count = 0;
    stand_up_count = 0;
    dynamic_count = 0;

    ///////// leg controll ==============================//////////////////
    //=========spring force forward compensation=============
    k_spring_calf = 3;
    k_p_rest_calf = -1.3;
    k_spring_thigh = 21; 
    k_p_rest_thigh = 0.75;
    k_spring_hip = 5;
    k_p_rest_hip = 0;

    torque_err.setZero();
    torque_err_intergration.setZero();
    Torque_ff_spring.setZero();
    Torque_ff_GRF.setZero();
    Torque_ff_GRF_opt.setZero();

    ///////let impedance control
    swing_kp.setZero();
    swing_kd.setZero();

    swing_kp(0,0) = 230;    swing_kp(1,1) = 150;    swing_kp(2,2) = 260;
    swing_kd(0,0) = 10;      swing_kd(1,1) = 10;      swing_kd(2,2) = 20;

    global_swing.setIdentity();
    global_swing(0,0) = 0.01;    global_swing(1,1) = 0.01;    global_swing(2,2) = 0.01;


    /////////===== feedback PID joint controller===========
    torq_kp_hip= 8;  
    torq_kd_hip= 0.3;  
    torq_ki_hip= 0.05; 

    torq_kp_thigh = 9; 
    torq_kd_thigh = 0.3; 
    torq_ki_thigh = 0.05;

    torq_kp_calf = 9;
    torq_kd_calf = 0.31;
    torq_ki_calf = 0.05;

   ///// landing controller
    hip_kp_scale = 1;
    hip_kd_scale = 1;   
    thigh_kp_scale = 1; ///
    thigh_kd_scale = 1;     
    calf_kp_scale = 1; ///
    calf_kd_scale = 1;



    for(int j=0; j<12; j++)
    {
        Kp_joint[j] = Kp_joint_ini[j];
        Kd_joint[j] = Kd_joint_ini[j];
    }
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
    body_p_Homing_Retarget << -0,
                              0,
                              0.26;

    body_r_Homing_Retarget << 0,
                              0,
                              0;                              
    FR_foot_Homing.setZero();
    FL_foot_Homing.setZero();
    RR_foot_Homing.setZero();
    RL_foot_Homing.setZero();

    body_p_Homing_dynamic.setZero();

    body_r_Homing.setZero();
    body_r_Homing_dynamic.setZero();
    body_p_des.setZero();
    body_r_des.setZero();

    body_p_des_old_ini.setZero();
    com_sensor_old_ini.setZero();
    init_count = 0; 
    init_count_old = 0;



    FR_foot_des.setZero(); 
    FL_foot_des.setZero(); 
    RR_foot_des.setZero(); 
    RL_foot_des.setZero();
    
    FR_footv_des.setZero();
    FL_footv_des.setZero(); 
    RR_footv_des.setZero(); 
    RL_footv_des.setZero();    
    //// measure global position /////
    FR_foot_mea.setZero(); FL_foot_mea.setZero();
    RR_foot_mea.setZero(); RL_foot_mea.setZero(); 
    body_p_est.setZero();
    body_r_est.setZero();

    FR_foot_mea_old.setZero(); 
    FL_foot_mea_old.setZero();
    RR_foot_mea_old.setZero(); 
    RL_foot_mea_old.setZero();
    FR_v_est.setZero(); 
    FL_v_est.setZero();
    RR_v_est.setZero(); 
    RL_v_est.setZero();


    leg_position.setZero();

    root_pos.setZero();
    root_quat.setIdentity();
    root_euler.setZero();
    root_euler_offset.setZero();
    root_euler_angular_velocity_offset.setZero();
    root_rot_mat.setZero();
    root_rot_mat_z.setZero();
    root_lin_vel.setZero();
    root_ang_vel.setZero();
    root_acc.setZero();
    yaw_angle = 0;

    rate = 0;
    ratex = 0;
    rate_stand_up = 0;
    q_ini.setZero();
    
    nt_slow_mpc = 0.0;

    //=== Force distribution 
    kpp<< 110,
          110,
          100;
    kdp<< 14,
          8,
          8;

    kpw<< 300,
          600,
          100;

    kdw<< 50,
          50,
          50;                    

    Grf_sub.setZero();
    F_sum.setZero();
    Momentum_sum << 0.0168352186, 0.0004636141, 0.0002367952,
                    0.0004636141, 0.0656071082, 3.6671e-05, 
                    0.0002367952, 3.6671e-05,   0.0742720659;
    // Momentum_sum *= 10;
    //cout<<Momentum_sum<<endl;

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
    FR_GRF_opt.setZero();
    FL_GRF_opt.setZero();
    RR_GRF_opt.setZero();
    RL_GRF_opt.setZero();


    FR_swing = true; 
    FL_swing = true;
    RR_swing = true;
    RL_swing = true;  

    FR_foot_desx = 0;
    FR_foot_desy = 0;
    FR_foot_desz = 0;  

    FL_foot_desx = 0;
    FL_foot_desy = 0;
    FL_foot_desz = 0;                  
    
    ////////////////////////=================================================//////////////////////////////////////
    ////////////////////////// state estimation///////////////////////
    state_kine.setZero();    
    
    support_flag = 0; /////left: 0; right: 1; double: 2
    omega_sensor = sqrt(gait::_g/body_p_Homing_Retarget(2,0));

        
    support_pos_sensor[0] = 0; ///left support by default
    support_pos_sensor[1] = 0;
    if(gait_mode==101)
    {
        support_pos_sensor[1] = gait::RobotParaClass_HALF_HIP_WIDTH; ///left support by default
        
    }
    support_pos_sensor[2] = 0; ///left support by default

    support_pos_des[0] = support_pos_sensor[0];
    support_pos_des[1] = support_pos_sensor[1];
    support_pos_des[2] = support_pos_sensor[2];


    com_sensor[0] = 0; /////assuming com is 10cm above the pelvis
    com_sensor[1] = 0;
    com_sensor[2] = (body_p_Homing_Retarget(2,0));
    com_sensor_hip[0] = 0; /////assuming com is 10cm above the pelvis
    com_sensor_hip[1] = 0;
    com_sensor_hip[2] = (body_p_Homing_Retarget(2,0));	

    using_ft_sensor = false;       

    for(int i = 0; i < 3; i++){

        com_sensor_pre[i] = 0;
        com_des[i] = 0;
        com_des_pre[i] = 0;
        comv_des[i] = 0;
        coma_des[i] = 0;
        rfoot_des[i] = 0;
        lfoot_des[i] = 0;
        theta_des[i] = 0;
        thetav_des[i] = 0;
        theta_acc_des[i] = 0;
        theta_des_pre[i] = 0;
        rfoot_theta_des[i] = 0;
        lfoot_theta_des[i] = 0;

        comv_sensor[i] = 0;
        comv_sensor_raw[i] = 0;
        coma_sensor[i] = 0;
        zmp_sensor[i] = 0;
        zmp_ref[i] = 0;
        dcm_ref[i]= 0;
        dcm_sensor[i] = 0;    
        w_pos_m[i] = 0;    
        w_rpy_m[i] = 0;   
        w_pos_m_filter[i] = 0;    
        w_rpy_m_filter[i] = 0;             

        frfoot_pose_sensor[i] = 0;
        flfoot_pose_sensor[i] = 0;
        rrfoot_pose_sensor[i] = 0;
        rlfoot_pose_sensor[i] = 0;

        com_des[2] = body_p_Homing_Retarget(2,0);
        
        if(gait_mode==101)
        {
            rfoot_des[1] = -gait::RobotParaClass_HALF_HIP_WIDTH;
            lfoot_des[1] = gait::RobotParaClass_HALF_HIP_WIDTH;
        } 

        body_relative_support_des_old[i] =  com_des[i] - support_pos_des[i];
        body_relative_support_sensor_old[i] =  com_sensor[i] - support_pos_sensor[i];
        body_relative_supportv_des[i] = 0; 
        body_relative_supportv_sensor[i] = 0;
    }

    comav_butterworth.setZero();
    theta_default.setZero();


    L_com.setZero();
    com_estkine.setZero();
    cop_estkine.setZero();
    theta_estkine.setZero();
    theta_estkine_pre.setZero();
    thetav_estkine.setZero();
    thetav_estkine_raw.setZero();
    thetaa_estkine.setZero();
    Fr_estkine = 0;
    Fl_estkine = 0;
    comv_estkine.setZero();
    dob_result.setZero();
    J_ini_xx_est = 0;
    J_ini_yy_est = 0;




//  =========================== real-time control loop ==================//////////////	
    //////////////// load config parameters/////////
    config_set();

	state_to_MPC.position.resize(25);
	state_feedback.setZero();
	slow_mpc_gait.setZero();   
	count_in_rt_loop = 0;
	count_in_rt_ros = 0;

	t_int = 0;  
    n_rt = round(1/dtx);
	n_t_int = (int) round(dt_mpc_slow /dtx); 
	n_t_int_fast = (int) round(gait::dt_mpc_fast /dtx); 
    start_grf = 0;

	
	//// gait filter
    f_sample_comx1 = 1/dtx;
	butterworthLPF1.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF2.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF3.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF4.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF5.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF6.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF7.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF8.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF9.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF10.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF11.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF12.init(f_sample_comx1,fcutoff_comx2);

	butterworthLPF31.init(f_sample_comx1,fcutoff_comx2);
	butterworthLPF32.init(f_sample_comx1,fcutoff_comx2);
	butterworthLPF33.init(f_sample_comx1,fcutoff_comx2);
	butterworthLPF34.init(f_sample_comx1,fcutoff_comx2);
	butterworthLPF35.init(f_sample_comx1,fcutoff_comx2);
	butterworthLPF36.init(f_sample_comx1,fcutoff_comx2);
	butterworthLPF37.init(f_sample_comx1,fcutoff_comx2);
	// butterworthLPF38.init(f_sample_comx1,fcutoff_comx2);
	// butterworthLPF39.init(f_sample_comx1,fcutoff_comx2);
	// butterworthLPF30.init(f_sample_comx1,fcutoff_comx2);    

	butterworthLPF13.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF14.init(f_sample_comx1,fcutoff_comx3);    
	butterworthLPF15.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF16.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF17.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF18.init(f_sample_comx1,fcutoff_comx3);

	butterworthLPF19.init(f_sample_comx1,fcutoff_comx4);	
	butterworthLPF20.init(f_sample_comx1,fcutoff_comx4);
	butterworthLPF21.init(f_sample_comx1,fcutoff_comx4);	
	butterworthLPF22.init(f_sample_comx1,fcutoff_comx4);
    butterworthLPF23.init(f_sample_comx1,fcutoff_comx4);    
	butterworthLPF24.init(f_sample_comx1,fcutoff_comx4);	
	butterworthLPF25.init(f_sample_comx1,fcutoff_comx4);	
	butterworthLPF26.init(f_sample_comx1,fcutoff_comx4);	
	butterworthLPF27.init(f_sample_comx1,fcutoff_comx4);	
	butterworthLPF28.init(f_sample_comx1,fcutoff_comx4);	
	butterworthLPF29.init(f_sample_comx1,fcutoff_comx4);	
	butterworthLPF30.init(f_sample_comx1,fcutoff_comx4);	
    
    f_sample_comx2 = 1/gait::dt_grf;
    fcutoff_comx5 = 500; 
    butterworthLPF41.init(f_sample_comx1,fcutoff_comx5); butterworthLPF42.init(f_sample_comx1,fcutoff_comx5); 
    butterworthLPF43.init(f_sample_comx1,fcutoff_comx5); butterworthLPF44.init(f_sample_comx1,fcutoff_comx5);
    butterworthLPF45.init(f_sample_comx1,fcutoff_comx5); butterworthLPF46.init(f_sample_comx1,fcutoff_comx5);
    butterworthLPF47.init(f_sample_comx1,fcutoff_comx5); butterworthLPF48.init(f_sample_comx1,fcutoff_comx5); 
    butterworthLPF49.init(f_sample_comx1,fcutoff_comx5); butterworthLPF50.init(f_sample_comx1,fcutoff_comx5);
    butterworthLPF51.init(f_sample_comx1,fcutoff_comx5); butterworthLPF52.init(f_sample_comx1,fcutoff_comx5);

    butterworthLPF53.init(f_sample_comx1,fcutoff_comx51); butterworthLPF54.init(f_sample_comx1,fcutoff_comx51); 
    butterworthLPF55.init(f_sample_comx1,fcutoff_comx51); butterworthLPF56.init(f_sample_comx1,fcutoff_comx51);
    butterworthLPF57.init(f_sample_comx1,fcutoff_comx51); butterworthLPF58.init(f_sample_comx1,fcutoff_comx51);
    butterworthLPF59.init(f_sample_comx1,fcutoff_comx51); butterworthLPF60.init(f_sample_comx1,fcutoff_comx51); 
    butterworthLPF61.init(f_sample_comx1,fcutoff_comx51); butterworthLPF62.init(f_sample_comx1,fcutoff_comx51);
    butterworthLPF63.init(f_sample_comx1,fcutoff_comx51);
    butterworthLPF64.init(f_sample_comx1,fcutoff_comx51); butterworthLPF65.init(f_sample_comx1,fcutoff_comx51); 
    butterworthLPF66.init(f_sample_comx1,fcutoff_comx51); butterworthLPF67.init(f_sample_comx1,fcutoff_comx51);
    butterworthLPF68.init(f_sample_comx1,fcutoff_comx51); butterworthLPF69.init(f_sample_comx1,fcutoff_comx51);
    butterworthLPF70.init(f_sample_comx1,fcutoff_comx51);
    /// for acceleration feeback
    butterworthLPF71.init(f_sample_comx1,fcutoff_comx51); butterworthLPF72.init(f_sample_comx1,fcutoff_comx51); 
    butterworthLPF73.init(f_sample_comx1,fcutoff_comx51); butterworthLPF74.init(f_sample_comx1,fcutoff_comx51);
    butterworthLPF75.init(f_sample_comx1,fcutoff_comx51); butterworthLPF76.init(f_sample_comx1,fcutoff_comx51);
    butterworthLPF77.init(f_sample_comx1,fcutoff_comx51); butterworthLPF78.init(f_sample_comx1,fcutoff_comx51);
    butterworthLPF79.init(f_sample_comx1,fcutoff_comx51);

	butterworthLPF80.init(f_sample_comx1,fcutoff_comx51);	
	butterworthLPF81.init(f_sample_comx1,fcutoff_comx51);    
	butterworthLPF82.init(f_sample_comx1,fcutoff_comx51);	
	butterworthLPF83.init(f_sample_comx1,fcutoff_comx51);	
	butterworthLPF84.init(f_sample_comx1,fcutoff_comx51);	
	butterworthLPF85.init(f_sample_comx1,fcutoff_comx51);


    for (int j = 0; j < 4; ++j) {
        recent_contact_x_filter[j] = MovingWindowFilter(10);
        recent_contact_y_filter[j] = MovingWindowFilter(10);
        recent_contact_z_filter[j] = MovingWindowFilter(10);
    }
    ground_angle.setZero();


    
    pitch_angle_W = 0;

    n_period = round(tstep / dtx); 

    rt_frequency = ctrl_estimation; /// frequency of lower_level control
    time_programming = 1.0/rt_frequency;

    n_count = 0;
    stand_duration = 5; /// stand up: 2s


    gait_data_pub = n.advertise<sensor_msgs::JointState>("go1_gait_data",10);
    gait_data_pubx = n.advertise<sensor_msgs::JointState>("go1_gait_datax",10);    
    gait_step_real_time = n.advertise<sensor_msgs::JointState>("/go1_gait_parameters",10);   
    state_estimate_ekf_pub_ = n.advertise<sensor_msgs::JointState>("go1_state_estmate",10);
    rt_to_nrt_pub_ = n.advertise<sensor_msgs::JointState>("/rt2nrt/state", 10);

    robot_mode_cmd = n.subscribe("/Robot_mode", 1, &Quadruped::keyboard_model_callback,this); //// for robot mode selection
    nrt_mpc_gait_subscribe_ = n.subscribe("/MPC/Gait", 10, &Quadruped::nrt_gait_sub_operation,this);
    Grf_gait_subscribe_ = n.subscribe("/go1/Grf_opt", 10, &Quadruped::Grf_sub_operation,this);
    gain_cmd = n.subscribe("/Base_offset", 1, &Quadruped::Base_offset_callback,this);
  

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
    }


    ////////////////////======================================///////////////////
    ////////////////        state estimation:variable declaration     //////////////////////////////    
    a1_ctrl_states.reset();
    
    // init leg kinematics
    // set leg kinematics related parameters
    // body_to_a1_body
    p_br = Eigen::Vector3d(-0.2293, 0.0, -0.067);
    R_br = Eigen::Matrix3d::Identity();

    // leg order: 0-FL  1-FR  2-RL  3-RR
    leg_offset_x[0] = 0.1881;
    leg_offset_x[1] = 0.1881;
    leg_offset_x[2] = -0.1881;
    leg_offset_x[3] = -0.1881;
    leg_offset_y[0] = 0.04675;
    leg_offset_y[1] = -0.04675;
    leg_offset_y[2] = 0.04675;
    leg_offset_y[3] = -0.04675;
    motor_offset[0] = 0.08;
    motor_offset[1] = -0.08;
    motor_offset[2] = 0.08;
    motor_offset[3] = -0.08;
    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = 0.213;
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = LOWER_LEG_LENGTH;

    for (int i = 0; i < NUM_LEG; i++) {
        Eigen::VectorXd rho_fix(5);
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
        Eigen::VectorXd rho_opt(3);
        rho_opt << 0.0, 0.0, 0.0;
        rho_fix_list.push_back(rho_fix);
        rho_opt_list.push_back(rho_opt);
    }

    acc_x = MovingWindowFilter(5);
    acc_y = MovingWindowFilter(5);
    acc_z = MovingWindowFilter(5);
    gyro_x = MovingWindowFilter(5);
    gyro_y = MovingWindowFilter(5);
    gyro_z = MovingWindowFilter(5);
    quat_w = MovingWindowFilter(5);
    quat_x = MovingWindowFilter(5);
    quat_y = MovingWindowFilter(5);
    quat_z = MovingWindowFilter(5);

    footforce_fr.setZero();
    footforce_fl.setZero();
    footforce_rr.setZero();
    footforce_rl.setZero();
    estimated_root_vel_rotz.setZero();
    linear_vel_rotz.setZero();
    estimated_root_pos_offset.setZero();
    estimation_offset.setZero();

    foot_contact_flag.setZero();
    foot_earlier_contact_flag.setZero();
    foot_earlier_contact_flag_old;

    foot_contact_position.setZero();

    fz_load_ratio = 0;
    



}

void Quadruped::UDPRecv()
{  
    udp.Recv();
}

void Quadruped::UDPSend()
{  
    udp.Send();
}





void Quadruped::RobotControl()
{

    auto t3 = std::chrono::high_resolution_clock::now();
    now = ros::Time::now();
    dt_rt_loop = now - prev;
    prev = now;

    udp.GetRecv(statex_quadrupedal);
    RecvLowROS = statex_quadrupedal;


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


    //// IMU quaternion
    root_quat = Eigen::Quaterniond(quat_w.CalculateAverage(RecvLowROS.imu.quaternion[0]),
                                    quat_x.CalculateAverage(RecvLowROS.imu.quaternion[1]),
                                    quat_y.CalculateAverage(RecvLowROS.imu.quaternion[2]),
                                    quat_z.CalculateAverage(RecvLowROS.imu.quaternion[3])); 

    //root_quat = root_quat.normalized();
    // euler angle: roll pitch yaw
    //root_rot_mat = (root_quat.normalized()).toRotationMatrix();
    root_euler = Utils::quat_to_euler(root_quat);

    if(n_count>5001)
    {
        body_r_est(0,0) = root_euler(0) - root_euler_offset(0);
        body_r_est(1,0) = root_euler(1) - root_euler_offset(1);
        body_r_est(2,0) = root_euler(2) - root_euler_offset(2);
    }
    /// modify the quaternion and 
    root_rot_mat =  Eigen::AngleAxisd(body_r_est(2,0), Eigen::Vector3d::UnitZ())
                    *Eigen::AngleAxisd(body_r_est(1,0), Eigen::Vector3d::UnitY())
                    *Eigen::AngleAxisd(body_r_est(0,0), Eigen::Vector3d::UnitX());

    // Eigen::Quaternionf root_quat(root_rot_mat);                

    yaw_angle = body_r_est(2,0);
    root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());


    Momentum_global = root_rot_mat * Momentum_sum * root_rot_mat.transpose();     

    ///===== global framework;======
    body_p_est.setZero();
    FR_foot_mea = Kine.Forward_kinematics_g(body_p_est, body_r_est,FR_angle_mea, 0);
    FR_Jaco_est = Kine.Jacobian_kin;
    FL_foot_mea = Kine.Forward_kinematics_g(body_p_est, body_r_est,FL_angle_mea, 1);
    FL_Jaco_est = Kine.Jacobian_kin;
    RR_foot_mea = Kine.Forward_kinematics_g(body_p_est, body_r_est,RR_angle_mea, 2);
    RR_Jaco_est = Kine.Jacobian_kin;
    RL_foot_mea = Kine.Forward_kinematics_g(body_p_est, body_r_est,RL_angle_mea, 3);
    RL_Jaco_est = Kine.Jacobian_kin;
    FR_foot_relative_mea = FR_foot_mea - body_p_est;
    FL_foot_relative_mea = FL_foot_mea - body_p_est;
    RR_foot_relative_mea = RR_foot_mea - body_p_est;
    RL_foot_relative_mea = RL_foot_mea - body_p_est;
    
    //// relative velocity in global framework: another way is to use differential of measured retlative position;
    FR_v_est_relative = FR_Jaco_est * FR_dq_mea;
    FL_v_est_relative = FL_Jaco_est * FL_dq_mea; 
    RR_v_est_relative = RR_Jaco_est * RR_dq_mea;
    RL_v_est_relative = RL_Jaco_est * RL_dq_mea; 
    
    // if (count_in_rt_ros > 1)
    // {
    //     FR_v_est_relative = (FR_foot_relative_mea - FR_foot_relative_mea_old)/dtx;
    //     FL_v_est_relative = (FL_foot_relative_mea - FL_foot_relative_mea_old)/dtx;
    //     RR_v_est_relative = (RR_foot_relative_mea - RR_foot_relative_mea_old)/dtx;
    //     RL_v_est_relative = (RL_foot_relative_mea - RL_foot_relative_mea_old)/dtx;            
    // }

    //////============ state_estimation ///////////////////////////////
    //// feet force generation
    if(count_in_rt_ros<5)
    {
        footforce_fr(count_in_rt_ros,0) = RecvLowROS.footForce[0];
        footforce_fl(count_in_rt_ros,0) = RecvLowROS.footForce[1];
        footforce_rr(count_in_rt_ros,0) = RecvLowROS.footForce[2];
        footforce_rl(count_in_rt_ros,0) = RecvLowROS.footForce[3];
    }
    else
    {
        footforce_fr.block<4,1>(0,0) = footforce_fr.block<4,1>(1,0);
        footforce_fl.block<4,1>(0,0) = footforce_fl.block<4,1>(1,0);
        footforce_rr.block<4,1>(0,0) = footforce_rr.block<4,1>(1,0);
        footforce_rl.block<4,1>(0,0) = footforce_rl.block<4,1>(1,0);

        footforce_fr(4,0) = RecvLowROS.footForce[0];
        footforce_fl(4,0) = RecvLowROS.footForce[1];
        footforce_rr(4,0) = RecvLowROS.footForce[2];
        footforce_rl(4,0) = RecvLowROS.footForce[3];
    }


    ///// estimate global COM position
    if(gait_status == STAND_INIT_STATUS) //// stand up generation;
    {
        com_sensor[0] = (FR_foot_des(0,0) - FR_foot_relative_mea(0,0) + FL_foot_des(0,0) - FL_foot_relative_mea(0,0)+
                            RR_foot_des(0,0) - FR_foot_relative_mea(0,0) + (RL_foot_des(0,0) - RL_foot_relative_mea(0,0)))/4;

        com_sensor[1] = (FR_foot_des(1,0) - FR_foot_relative_mea(1,0) + FL_foot_des(1,0) - FL_foot_relative_mea(1,0)+
                            RR_foot_des(1,0) - FR_foot_relative_mea(1,0) + (RL_foot_des(1,0) - RL_foot_relative_mea(1,0)))/4;

        com_sensor[2] = (FR_foot_des(2,0) - FR_foot_relative_mea(2,0) + FL_foot_des(2,0) - FL_foot_relative_mea(2,0)+
                            RR_foot_des(2,0) - FR_foot_relative_mea(2,0) + (RL_foot_des(2,0) - RL_foot_relative_mea(2,0)))/4; 
        for(int j=0; j<3;j++)
        {
            comv_sensor[j] = (com_sensor[j] - com_sensor_pre[j])/dtx;
            /// initialization for kinematic-based estimator
            frfoot_pose_sensor[j] = FR_foot_relative_mea(j,0) + body_p_des(j,0);
            flfoot_pose_sensor[j] = FL_foot_relative_mea(j,0) + body_p_des(j,0);
            rrfoot_pose_sensor[j] = RR_foot_relative_mea(j,0) + body_p_des(j,0);
            rlfoot_pose_sensor[j] = RL_foot_relative_mea(j,0) + body_p_des(j,0); 
            /// initialization for ekf-based estimator
            a1_ctrl_states.estimated_root_pos(j,0) = com_sensor[j];
            a1_ctrl_states.estimated_root_vel(j,0) = comv_sensor[j];
            a1_ctrl_states.root_euler(j,0) = root_euler(j,0);
            a1_ctrl_states.foot_pos_world(j,1) = FR_foot_relative_mea(j,0) + body_p_des(j,0);
            a1_ctrl_states.foot_pos_world(j,0) = FL_foot_relative_mea(j,0) + body_p_des(j,0);
            a1_ctrl_states.foot_pos_world(j,3) = RR_foot_relative_mea(j,0) + body_p_des(j,0);
            a1_ctrl_states.foot_pos_world(j,2) = RL_foot_relative_mea(j,0) + body_p_des(j,0);

            a1_ctrl_states.root_pos(j,0) = com_sensor[j];
            a1_ctrl_states.root_lin_vel(j,0) = comv_sensor[j];

            go1_estimate.x(j,0) = com_sensor[j];
            go1_estimate.x(j+3,0) = comv_sensor[j];
        }
    }
    else
    {
        ///// kinematic-based open-loop state estimation //////////
        comav_butterworth = state_est_kine.state_estimator(gait_mode, support_flag, FR_foot_relative_mea, FL_foot_relative_mea,RR_foot_relative_mea, RL_foot_relative_mea,
                                                footforce_fr, footforce_fl,footforce_rr,footforce_rl,frfoot_pose_sensor, flfoot_pose_sensor, rrfoot_pose_sensor, rlfoot_pose_sensor, 
                                                support_pos_sensor, com_sensor, dcm_sensor, omega_sensor,estimated_root_pos_offset);        

        /////////// global state: EKF based estination //////////
        gait_pose_callback(RecvLowROS);
        if(using_ros_time>0.5)
        {
            state_gait_ekf = state_est_main_update(dt_rt_loop.toSec());////using the actual time inteval
        }
        else
        {
            state_gait_ekf = state_est_main_update(dtx); /// assume a fixed time inteval
        }
        estimated_root_vel_rotz = root_rot_mat.transpose() * a1_ctrl_states.estimated_root_vel;
        linear_vel_rotz = root_rot_mat.transpose()  * a1_ctrl_states.root_ang_vel;
        if(using_ekf>0.5) /////using the ekf estimator
        {
            for(int j=0; j<3;j++)
            {
                com_sensor[j] = (3*a1_ctrl_states.estimated_root_pos(j,0) + 0*com_sensor[j])/3;
                
                if(using_rotz>0.5)
                {
                    comv_sensor[j] = (3*estimated_root_vel_rotz(j,0) + 0*comv_sensor[j])/3;
                }
                else
                {
                    comv_sensor[j] = (3*a1_ctrl_states.estimated_root_vel(j,0) + 0*comv_sensor[j])/3;
                }
                
                //////considering the kinematical relationship
                a1_ctrl_states.foot_pos_world(j,1) = (FR_foot_relative_mea(j,0) + a1_ctrl_states.foot_pos_world(j,1) 
                                                - a1_ctrl_states.estimated_root_pos(j,0))/2 + a1_ctrl_states.estimated_root_pos(j,0);
                a1_ctrl_states.foot_pos_world(j,0) = (FL_foot_relative_mea(j,0) + a1_ctrl_states.foot_pos_world(j,0)
                                                - a1_ctrl_states.estimated_root_pos(j,0))/2 + a1_ctrl_states.estimated_root_pos(j,0);
                a1_ctrl_states.foot_pos_world(j,3) = (RR_foot_relative_mea(j,0) + a1_ctrl_states.foot_pos_world(j,3) 
                                                - a1_ctrl_states.estimated_root_pos(j,0))/2 + a1_ctrl_states.estimated_root_pos(j,0);
                a1_ctrl_states.foot_pos_world(j,2) = (RL_foot_relative_mea(j,0) + a1_ctrl_states.foot_pos_world(j,2) 
                                                - a1_ctrl_states.estimated_root_pos(j,0))/2 + a1_ctrl_states.estimated_root_pos(j,0);                 
                state_gait_ekf.block<3,1>(12,0) = a1_ctrl_states.foot_pos_world.col(1);
                state_gait_ekf.block<3,1>(15,0) = a1_ctrl_states.foot_pos_world.col(0);
                state_gait_ekf.block<3,1>(18,0) = a1_ctrl_states.foot_pos_world.col(3);
                state_gait_ekf.block<3,1>(21,0) = a1_ctrl_states.foot_pos_world.col(2);
            
                FR_foot_mea[j] = a1_ctrl_states.foot_pos_world(j,1);
                FL_foot_mea[j] = a1_ctrl_states.foot_pos_world(j,0);
                RR_foot_mea[j] = a1_ctrl_states.foot_pos_world(j,3);
                RL_foot_mea[j] = a1_ctrl_states.foot_pos_world(j,2);                
            }
            FR_v_est = (FR_foot_mea - FR_foot_mea_old)/dtx; 
            FL_v_est = (FL_foot_mea - FL_foot_mea_old)/dtx;   
            RR_v_est = (RR_foot_mea - RR_foot_mea_old)/dtx; 
            RL_v_est = (RL_foot_mea - RL_foot_mea_old)/dtx;                  
        }
        else
        {
            ////merely modify the height;
            frfoot_pose_sensor[2] = a1_ctrl_states.foot_pos_world(2,1);
            flfoot_pose_sensor[2] = a1_ctrl_states.foot_pos_world(2,0);
            rrfoot_pose_sensor[2] = a1_ctrl_states.foot_pos_world(2,3);
            rlfoot_pose_sensor[2] = a1_ctrl_states.foot_pos_world(2,2); 
            com_sensor[2] = a1_ctrl_states.estimated_root_pos(2,0); 

            for (int i = 0; i < 3; i++)
            {
                comv_sensor[i] = (com_sensor[i] - com_sensor_pre[i])/dtx;
                FR_foot_mea[i] = frfoot_pose_sensor[i];
                FL_foot_mea[i] = flfoot_pose_sensor[i];
                RR_foot_mea[i] = rrfoot_pose_sensor[i];
                RL_foot_mea[i] = rlfoot_pose_sensor[i];
            }
            FR_v_est = (FR_foot_mea - FR_foot_mea_old)/dtx; 
            FL_v_est = (FL_foot_mea - FL_foot_mea_old)/dtx;   
            RR_v_est = (RR_foot_mea - RR_foot_mea_old)/dtx; 
            RL_v_est = (RL_foot_mea - RL_foot_mea_old)/dtx;                               
        }   
    }
    theta_estkine = body_r_est;


    body_p_est(0,0) = com_sensor[0];
    body_p_est(1,0) = com_sensor[1];
    body_p_est(2,0) = com_sensor[2];
    


    if (count_in_rt_ros > 1)
    {
        if(using_rotz>0.5)
        {
            thetav_estkine = linear_vel_rotz;
        }
        else
        {
            thetav_estkine = a1_ctrl_states.root_ang_vel;
        }
    }

    
    // comv_sensor[0] = butterworthLPF80.filter(comv_sensor_raw[0]);
    // comv_sensor[1] = butterworthLPF81.filter(comv_sensor_raw[1]);
    // comv_sensor[2] = butterworthLPF82.filter(comv_sensor_raw[2]); 
    // thetav_estkine[0] = butterworthLPF83.filter(thetav_estkine_raw[0]);
    // thetav_estkine[1] = butterworthLPF84.filter(thetav_estkine_raw[1]);
    // thetav_estkine[2] = butterworthLPF85.filter(thetav_estkine_raw[2]);

    // comv_sensor[0] = (comv_sensor_raw[0]);
    // comv_sensor[1] = (comv_sensor_raw[1]);
    // comv_sensor[2] = (comv_sensor_raw[2]); 
    // thetav_estkine[0] = (thetav_estkine_raw[0]);
    // thetav_estkine[1] = (thetav_estkine_raw[1]);
    // thetav_estkine[2] = (thetav_estkine_raw[2]);        


    //////==== send data to nlp node=====///////////////////
    t_int += (int) floor(count_in_rt_loop/n_t_int);

    state_feedback(0,0) = t_int;
    /// comx(pva),comy(pva),comz(pva),thetax(pva),thetay(pva),thetaz(pva)
    // state_feedback.block<3,1>(1,0) = state_gait_ekf.block<3,1>(0,0);
    // state_feedback.block<3,1>(4,0) = state_gait_ekf.block<3,1>(3,0);
    // state_feedback.block<3,1>(10,0) = state_gait_ekf.block<3,1>(6,0);
    // state_feedback.block<3,1>(13,0) = state_gait_ekf.block<3,1>(9,0);
    state_feedback(1,0) = com_sensor[0] - body_p_Homing_dynamic[0] + body_p_Homing_Retarget(0,0);
    state_feedback(2,0) = comv_sensor[0];
    state_feedback(3,0) = comav_butterworth(0,0);
    state_feedback(4,0) = com_sensor[1] - body_p_Homing_dynamic[1]+ body_p_Homing_Retarget(1,0);
    state_feedback(5,0) = comv_sensor[1];
    state_feedback(6,0) = comav_butterworth(1,0);
    state_feedback(7,0) = com_sensor[2];
    state_feedback(8,0) = comv_sensor[2];
    state_feedback(9,0) = comav_butterworth(2,0);
    state_feedback(10,0) = body_r_est(0,0);
    state_feedback(11,0) = body_r_est(1,0);
    state_feedback(12,0) = body_r_est(2,0); ////send back the current yaw angle;

    if(gait_mode==102)// Lfoot_location & Rfoot_location
    {
        state_feedback.block<3,1>(19,0) = (FR_foot_mea- FR_foot_Homing + RL_foot_mea -RL_foot_Homing)/2; ////left foot
        state_feedback.block<3,1>(22,0) = (FL_foot_mea- FL_foot_Homing + RR_foot_mea- RR_foot_Homing)/2; ////right foot
    }

    for (int jx = 0; jx<25; jx++)
    {
        state_to_MPC.position[jx] = state_feedback(jx,0);
    } 
    rt_to_nrt_pub_.publish(state_to_MPC);


    // ============== Key board controller ======================
    base_offset_x = clamp_func(base_offset_x,base_offset_x_old, 0.001);
    base_offset_y = clamp_func(base_offset_y,base_offset_y_old, 0.001);
    base_offset_z = clamp_func(base_offset_z,base_offset_z_old, 0.001);        
    base_offset_pitch = clamp_func(base_offset_pitch,base_offset_pitch_old, 0.001);
    base_offset_roll = clamp_func(base_offset_roll,base_offset_roll_old, 0.001);
    base_offset_yaw = clamp_func(base_offset_yaw,base_offset_yaw_old, 0.001);

    Grf_sub_filter[0] = butterworthLPF41.filter(Grf_sub(0));
    Grf_sub_filter[1] = butterworthLPF42.filter(Grf_sub(1));
    Grf_sub_filter[3-1] = butterworthLPF43.filter(Grf_sub(2));
    Grf_sub_filter[4-1] = butterworthLPF44.filter(Grf_sub(3));
    Grf_sub_filter[5-1] = butterworthLPF45.filter(Grf_sub(4));
    Grf_sub_filter[6-1] = butterworthLPF46.filter(Grf_sub(5));
    Grf_sub_filter[7-1] = butterworthLPF47.filter(Grf_sub(6));
    Grf_sub_filter[8-1] = butterworthLPF48.filter(Grf_sub(7));
    Grf_sub_filter[9-1] = butterworthLPF49.filter(Grf_sub(8));
    Grf_sub_filter[10-1] = butterworthLPF50.filter(Grf_sub(9));
    Grf_sub_filter[11-1] = butterworthLPF51.filter(Grf_sub(10));
    Grf_sub_filter[12-1] = butterworthLPF52.filter(Grf_sub(11));  


    Grf_sub_filter[13-1] = butterworthLPF53.filter(Grf_sub(12));
    Grf_sub_filter[14-1] = butterworthLPF54.filter(Grf_sub(13));
    Grf_sub_filter[15-1] = butterworthLPF55.filter(Grf_sub(14));

    Grf_sub_filter[16-1] = butterworthLPF56.filter(Grf_sub(15));
    Grf_sub_filter[17-1] = butterworthLPF57.filter(Grf_sub(16));
    Grf_sub_filter[18-1] = butterworthLPF58.filter(Grf_sub(17));

    Grf_sub_filter[19-1] = butterworthLPF59.filter(Grf_sub(18));
    Grf_sub_filter[20-1] = butterworthLPF60.filter(Grf_sub(19));                            
    Grf_sub_filter[21-1] = butterworthLPF61.filter(Grf_sub(20));

    Grf_sub_filter[22-1] = butterworthLPF62.filter(Grf_sub(21));                            
    Grf_sub_filter[23-1] = butterworthLPF63.filter(Grf_sub(22));
    Grf_sub_filter[24-1] = butterworthLPF64.filter(Grf_sub(23));

    Grf_sub_filter[25-1] = butterworthLPF65.filter(Grf_sub(24));
    Grf_sub_filter[26-1] = butterworthLPF66.filter(Grf_sub(25));
    Grf_sub_filter[27-1] = butterworthLPF67.filter(Grf_sub(26));

    Grf_sub_filter[28-1] = butterworthLPF68.filter(Grf_sub(27));
    Grf_sub_filter[29-1] = butterworthLPF69.filter(Grf_sub(28));
    Grf_sub_filter[30-1] = butterworthLPF70.filter(Grf_sub(29));   


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
                    }
                    break;
                case STAND:
                    gait_status = STAND_STATUS;
                    stand_count++;
                    if(stand_count==1)
                    {
                        printf("===== RETARGETING HOMING POSE =======\n");
                    }
                    if(stand_count<=2000)
                    {
                        if(using_ekf>0.5)
                        {
                            estimation_offset(0,0) += a1_ctrl_states.estimated_root_pos(0,0);
                            estimation_offset(1,0) += a1_ctrl_states.estimated_root_pos(1,0);
                            // estimation_offset(2,0) += a1_ctrl_states.estimated_root_pos(2,0);
                            estimated_root_pos_offset = estimation_offset/stand_count;
                        }
                        else
                        {
                            estimation_offset(0,0) += com_sensor[0];
                            estimation_offset(1,0) += com_sensor[1];
                            // estimation_offset(2,0) += a1_ctrl_states.estimated_root_pos(2,0);
                            estimated_root_pos_offset = estimation_offset/stand_count;                                
                        }
                    }
                    else
                    {
                        if(using_ekf>0.5)
                        {
                            estimated_root_pos_offset.setZero();
                        } 
                        
                    }
                    /////// for normal walking: preparing for stand up
                    ratex = pow(stand_count/1000.0,2); 
                    if(ratex<=1000)
                    {
                        for(int j=0; j<3;j++)
                        {
                            body_p_des[j] = jointLinearInterpolation(body_p_Homing(j,0), body_p_Homing_Retarget(j,0), ratex, 0); 
                            body_r_des[j] = jointLinearInterpolation(body_r_Homing(j,0), body_r_Homing_Retarget(j,0), ratex, 0);                    
                        }
                        body_p_Homing = body_p_des;
                        body_r_Homing = body_r_des;

                            for(int j=0; j<12;j++)
                        {
                            Kp_joint[j] = jointLinearInterpolation(Kp_joint_ini[j], Kp_joint_retarget[j], ratex, 0);  
                            Kd_joint[j] = jointLinearInterpolation(Kd_joint_ini[j], Kd_joint_retarget[j], ratex, 0);

                        }                           
                    }
                    else
                    {   
                        body_p_des[0] = body_p_Homing[0] + base_offset_x;
                        body_p_des[1] = body_p_Homing[1] + base_offset_y;
                        body_p_des[2] = body_p_Homing[2] + base_offset_z;
                        body_r_des[0] = body_r_Homing[0] + base_offset_roll;
                        body_r_des[1] = body_r_Homing[1] + base_offset_pitch;
                        body_r_des[2] = body_r_Homing[2] + base_offset_yaw;                            
                    }

                    ///cout << "xyyyy"<<endl;
                    leg_kinematic();

                    break;
                case STAND_UP:
                    gait_status = STAND_UP_STATUS;
                    stand_up_count++;
                    if(stand_up_count==1)
                    {
                        body_p_Homing[0] = body_p_des[0];
                        body_p_Homing[1] = body_p_des[1];
                        body_p_Homing[2] = body_p_des[2];
                        body_r_Homing[0] = body_r_des[0];
                        body_r_Homing[1] = body_r_des[1];
                        body_r_Homing[2] = body_r_des[2];    
                        for(int ix =0; ix<12;ix++)
                        {
                            sin_mid_q[ix] = qDes[ix];
                        }
                    
                        printf("===== STAND UP GRF compensation======\n");
                        cout<<"body_p_Homing"<<body_p_Homing<<endl;
                        cout<<"body_r_Homing"<<body_r_Homing<<endl;
                    }
                    right_support = 2; 
                    // ////========= key board control for homing posing modulation ========= ///////////////////
                    body_p_des[0] = body_p_Homing[0] + base_offset_x;
                    body_p_des[1] = body_p_Homing[1] + base_offset_y;
                    body_p_des[2] = body_p_Homing[2] + base_offset_z;
                    body_r_des[0] = body_r_Homing[0] + base_offset_roll;
                    body_r_des[1] = body_r_Homing[1] + base_offset_pitch;
                    body_r_des[2] = body_r_Homing[2] + base_offset_yaw; 

                    body_p_des_old_ini = body_p_des;

                    
                    /////////////////=============== base status control ===================/////////////////////
                    ////// body pose feedback
                    base_pos_fb_controler();

                    ratex = pow(stand_up_count/500.0,2);
                    // ratex = std::min(ratex,1);
                    if(stand_up_count<=500)
                    {
                        w_pos_m_filter[0] *= ratex;
                        w_pos_m_filter[1] *= ratex;
                        w_pos_m_filter[2] *= ratex;
                        w_rpy_m_filter[0] *= ratex;
                        w_rpy_m_filter[1] *= ratex;
                        w_rpy_m_filter[2] *= ratex;
                    }
                        
                    body_p_des[0] +=  w_pos_m_filter[0];
                    body_p_des[1] +=  w_pos_m_filter[1];
                    body_p_des[2] +=  w_pos_m_filter[2];
                    body_r_des[0] +=  w_rpy_m_filter[0];
                    body_r_des[1] +=  w_rpy_m_filter[1];
                    body_r_des[2] +=  w_rpy_m_filter[2];

                    if (count_in_rt_loop>1)
                    {
                        comv_des[0] = (body_p_des[0] - com_des_pre[0]) /dtx;
                        comv_des[1] = (body_p_des[1] - com_des_pre[1]) /dtx;
                        comv_des[2] = (body_p_des[2] - com_des_pre[2]) /dtx;

                        thetav_des[0] = (body_r_des[0] - theta_des_pre[0]) /dtx;
                        thetav_des[1] = (body_r_des[1] - theta_des_pre[1]) /dtx;
                        thetav_des[2] = (body_r_des[2] - theta_des_pre[2]) /dtx;                           
                    }
                    ////////===== groun slope estimation===============////////
                    // if (foot_contact_flag(0,0)>0.5) {
                    //     foot_contact_position.block<3, 1>(0, 0)
                    //             << recent_contact_x_filter[0].CalculateAverage(FR_foot_mea(0, 0)),
                    //             recent_contact_y_filter[0].CalculateAverage(FR_foot_mea(1, 0)),
                    //             recent_contact_z_filter[0].CalculateAverage(FR_foot_mea(2, 0));
                    // }
                    // if (foot_contact_flag(1,0)>0.5) {
                    //     foot_contact_position.block<3, 1>(0, 1)
                    //             << recent_contact_x_filter[1].CalculateAverage(FL_foot_mea(0, 0)),
                    //             recent_contact_y_filter[1].CalculateAverage(FL_foot_mea(1, 0)),
                    //             recent_contact_z_filter[1].CalculateAverage(FL_foot_mea(2, 0));
                    // }
                    // if (foot_contact_flag(2,0)>0.5) {
                    //     foot_contact_position.block<3, 1>(0, 2)
                    //             << recent_contact_x_filter[2].CalculateAverage(RR_foot_mea(0, 0)),
                    //             recent_contact_y_filter[2].CalculateAverage(RR_foot_mea(1, 0)),
                    //             recent_contact_z_filter[2].CalculateAverage(RR_foot_mea(2, 0));
                    // }
                    // if (foot_contact_flag(3,0)>0.5) {
                    //     foot_contact_position.block<3, 1>(0, 3)
                    //             << recent_contact_x_filter[3].CalculateAverage(RL_foot_mea(0, 0)),
                    //             recent_contact_y_filter[3].CalculateAverage(RL_foot_mea(1, 0)),
                    //             recent_contact_z_filter[3].CalculateAverage(RL_foot_mea(2, 0));
                    // }
                    /////////
                    foot_contact_position.block<3, 1>(0, 0)=FR_foot_mea;
                    foot_contact_position.block<3, 1>(0, 1)=FL_foot_mea;
                    foot_contact_position.block<3, 1>(0, 2)=RR_foot_mea;
                    foot_contact_position.block<3, 1>(0, 3)=RL_foot_mea;
                    ground_angle = state_est_kine.compute_ground_inclination(body_r_des, body_r_est, body_p_est,foot_contact_position,right_support); 
                    if (use_terrain_adapt) {
                        body_r_des(0,0) += ground_angle(0,0);
                        body_r_des(1,0) += ground_angle(1,0);
                    }
                    base_acc_ff_controler();  

                    leg_kinematic();
                    //////////// add GRF compensation=========///////
                    F_sum(0) = gait::mass * coma_des[0];
                    F_sum(1) = gait::mass * coma_des[1];
                    F_sum(2) = gait::mass * (gait::_g + coma_des[2]);
                    ///////==== momentum check !!!!!!!!!!!!!!!!!!!!!!!! =================
                    F_sum(3,0) = Momentum_global(0,0) * theta_acc_des[0] + Momentum_global(0,1) * theta_acc_des[1] + Momentum_global(0,2) * theta_acc_des[2];
                    F_sum(4,0) = Momentum_global(1,0) * theta_acc_des[0] + Momentum_global(1,1) * theta_acc_des[1] + Momentum_global(1,2) * theta_acc_des[2];
                    F_sum(5,0) = Momentum_global(2,0) * theta_acc_des[0] + Momentum_global(2,1) * theta_acc_des[1] + Momentum_global(2,2) * theta_acc_des[2];
                    
                    foot_contact_flag.setConstant(1);                       
                    if(using_ff>0.5)
                    {
                        Dynam.force_opt(body_p_des,FR_foot_des, FL_foot_des, RR_foot_des, RL_foot_des,
                                        F_sum, gait_mode, right_support, y_offset,foot_contact_flag);
                    } 

                    FR_GRF_opt = (1 * (Dynam.grf_opt.block<3,1>(0,0) - FR_GRF)) + FR_GRF;
                    FL_GRF_opt = (1 * (Dynam.grf_opt.block<3,1>(3,0) - FL_GRF)) + FL_GRF;
                    RR_GRF_opt = (1 * (Dynam.grf_opt.block<3,1>(6,0) - RR_GRF)) + RR_GRF;
                    RL_GRF_opt = (1 * (Dynam.grf_opt.block<3,1>(9,0) - RL_GRF)) + RL_GRF;                                                               
                    if(uisng_current_jaco>0.5)
                    {
                        Torque_ff_GRF.block<3,1>(0,0) = - FR_Jaco_est.transpose() *  root_rot_mat.transpose() * FR_GRF_opt;
                        Torque_ff_GRF.block<3,1>(3,0) = - FL_Jaco_est.transpose() *  root_rot_mat.transpose() * FL_GRF_opt;
                        Torque_ff_GRF.block<3,1>(6,0) = - RR_Jaco_est.transpose() *  root_rot_mat.transpose() * RR_GRF_opt;
                        Torque_ff_GRF.block<3,1>(9,0) = - RL_Jaco_est.transpose() *  root_rot_mat.transpose() * RL_GRF_opt;                             
                    }
                    else
                    {
                        Torque_ff_GRF.block<3,1>(0,0) = - FR_Jaco.transpose() *  root_rot_mat.transpose() * FR_GRF_opt;
                        Torque_ff_GRF.block<3,1>(3,0) = - FL_Jaco.transpose() *  root_rot_mat.transpose() * FL_GRF_opt;
                        Torque_ff_GRF.block<3,1>(6,0) = - RR_Jaco.transpose() *  root_rot_mat.transpose() * RR_GRF_opt;
                        Torque_ff_GRF.block<3,1>(9,0) = - RL_Jaco.transpose() *  root_rot_mat.transpose() * RL_GRF_opt; 
                    }
                    
                    rate_stand_up = pow(stand_up_count/500.0,2); 
                    for(int j=0; j<12;j++)
                    {
                        Torque_ff_GRF[j] = jointLinearInterpolation(0, Torque_ff_GRF(j,0), rate_stand_up, 0);
                    }
                    FR_swing = false; 
                    FL_swing = false;
                    RR_swing = false;
                    RL_swing = false; 
                    
                    //// only for test
                    Legs_torque = Torque_ff_GRF;
                    

                    break;                        
                case DYNAMIC:
                    gait_status = DYNAMIC_STATUS;
                    dynamic_count++;
                    count_in_rt_loop++;
                    start_grf = 1;
                    if(dynamic_count==1)
                    {
                        body_p_Homing_dynamic[0] = body_p_Homing[0];
                        body_p_Homing_dynamic[1] = body_p_Homing[1];
                        body_p_Homing_dynamic[2] = body_p_Homing[2];
                        body_r_Homing_dynamic[0] = body_r_Homing[0];
                        body_r_Homing_dynamic[1] = body_r_Homing[1];
                        body_r_Homing_dynamic[2] = body_r_Homing[2];                               
                        printf("===PERFORMING DYNAMIC WALKING===\n");
                        cout<<"body_p_Homing_dynamic"<<body_p_Homing_dynamic<<endl;
                        cout<<"body_r_Homing_dynamic"<<body_r_Homing_dynamic<<endl;
                    }   
                    if(using_grf_node>0.5) ///// receiveing gait data from the grf node;
                    {
                        // if (count_in_rt_loop * dtx >= 0.5)
                        // {
                            // body_p_des = Grf_sub_filter.block<3,1>(12,0);
                            // body_r_des = Grf_sub_filter.block<3,1>(15,0);
                            // FR_foot_des = Grf_sub_filter.block<3,1>(18,0);
                            // FL_foot_des = Grf_sub_filter.block<3,1>(21,0);
                            // RR_foot_des = Grf_sub_filter.block<3,1>(24,0);
                            // RL_foot_des = Grf_sub_filter.block<3,1>(27,0); 

                            // FR_GRF_opt = (1 * (Grf_sub_filter.block<3,1>(0,0) - FR_GRF)) + FR_GRF;
                            // FL_GRF_opt = (1 * (Grf_sub_filter.block<3,1>(3,0) - FL_GRF)) + FL_GRF;
                            // RR_GRF_opt = (1 * (Grf_sub_filter.block<3,1>(6,0) - RR_GRF)) + RR_GRF;
                            // RL_GRF_opt = (1 * (Grf_sub_filter.block<3,1>(9,0) - RL_GRF)) + RL_GRF; 

                            body_p_des = (Grf_sub.block<3,1>(12,0) + body_p_des)/2;
                            body_r_des = (Grf_sub.block<3,1>(15,0) + body_r_des)/2;
                            FR_foot_des = (Grf_sub.block<3,1>(18,0) + Grf_sub_filter.block<3,1>(18,0))/2;
                            FL_foot_des = (Grf_sub.block<3,1>(21,0) + Grf_sub_filter.block<3,1>(21,0))/2;
                            RR_foot_des = (Grf_sub.block<3,1>(24,0) + RR_foot_des)/2;
                            RL_foot_des = (Grf_sub.block<3,1>(27,0) + RL_foot_des)/2; 

                            FR_GRF_opt = (1 * (Grf_sub.block<3,1>(0,0) - FR_GRF)) + FR_GRF;
                            FL_GRF_opt = (1 * (Grf_sub.block<3,1>(3,0) - FL_GRF)) + FL_GRF;
                            RR_GRF_opt = (1 * (Grf_sub.block<3,1>(6,0) - RR_GRF)) + RR_GRF;
                            RL_GRF_opt = (1 * (Grf_sub.block<3,1>(9,0) - RL_GRF)) + RL_GRF;                                   

                            // if (count_in_rt_loop>1)
                            // {
                            //     comv_des[0] = (body_p_des[0] - com_des_pre[0]) /dtx;
                            //     comv_des[1] = (body_p_des[1] - com_des_pre[1]) /dtx;
                            //     comv_des[2] = (body_p_des[2] - com_des_pre[2]) /dtx;

                            //     thetav_des[0] = (body_r_des[0] - theta_des_pre[0]) /dtx;
                            //     thetav_des[1] = (body_r_des[1] - theta_des_pre[1]) /dtx;
                            //     thetav_des[2] = (body_r_des[2] - theta_des_pre[2]) /dtx;                           
                            // }

                            // // right_support = 2;  
                            /////////////////////// leg movement test//////////////////////
                            // FR_foot_desx = abs(0.04 * sin(4*M_PI*dynamic_count/n_rt))-0.01;
                            // FR_foot_desy = abs(0.02 * sin(4*M_PI*dynamic_count/n_rt))-0.01;
                            FR_foot_desz = (clear_height * sin(8*M_PI*dynamic_count/n_rt))-0.02;
                            // FL_foot_desx = abs(0.04 * sin(4*M_PI*dynamic_count/n_rt + M_PI))-0.01;
                            // FL_foot_desy = abs(0.02 * sin(4*M_PI*dynamic_count/n_rt + M_PI))-0.01;
                            FL_foot_desz = (clear_height * sin(8*M_PI*dynamic_count/n_rt + M_PI))-0.02;                       
                            if((FR_foot_desz<=0))
                            {
                                FR_foot_desz = 0;
                            }
                            if(FL_foot_desz<=0)
                            {
                                FL_foot_desz = 0;
                            }



                            if(FR_foot_desx <=0)
                            {
                                FR_foot_desx = 0;
                            }

                            if(FR_foot_desy <=0)
                            {
                                FR_foot_desy = 0;
                            }                       

                            dynamic_count_period = floor(dynamic_count * 2/n_rt ); 
                            FR_foot_desx *= pow((-1), dynamic_count_period % 2);
                            FR_foot_desy *= pow((-1), dynamic_count_period % 2);

                            if((FR_foot_desz<=0))
                            {
                                if(FL_foot_desz<=0)
                                {
                                right_support = 2;
                                }                           
                                else
                                {
                                right_support = 0;
                                }

                            }
                            else
                            {
                                right_support = 1;
                            }

                            FR_foot_des[0] = FR_foot_Homing[0] + FR_foot_desx;
                            FR_foot_des[1] = FR_foot_Homing[1] + FR_foot_desy;
                            FR_foot_des[2] = FR_foot_Homing[2] + FR_foot_desz;
                            RL_foot_des[0] = RL_foot_Homing[0] + FR_foot_desx;
                            RL_foot_des[1] = RL_foot_Homing[1] + FR_foot_desy;
                            RL_foot_des[2] = RL_foot_Homing[2] + FR_foot_desz;

                            FL_foot_des[0] = FL_foot_Homing[0] + FL_foot_desx;
                            FL_foot_des[1] = FL_foot_Homing[1] + FL_foot_desy;
                            FL_foot_des[2] = FL_foot_Homing[2] + FL_foot_desz;
                            RR_foot_des[0] = RR_foot_Homing[0] + FL_foot_desx;
                            RR_foot_des[1] = RR_foot_Homing[1] + FL_foot_desy;
                            RR_foot_des[2] = RR_foot_Homing[2] + FL_foot_desz;
                            
                            body_p_des = (Grf_sub.block<3,1>(12,0) + body_p_des)/2;
                            body_r_des = (Grf_sub.block<3,1>(15,0) + body_r_des)/2;
                            FR_foot_des = (Grf_sub.block<3,1>(18,0) + Grf_sub_filter.block<3,1>(18,0))/2;
                            FL_foot_des = (Grf_sub.block<3,1>(21,0) + Grf_sub_filter.block<3,1>(21,0))/2;
                            RR_foot_des = (Grf_sub_filter.block<3,1>(24,0) + RR_foot_des)/2;
                            RL_foot_des = (Grf_sub_filter.block<3,1>(27,0) + RL_foot_des)/2;

                        // }
                        // else
                        // {
                        //     body_p_des[0] = body_p_Homing_dynamic[0] + 0.00 * sin(8*M_PI*dynamic_count/n_rt);
                        //     body_p_des[1] = body_p_Homing_dynamic[1] + 0.00 * sin(8*M_PI*dynamic_count/n_rt);
                        //     body_p_des[2] = body_p_Homing_dynamic[2] + 0.00 *(sin(8*M_PI*dynamic_count/n_rt));
                        //     body_r_des[0] = body_r_Homing_dynamic[0] + 0.00 * sin(6*M_PI*dynamic_count/n_rt);
                        //     body_r_des[1] = body_r_Homing_dynamic[1] + 0.00 * sin(6*M_PI*dynamic_count/n_rt);
                        //     body_r_des[2] = body_r_Homing_dynamic[2] + 0.00 * sin(8*M_PI*dynamic_count/n_rt); 
                            
                        //     comv_des[0] = -0.00 * cos(8*M_PI*dynamic_count/n_rt) * (8*M_PI/n_rt);
                        //     comv_des[1] = -0.00 * cos(8*M_PI*dynamic_count/n_rt) * (8*M_PI/n_rt);
                        //     comv_des[2] = -0.00 * cos(8*M_PI*dynamic_count/n_rt) * (8*M_PI/n_rt);

                        //     thetav_des[0] = -0.00 * cos(6*M_PI*dynamic_count/n_rt) * (6*M_PI/n_rt);
                        //     thetav_des[1] = -0.00 * cos(6*M_PI*dynamic_count/n_rt) * (6*M_PI/n_rt);
                        //     thetav_des[2] = -0.00 * cos(6*M_PI*dynamic_count/n_rt) * (8*M_PI/n_rt);   

                        //     FR_foot_des[0] = FR_foot_Homing[0] + FR_foot_desx;
                        //     FR_foot_des[1] = FR_foot_Homing[1] + FR_foot_desy;
                        //     FR_foot_des[2] = FR_foot_Homing[2] + FR_foot_desz;
                        //     RL_foot_des[0] = RL_foot_Homing[0] + FR_foot_desx;
                        //     RL_foot_des[1] = RL_foot_Homing[1] + FR_foot_desy;
                        //     RL_foot_des[2] = RL_foot_Homing[2] + FR_foot_desz;

                        //     FL_foot_des[0] = FL_foot_Homing[0] + FL_foot_desx;
                        //     FL_foot_des[1] = FL_foot_Homing[1] + FL_foot_desy;
                        //     FL_foot_des[2] = FL_foot_Homing[2] + FL_foot_desz;
                        //     RR_foot_des[0] = RR_foot_Homing[0] + FL_foot_desx;
                        //     RR_foot_des[1] = RR_foot_Homing[1] + FL_foot_desy;
                        //     RR_foot_des[2] = RR_foot_Homing[2] + FL_foot_desz;                                
                        // }
                    }
                    else
                    {
                        if(test_stepping_in_place>0.5)
                        {
                            // ////////===========dyanmic walking COM movement test/////////////////////////
                            body_p_des[0] = body_p_Homing_dynamic[0] + 0.00 * sin(6*M_PI*dynamic_count/n_rt);
                            body_p_des[1] = body_p_Homing_dynamic[1] + 0.00 * sin(6*M_PI*dynamic_count/n_rt);
                            body_p_des[2] = body_p_Homing_dynamic[2] + 0.02 *(sin(3*M_PI*dynamic_count/n_rt));
                            body_r_des[0] = body_r_Homing_dynamic[0] + 0.00 * sin(6*M_PI*dynamic_count/n_rt);
                            body_r_des[1] = body_r_Homing_dynamic[1] + 0.00 * sin(6*M_PI*dynamic_count/n_rt);
                            body_r_des[2] = body_r_Homing_dynamic[2] + 0.00 * sin(6*M_PI*dynamic_count/n_rt);                  
                            comv_des[0] = -0.00 * cos(6*M_PI*dynamic_count/n_rt) * (6*M_PI/n_rt);
                            comv_des[1] = -0.00 * cos(6*M_PI*dynamic_count/n_rt) * (6*M_PI/n_rt);
                            comv_des[2] = -0.02 * cos(3*M_PI*dynamic_count/n_rt) * (3*M_PI/n_rt);
                            thetav_des[0] = -0.00 * cos(6*M_PI*dynamic_count/n_rt) * (6*M_PI/n_rt);
                            thetav_des[1] = -0.00 * cos(6*M_PI*dynamic_count/n_rt) * (6*M_PI/n_rt);
                            thetav_des[2] = -0.00 * cos(6*M_PI*dynamic_count/n_rt) * (6*M_PI/n_rt);   

                            right_support = 2;  
                            // /////////////////////// leg movement test//////////////////////
                            // if((swing_leg_test==0)||(swing_leg_test==2))
                            // {
                            //     // FR_foot_desx = abs(0.04 * sin(4*M_PI*dynamic_count/n_rt))-0.01;
                            //     // FR_foot_desy = abs(0.02 * sin(4*M_PI*dynamic_count/n_rt))-0.01;
                            //     FR_foot_desz = (clear_height * sin(4*M_PI*dynamic_count/n_rt))-0.02;
                            // }
                            // if((swing_leg_test==1)||(swing_leg_test==2))
                            // {
                            //     // FL_foot_desx = abs(0.04 * sin(4*M_PI*dynamic_count/n_rt + M_PI))-0.01;
                            //     // FL_foot_desy = abs(0.02 * sin(4*M_PI*dynamic_count/n_rt + M_PI))-0.01;
                            //     FL_foot_desz = (clear_height * sin(4*M_PI*dynamic_count/n_rt + M_PI))-0.02; 
                            // }                      
                            // if((FR_foot_desz<=-0.005))
                            // {
                            //     FR_foot_desz = -0.005;
                            // }
                            // if(FL_foot_desz<=-0.005)
                            // {
                            //     FL_foot_desz = -0.005;
                            // }
                            // if(FR_foot_desx <=0)
                            // {
                            //     FR_foot_desx = 0;
                            // }
                            // if(FR_foot_desy <=0)
                            // {
                            //     FR_foot_desy = 0;
                            // }                       
                            // dynamic_count_period = floor(dynamic_count * 2/n_rt ); 
                            // FR_foot_desx *= pow((-1), dynamic_count_period % 2);
                            // FR_foot_desy *= pow((-1), dynamic_count_period % 2);

                            // if((FR_foot_desz<=0))
                            // {
                            //     if(FL_foot_desz<=0)
                            //     {
                            //         right_support = 2;
                            //     }                           
                            //     else
                            //     {
                            //         right_support = 0;
                            //     }
                            // }
                            // else
                            // {
                            //     right_support = 1;
                            // }

                            FR_foot_des[0] = FR_foot_Homing[0] + FR_foot_desx;
                            FR_foot_des[1] = FR_foot_Homing[1] + FR_foot_desy;
                            FR_foot_des[2] = FR_foot_Homing[2] + FR_foot_desz;
                            RL_foot_des[0] = RL_foot_Homing[0] + FR_foot_desx;
                            RL_foot_des[1] = RL_foot_Homing[1] + FR_foot_desy;
                            RL_foot_des[2] = RL_foot_Homing[2] + FR_foot_desz;
                            FL_foot_des[0] = FL_foot_Homing[0] + FL_foot_desx;
                            FL_foot_des[1] = FL_foot_Homing[1] + FL_foot_desy;
                            FL_foot_des[2] = FL_foot_Homing[2] + FL_foot_desz;
                            RR_foot_des[0] = RR_foot_Homing[0] + FL_foot_desx;
                            RR_foot_des[1] = RR_foot_Homing[1] + FL_foot_desy;
                            RR_foot_des[2] = RR_foot_Homing[2] + FL_foot_desz;
                        }
                        else
                        {
                        // // ================non -real-time intepoloation: data filter: from hierachical convex optimization:                    
                            bjx1 = slow_mpc_gait(27);
                            right_support = slow_mpc_gait(99); 
                            init_count = slow_mpc_gait(39); 

                            com_des[0] = butterworthLPF1.filter(slow_mpc_gait(0,0)) + body_p_Homing_dynamic[0];
                            com_des[1] = butterworthLPF2.filter(slow_mpc_gait(1,0)) + body_p_Homing_dynamic[1];
                            com_des[2] = butterworthLPF3.filter(slow_mpc_gait(2,0));
                            // theta_des[0] = butterworthLPF4.filter(slow_mpc_gait(3));
                            // theta_des[1] = -abs(butterworthLPF5.filter(slow_mpc_gait(4)));	    
                            theta_des[2] = butterworthLPF6.filter(slow_mpc_gait(5)); 
                            theta_des[0] = 0;
                            theta_des[1] = 0;   
                            //theta_des[2] = 0;                                                
                            rfoot_des[0] = butterworthLPF7.filter(slow_mpc_gait(9));
                            rfoot_des[1] = butterworthLPF8.filter(slow_mpc_gait(10));
                            rfoot_des[2] = butterworthLPF9.filter(slow_mpc_gait(11));
                            lfoot_des[0] = butterworthLPF10.filter(slow_mpc_gait(6));
                            lfoot_des[1] = butterworthLPF11.filter(slow_mpc_gait(7));
                            lfoot_des[2] = butterworthLPF12.filter(slow_mpc_gait(8));
                            // if (count_in_rt_loop>1)
                            // {
                            //     comv_des[0] = (com_des[0] - com_des_pre[0]) /dtx;
                            //     comv_des[1] = (com_des[1] - com_des_pre[1]) /dtx;
                            //     comv_des[2] = (com_des[2] - com_des_pre[2]) /dtx;
                            //     thetav_des[0] = (theta_des[0] - theta_des_pre[0]) /dtx;
                            //     thetav_des[1] = (theta_des[1] - theta_des_pre[1]) /dtx;
                            //     thetav_des[2] = (theta_des[2] - theta_des_pre[2]) /dtx;                           
                            // }
                            // coma_des[0] = butterworthLPF13.filter(slow_mpc_gait(39));
                            // coma_des[1] = butterworthLPF14.filter(slow_mpc_gait(40));
                            // coma_des[2] = butterworthLPF15.filter(slow_mpc_gait(41));		
                            // theta_acc_des[0] = butterworthLPF16.filter(slow_mpc_gait(73));
                            // theta_acc_des[1] = butterworthLPF17.filter(slow_mpc_gait(74));
                            // theta_acc_des[2] = butterworthLPF18.filter(slow_mpc_gait(75));	
                            if (count_in_rt_loop * dtx >= 0.5)
                            {
                                switch (gait_mode)
                                {
                                    case 101:  ////biped walking
                                        body_p_des[0] = com_des[0];
                                        body_p_des[1] = com_des[1] * y_offset;
                                        body_p_des[2] = com_des[2];
                                        body_r_des[0] = theta_des[0];
                                        body_r_des[1] = theta_des[1];
                                        body_r_des[2] = theta_des[2];
                                        //// right two legs move synchronous
                                        FR_foot_des[0] = FR_foot_Homing[0] + rfoot_des[0];
                                        FR_foot_des[1] = FR_foot_Homing[1] + rfoot_des[1] + gait::RobotParaClass_HALF_HIP_WIDTH;
                                        FR_foot_des[2] = FR_foot_Homing[2] + rfoot_des[2];
                                        RR_foot_des[0] = RR_foot_Homing[0] + rfoot_des[0];
                                        RR_foot_des[1] = RR_foot_Homing[1] + rfoot_des[1] + gait::RobotParaClass_HALF_HIP_WIDTH;
                                        RR_foot_des[2] = RR_foot_Homing[2] + rfoot_des[2];
                                        //// left two legs move synchronous
                                        FL_foot_des[0] = FL_foot_Homing[0] + lfoot_des[0];
                                        FL_foot_des[1] = FL_foot_Homing[1] + lfoot_des[1] - gait::RobotParaClass_HALF_HIP_WIDTH;
                                        FL_foot_des[2] = FL_foot_Homing[2] + lfoot_des[2];
                                        RL_foot_des[0] = RL_foot_Homing[0] + lfoot_des[0];
                                        RL_foot_des[1] = RL_foot_Homing[1] + lfoot_des[1] - gait::RobotParaClass_HALF_HIP_WIDTH;
                                        RL_foot_des[2] = RL_foot_Homing[2] + lfoot_des[2];
                                        break;
                                    case 102:  ///troting
                                        body_p_des[0] = com_des[0];
                                        body_p_des[1] = com_des[1];
                                        body_p_des[2] = com_des[2];
                                        body_r_des[0] = theta_des[0];
                                        body_r_des[1] = theta_des[1];
                                        body_r_des[2] = theta_des[2];
                                        //// FR, RL two legs move synchronous
                                        FR_foot_des[0] = FR_foot_Homing[0] + lfoot_des[0];
                                        FR_foot_des[1] = FR_foot_Homing[1] + lfoot_des[1];
                                        FR_foot_des[2] = FR_foot_Homing[2] + lfoot_des[2];
                                        RL_foot_des[0] = RL_foot_Homing[0] + lfoot_des[0];
                                        RL_foot_des[1] = RL_foot_Homing[1] + lfoot_des[1];
                                        RL_foot_des[2] = RL_foot_Homing[2] + lfoot_des[2];
                                        //// FL, RR two legs move synchronous
                                        FL_foot_des[0] = FL_foot_Homing[0] + rfoot_des[0];
                                        FL_foot_des[1] = FL_foot_Homing[1] + rfoot_des[1];
                                        FL_foot_des[2] = FL_foot_Homing[2] + rfoot_des[2];
                                        RR_foot_des[0] = RR_foot_Homing[0] + rfoot_des[0];
                                        RR_foot_des[1] = RR_foot_Homing[1] + rfoot_des[1];
                                        RR_foot_des[2] = RR_foot_Homing[2] + rfoot_des[2];                                                    
                                        break;
                                    case 103:  ///gallop: alter the  x-y direction
                                        body_p_des[0] = com_des[0];
                                        body_p_des[1] = com_des[1] * y_offset;
                                        body_p_des[2] = com_des[2];                                  
                                        // pitch angle generation //////                            
                                        pitch_angle_W = 2 * gait::pi / (2 * tstep);
                                        body_r_des[0] = theta_des[0];
                                        ///// 
                                        if (count_in_rt_loop - 3* n_period <= 0)
                                        {
                                            body_r_des[0] = theta_des[1];
                                        }
                                        else
                                        {
                                            body_r_des[1] = -(0.1 * (sin(pitch_angle_W * (count_in_rt_loop - 3* n_period) * dtx))); /// pitch angle//
                                        }     
                                        body_r_des[2] = theta_des[2];                    
                                        //// fore two legs move synchronous
                                        FR_foot_des[0] = FR_foot_Homing[0] + rfoot_des[0];
                                        FR_foot_des[1] = FR_foot_Homing[1] + rfoot_des[1] + gait::RobotParaClass_HALF_HIP_WIDTH;
                                        FR_foot_des[2] = FR_foot_Homing[2] + rfoot_des[2];
                                        FL_foot_des[0] = FL_foot_Homing[0] + rfoot_des[0];
                                        FL_foot_des[1] = FL_foot_Homing[1] + rfoot_des[1] + gait::RobotParaClass_HALF_HIP_WIDTH;
                                        FL_foot_des[2] = FL_foot_Homing[2] + rfoot_des[2];
                                        //// rear two legs move synchronous
                                        RR_foot_des[0] = RR_foot_Homing[0] + lfoot_des[0];
                                        RR_foot_des[1] = RR_foot_Homing[1] + lfoot_des[1] - gait::RobotParaClass_HALF_HIP_WIDTH;
                                        RR_foot_des[2] = RR_foot_Homing[2] + lfoot_des[2];             
                                        RL_foot_des[0] = RL_foot_Homing[0] + lfoot_des[0];
                                        RL_foot_des[1] = RL_foot_Homing[1] + lfoot_des[1] - gait::RobotParaClass_HALF_HIP_WIDTH;
                                        RL_foot_des[2] = RL_foot_Homing[2] + lfoot_des[2];  
                                        break;            
                                    default:
                                        break;
                                }
                            }
                            else
                            {
                                body_p_des[0] = body_p_Homing_dynamic[0];
                                body_p_des[1] = body_p_Homing_dynamic[1];
                                body_p_des[2] = body_p_Homing_dynamic[2];  
                                body_r_des[0] = body_r_Homing_dynamic[0];
                                body_r_des[1] = body_r_Homing_dynamic[1];
                                body_r_des[2] = body_r_Homing_dynamic[2];                                                    
                            }
                        }


                        ////=========  ///////////////////////////
                        ////// body pose feedback!!! set swing_flag according to the reference
                        base_pos_fb_controler();          
                        body_p_des[0] +=  w_pos_m_filter[0];
                        body_p_des[1] +=  w_pos_m_filter[1];
                        body_p_des[2] +=  w_pos_m_filter[2];
                        body_r_des[0] +=  w_rpy_m_filter[0];
                        body_r_des[1] +=  w_rpy_m_filter[1];
                        body_r_des[2] +=  w_rpy_m_filter[2];
                        if (count_in_rt_loop>1)
                        {
                            comv_des[0] = (body_p_des[0] - com_des_pre[0]) /dtx;
                            comv_des[1] = (body_p_des[1] - com_des_pre[1]) /dtx;
                            comv_des[2] = (body_p_des[2] - com_des_pre[2]) /dtx;
                            thetav_des[0] = (body_r_des[0] - theta_des_pre[0]) /dtx;
                            thetav_des[1] = (body_r_des[1] - theta_des_pre[1]) /dtx;
                            thetav_des[2] = (body_r_des[2] - theta_des_pre[2]) /dtx;                           
                        }


                        foot_earlier_contact_flag.setZero();  
                        //////////////judge if earlier contact
                        if(fsr_contact>0.5) /////rely on the FSR
                        {
                            if(footforce_fr.mean()>fz_limit)
                            {
                                foot_contact_flag(0,0) = 1;
                            }
                            else
                            {
                                foot_contact_flag(0,0) = 0; 
                            }                                
                            if(footforce_fl.mean()>fz_limit)
                            {
                                foot_contact_flag(1,0) = 1;
                            }
                            else
                            {
                                foot_contact_flag(1,0) = 0; 
                            }
                            if(footforce_rr.mean()>fz_limit)
                            {
                                foot_contact_flag(2,0) = 1;
                            }
                            else
                            {
                                foot_contact_flag(2,0) = 0; 
                            }                                
                            if(footforce_rl.mean()>fz_limit)
                            {
                                foot_contact_flag(3,0) = 1;
                            }
                            else
                            {
                                foot_contact_flag(3,0) = 0; 
                            }
                        }
                        else
                        {
                            /////judge if it is the stance foot in planning
                            if(FR_swing == false)
                            {
                                foot_contact_flag(0,0) = 1;
                                if((FR_foot_des[2] < FR_foot_des_old[2])&&(foot_earlier_contact_flag_old(0,0) >0.5))
                                {
                                for(int j=0;j<3;j++)
                                {
                                    FR_foot_des[j] = FR_foot_des_old[j];
                                }
                                foot_earlier_contact_flag(0,0) = 1;
                                }
                            }
                            else
                            {
                                foot_contact_flag(0,0) = 0; 
                            }                                
                            if(FL_swing == false)
                            {
                                foot_contact_flag(1,0) = 1;
                                if((FL_foot_des[2] < FL_foot_des_old[2])&&(foot_earlier_contact_flag_old(1,0) >0.5))
                                {
                                for(int j=0;j<3;j++)
                                {
                                    FL_foot_des[j] = FL_foot_des_old[j];
                                }
                                foot_earlier_contact_flag(1,0) = 1;
                                }                                   
                            }
                            else
                            {
                                foot_contact_flag(1,0) = 0; 
                            }
                            if(RR_swing == false)
                            {
                                foot_contact_flag(2,0) = 1;
                                if( (RR_foot_des[2] < RR_foot_des_old[2])&&(foot_earlier_contact_flag_old(2,0) >0.5))
                                {
                                for(int j=0;j<3;j++)
                                {
                                    RR_foot_des[j] = RR_foot_des_old[j];
                                }
                                foot_earlier_contact_flag(2,0) = 1;
                                }                                   
                            }
                            else
                            {
                                foot_contact_flag(2,0) = 0; 
                            }                                
                            if(RL_swing == false)
                            {
                                foot_contact_flag(3,0) = 1;
                                if( (RL_foot_des[2] < RL_foot_des_old[2])&&(foot_earlier_contact_flag_old(3,0) >0.5))
                                {
                                for(int j=0;j<3;j++)
                                {
                                    RL_foot_des[j] = RL_foot_des_old[j];
                                }
                                foot_earlier_contact_flag(3,0) = 1;
                                }                                     
                            }
                            else
                            {
                                foot_contact_flag(3,0) = 0; 
                            }
                                                    
                            if(judge_early_contact>0.5)////// obstacle & ground inclination test/////
                                {
                                    //earlier contact judgement: not moving the leg position: keep the current leg_position
                                    if((FL_swing == true) && (footforce_fl.mean()>2*fz_limit)&&(dynamic_count * dtx - slow_mpc_gait(12)>slow_mpc_gait(94)/2)&&(dynamic_count * dtx - slow_mpc_gait(12)<slow_mpc_gait(94)*0.9))
                                    {
                                        fz_load_ratio = 2 - pow(footforce_fl.mean()/(2*fz_limit),1);
                                        if(fz_load_ratio<0)
                                        {
                                        fz_load_ratio = 0;
                                        }
                                        for(int j=0;j<3;j++)
                                        {
                                                FL_foot_des[j] = (FL_foot_des_old[j] + fz_load_ratio*(FL_foot_des[j]-FL_foot_des_old[j]));
                                        }
                                        foot_contact_flag(1,0) = 1;
                                        foot_earlier_contact_flag(1,0) = 1;
                                    }
                                    else ////early contact to swing leg: only check the z direction motion
                                    {
                                        if((foot_earlier_contact_flag_old(1,0) >0.5)&&(dynamic_count * dtx - slow_mpc_gait(12)>=slow_mpc_gait(94)*0.9))
                                        {
                                            for(int j=0;j<3;j++)
                                            {
                                                    FL_foot_des[j] = (FL_foot_des_old[j]);
                                            }
                                            foot_contact_flag(1,0) = 1;
                                            foot_earlier_contact_flag(1,0) = 1;
                                        }
                                        else
                                        {
                                            // if((FL_foot_des[2] < FL_foot_des_old[2])&&(foot_earlier_contact_flag_old(1,0) >0.5)&&(dynamic_count * dtx - slow_mpc_gait(12)<=slow_mpc_gait(94)/2))
                                            // {
                                            //     for(int j=2;j<3;j++)
                                            //     {
                                            //         FL_foot_des[j] = FL_foot_des_old[j];
                                            //     }
                                            //     foot_earlier_contact_flag(1,0) = 1;
                                            // }
                                        }                                   
                                    }
                                    if((FR_swing ==true) && (footforce_fr.mean()>2*fz_limit)&&(dynamic_count * dtx - slow_mpc_gait(12)>slow_mpc_gait(94)/2)&&(dynamic_count * dtx - slow_mpc_gait(12)<slow_mpc_gait(94)*0.9))
                                    {
                                        fz_load_ratio = 2 - pow(footforce_fr.mean()/(2*fz_limit),1);
                                        if(fz_load_ratio<0)
                                        {
                                        fz_load_ratio = 0;
                                        }
                                        for(int j=0;j<3;j++)
                                        {
                                                FR_foot_des[j] = (FR_foot_des_old[j] + fz_load_ratio*(FR_foot_des[j]-FR_foot_des_old[j]));
                                        }
                                        foot_contact_flag(0,0) = 1;
                                        foot_earlier_contact_flag(0,0) = 1;
                                    }
                                    else ////early contact to swing leg: only check the z direction motion
                                    {
                                        if((foot_earlier_contact_flag_old(0,0) >0.5)&&(dynamic_count * dtx - slow_mpc_gait(12)>=slow_mpc_gait(94)*0.9))
                                        {
                                            for(int j=0;j<3;j++)
                                            {
                                                    FR_foot_des[j] = (FR_foot_des_old[j]);
                                            }
                                            foot_contact_flag(0,0) = 1;
                                            foot_earlier_contact_flag(0,0) = 1;
                                        }
                                        else
                                        {                                            
                                            // if((FR_foot_des[2] < FR_foot_des_old[2])&&(foot_earlier_contact_flag_old(0,0) >0.5)&&(dynamic_count * dtx - slow_mpc_gait(12)<=slow_mpc_gait(94)/2))
                                            // {
                                            //     for(int j=2;j<3;j++)
                                            //     {
                                            //         FR_foot_des[j] = FR_foot_des_old[j];
                                            //     }
                                            //     foot_earlier_contact_flag(0,0) = 1;
                                            // }
                                        }
                                        
                                        
                                    }                                        
                                    if((RL_swing == true) && (footforce_rl.mean()>2*fz_limit)&&(dynamic_count * dtx - slow_mpc_gait(12)>slow_mpc_gait(94)/2)&&(dynamic_count * dtx - slow_mpc_gait(12)<slow_mpc_gait(94)*0.9))
                                    {
                                        fz_load_ratio = 2 - pow(footforce_rl.mean()/(2*fz_limit),1);
                                        if(fz_load_ratio<0)
                                        {
                                        fz_load_ratio = 0;
                                        }
                                        for(int j=0;j<3;j++)
                                        {
                                                RL_foot_des[j] = (RL_foot_des_old[j] + fz_load_ratio*(RL_foot_des[j]-RL_foot_des_old[j]));
                                        }
                                        foot_contact_flag(3,0) = 1;
                                        foot_earlier_contact_flag(3,0) = 1;
                                    }
                                    else ////early contact to swing leg: only check the z direction motion
                                    {
                                        if((foot_earlier_contact_flag_old(3,0) >0.5)&&(dynamic_count * dtx - slow_mpc_gait(12)>=slow_mpc_gait(94)*0.9))
                                        {
                                            for(int j=0;j<3;j++)
                                            {
                                                    RL_foot_des[j] = (RL_foot_des_old[j]);
                                            }
                                            foot_contact_flag(3,0) = 1;
                                            foot_earlier_contact_flag(3,0) = 1;
                                        }
                                        else
                                        {                                            
                                            // if((RL_foot_des[2] < RL_foot_des_old[2])&&(foot_earlier_contact_flag_old(3,0) >0.5)&&(dynamic_count * dtx - slow_mpc_gait(12)<=slow_mpc_gait(94)/2))
                                            // {
                                            //     for(int j=2;j<3;j++)
                                            //     {
                                            //         RL_foot_des[j] = RL_foot_des_old[j];
                                            //     }
                                            //     foot_earlier_contact_flag(3,0) = 1;
                                            // }
                                        }
                                        
                                    }                                        
                                    if((RR_swing == true) && (footforce_rr.mean()>2*fz_limit)&&(dynamic_count * dtx - slow_mpc_gait(12)>slow_mpc_gait(94)/2)&&(dynamic_count * dtx - slow_mpc_gait(12)<slow_mpc_gait(94)*0.9))
                                    {
                                        fz_load_ratio = 2 - pow(footforce_rr.mean()/(2*fz_limit),1);
                                        if(fz_load_ratio<0)
                                        {
                                        fz_load_ratio = 0;
                                        }
                                        for(int j=0;j<3;j++)
                                        {
                                                RR_foot_des[j] = (RR_foot_des_old[j] + fz_load_ratio*(RR_foot_des[j]-RR_foot_des_old[j]));
                                        }
                                        foot_contact_flag(2,0) = 1;
                                        foot_earlier_contact_flag(2,0) = 1;
                                    }
                                    else ////early contact to swing leg: only check the z direction motion
                                    {
                                        if((foot_earlier_contact_flag_old(2,0) >0.5)&&(dynamic_count * dtx - slow_mpc_gait(12)>=slow_mpc_gait(94)*0.9))
                                        {
                                            for(int j=0;j<3;j++)
                                            {
                                                    RR_foot_des[j] = (RR_foot_des_old[j]);
                                            }
                                            foot_contact_flag(2,0) = 1;
                                            foot_earlier_contact_flag(2,0) = 1;
                                        }
                                        else
                                        {                                             
                                            // if((RR_foot_des[2] < RR_foot_des_old[2])&&(foot_earlier_contact_flag_old(2,0) >0.5)&&(dynamic_count * dtx - slow_mpc_gait(12)<=slow_mpc_gait(94)/2))
                                            // {
                                            //     for(int j=2;j<3;j++)
                                            //     {
                                            //         RR_foot_des[j] = RR_foot_des_old[j];
                                            //     }
                                            //     foot_earlier_contact_flag(2,0) = 1;
                                            // }
                                        }
                                    }                                         
                                }
                        }
                        ////////===== groun slope estimation===============////////
                        // if (foot_contact_flag(0,0)>0.5) {
                            foot_contact_position.block<3, 1>(0, 0)
                                    << recent_contact_x_filter[0].CalculateAverage(FR_foot_mea(0, 0)),
                                    recent_contact_y_filter[0].CalculateAverage(FR_foot_mea(1, 0)),
                                    recent_contact_z_filter[0].CalculateAverage(FR_foot_mea(2, 0));
                        // }
                        // if (foot_contact_flag(1,0)>0.5) {
                            foot_contact_position.block<3, 1>(0, 1)
                                    << recent_contact_x_filter[1].CalculateAverage(FL_foot_mea(0, 0)),
                                    recent_contact_y_filter[1].CalculateAverage(FL_foot_mea(1, 0)),
                                    recent_contact_z_filter[1].CalculateAverage(FL_foot_mea(2, 0));
                        // }
                        // if (foot_contact_flag(2,0)>0.5) {
                            foot_contact_position.block<3, 1>(0, 2)
                                    << recent_contact_x_filter[2].CalculateAverage(RR_foot_mea(0, 0)),
                                    recent_contact_y_filter[2].CalculateAverage(RR_foot_mea(1, 0)),
                                    recent_contact_z_filter[2].CalculateAverage(RR_foot_mea(2, 0));
                        // }
                        // if (foot_contact_flag(3,0)>0.5) {
                            foot_contact_position.block<3, 1>(0, 3)
                                    << recent_contact_x_filter[3].CalculateAverage(RL_foot_mea(0, 0)),
                                    recent_contact_y_filter[3].CalculateAverage(RL_foot_mea(1, 0)),
                                    recent_contact_z_filter[3].CalculateAverage(RL_foot_mea(2, 0));
                        // }
                        // /////////
                        // foot_contact_position.block<3, 1>(0, 0)=FR_foot_mea;
                        // foot_contact_position.block<3, 1>(0, 1)=FL_foot_mea;
                        // foot_contact_position.block<3, 1>(0, 2)=RR_foot_mea;
                        // foot_contact_position.block<3, 1>(0, 3)=RL_foot_mea;
                        ground_angle = state_est_kine.compute_ground_inclination(body_r_des, body_r_est, body_p_est,foot_contact_position,right_support); 
                        if (use_terrain_adapt) {
                            body_r_des(0,0) += ground_angle(0,0);
                            body_r_des(1,0) += ground_angle(1,0);
                        }

                        foot_earlier_contact_flag_old = foot_earlier_contact_flag;

                        // ///////////////===========================================///////////////////////////
                        ///======= adjust the com acceleration in real-time!!!!!!!!!!!!!!!!!!!
                        base_acc_ff_controler(); 
                        // //////=================== QP-based force distribution ================//////////////////
                        F_sum(0) = gait::mass * coma_des[0];
                        F_sum(1) = gait::mass * coma_des[1];
                        F_sum(2) = gait::mass * (gait::_g + coma_des[2]);
                        ///////==== momentum check !!!!!!!!!!!!!!!!!!!!!!!! =================
                        F_sum(3,0) = Momentum_global(0,0) * theta_acc_des[0] + Momentum_global(0,1) * theta_acc_des[1] + Momentum_global(0,2) * theta_acc_des[2];
                        F_sum(4,0) = Momentum_global(1,0) * theta_acc_des[0] + Momentum_global(1,1) * theta_acc_des[1] + Momentum_global(1,2) * theta_acc_des[2];
                        F_sum(5,0) = Momentum_global(2,0) * theta_acc_des[0] + Momentum_global(2,1) * theta_acc_des[1] + Momentum_global(2,2) * theta_acc_des[2];        

                        if(using_ff>0.5)
                        {
                            Dynam.force_opt(body_p_des,FR_foot_des, FL_foot_des, RR_foot_des, RL_foot_des,
                                            F_sum, gait_mode, right_support, y_offset,foot_contact_flag);
                        }
                        FR_GRF_opt = (1 * (Dynam.grf_opt.block<3,1>(0,0) - FR_GRF)) + FR_GRF;
                        FL_GRF_opt = (1 * (Dynam.grf_opt.block<3,1>(3,0) - FL_GRF)) + FL_GRF;
                        RR_GRF_opt = (1 * (Dynam.grf_opt.block<3,1>(6,0) - RR_GRF)) + RR_GRF;
                        RL_GRF_opt = (1 * (Dynam.grf_opt.block<3,1>(9,0) - RL_GRF)) + RL_GRF; 
                    }        

                    leg_kinematic();               

                    if(uisng_current_jaco>0.5)
                    {
                        Torque_ff_GRF.block<3,1>(0,0) = - FR_Jaco_est.transpose() *  root_rot_mat.transpose() * FR_GRF_opt;
                        Torque_ff_GRF.block<3,1>(3,0) = - FL_Jaco_est.transpose() *  root_rot_mat.transpose() * FL_GRF_opt;
                        Torque_ff_GRF.block<3,1>(6,0) = - RR_Jaco_est.transpose() *  root_rot_mat.transpose() * RR_GRF_opt;
                        Torque_ff_GRF.block<3,1>(9,0) = - RL_Jaco_est.transpose() *  root_rot_mat.transpose() * RL_GRF_opt;                             
                    }
                    else
                    {
                        Torque_ff_GRF.block<3,1>(0,0) = - FR_Jaco.transpose() *  root_rot_mat.transpose() * FR_GRF_opt;
                        Torque_ff_GRF.block<3,1>(3,0) = - FL_Jaco.transpose() *  root_rot_mat.transpose() * FL_GRF_opt;
                        Torque_ff_GRF.block<3,1>(6,0) = - RR_Jaco.transpose() *  root_rot_mat.transpose() * RR_GRF_opt;
                        Torque_ff_GRF.block<3,1>(9,0) = - RL_Jaco.transpose() *  root_rot_mat.transpose() * RL_GRF_opt; 
                    }

                    break;
                default:
                    FR_swing = false; 
                    FL_swing = false;
                    RR_swing = false;
                    RL_swing = false;
                    break;
            }


            ////======= torque controller: feedback + feedforward ========////////
            torque.setZero();

            if(judge_later_contact>0.5)
            {
                //adjust for landing controller, if later contact, keeping the impedance control
                if((FL_swing == false) && (footforce_fl.mean()<fz_limit))
                {
                FL_swing = true;
                }
                if((FR_swing == false) && (footforce_fr.mean()<fz_limit))
                {
                FR_swing = true;
                }
                if((RR_swing == false) && (footforce_rl.mean()<fz_limit))
                {
                RR_swing = true;
                }
                if((RR_swing == false) && (footforce_rr.mean()<fz_limit))
                {
                RR_swing = true;
                } 
            } 
            
            //===== joint space: pid joint tracking feedback + spring softplus feedfoward (when enable spring)====////////
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
                    if(j /3 ==0)  //FR
                    {
                    //     //// support leg
                    //     if(!FR_swing)
                    //     {
                            torq_kp_hip = 8 * hip_kp_scale;
                            torq_kd_hip = 0.3 * hip_kd_scale;
                    //     }  
                    }
                    if(j /3 ==1)  //FL
                    {
                    //     //// support leg
                    //     if(!FL_swing)
                    //     {
                            torq_kp_hip = 8 * hip_kp_scale;
                            torq_kd_hip = 0.3 * hip_kd_scale;
                    //     }  
                    }                                                                         
                    if(j /3 ==2)  //RR
                    {
                    //     //// support leg
                    //     if(!RR_swing)
                    //     {
                            torq_kp_hip = 8 * hip_kp_scale;
                            torq_kd_hip = 0.3 * hip_kd_scale; 
                    //     }  
                    }
                    if(j /3 ==3)  //RL
                    {
                    //     //// support leg
                    //     if(!RL_swing)
                    //     {
                            torq_kp_hip = 8 * hip_kp_scale;
                            torq_kd_hip = 0.3 * hip_kd_scale; 
                    //     }  
                    }
                    k_p_rest_hip = sin_mid_q[j];                                                  
                    //// hip joint tracking
                    torque_err.block<torque_err_row-1,1>(0,j) = torque_err.block<torque_err_row-1,1>(1,j);
                    torque_err(torque_err_row-1,j) = qDes[j] - RecvLowROS.motorState[j].q;
                    torque_err_intergration.setZero();
                    for(int ij=0; ij<torque_err_row; ij++)
                    {
                    torque_err_intergration(j,0) += torque_err(ij,j);
                    }                 
                    torque(j,0) = (qDes[j] - RecvLowROS.motorState[j].q)*torq_kp_hip + (0 - RecvLowROS.motorState[j].dq)*torq_kd_hip + torque_err_intergration(j,0)*torq_ki_hip;                      
                    // ff control
                    // ///// tuning the parameters carefully
                    if((gait_status == DYNAMIC_STATUS)||(gait_status == STAND_UP_STATUS))
                    {
                        if(enable_spring>0.5)
                        {
                            if(j==0 || j==6) ///right leg: negetive position, negative torque
                            {
                                //////// softplus funtion ///////////
                                Torque_ff_spring(j,0) = -std::min(pow(stand_up_count/500.0,2),1.0) * (log(1+exp(-k_spring_hip*(qDes[j] + (k_p_rest_hip))))); 
                            }
                            else
                            {
                                Torque_ff_spring(j,0) = std::min(pow(stand_up_count/500.0,2),1.0) * (log(1+exp(k_spring_hip*(qDes[j] - (k_p_rest_hip)))));
                            }                                                    
                        }
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
                            // if(!FR_swing)
                            // {
                            //// support leg
                            torq_kp_thigh = 7 * thigh_kp_scale;
                            torq_kd_thigh = 0.3 * thigh_kd_scale;
                            // }
                            torq_ki_thigh = 0.01*1;  
                            k_spring_thigh = FR_k_spring_thigh;                              
                            // k_p_rest_thigh = 0.49;
                            // k_p_rest_thigh = 0.52;   
                            k_p_rest_thigh = sin_mid_q[j];                                                    
                        }                        
                        if(j /3 ==1)
                        {
                            // //// too soft: only for swing leg
                            torq_kp_thigh = 8* thigh_kp_scale;
                            torq_kd_thigh = 0.3* thigh_kd_scale;
                            // } 
                            torq_ki_thigh  = 0.01*1;                                 
                            k_spring_thigh = FL_k_spring_thigh;
                            // k_p_rest_thigh = 0.41;
                            //k_p_rest_thigh = 0.37;    
                            k_p_rest_thigh = sin_mid_q[j];                                                 
                        }
                        if(j /3 ==2)
                        {
                            // //// too soft: only for swing leg
                            // //// support leg
                            torq_kp_thigh = 8* thigh_kp_scale;
                            torq_kd_thigh = 0.3* thigh_kd_scale;
                            // }
                            torq_ki_thigh = 0.01*1;                                 
                            k_spring_thigh = RR_k_spring_thigh;
                            //k_p_rest_thigh = 0.45;
                            //k_p_rest_thigh = 0.33;
                            k_p_rest_thigh = sin_mid_q[j];
                        } 
                        if(j /3 ==3)
                        {
                            torq_kp_thigh = 9* thigh_kp_scale;
                            torq_kd_thigh = 0.3* thigh_kd_scale;
                            // }
                            torq_ki_thigh  = 0.01*1;                                 
                            k_spring_thigh = RL_k_spring_thigh;
                            //k_p_rest_thigh = 0.82;  
                            //k_p_rest_thigh = 0.83;
                            k_p_rest_thigh = sin_mid_q[j];
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
                        // joint-level ff control
                        if((gait_status == DYNAMIC_STATUS)||(gait_status == STAND_UP_STATUS))
                        {
                            if(enable_spring>0.5)
                            {
                                //////// softplus funtion ///////////
                                Torque_ff_spring(j,0) = std::min(pow(stand_up_count/500.0,2),1.0) * (log(1+exp(k_spring_thigh*(qDes[j] - (k_p_rest_thigh)))));                                                     
                            }
                            // else /// friction compensation
                            // {
                            //     Torque_ff_spring(j,0) = 0.5/(1+exp(k_spring_thigh*(qDes[j] - (k_p_rest_thigh))));
                            // }
                        }

                        torque(j,0) += Torque_ff_spring(j,0);                             
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
                            //// basic value is for swing leg
                            torq_kp_calf = 9 * calf_kp_scale;
                            torq_kd_calf = 0.31 * calf_kd_scale;
                            torq_ki_calf = 0.01*1;  
                            k_spring_calf = FR_k_spring_calf;                              
                            // k_p_rest_calf = -1.3;
                            //k_p_rest_calf = -1.38;    
                            k_p_rest_calf = sin_mid_q[j];                          
                        }                           
                        if(j /3 ==1)
                        {
                            //     //// basic value is for swing leg
                            torq_kp_calf = 9 * calf_kp_scale;
                            torq_kd_calf = 0.31 * calf_kd_scale;
                            torq_ki_calf = 0.01*1;                                 
                            k_spring_calf = FL_k_spring_calf;
                            // k_p_rest_calf = -1.26;
                            //k_p_rest_calf = -1.33;
                            k_p_rest_calf = sin_mid_q[j]; 
                        }
                        if(j /3 ==2)
                        {
                            //// basic value is for swing leg
                            torq_kp_calf = 9 * calf_kp_scale;
                            torq_kd_calf = 0.31 * calf_kd_scale;
                            torq_ki_calf = 0.01*1;                                  
                            k_spring_calf = RR_k_spring_calf;
                            // k_p_rest_calf = -1.2;
                            //k_p_rest_calf = -1.28;
                            k_p_rest_calf = sin_mid_q[j]; 
                        } 
                        if(j /3 ==3)
                        {
                            //// basic value is for swing leg
                            torq_kp_calf = 9 * calf_kp_scale;
                            torq_kd_calf = 0.31 * calf_kd_scale;
                            torq_ki_calf = 0.01*1;                                 
                            k_spring_calf = RL_k_spring_calf;
                            // k_p_rest_calf = -1.45;
                            //k_p_rest_calf = -1.47;
                            k_p_rest_calf = sin_mid_q[j]; 
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
                        //// joint-level ff control
                        if((gait_status == DYNAMIC_STATUS)||(gait_status == STAND_UP_STATUS))
                        {
                            if(enable_spring>0.5)
                            {
                                /////// softplus funtion ///////////
                                Torque_ff_spring(j,0) = std::min(pow(stand_up_count/500.0,2),1.0) * (-log(1+exp(k_spring_calf*(-(qDes[j] - (k_p_rest_calf))))));  
                            }
                            // else
                            // {
                            //     /////// friction funtion ///////////
                            //     Torque_ff_spring(j,0) = 0.5/(1+exp(k_spring_calf*(-(qDes[j] - (k_p_rest_calf)))));
                            // }
                        }                            
                        

                        torque(j,0) += Torque_ff_spring(j,0);
                    }                      
                }
            }
            ////// Gravity constant:
            if(FR_swing)
            {
                torque(0,0) += (-0.8f);
            }
            if(RR_swing)
            {
                torque(6,0) += (-0.8f);
            }
            if(FL_swing)
            {
                torque(3,0) += (0.8f);
            }
            if(RL_swing)
            {
                torque(9,0) += (0.8f);
            }                                 
            
            for(int j=0; j<12;j++)
            {
                if(motiontime>=1)
                {
                    dqDes[j] = (qDes[j] - qDes_pre[j]) /dtx;
                } 
                qDes_pre[j] = qDes[j];
            }
            //=== Catesian-space PD-type swing leg impedance control + stance-leg Grf feedforward====////////               
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

                FR_footv_des = (FR_foot_des - FR_foot_des_old)/dtx;
                FL_footv_des = (FL_foot_des - FL_foot_des_old)/dtx;   
                RR_footv_des = (RR_foot_des - RR_foot_des_old)/dtx;
                RL_footv_des = (RL_foot_des - RL_foot_des_old)/dtx; 

                if(!FR_swing)
                {
                    FR_torque_impedance.setZero();  
                }
                else
                {
                    if(uisng_current_jaco>0.5)
                    {
                        if(tracking_global_foot>0.5)
                        {
                            if(using_ekf>0.5)
                            {

                                ////        a1_ctrl_states.foot_pos_world.block<3, 1>(0, i) ;a1_ctrl_states.foot_vel_world.block<3, 1>(0, i);
                                FR_torque_impedance = global_swing*(FR_Jaco_est.transpose() * (swing_kp*(FR_foot_des - a1_ctrl_states.foot_pos_world.block<3, 1>(0, 1)) 
                                                                                + swing_kd*(FR_footv_des - a1_ctrl_states.foot_vel_world.block<3, 1>(0, 1))));                                    
                            }
                            else
                            {
                                FR_torque_impedance = global_swing*(FR_Jaco_est.transpose() * (swing_kp*(FR_foot_des - FR_foot_mea) 
                                                                                + swing_kd*(FR_footv_des - FR_v_est)));
                            }                               
                        }
                        else
                        {
                            FR_torque_impedance = (FR_Jaco_est.transpose() * (swing_kp*(FR_foot_relative_des - FR_foot_relative_mea) 
                                                                                + swing_kd*(FR_v_relative - FR_v_est_relative)));
                        }
                    }
                    else
                    {
                        if(tracking_global_foot>0.5)
                        {                            
                            if(using_ekf>0.5)
                            {

                                ////        a1_ctrl_states.foot_pos_world.block<3, 1>(0, i) ;a1_ctrl_states.foot_vel_world.block<3, 1>(0, i);
                                FR_torque_impedance = global_swing*(FR_Jaco.transpose() * (swing_kp*(FR_foot_des - a1_ctrl_states.foot_pos_world.block<3, 1>(0, 1)) 
                                                                                + swing_kd*(FR_footv_des - a1_ctrl_states.foot_vel_world.block<3, 1>(0, 1))));                                    
                            }
                            else
                            {
                                FR_torque_impedance = global_swing*(FR_Jaco.transpose() * (swing_kp*(FR_foot_des - FR_foot_mea) 
                                                                                + swing_kd*(FR_footv_des - FR_v_est)));
                            }
                        }
                        else
                        {
                            FR_torque_impedance = (FR_Jaco.transpose() * (swing_kp*(FR_foot_relative_des - FR_foot_relative_mea) 
                                                                                + swing_kd*(FR_v_relative - FR_v_est_relative))); 
                        }                             
                        
                    }
                }
                if(!FL_swing)
                {
                    FL_torque_impedance.setZero();  
                }
                else
                {      
                    if(uisng_current_jaco>0.5)
                    { 
                        if(tracking_global_foot>0.5)
                        {
                            if(using_ekf>0.5)
                            {

                                ////        a1_ctrl_states.foot_pos_world.block<3, 1>(0, i) ;a1_ctrl_states.foot_vel_world.block<3, 1>(0, i);
                                FL_torque_impedance = global_swing*(FL_Jaco_est.transpose() * (swing_kp*(FL_foot_des - a1_ctrl_states.foot_pos_world.block<3, 1>(0, 0)) 
                                                                                + swing_kd*(FL_footv_des - a1_ctrl_states.foot_vel_world.block<3, 1>(0, 0))));                                    
                            }
                            else
                            {
                                FL_torque_impedance = global_swing*(FL_Jaco_est.transpose() * (swing_kp*(FL_foot_des - FL_foot_mea) 
                                                                                + swing_kd*(FL_footv_des - FL_v_est)));
                            }                               
                        }
                        else
                        {                            
                            FL_torque_impedance = (FL_Jaco_est.transpose() * (swing_kp*(FL_foot_relative_des - FL_foot_relative_mea) 
                                                                                + swing_kd*(FL_v_relative - FL_v_est_relative)));   
                        }                                     
                    }
                    else
                    {
                        if(tracking_global_foot>0.5)
                        {
                            if(using_ekf>0.5)
                            {

                                ////        a1_ctrl_states.foot_pos_world.block<3, 1>(0, i) ;a1_ctrl_states.foot_vel_world.block<3, 1>(0, i);
                                FL_torque_impedance = global_swing*(FL_Jaco.transpose() * (swing_kp*(FL_foot_des - a1_ctrl_states.foot_pos_world.block<3, 1>(0, 0)) 
                                                                                + swing_kd*(FL_footv_des - a1_ctrl_states.foot_vel_world.block<3, 1>(0, 0))));                                    
                            }
                            else
                            {
                                FL_torque_impedance = global_swing*(FL_Jaco.transpose() * (swing_kp*(FL_foot_des - FL_foot_mea) 
                                                                                + swing_kd*(FL_footv_des - FL_v_est)));
                            }                               
                        }
                        else
                        {                              
                            FL_torque_impedance = (FL_Jaco.transpose() * (swing_kp*(FL_foot_relative_des - FL_foot_relative_mea) 
                                                                                + swing_kd*(FL_v_relative - FL_v_est_relative)));
                        }
                    } 
                }
                if(!RR_swing)
                {
                    RR_torque_impedance.setZero();  
                }
                else
                { 
                    if(uisng_current_jaco>0.5)
                    {
                        if(tracking_global_foot>0.5)
                        {
                            if(using_ekf>0.5)
                            {

                                ////        a1_ctrl_states.foot_pos_world.block<3, 1>(0, i) ;a1_ctrl_states.foot_vel_world.block<3, 1>(0, i);
                                RR_torque_impedance = global_swing*(RR_Jaco_est.transpose() * (swing_kp*(RR_foot_des - a1_ctrl_states.foot_pos_world.block<3, 1>(0, 3)) 
                                                                                + swing_kd*(RR_footv_des - a1_ctrl_states.foot_vel_world.block<3, 1>(0, 3))));                                    
                            }
                            else
                            {
                                RR_torque_impedance = global_swing*(RR_Jaco_est.transpose() * (swing_kp*(RR_foot_des - RR_foot_mea) 
                                                                                + swing_kd*(RR_footv_des - RR_v_est)));
                            }                               
                        }
                        else
                        {                            
                            RR_torque_impedance = (RR_Jaco_est.transpose() * (swing_kp*(RR_foot_relative_des - RR_foot_relative_mea) 
                                                                                + swing_kd*(RR_v_relative - RR_v_est_relative)));   
                        }                      
                    } 
                    else
                    {
                        if(tracking_global_foot>0.5)
                        {
                            if(using_ekf>0.5)
                            {

                                ////        a1_ctrl_states.foot_pos_world.block<3, 1>(0, i) ;a1_ctrl_states.foot_vel_world.block<3, 1>(0, i);
                                RR_torque_impedance = global_swing*(RR_Jaco.transpose() * (swing_kp*(RR_foot_des - a1_ctrl_states.foot_pos_world.block<3, 1>(0, 3)) 
                                                                                + swing_kd*(RR_footv_des - a1_ctrl_states.foot_vel_world.block<3, 1>(0, 3))));                                    
                            }
                            else
                            {
                                RR_torque_impedance = global_swing*(RR_Jaco.transpose() * (swing_kp*(RR_foot_des - RR_foot_mea) 
                                                                                + swing_kd*(RR_footv_des - RR_v_est)));
                            }                               
                        }
                        else
                        {                             
                            RR_torque_impedance = (RR_Jaco.transpose() * (swing_kp*(RR_foot_relative_des - RR_foot_relative_mea) 
                                                                                + swing_kd*(RR_v_relative - RR_v_est_relative))); 
                        }                            
                    }                       
                }
                if(!RL_swing)
                {
                    RL_torque_impedance.setZero();  
                }
                else
                { 
                    if(uisng_current_jaco>0.5)
                    {          
                        if(tracking_global_foot>0.5)
                        {
                            if(using_ekf>0.5)
                            {

                                ////        a1_ctrl_states.foot_pos_world.block<3, 1>(0, i) ;a1_ctrl_states.foot_vel_world.block<3, 1>(0, i);
                                RL_torque_impedance = global_swing*(RL_Jaco_est.transpose() * (swing_kp*(RL_foot_des - a1_ctrl_states.foot_pos_world.block<3, 1>(0, 2)) 
                                                                                + swing_kd*(RL_footv_des - a1_ctrl_states.foot_vel_world.block<3, 1>(0, 2))));                                    
                            }
                            else
                            {
                                RL_torque_impedance = global_swing*(RL_Jaco_est.transpose() * (swing_kp*(RL_foot_des - RL_foot_mea) 
                                                                                + swing_kd*(RL_footv_des - RL_v_est)));
                            }                               
                        }
                        else
                        {                                           
                            RL_torque_impedance = (RL_Jaco_est.transpose() * (swing_kp*(RL_foot_relative_des - RL_foot_relative_mea) 
                                                                                + swing_kd*(RL_v_relative - RL_v_est_relative))); 
                        }
                    } 
                    else
                    {
                        if(tracking_global_foot>0.5)
                        {
                            if(using_ekf>0.5)
                            {

                                ////        a1_ctrl_states.foot_pos_world.block<3, 1>(0, i) ;a1_ctrl_states.foot_vel_world.block<3, 1>(0, i);
                                RL_torque_impedance = global_swing*(RL_Jaco.transpose() * (swing_kp*(RL_foot_des - a1_ctrl_states.foot_pos_world.block<3, 1>(0, 2)) 
                                                                                + swing_kd*(RL_footv_des - a1_ctrl_states.foot_vel_world.block<3, 1>(0, 2))));                                    
                            }
                            else
                            {
                                RL_torque_impedance = global_swing*(RL_Jaco.transpose() * (swing_kp*(RL_foot_des - RL_foot_mea) 
                                                                                + swing_kd*(RL_footv_des - RL_v_est)));
                            }                               
                        }
                        else
                        {                              
                            RL_torque_impedance = (RL_Jaco.transpose() * (swing_kp*(RL_foot_relative_des - RL_foot_relative_mea) 
                                                                                + swing_kd*(RL_v_relative - RL_v_est_relative))); 
                        }                            
                    }
                }                                                                                                               
                Legs_torque.block<3,1>(0,0) = FR_torque_impedance;
                Legs_torque.block<3,1>(3,0) = FL_torque_impedance;
                Legs_torque.block<3,1>(6,0) = RR_torque_impedance;
                Legs_torque.block<3,1>(9,0) = RL_torque_impedance; 
                /////// 222 Grf feedforward control///////////
                if(!FR_swing)
                {
                    torque.block<3,1>(0,0) += Torque_ff_GRF.block<3,1>(0,0);
                }
                else
                {
                    torque.block<3,1>(0,0) += FR_torque_impedance;
                }
                if(!FL_swing)
                {
                    torque.block<3,1>(3,0) += Torque_ff_GRF.block<3,1>(3,0);
                }
                else
                {
                    torque.block<3,1>(3,0) += FL_torque_impedance;
                }
                if(!RR_swing)
                {
                    torque.block<3,1>(6,0) += Torque_ff_GRF.block<3,1>(6,0);
                }
                else
                {
                    torque.block<3,1>(6,0) += RR_torque_impedance;
                }
                if(!RL_swing)
                {
                    torque.block<3,1>(9,0) += Torque_ff_GRF.block<3,1>(9,0);
                }
                else
                {
                    torque.block<3,1>(9,0) += RL_torque_impedance;
                }

            }
            ///// joint torque filtering&clamp//////////////////////////////////////
            torque_ff(0,0)  = butterworthLPF19.filter(torque(0,0));
            torque_ff(1,0)  = butterworthLPF20.filter(torque(1,0));
            torque_ff(2,0)  = butterworthLPF21.filter(torque(2,0));
            torque_ff(3,0)  = butterworthLPF22.filter(torque(3,0));
            torque_ff(4,0)  = butterworthLPF23.filter(torque(4,0));
            torque_ff(5,0)  = butterworthLPF24.filter(torque(5,0));
            torque_ff(6,0)  = butterworthLPF25.filter(torque(6,0));
            torque_ff(7,0)  = butterworthLPF26.filter(torque(7,0));
            torque_ff(8,0)  = butterworthLPF27.filter(torque(8,0));
            torque_ff(9,0)  = butterworthLPF28.filter(torque(9,0));
            torque_ff(10,0)  = butterworthLPF29.filter(torque(10,0));
            torque_ff(11,0)  = butterworthLPF30.filter(torque(11,0));                
            for(int j=0; j<12;j++)
            {
                if(j % 3 == 0)
                {
                    if(torque_ff(j,0) > 10.0f) torque_ff(j,0) = 10.0f;
                    if(torque_ff(j,0) < -10.0f) torque_ff(j,0) = -10.0f;
                }
                if(j % 3 == 1)
                {
                    if(torque_ff(j,0) > 15.0f) torque_ff(j,0) = 15.0f;
                    if(torque_ff(j,0) < -15.0f) torque_ff(j,0) = -15.0f;
                }
                if(j % 3 == 2)
                {
                    if(torque_ff(j,0) > 15.0f) torque_ff(j,0) = 15.0f;
                    if(torque_ff(j,0) < -15.0f) torque_ff(j,0) = -15.0f;
                }
            }
            /////////////// send commanded torque to LCM ///////////////////////
            for(int j=0; j<12;j++)
            {
                /////// purely torque control
            //     SendLowROS.motorCmd[j].q = PosStopF;
            //     SendLowROS.motorCmd[j].dq = VelStopF;
            //     SendLowROS.motorCmd[j].Kp = 0;
            //     SendLowROS.motorCmd[j].Kd = 0;
            //     SendLowROS.motorCmd[j].tau = torque(j,0);
            //    // SendLowROS.motorCmd[j].tau = 0;     ///for rest length measurement  
                ////// joint-level + toruqe
                SendLowROS.motorCmd[j].q = qDes[j];
                SendLowROS.motorCmd[j].dq = 0;
                SendLowROS.motorCmd[j].Kp = Kp_joint[j];
                SendLowROS.motorCmd[j].Kd = Kd_joint[j];
                if(using_ff>0.5)
                {
                    SendLowROS.motorCmd[j].tau = torque_ff(j,0); 
                }
                else
                {
                    SendLowROS.motorCmd[j].tau = 0;
                }                
            }
        }
    }
    

    /////====== last time data saving ===================/////////// 
    FR_foot_relative_des_old = FR_foot_relative_des;
    FL_foot_relative_des_old = FL_foot_relative_des;
    RR_foot_relative_des_old = RR_foot_relative_des;
    RL_foot_relative_des_old = RL_foot_relative_des;
    FR_foot_relative_mea_old = FR_foot_relative_mea;
    FL_foot_relative_mea_old = FL_foot_relative_mea;
    RR_foot_relative_mea_old = RR_foot_relative_mea;
    RL_foot_relative_mea_old = RL_foot_relative_mea;
    FR_foot_mea_old = FR_foot_mea; 
    FL_foot_mea_old = FL_foot_mea;
    RR_foot_mea_old = RR_foot_mea; 
    RL_foot_mea_old = RL_foot_mea;
    FR_foot_des_old = FR_foot_des;
    FL_foot_des_old = FL_foot_des;
    RR_foot_des_old = RR_foot_des;
    RL_foot_des_old = RL_foot_des;

    for(int i=0;i<3;i++)
    {
        body_relative_support_des_old[i] =  body_p_des[i] - support_pos_des[i];
        body_relative_support_sensor_old[i] =  com_sensor[i] - support_pos_sensor[i];
        com_des_pre[i] = body_p_des[i];
        theta_des_pre[i] = theta_des[i];
        com_sensor_pre[i] = com_sensor[i];                    
    }
    theta_estkine_pre = theta_estkine;
    base_offset_x_old = base_offset_x;
    base_offset_y_old = base_offset_y; 
    base_offset_z_old = base_offset_z;
    base_offset_roll_old = base_offset_roll;
    base_offset_pitch_old = base_offset_pitch;
    base_offset_yaw_old = base_offset_yaw;
    
    

    init_count_old = init_count;


    auto t4 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> ms_double = t4 - t3;
    ///********************* data saving & publisher ************************************///////
    joint2simulation.header.stamp = ros::Time::now();
    joint2simulationx.header.stamp = ros::Time::now();
    leg2sim.header.stamp = ros::Time::now();
    ////
    for(int j=0; j<12; j++){
            joint2simulation.position[j] = qDes[j]; // desired joint angles; 
    }
    for(int j=0; j<12; j++)
    {
        joint2simulation.position[12+j] = RecvLowROS.motorState[j].q;   // measured joint angles;
    } 

    for(int j=0; j<3; j++)
    {
        joint2simulation.position[24+j] = FR_foot_des(j,0);   // desired FR Leg position;
    }         
    for(int j=0; j<3; j++)
    {
        joint2simulation.position[27+j] = FL_foot_des(j,0);   // desired position;
    }
    for(int j=0; j<3; j++)
    {
        joint2simulation.position[30+j] = RR_foot_des(j,0);   // desired position;
    }
    for(int j=0; j<3; j++)
    {
        joint2simulation.position[33+j] = RL_foot_des(j,0);   // desired position;
    } 

    for(int j=0; j<3; j++)
    {
        // joint2simulation.position[36+j] = FR_foot_relative_mea(j,0) + body_p_des(j,0);   // measured position;
        joint2simulation.position[36+j] = frfoot_pose_sensor[j];   // measured position;
    }         
    for(int j=0; j<3; j++)
    {
        // joint2simulation.position[39+j] = FL_foot_relative_mea(j,0) + body_p_des(j,0);   // measured position;
        joint2simulation.position[39+j] = flfoot_pose_sensor[j];    // measured position;
    }
    for(int j=0; j<3; j++)
    {
        //joint2simulation.position[42+j] = RR_foot_relative_mea(j,0) + body_p_des(j,0);   // measured position;
        joint2simulation.position[42+j] = rrfoot_pose_sensor[j];   // measured position;
    }
    for(int j=0; j<3; j++)
    {
        // joint2simulation.position[45+j] = RL_foot_relative_mea(j,0) + body_p_des(j,0);   // measured position;
        joint2simulation.position[45+j] = rlfoot_pose_sensor[j];   // measured position;
    } 
    
    /// body_pos_des
    for(int j=0; j<3; j++)
    {
        joint2simulation.position[48+j] = body_p_des(j,0);   // desired body position;
    }
    /// body_R_des
    for(int j=0; j<3; j++)
    {
        joint2simulation.position[51+j] = body_r_des(j,0);   // desired body ori;
    }
    /// body_R_mea
    for(int j=0; j<3; j++)
    {
        joint2simulation.position[90+j] = body_r_est(j,0);   // measure body ori;
    }
    
    ////////
    for(int j=0; j<6; j++)
    {
        joint2simulation.position[54+j] = Force_L_R(j);  // desired force on left and right leg;
    }


    // FR force
    for(int j=0; j<3; j++)
    {
        joint2simulation.position[66+j] = Dynam.F_leg_ref(j,0); 
    }

    // FL force
    for(int j=0; j<3; j++)
    {
        joint2simulation.position[69+j] = Dynam.F_leg_ref(j,1);
    } 

    // RR force
    for(int j=0; j<3; j++)
    {
        joint2simulation.position[72+j] = Dynam.F_leg_ref(j,2); 
    }  

    // RL force
    for(int j=0; j<3; j++)
    {
        joint2simulation.position[75+j] = Dynam.F_leg_ref(j,3);
    } 

    // joint torque desired: fb + ff //////// 
    for(int j=0; j<12; j++)
    {
        joint2simulation.position[78+j] = SendLowROS.motorCmd[j].tau;
    }

    // estimated CoM position //////// 
    for(int j=0; j<3; j++)
    {
        joint2simulation.position[93+j] = com_sensor[j];
    }
    for(int j=0; j<3; j++)
    {
        joint2simulation.position[96+j] = w_pos_m_filter[j];
    }

    //////////////////////torque display
    //// torque measured //////////////////////
    for(int j=0; j<12; j++)
    {
        joint2simulationx.position[j] = RecvLowROS.motorState[j].tauEst;
    }
    
    ////// optimal grf compensation/////////
    for(int j=0; j<3; j++)
    {
        joint2simulationx.position[12+j] = FR_GRF_opt[j];
    }
    for(int j=0; j<3; j++)
    {
        joint2simulationx.position[15+j] = FL_GRF_opt[j];
    }
    for(int j=0; j<3; j++)
    {
        joint2simulationx.position[18+j] = RR_GRF_opt[j];
    }
    for(int j=0; j<3; j++)
    {
        joint2simulationx.position[21+j] = RL_GRF_opt[j];
    }                 

    /////// spring torque compensation//////
    for(int j=0; j<12; j++)
    {
        if(enable_spring)
        {
            joint2simulationx.position[24+j] = Torque_ff_spring(j,0);
        }
        else
        {
            joint2simulationx.position[24+j] = Torque_ff_GRF(j,0);
        }
        
    } 

    /////// leg impedance-control torque compensation//////
    for(int j=0; j<12; j++)
    {
        joint2simulationx.position[36+j] = Legs_torque(j,0);
    }   

    for(int j=0; j<4; j++)
    {
        joint2simulationx.position[48+j] = RecvLowROS.footForce[j];
    }

        for(int j=0; j<3; j++)
    {
        joint2simulationx.position[52+j] = a1_ctrl_states.imu_acc[j];
    }

    for(int j=0; j<3; j++)
    {
        joint2simulationx.position[55+j] = ground_angle[j];
    }        
                

    // robot data publisher
    for(int j=0;j<100;j++)
    {
        leg2sim.position[j] = state_gait_ekf[j];
    }
    leg2sim.position[99] = 1000*dt_rt_loop.toSec();
    leg2sim.position[98] = ms_double.count();


    // /// publisher to wbc:;
    for(int j=0; j<3; j++)
    {
        date2wbc.position[j] = FR_foot_Homing(j,0);   // homing FR Leg position;
    }         
    for(int j=0; j<3; j++)
    {
        date2wbc.position[3+j] = FL_foot_Homing(j,0);   // homing position;
    }
    for(int j=0; j<3; j++)
    {
        date2wbc.position[6+j] = RR_foot_Homing(j,0);   // homing position;
    }
    for(int j=0; j<3; j++)
    {
        date2wbc.position[9+j] = RL_foot_Homing(j,0);   // homing position;
    } 
    for(int j=0; j<36; j++)
    {
        date2wbc.position[12+j] = state_gait_ekf(j,0);   // estimated;
    }

    date2wbc.position[99] = start_grf;


    gait_data_pub.publish(joint2simulation);
    gait_data_pubx.publish(joint2simulationx);
    state_estimate_ekf_pub_.publish(leg2sim);     
    gait_step_real_time.publish(date2wbc);     


    // /////sending command ////////////
    if(debug_mode<0.5)
    {
        udp.SetSend(SendLowROS);        
    }


    count++;
    if(count > 1000){
        count = 1000;
        initiated_flag = true;
    }

    ////// IMU drift calibration///////
    n_count++;  
    if(n_count==5001)
    {
        root_euler_offset(0) *= (1.0/5000);
        root_euler_offset(1) *= (1.0/5000);
        root_euler_offset(2) *= (1.0/5000); 
        root_euler_angular_velocity_offset(0) *= (1.0/5000);  
        root_euler_angular_velocity_offset(1) *= (1.0/5000);  
        root_euler_angular_velocity_offset(2) *= (1.0/5000);                       
            
    }
    else
    {
        if(n_count>5001)
        {
            n_count =5002;
        }
        else
        {
            root_euler_offset(0) += root_euler(0);
            root_euler_offset(1) += root_euler(1);
            root_euler_offset(2) += root_euler(2);  
            root_euler_angular_velocity_offset(0) +=  RecvLowROS.imu.gyroscope[0];  
            root_euler_angular_velocity_offset(1) +=  RecvLowROS.imu.gyroscope[1];  
            root_euler_angular_velocity_offset(2) +=  RecvLowROS.imu.gyroscope[2];             
        }
            
    }

}






// This main() is here for testing/debugging - remove when using the landing_controller in another function.
int main(int argc, char *argv[]){

    ros::init(argc, argv, "torque_jumping_controller");
    

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    Quadruped robot(LOWLEVEL);
    double dt = 0.002; // Change the operational period (in sec).
    // InitEnvironment();
    LoopFunc loop_control("control_loop", dt,    boost::bind(&Quadruped::RobotControl, &robot));
    LoopFunc loop_udpSend("udp_send",     dt, 3, boost::bind(&Quadruped::UDPSend,      &robot));
    LoopFunc loop_udpRecv("udp_recv",     dt, 3, boost::bind(&Quadruped::UDPRecv,      &robot));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    ros::spin();


    return 0;

}