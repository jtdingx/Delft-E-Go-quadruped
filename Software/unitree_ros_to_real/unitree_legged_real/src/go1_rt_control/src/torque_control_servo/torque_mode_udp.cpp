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
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>




using namespace UNITREE_LEGGED_SDK;
using namespace std;

// global varable generation

double qDes[12]={0};
double qDes_pre[12]={0};
double dqDes[12] = {0}; 
double dqDes_old[12] = {0}; 
Eigen::Matrix<double, 12, 1> dqdes_raw;
double Kp_joint_ini[12] = {50/2,60/2,60/2,50/2,60/2,60/2,60/2,60/2,60/2,60/2,60/2,60/2};
double Kd_joint_ini[12] = {3/2,4/2,4/2,3/2,4/2,4/2,2/2,4/2,4/2,2/2,4/2,4/2};
// float Kp_joint_retarget[12] = {50,150,150,50,150,150,80,150,150,80,150,150};
// float Kd_joint_retarget[12] = {3,5,5,3,5,5,2,5,5,2,5,5};
double Kp_joint_retarget[12] = {60,60,40,60,60,40,60,75,70,60,75,70};
double Kd_joint_retarget[12] = {1.25,1.25,1.25,1.25,1.25,1.25,1.25,1.5,1.5,1.25,1.5,1.5};  ///smaller kd causes quiet motion

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

///////
double accelerometerx = 0;
double accelerometery = 0;
double accelerometerz = 0;
double accelerometerx_avg = 0;
double accelerometery_avg = 0;
double accelerometerz_avg = 0;
geometry_msgs::PoseStamped currentDroneState; 
geometry_msgs::PoseStamped previousDroneState; 
gazebo_msgs::ModelStates current_states;


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
    return p;
}


void Quadruped::keyboard_model_callback(const geometry_msgs::Twist::ConstPtr &msgIn) 
{
    if((msgIn->linear.x == 1)&&(gait_status == STAND_STATUS)&&(stand_count*dtx >= 5))
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

void Quadruped::gait_pose_callback(LowState RecvLowROS,double dt, int j)
{
    /// FL, FR, RL, RR;
    Go1_ctrl_states.joint_pos[0] = RecvLowROS.motorState[3].q;
    Go1_ctrl_states.joint_vel[0] = RecvLowROS.motorState[3].dq;
    Go1_ctrl_states.joint_pos[1] = RecvLowROS.motorState[4].q;
    Go1_ctrl_states.joint_vel[1] = RecvLowROS.motorState[4].dq;
    Go1_ctrl_states.joint_pos[2] = RecvLowROS.motorState[5].q;
    Go1_ctrl_states.joint_vel[2] = RecvLowROS.motorState[5].dq;

    Go1_ctrl_states.joint_pos[3] = RecvLowROS.motorState[0].q;
    Go1_ctrl_states.joint_vel[3] = RecvLowROS.motorState[0].dq;
    Go1_ctrl_states.joint_pos[4] = RecvLowROS.motorState[1].q;
    Go1_ctrl_states.joint_vel[4] = RecvLowROS.motorState[1].dq;
    Go1_ctrl_states.joint_pos[5] = RecvLowROS.motorState[2].q;
    Go1_ctrl_states.joint_vel[5] = RecvLowROS.motorState[2].dq;

    Go1_ctrl_states.joint_pos[6] = RecvLowROS.motorState[9].q;
    Go1_ctrl_states.joint_vel[6] = RecvLowROS.motorState[9].dq;   
    Go1_ctrl_states.joint_pos[7] = RecvLowROS.motorState[10].q;
    Go1_ctrl_states.joint_vel[7] = RecvLowROS.motorState[10].dq;
    Go1_ctrl_states.joint_pos[8] = RecvLowROS.motorState[11].q;
    Go1_ctrl_states.joint_vel[8] = RecvLowROS.motorState[11].dq;

    Go1_ctrl_states.joint_pos[9] = RecvLowROS.motorState[6].q;
    Go1_ctrl_states.joint_vel[9] = RecvLowROS.motorState[6].dq;
    Go1_ctrl_states.joint_pos[10] = RecvLowROS.motorState[7].q;
    Go1_ctrl_states.joint_vel[10] = RecvLowROS.motorState[7].dq;
    Go1_ctrl_states.joint_pos[11] = RecvLowROS.motorState[8].q;
    Go1_ctrl_states.joint_vel[11] = RecvLowROS.motorState[8].dq;

    Go1_ctrl_states.foot_force[0] = (footforce_fl.mean()); 
    Go1_ctrl_states.foot_force[1] = (footforce_fr.mean()); 
    Go1_ctrl_states.foot_force[2] = (footforce_rl.mean()); 
    Go1_ctrl_states.foot_force[3] = (footforce_rr.mean()); 

    // Go1_ctrl_states.foot_force[0] = (footforce_fl(4,0)); 
    // Go1_ctrl_states.foot_force[1] = (footforce_fr(4,0)); 
    // Go1_ctrl_states.foot_force[2] = (footforce_rl(4,0)); 
    // Go1_ctrl_states.foot_force[3] = (footforce_rr(4,0));   

    Go1_ctrl_states.imu_acc = Eigen::Vector3d(
            acc_x.CalculateAverage(RecvLowROS.imu.accelerometer[0]),
            acc_y.CalculateAverage(RecvLowROS.imu.accelerometer[1]),
            acc_z.CalculateAverage(RecvLowROS.imu.accelerometer[2])
    );
    Go1_ctrl_states.imu_ang_vel = Eigen::Vector3d(
            gyro_x.CalculateAverage(RecvLowROS.imu.gyroscope[0]),
            gyro_y.CalculateAverage(RecvLowROS.imu.gyroscope[1]),
            gyro_z.CalculateAverage(RecvLowROS.imu.gyroscope[2])
    );            
    rot_imu_acc = Go1_ctrl_states.root_rot_mat * Go1_ctrl_states.imu_acc;

    /////// mean value of the imu_linear acc, angular velocity;//// detect the IMU bias
    if(j<2000) /////// before calibration /// initialize the EKF estimator
    {
        if(simulation_mode<=1) /// in simulation; using the ground truth from pybullet or gazebo
        {
           Go1_ctrl_states.estimated_root_pos[0] =  state_gait_ekf[36];
           Go1_ctrl_states.estimated_root_pos[1] =  state_gait_ekf[37];
           Go1_ctrl_states.estimated_root_pos[2] =  state_gait_ekf[38];
           Go1_ctrl_states.estimated_root_vel[0] =  state_gait_ekf[39];
           Go1_ctrl_states.estimated_root_vel[1] =  state_gait_ekf[40];
           Go1_ctrl_states.estimated_root_vel[2] =  state_gait_ekf[41];

        }
        else //// in hardware test: using the kinematic variables;
        {
           Go1_ctrl_states.estimated_root_pos[0] =  com_sensor[0];
           Go1_ctrl_states.estimated_root_pos[1] =  com_sensor[1];
           Go1_ctrl_states.estimated_root_pos[2] =  com_sensor[2];
           Go1_ctrl_states.estimated_root_vel[0] =  comv_sensor[0];
           Go1_ctrl_states.estimated_root_vel[1] =  comv_sensor[1];
           Go1_ctrl_states.estimated_root_vel[2] =  comv_sensor[2];           
        }
    }
    else{
        if(gait_status == STAND_STATUS) //// in the calibration process;
        {
            // accelerometerx += acc_x.CalculateAverage(RecvLowROS.imu.accelerometer[0]);
            // accelerometery += acc_y.CalculateAverage(RecvLowROS.imu.accelerometer[1]);
            // accelerometerz += acc_z.CalculateAverage(RecvLowROS.imu.accelerometer[2]);
            // accelerometerx += RecvLowROS.imu.accelerometer[0];
            // accelerometery += RecvLowROS.imu.accelerometer[1];
            // accelerometerz += RecvLowROS.imu.accelerometer[2];  

            accelerometerx += rot_imu_acc[0];
            accelerometery += rot_imu_acc[1];
            accelerometerz += rot_imu_acc[2];            

            n_count_calibration += 1;  
        }
        else
        {
            if(stand_up_count == 1)
            {
                accelerometerx_avg = accelerometerx / n_count_calibration;
                accelerometery_avg = accelerometery / n_count_calibration;
                accelerometerz_avg = accelerometerz / n_count_calibration;
                gra_acc_est(0) = -accelerometerx_avg;
                gra_acc_est(1) = -accelerometery_avg;
                gra_acc_est(2) = -accelerometerz_avg;
                cout<<"accelerometerx_avg:"<<accelerometerx_avg<<endl;
                cout<<"accelerometery_avg:"<<accelerometery_avg<<endl;
                cout<<"gra_acc_est:"<<gra_acc_est<<endl;
            }

        }
    }



    // calculate several useful variables
    // euler should be roll pitch yaw
    Go1_ctrl_states.root_quat = root_quat.normalized();
    ///// built in function is not right
    // Go1_ctrl_states.root_rot_mat = Go1_ctrl_states.root_quat.normalized().toRotationMatrix();
    // Go1_ctrl_states.root_euler = Utils::quat_to_euler(Go1_ctrl_states.root_quat);
    Go1_ctrl_states.root_rot_mat = root_rot_mat;
    Go1_ctrl_states.root_euler = body_r_est;

    double yaw_angle = Go1_ctrl_states.root_euler[2];

    Go1_ctrl_states.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

    Go1_ctrl_states.root_ang_vel = Go1_ctrl_states.root_rot_mat * Go1_ctrl_states.imu_ang_vel;


    //////*****************************************main loop **************************************************////
    // TODO: we call estimator update here, be careful the runtime should smaller than the HARDWARE_UPDATE_FREQUENCY,i.e., 2ms
    // state estimation
    if (!go1_estimate.is_inited() || (gait_status == STAND_INIT_STATUS) || (gait_status == STAND_STATUS)) {
        go1_estimate.init_state(Go1_ctrl_states); ///// 
    } else {
        go1_estimate.update_estimation(Go1_ctrl_states, dt,simulation_mode, gra_acc_est);
    }    

    // FL, FR, RL, RR
    // use estimation base pos and vel to get foot pos and foot vel in world frame
    for (int i = 0; i < NUM_LEG; ++i) {
        Go1_ctrl_states.foot_pos_rel.block<3, 1>(0, i) = go1_kin.fk(
                Go1_ctrl_states.joint_pos.segment<3>(3 * i),
                rho_opt_list[i], rho_fix_list[i]);
        Go1_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i) = go1_kin.jac(
                Go1_ctrl_states.joint_pos.segment<3>(3 * i),
                rho_opt_list[i], rho_fix_list[i]);
        Eigen::Matrix3d tmp_mtx = Go1_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i);
        Eigen::Vector3d tmp_vec = Go1_ctrl_states.joint_vel.segment<3>(3 * i);
        Go1_ctrl_states.foot_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

        Go1_ctrl_states.foot_pos_abs.block<3, 1>(0, i) =
                Go1_ctrl_states.root_rot_mat * Go1_ctrl_states.foot_pos_rel.block<3, 1>(0, i);
        Go1_ctrl_states.foot_vel_abs.block<3, 1>(0, i) =
                Go1_ctrl_states.root_rot_mat * Go1_ctrl_states.foot_vel_rel.block<3, 1>(0, i);

        // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
        // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
        // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
        Go1_ctrl_states.foot_pos_world.block<3, 1>(0, i) =
                Go1_ctrl_states.foot_pos_abs.block<3, 1>(0, i) + Go1_ctrl_states.root_pos;
        Go1_ctrl_states.foot_vel_world.block<3, 1>(0, i) =
                Go1_ctrl_states.foot_vel_abs.block<3, 1>(0, i) + Go1_ctrl_states.root_lin_vel;
    }

    
    state_gait_ekf.block<3,1>(0,0) = Go1_ctrl_states.estimated_root_pos;
    state_gait_ekf.block<3,1>(3,0) = Go1_ctrl_states.estimated_root_vel;
    state_gait_ekf.block<3,1>(6,0) = Go1_ctrl_states.root_euler;
    state_gait_ekf.block<3,1>(9,0) = Go1_ctrl_states.root_ang_vel;
    state_gait_ekf.block<3,1>(12,0) = Go1_ctrl_states.foot_pos_world.col(1);
    state_gait_ekf.block<3,1>(15,0) = Go1_ctrl_states.foot_pos_world.col(0);
    state_gait_ekf.block<3,1>(18,0) = Go1_ctrl_states.foot_pos_world.col(3);
    state_gait_ekf.block<3,1>(21,0) = Go1_ctrl_states.foot_pos_world.col(2);
    state_gait_ekf.block<3,1>(24,0) = Go1_ctrl_states.foot_vel_world.col(1);
    state_gait_ekf.block<3,1>(27,0) = Go1_ctrl_states.foot_vel_world.col(0);
    state_gait_ekf.block<3,1>(30,0) = Go1_ctrl_states.foot_vel_world.col(3);
    state_gait_ekf.block<3,1>(33,0) = Go1_ctrl_states.foot_vel_world.col(2); 
    state_gait_ekf[42] = RecvLowROS.imu.accelerometer[0];
    state_gait_ekf[43] = RecvLowROS.imu.accelerometer[1];
    state_gait_ekf[44] = RecvLowROS.imu.accelerometer[2];
    state_gait_ekf.block<3,1>(45,0) = rot_imu_acc;
    state_gait_ekf.block<4,1>(48,0) = Go1_ctrl_states.estimated_contacts;
    state_gait_ekf.block<3,1>(52,0) = rot_imu_acc + gra_acc_est;


    WBDpino.computeInertiaMatrix(Go1_ctrl_states.estimated_root_vel,Go1_ctrl_states.imu_ang_vel,
    Go1_ctrl_states.joint_pos, Go1_ctrl_states.joint_vel,Go1_ctrl_states.estimated_root_pos,Go1_ctrl_states.root_quat);

    WBDpino.compute_gravity(Go1_ctrl_states.estimated_root_vel,Go1_ctrl_states.imu_ang_vel,
    Go1_ctrl_states.joint_pos, Go1_ctrl_states.joint_vel,Go1_ctrl_states.estimated_root_pos,Go1_ctrl_states.root_quat); 

    torque_bias = WBDpino.torque_bias;
    torque_gravity = WBDpino.torque_gravity;
    torque_coriolis = WBDpino.torque_coriolis;

    state_gait_ekf.block<12,1>(55,0) = torque_bias;
    state_gait_ekf.block<12,1>(67,0) = torque_gravity;
    state_gait_ekf.block<12,1>(79,0) = torque_coriolis;

    // if((stand_up_count % 100 == 1)|| (dynamic_count % 50 ==1))
    // {
    //     std::cout<<"centriodal angular momentum:"<<WBDpino.Ig<<endl;
    //     std::cout<<"upper_body:"<<Go1_ctrl_states.root_rot_mat.transpose() * Momentum_sum *  Go1_ctrl_states.root_rot_mat.transpose() <<endl;
    //     std::cout<<"CoriolisMatrix:"<<WBDpino.Coriolis_M.block<3,3>(6,6) <<endl;
    //     std::cout<<"Gravity Matrix:"<<WBDpino.gravity_M.block<3,1>(6,0) <<endl;
    //     std::cout<<"torque_drift:"<<WBDpino.torque_drift.block<3,1>(6,0) <<endl;
           
    // }


}


Eigen::Matrix<double, 100,1>  Quadruped::state_est_main_update(double dt) {

    Eigen::Matrix<double,100,1> state;
    state.setZero();
    // // state estimation
    // if (!go1_estimate.is_inited()) {
    //     go1_estimate.init_state(Go1_ctrl_states);
    // } else {
    //     go1_estimate.update_estimation(Go1_ctrl_states, dt, estimated_root_pos_offset);
    // }

    // state.block<3,1>(0,0) = Go1_ctrl_states.estimated_root_pos;
    // state.block<3,1>(3,0) = Go1_ctrl_states.estimated_root_vel;
    // state.block<3,1>(6,0) = Go1_ctrl_states.root_euler;
    // state.block<3,1>(9,0) = Go1_ctrl_states.imu_ang_vel;
    // state.block<3,1>(12,0) = Go1_ctrl_states.foot_pos_world.col(1);
    // state.block<3,1>(15,0) = Go1_ctrl_states.foot_pos_world.col(0);
    // state.block<3,1>(18,0) = Go1_ctrl_states.foot_pos_world.col(3);
    // state.block<3,1>(21,0) = Go1_ctrl_states.foot_pos_world.col(2);
    // state.block<3,1>(24,0) = Go1_ctrl_states.foot_vel_world.col(1);
    // state.block<3,1>(27,0) = Go1_ctrl_states.foot_vel_world.col(0);
    // state.block<3,1>(30,0) = Go1_ctrl_states.foot_vel_world.col(3);
    // state.block<3,1>(33,0) = Go1_ctrl_states.foot_vel_world.col(2);

    return state;
}

void Quadruped::ground_truth_state_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{

   current_states = *msg;
   int irisArrPos = 999;
 
   //search for the drone
   for (int i=0; i< current_states.name.size(); i++)
   {
     if(current_states.name[i] == "go1_gazebo")
     {
         irisArrPos = i;
         break;
     }
     
   }
   if (irisArrPos == 999)
   {
     std::cout << "go1_gazebo is not in world" << std::endl;
   }else{
     //assign drone pose to pose stamped
     currentDroneState.pose = current_states.pose[irisArrPos];
     currentDroneState.header.stamp = ros::Time::now();
   }
   

   state_gait_ekf[36] = currentDroneState.pose.position.x;
   state_gait_ekf[37] = currentDroneState.pose.position.y;
   state_gait_ekf[38] = currentDroneState.pose.position.z;
   if(n_count==0)
   {
        state_gait_ekf[39] = currentDroneState.pose.position.x;
        state_gait_ekf[40] = currentDroneState.pose.position.y;
        state_gait_ekf[41] = currentDroneState.pose.position.z;    
   }
   else
   {
        state_gait_ekf[39] = (currentDroneState.pose.position.x - previousDroneState.pose.position.x)/dtx;
        state_gait_ekf[40] = (currentDroneState.pose.position.y - previousDroneState.pose.position.y)/dtx;
        state_gait_ekf[41] = (currentDroneState.pose.position.z - previousDroneState.pose.position.z)/dtx;    
   }

   previousDroneState = currentDroneState;

//    state_gait_ekf[39] = currentDroneState.pose.orientation.w
//    state_gait_ekf[40] = currentDroneState.pose.position.y
//    state_gait_ekf[41] = currentDroneState.pose.position.z   

//   std::cout<<"subscribe"<<std::endl;
}



void Quadruped::nrt_gait_sub_operation(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int jx = 0; jx<100; jx++)
    {
        slow_mpc_gait(jx) = msg->position[jx]; 
    }
}

void Quadruped::rt_gait_sub_operation(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int jx = 0; jx<51; jx++)
    {
        fast_mpc_gait(jx) = msg->position[36+jx]; 
    }
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
                    support_pos_sensor[0] = (Go1_ctrl_states.foot_pos_world(0,0) + Go1_ctrl_states.foot_pos_world(0,2))/2;
                    support_pos_sensor[1] = (Go1_ctrl_states.foot_pos_world(1,0) + Go1_ctrl_states.foot_pos_world(1,2))/2;
                    support_pos_sensor[2] = (Go1_ctrl_states.foot_pos_world(2,0) + Go1_ctrl_states.foot_pos_world(2,2))/2;
                }
                else
                {
                    support_pos_sensor[2] = (Go1_ctrl_states.foot_pos_world(2,0) + Go1_ctrl_states.foot_pos_world(2,2))/2;
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
                    support_pos_sensor[0] = (Go1_ctrl_states.foot_pos_world(0,1) + Go1_ctrl_states.foot_pos_world(0,2))/2;
                    support_pos_sensor[1] = (Go1_ctrl_states.foot_pos_world(1,1) + Go1_ctrl_states.foot_pos_world(1,2))/2;
                    support_pos_sensor[2] = (Go1_ctrl_states.foot_pos_world(2,1) + Go1_ctrl_states.foot_pos_world(2,2))/2;
                }
                else
                {
                    support_pos_sensor[2] = (Go1_ctrl_states.foot_pos_world(2,1) + Go1_ctrl_states.foot_pos_world(2,2))/2;
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
                        support_pos_sensor[0] = (Go1_ctrl_states.foot_pos_world(0,1) + Go1_ctrl_states.foot_pos_world(0,3))/2;
                        support_pos_sensor[1] = (Go1_ctrl_states.foot_pos_world(1,1) + Go1_ctrl_states.foot_pos_world(1,3))/2;
                        support_pos_sensor[2] = (Go1_ctrl_states.foot_pos_world(2,1) + Go1_ctrl_states.foot_pos_world(2,3))/2;
                    }
                    else
                    {
                        support_pos_sensor[2] = (Go1_ctrl_states.foot_pos_world(2,1) + Go1_ctrl_states.foot_pos_world(2,3))/2;
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
                        support_pos_sensor[0] = (Go1_ctrl_states.foot_pos_world(0,0) + Go1_ctrl_states.foot_pos_world(0,3))/2;
                        support_pos_sensor[1] = (Go1_ctrl_states.foot_pos_world(1,0) + Go1_ctrl_states.foot_pos_world(1,3))/2;
                        support_pos_sensor[2] = (Go1_ctrl_states.foot_pos_world(2,0) + Go1_ctrl_states.foot_pos_world(2,3))/2;
                    }
                    else
                    {
                        support_pos_sensor[2] = (Go1_ctrl_states.foot_pos_world(2,0) + Go1_ctrl_states.foot_pos_world(2,3))/2;
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
                support_pos_sensor[0] = (Go1_ctrl_states.foot_pos_world(0,0) + Go1_ctrl_states.foot_pos_world(0,3) 
                                    + Go1_ctrl_states.foot_pos_world(0,1) + Go1_ctrl_states.foot_pos_world(0,2))/4;
                support_pos_sensor[1] = (Go1_ctrl_states.foot_pos_world(1,0) + Go1_ctrl_states.foot_pos_world(1,3)
                                    + Go1_ctrl_states.foot_pos_world(1,1) + Go1_ctrl_states.foot_pos_world(1,2))/4;
                support_pos_sensor[2] = (Go1_ctrl_states.foot_pos_world(2,0) + Go1_ctrl_states.foot_pos_world(2,3)
                                    + Go1_ctrl_states.foot_pos_world(2,1) + Go1_ctrl_states.foot_pos_world(2,2))/4;
                support_pos_des[0] = (FL_foot_des[0] + RR_foot_des[0] + FR_foot_des[0] + RL_foot_des[0])/4;
                support_pos_des[1] = (FL_foot_des[1] + RR_foot_des[1] + FR_foot_des[1] + RL_foot_des[1])/4;
                support_pos_des[2] = (FL_foot_des[2] + RR_foot_des[2] + FR_foot_des[2] + RL_foot_des[2])/4;                                    
            }
            else
            {
                support_pos_sensor[2] = (Go1_ctrl_states.foot_pos_world(2,0) + Go1_ctrl_states.foot_pos_world(2,3)
                                    + Go1_ctrl_states.foot_pos_world(2,1) + Go1_ctrl_states.foot_pos_world(2,2))/4;                
            
                support_pos_des[0] = support_pos_sensor[0];
                support_pos_des[1] = support_pos_sensor[1];
                support_pos_des[2] = support_pos_sensor[2];             
            }
            
               
        }                
        
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
    w_pos_m_filter[0] = std::min(std::max(butterworthLPF31.filter(w_pos_m[0]),com_pos_min),com_pos_max);
    w_pos_m_filter[1] = std::min(std::max(butterworthLPF32.filter(w_pos_m[1]),com_pos_min),com_pos_max);
    w_pos_m_filter[2] = std::min(std::max(butterworthLPF33.filter(w_pos_m[2]),com_pos_min),com_pos_max);

    w_rpy_m[0] = w_rpy_m_kp[0] * (body_r_des[0]- theta_estkine[0]) + w_rpy_m_kp[0] * (0-thetav_estkine[0]); /// x;
    w_rpy_m[1] = w_rpy_m_kp[1] * (body_r_des[1]- theta_estkine[1]) + w_rpy_m_kp[1] * (0-thetav_estkine[1]);  /// y;
    w_rpy_m[2] = w_rpy_m_kp[2] * (body_p_des[2]- theta_estkine[2]) + w_rpy_m_kp[2] * (0-thetav_estkine[2]); /// z:
    w_rpy_m_filter[0] = std::min(std::max(butterworthLPF34.filter(w_rpy_m[0]),com_rpy_min),com_rpy_max);
    w_rpy_m_filter[1] = std::min(std::max(butterworthLPF35.filter(w_rpy_m[1]),com_rpy_min),com_rpy_max);
    w_rpy_m_filter[2] = std::min(std::max(butterworthLPF36.filter(w_rpy_m[2]),com_rpy_min),com_rpy_max);             
}

void Quadruped::base_acc_ff_controler()
{
    /// update planning loop: reinitiliazation of global CoM position;
    if(init_count>init_count_old)
    {
    //    std::cout<<"re_initialization in rt_loop:" <<std::endl;
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
    Eigen::Matrix3d R_des =  Utils::eulerAnglesToRotationMatrix(body_r_des);
    Eigen::Matrix3d R_est =  Utils::eulerAnglesToRotationMatrix(theta_estkine);
    Eigen::Matrix3d R_error = R_des * R_est.transpose();
    Eigen::Vector3d theta_error = Utils::rotationMatrixToEulerAngles(R_error);
    theta_acc_des[0] = kpw[0] * (theta_error[0]) + kdw[0]*(0 - thetav_estkine[0]);
    theta_acc_des[1] = kpw[1] * (theta_error[1]) + kdw[1]*(0 - thetav_estkine[1]);
    theta_acc_des[2] = kpw[2] * (theta_error[2]) + kdw[2]*(0 - thetav_estkine[2]);     

    // theta_acc_des[0] = kpw[0] * (body_r_des(0,0) - theta_estkine[0]) + kdw[0]*(0 - thetav_estkine[0]);
    // theta_acc_des[1] = kpw[1] * (body_r_des(1,0) - theta_estkine[1]) + kdw[1]*(0 - thetav_estkine[1]);
    // theta_acc_des[2] = kpw[2] * (body_r_des(2,0) - theta_estkine[2]) + kdw[2]*(0 - thetav_estkine[2]); 

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
    using_real_jaco = config["using_real_jaco1"].as<double>();
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
    using_real_rotz =   config["using_real_rotz"].as<double>();    
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

    use_terrain_adapt = config["use_terrain_adapt"].as<double>(); 

    using_fast_mpc = config["using_hie_using"].as<double>(); 

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
    root_rot_mat.setIdentity();
    root_rot_mat_ref.setIdentity();
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

    RR_foot_desx = 0;
    RR_foot_desy = 0;
    RR_foot_desz = 0;  

    RL_foot_desx = 0;
    RL_foot_desy = 0;
    RL_foot_desz = 0;      

    
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

	state_to_MPC.position.resize(53);
	state_feedback.setZero();
	slow_mpc_gait.setZero(); 
    fast_mpc_gait.setZero();   
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


    //////// low-pass-filter: foot support position rotation, hip position=== 
	butterworthLPF101.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF102.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF103.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF104.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF105.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF106.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF107.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF108.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF109.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF110.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF111.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF112.init(f_sample_comx1,fcutoff_comx2);



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
    fcutoff_comx5 = 50; 
    butterworthLPF41.init(f_sample_comx1,fcutoff_comx5); butterworthLPF42.init(f_sample_comx1,fcutoff_comx5); 
    butterworthLPF43.init(f_sample_comx1,fcutoff_comx5); butterworthLPF44.init(f_sample_comx1,fcutoff_comx5);
    butterworthLPF45.init(f_sample_comx1,fcutoff_comx5); butterworthLPF46.init(f_sample_comx1,fcutoff_comx5);
    butterworthLPF47.init(f_sample_comx1,fcutoff_comx5); butterworthLPF48.init(f_sample_comx1,fcutoff_comx5); 
    butterworthLPF49.init(f_sample_comx1,fcutoff_comx5); butterworthLPF50.init(f_sample_comx1,fcutoff_comx5);
    butterworthLPF51.init(f_sample_comx1,fcutoff_comx5); butterworthLPF52.init(f_sample_comx1,fcutoff_comx5);
    


    ////// joint velocity /////

    butterworthLPF53.init(f_sample_comx1,fcutoff_comx51); butterworthLPF54.init(f_sample_comx1,fcutoff_comx51); 
    butterworthLPF55.init(f_sample_comx1,fcutoff_comx51); butterworthLPF56.init(f_sample_comx1,fcutoff_comx51);
    butterworthLPF57.init(f_sample_comx1,fcutoff_comx51); butterworthLPF58.init(f_sample_comx1,fcutoff_comx51);
    butterworthLPF59.init(f_sample_comx1,fcutoff_comx51); butterworthLPF60.init(f_sample_comx1,fcutoff_comx51); 
    butterworthLPF61.init(f_sample_comx1,fcutoff_comx51); butterworthLPF62.init(f_sample_comx1,fcutoff_comx51);
    butterworthLPF63.init(f_sample_comx1,fcutoff_comx51);
    butterworthLPF64.init(f_sample_comx1,fcutoff_comx51); 
    
    butterworthLPF65.init(f_sample_comx1,fcutoff_comx51); 
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

    energy_cost = 0;


    
    pitch_angle_W = 0;

    n_period = round(tstep / dtx); 

    rt_frequency = ctrl_estimation; /// frequency of lower_level control
    time_programming = 1.0/rt_frequency;

    n_count = 0;
    n_count_calibration = 0;
    stand_duration = 5; /// stand up: 2s


    gait_data_pub = n.advertise<sensor_msgs::JointState>("go1_gait_data",10);
    gait_data_pubx = n.advertise<sensor_msgs::JointState>("go1_gait_datax",10);    
    gait_step_real_time = n.advertise<sensor_msgs::JointState>("/go1_gait_parameters",10);   
    state_estimate_ekf_pub_ = n.advertise<sensor_msgs::JointState>("go1_state_estmate",10);
    rt_to_nrt_pub_ = n.advertise<sensor_msgs::JointState>("/control2rtmpc/state", 10);

    robot_mode_cmd = n.subscribe("/Robot_mode", 1, &Quadruped::keyboard_model_callback,this); //// for robot mode selection
    nrt_mpc_gait_subscribe_ = n.subscribe("/MPC/Gait", 10, &Quadruped::nrt_gait_sub_operation,this);
    Grf_gait_subscribe_ = n.subscribe("/go1/Grf_opt", 10, &Quadruped::Grf_sub_operation,this);
    gain_cmd = n.subscribe("/Base_offset", 1, &Quadruped::Base_offset_callback,this);
    rt_mpc_gait_subscribe_ = n.subscribe("/rtMPC/traj", 10, &Quadruped::rt_gait_sub_operation,this);
    ground_truch_model_state = n.subscribe("/gazebo/model_states",10,&Quadruped::ground_truth_state_callback,this);

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
    }


    ////////////////////======================================///////////////////
    ////////////////        state estimation:variable declaration     //////////////////////////////    
    Go1_ctrl_states.reset();
    
    // init leg kinematics
    // set leg kinematics related parameters
    // body_to_Go1_body
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
    
    gra_acc_est << 0,
                   0,
                   -9.8;

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
    std::clock_t start;
    start = std::clock();

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

    // euler angle: roll pitch yaw//// not right ,,,,
    //root_quat = root_quat.normalized();
    //root_rot_mat = root_quat.normalized().toRotationMatrix();

    root_euler = Utils::quat_to_euler(root_quat);

    if(n_count>5001)
    {
        body_r_est(0,0) = root_euler(0) - root_euler_offset(0);
        body_r_est(1,0) = root_euler(1) - root_euler_offset(1);
        body_r_est(2,0) = root_euler(2) - root_euler_offset(2);
    }
    // /// modify the quaternion ///////
    root_rot_mat =  Eigen::AngleAxisd(body_r_est(2,0), Eigen::Vector3d::UnitZ())
                    *Eigen::AngleAxisd(body_r_est(1,0), Eigen::Vector3d::UnitY())
                    *Eigen::AngleAxisd(body_r_est(0,0), Eigen::Vector3d::UnitX());

           

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
    
    ////////////=========================================================================//////////////////////////////
    //////============ state_estimation ========== ///////////////////////////////
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


    ///// estimate global COM position ////////////////////////////
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
        }
    }
    else
    {
        ///// kinematic-based open-loop state estimation //////////
        comav_butterworth = state_est_kine.state_estimator(gait_mode, support_flag, FR_foot_relative_mea, FL_foot_relative_mea,RR_foot_relative_mea, RL_foot_relative_mea,
                                                footforce_fr, footforce_fl,footforce_rr,footforce_rl,frfoot_pose_sensor, flfoot_pose_sensor, rrfoot_pose_sensor, rlfoot_pose_sensor, 
                                                support_pos_sensor, com_sensor, dcm_sensor, omega_sensor,estimated_root_pos_offset);        

        /////////// global state: EKF based estination //////////
        if(using_ros_time>0.5)
        {
            double t_est = std::min(dt_rt_loop.toSec(),dtx);
            gait_pose_callback(RecvLowROS,dt_rt_loop.toSec(),stand_count);
        }
        else
        {
            gait_pose_callback(RecvLowROS,dtx,stand_count); /// assume a fixed time inteval
        }
        estimated_root_vel_rotz = root_rot_mat.transpose() * Go1_ctrl_states.estimated_root_vel;
        linear_vel_rotz = root_rot_mat.transpose()  * Go1_ctrl_states.root_ang_vel;
        if(using_ekf>0.5) /////using the ekf estimator
        {
            for(int j=0; j<3;j++)
            {
                com_sensor[j] = Go1_ctrl_states.estimated_root_pos(j,0);
                comv_sensor[j] = Go1_ctrl_states.estimated_root_vel(j,0);                           
            }
            FR_foot_mea = Go1_ctrl_states.foot_pos_world.col(1);
            FL_foot_mea = Go1_ctrl_states.foot_pos_world.col(0);
            RR_foot_mea = Go1_ctrl_states.foot_pos_world.col(3);
            RL_foot_mea = Go1_ctrl_states.foot_pos_world.col(2);             
            FR_v_est = Go1_ctrl_states.foot_vel_world.col(1); 
            FL_v_est = Go1_ctrl_states.foot_vel_world.col(0);   
            RR_v_est = Go1_ctrl_states.foot_vel_world.col(3); 
            RL_v_est = Go1_ctrl_states.foot_vel_world.col(2);                  
        }
        else
        {
            ////merely modify the height;
            frfoot_pose_sensor[2] = Go1_ctrl_states.foot_pos_world(2,1);
            flfoot_pose_sensor[2] = Go1_ctrl_states.foot_pos_world(2,0);
            rrfoot_pose_sensor[2] = Go1_ctrl_states.foot_pos_world(2,3);
            rlfoot_pose_sensor[2] = Go1_ctrl_states.foot_pos_world(2,2); 
            com_sensor[2] = Go1_ctrl_states.estimated_root_pos(2,0); 

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
    


    thetav_estkine = Go1_ctrl_states.root_ang_vel;

    // //////==== send data to rt+mpc node=====///////////////////
    // t_int = (int) floor(count_in_rt_loop/n_t_int_fast);
    t_int = (int) floor(count_in_rt_loop/n_t_int); /////// dynamic count in the slow_mpc_node....

    state_feedback(0,0) = t_int;
    /// comx(pva),comy(pva),comz(pva),thetax(pva),thetay(pva),thetaz(pva)
    // state_feedback(1,0) = com_sensor[0] - body_p_Homing_dynamic[0] + body_p_Homing_Retarget(0,0);
    state_feedback(1,0) = com_sensor[0] ;
    state_feedback(2,0) = comv_sensor[0];
    state_feedback(3,0) = comav_butterworth(0,0);
    // state_feedback(4,0) = com_sensor[1] - body_p_Homing_dynamic[1]+ body_p_Homing_Retarget(1,0);
    state_feedback(4,0) = com_sensor[1] ;
    state_feedback(5,0) = comv_sensor[1];
    state_feedback(6,0) = comav_butterworth(1,0);
    state_feedback(7,0) = com_sensor[2];
    state_feedback(8,0) = comv_sensor[2];
    state_feedback(9,0) = comav_butterworth(2,0);
    state_feedback(10,0) = body_r_est(0,0);
    state_feedback(11,0) = body_r_est(1,0);
    state_feedback(12,0) = body_r_est(2,0); ////send back the current yaw angle;
    state_feedback(13,0) = thetav_estkine(1,0);
    state_feedback(14,0) = thetav_estkine(1,0);
    state_feedback(15,0) = thetav_estkine(2,0); ////send back the current angular velocity;    

    if(gait_mode==102)// Lfoot_location & Rfoot_location
    {
        state_feedback.block<3,1>(19,0) = (FR_foot_mea- FR_foot_Homing + RL_foot_mea -RL_foot_Homing)/2; ////left foot
        state_feedback.block<3,1>(22,0) = (FL_foot_mea- FL_foot_Homing + RR_foot_mea- RR_foot_Homing)/2; ////right foot    
    }
    
    ////these are the real leg position /////
    state_feedback.block<3,1>(25,0) = FR_foot_mea;
    state_feedback.block<3,1>(28,0) = FL_foot_mea;
    state_feedback.block<3,1>(31,0) = RR_foot_mea;
    state_feedback.block<3,1>(34,0) = RL_foot_mea;


    /////======= support location =======/////
    support_position_feedback.block<3,1>(0,0) = FR_foot_Homing;
    support_position_feedback.block<3,1>(3,0) = FL_foot_Homing;
    support_position_feedback.block<3,1>(6,0) = RR_foot_Homing;
    support_position_feedback.block<3,1>(9,0) = RL_foot_Homing;


    for (int jx = 0; jx<25; jx++)
    {
        state_to_MPC.position[jx] = state_feedback(jx,0);
    } 

    for (int jx = 25; jx<37; jx++)
    {
        state_to_MPC.position[jx] = support_position_feedback(jx-25,0);
    }   
    
    for (int jx = 37; jx<49; jx++)
    {
        state_to_MPC.position[jx] = state_feedback(jx-12,0);
    }
    for (int jx = 49; jx<53; jx++)
    {
        state_to_MPC.position[jx] = foot_earlier_contact_flag(jx-49,0);
    }    
    

    rt_to_nrt_pub_.publish(state_to_MPC);


    // ============== Key board controller ======================
    base_offset_x = clamp_func(base_offset_x,base_offset_x_old, 0.001);
    base_offset_y = clamp_func(base_offset_y,base_offset_y_old, 0.001);
    base_offset_z = clamp_func(base_offset_z,base_offset_z_old, 0.001);        
    base_offset_pitch = clamp_func(base_offset_pitch,base_offset_pitch_old, 0.001);
    base_offset_roll = clamp_func(base_offset_roll,base_offset_roll_old, 0.001);
    base_offset_yaw = clamp_func(base_offset_yaw,base_offset_yaw_old, 0.001); 


    if(initiated_flag == true){
        motiontime++;  

        if( motiontime >= 0){
            //gait status switch begin
            switch (cmd_gait){
                case STAND_INIT:
                    if(1>0)
                    {
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
                    }

                case STAND:
                    if(1>0)
                    {
                        gait_status = STAND_STATUS;
                        stand_count++;
                        if(stand_count==1)
                        {
                            printf("===== RETARGETING HOMING POSE =======\n");
                            cout<<"FR_foot_Homing\n"<<FR_foot_Homing<<endl;
                            cout<<"FL_foot_Homing\n"<<FL_foot_Homing<<endl;
                        }
                        if(stand_count<=2000)
                        {
                            if(using_ekf>0.5)
                            {
                                estimation_offset(0,0) += Go1_ctrl_states.estimated_root_pos(0,0);
                                estimation_offset(1,0) += Go1_ctrl_states.estimated_root_pos(1,0);
                                // estimation_offset(2,0) += Go1_ctrl_states.estimated_root_pos(2,0);
                                estimated_root_pos_offset = estimation_offset/stand_count;
                            }
                            else
                            {
                                estimation_offset(0,0) += com_sensor[0];
                                estimation_offset(1,0) += com_sensor[1];
                                // estimation_offset(2,0) += Go1_ctrl_states.estimated_root_pos(2,0);
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
                        ratex = std::min(pow(stand_count/1000.0,2),1.0); 
                        if(ratex<=1)
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
                    }
                case STAND_UP:
                    if(1>0)
                    {                
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


                        body_p_des_old_ini = body_p_des;

                        
                        /////////////////=============== base status control ===================/////////////////////
                        // ////// body pose feedback
                        base_pos_fb_controler();
                        if (stand_up_count>1)
                        {
                            comv_des[0] = (body_p_des[0] - com_des_pre[0]) /dtx;
                            comv_des[1] = (body_p_des[1] - com_des_pre[1]) /dtx;
                            comv_des[2] = (body_p_des[2] - com_des_pre[2]) /dtx;

                            thetav_des[0] = (body_r_des[0] - theta_des_pre[0]) /dtx;
                            thetav_des[1] = (body_r_des[1] - theta_des_pre[1]) /dtx;
                            thetav_des[2] = (body_r_des[2] - theta_des_pre[2]) /dtx;                           
                        }
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

                        Dynam.force_opt(body_p_des,FR_foot_des, FL_foot_des, RR_foot_des, RL_foot_des,
                                        F_sum, gait_mode, right_support, y_offset,foot_contact_flag);


                        FR_GRF_opt = (1 * (Dynam.grf_opt.block<3,1>(0,0) - FR_GRF)) + FR_GRF;
                        FL_GRF_opt = (1 * (Dynam.grf_opt.block<3,1>(3,0) - FL_GRF)) + FL_GRF;
                        RR_GRF_opt = (1 * (Dynam.grf_opt.block<3,1>(6,0) - RR_GRF)) + RR_GRF;
                        RL_GRF_opt = (1 * (Dynam.grf_opt.block<3,1>(9,0) - RL_GRF)) + RL_GRF;                                                               

                        Torque_ff_GRF.block<3,1>(0,0) = - FR_Jaco_est.transpose() *  root_rot_mat.transpose() * FR_GRF_opt;
                        Torque_ff_GRF.block<3,1>(3,0) = - FL_Jaco_est.transpose() *  root_rot_mat.transpose() * FL_GRF_opt;
                        Torque_ff_GRF.block<3,1>(6,0) = - RR_Jaco_est.transpose() *  root_rot_mat.transpose() * RR_GRF_opt;
                        Torque_ff_GRF.block<3,1>(9,0) = - RL_Jaco_est.transpose() *  root_rot_mat.transpose() * RL_GRF_opt;                           

                        
                        rate_stand_up = pow(stand_up_count/500.0,2); 
                        for(int j=0; j<12;j++)
                        {
                            Torque_ff_GRF[j] = jointLinearInterpolation(0, Torque_ff_GRF(j,0), rate_stand_up, 0);
                        }
                        FR_swing = false; 
                        FL_swing = false;
                        RR_swing = false;
                        RL_swing = false; 

                        Torque_ff_GRF_standing = Torque_ff_GRF;
                        
                        //// only for test
                        Legs_torque = Torque_ff_GRF;
                        

                        break;
                    }                        
                case DYNAMIC:
                    if(1>0)
                    {                
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
                            cout<<"body_p_Homing_dynamic:\n"<<body_p_Homing_dynamic<<endl;
                            cout<<"body_r_Homing_dynamic:\n"<<body_r_Homing_dynamic<<endl;
                        
                        }   
                        {
                            ////// test--- step in place /////
                            if(test_stepping_in_place>0.5)
                            {
                                // ////////===========dyanmic walking COM movement test/////////////////////////
                                body_p_des[0] = body_p_Homing_dynamic[0] + 0.00 * sin(6*M_PI*dynamic_count/n_rt);
                                body_p_des[1] = body_p_Homing_dynamic[1] + 0.00 * sin(6*M_PI*dynamic_count/n_rt);
                                body_p_des[2] = body_p_Homing_dynamic[2] + 0.00 *(sin(3*M_PI*dynamic_count/n_rt));
                                body_r_des[0] = body_r_Homing_dynamic[0] + 0.00 * sin(6*M_PI*dynamic_count/n_rt);
                                body_r_des[1] = body_r_Homing_dynamic[1] + 0.00 * sin(6*M_PI*dynamic_count/n_rt);
                                body_r_des[2] = body_r_Homing_dynamic[2] + 0.00 * sin(6*M_PI*dynamic_count/n_rt);                  
                                comv_des[0] = -0.00 * cos(6*M_PI*dynamic_count/n_rt) * (6*M_PI/n_rt);
                                comv_des[1] = -0.00 * cos(6*M_PI*dynamic_count/n_rt) * (6*M_PI/n_rt);
                                comv_des[2] = -0.00 * cos(3*M_PI*dynamic_count/n_rt) * (3*M_PI/n_rt);
                                thetav_des[0] = -0.00 * cos(6*M_PI*dynamic_count/n_rt) * (6*M_PI/n_rt);
                                thetav_des[1] = -0.00 * cos(6*M_PI*dynamic_count/n_rt) * (6*M_PI/n_rt);
                                thetav_des[2] = -0.00 * cos(6*M_PI*dynamic_count/n_rt) * (6*M_PI/n_rt);   

                                right_support = 2;  
                                /////////////////////// leg movement test//////////////////////
                                // if((swing_leg_test==0)||(swing_leg_test==2))
                                // {
                                //     // FR_foot_desx = abs(0.04 * sin(4*M_PI*dynamic_count/n_rt))-0.01;
                                //     //FR_foot_desy = (0.06 * sin(4*M_PI*dynamic_count/n_rt));
                                //     //FR_foot_desz = (clear_height * sin(4*M_PI*dynamic_count/n_rt))-0.02;
                                // }
                                // if((swing_leg_test==1)||(swing_leg_test==2))
                                // {
                                //     // FL_foot_desx = abs(0.04 * sin(4*M_PI*dynamic_count/n_rt + M_PI))-0.01;
                                //     // FL_foot_desy = (0.06 * sin(4*M_PI*dynamic_count/n_rt + M_PI));
                                //     //FL_foot_desz = (clear_height * sin(4*M_PI*dynamic_count/n_rt + M_PI))-0.02; 
                                // }                                           
                                // dynamic_count_period = floor(dynamic_count * 2/n_rt ); 
                                // FR_foot_desx *= pow((-1), dynamic_count_period % 2);
                                // FR_foot_desy *= pow((-1), dynamic_count_period % 2);
                                

                                FR_foot_desy = (0.06 * sin(4*M_PI*dynamic_count/n_rt));
                                FL_foot_desy = -(0.06 * sin(4*M_PI*dynamic_count/n_rt));
                                FR_foot_des[0] = FR_foot_Homing[0] + FR_foot_desx ;
                                FR_foot_des[1] = FR_foot_Homing[1] + FR_foot_desy ;
                                FR_foot_des[2] = FR_foot_Homing[2] + FR_foot_desz ;

                                RL_foot_des[0] = RL_foot_Homing[0] + FL_foot_desx ;
                                RL_foot_des[1] = RL_foot_Homing[1] + FL_foot_desy ;
                                RL_foot_des[2] = RL_foot_Homing[2] + FL_foot_desz ;
                                FL_foot_des[0] = FL_foot_Homing[0] + FL_foot_desx ;
                                FL_foot_des[1] = FL_foot_Homing[1] + FL_foot_desy ;
                                FL_foot_des[2] = FL_foot_Homing[2] + FL_foot_desz ;
                                RR_foot_des[0] = RR_foot_Homing[0] + FR_foot_desx;
                                RR_foot_des[1] = RR_foot_Homing[1] + FR_foot_desy ;
                                RR_foot_des[2] = RR_foot_Homing[2] + FR_foot_desz ;
                            }
                            else  /////// gait planned by the real time loop
                            {

                                // // ================non -real-time intepoloation: data filter: from hierachical convex optimization:                    
                                bjx1 = slow_mpc_gait(27);
                                
                                init_count = slow_mpc_gait(13); /// mpc reinitialization test
                                

                                right_support = slow_mpc_gait(99);
                                /////// high-level MPC
                                com_des[0] = butterworthLPF1.filter(slow_mpc_gait(0,0));
                                com_des[1] = butterworthLPF2.filter(slow_mpc_gait(1,0));
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

                                FR_foot_desx = butterworthLPF101.filter(slow_mpc_gait(58,0));
                                FR_foot_desy = butterworthLPF102.filter(slow_mpc_gait(59,0)); 
                                FR_foot_desz = butterworthLPF103.filter(slow_mpc_gait(60,0));
                                FL_foot_desx = butterworthLPF104.filter(slow_mpc_gait(61,0)); 
                                FL_foot_desy = butterworthLPF105.filter(slow_mpc_gait(62,0)); 
                                FL_foot_desz = butterworthLPF106.filter(slow_mpc_gait(63,0)); 
                                RR_foot_desx = butterworthLPF107.filter(slow_mpc_gait(64,0)); 
                                RR_foot_desy = butterworthLPF108.filter(slow_mpc_gait(65,0));
                                RR_foot_desz = butterworthLPF109.filter(slow_mpc_gait(66,0)); 
                                RL_foot_desx = butterworthLPF110.filter(slow_mpc_gait(67,0)); 
                                RL_foot_desy = butterworthLPF111.filter(slow_mpc_gait(68,0)); 
                                RL_foot_desz = butterworthLPF112.filter(slow_mpc_gait(69,0));   
                                // coma_des[0] = butterworthLPF13.filter(slow_mpc_gait(39));
                                // coma_des[1] = butterworthLPF14.filter(slow_mpc_gait(40));
                                // coma_des[2] = butterworthLPF15.filter(slow_mpc_gait(41));		
                                // theta_acc_des[0] = butterworthLPF16.filter(slow_mpc_gait(73));
                                // theta_acc_des[1] = butterworthLPF17.filter(slow_mpc_gait(74));
                                // theta_acc_des[2] = butterworthLPF18.filter(slow_mpc_gait(75));
                                if (dynamic_count==1)
                                {  
                                    com_des_ini[0] = slow_mpc_gait(0,0);
                                    com_des_ini[1] = slow_mpc_gait(1,0);
                                    com_des_ini[2] = slow_mpc_gait(2,0);
                                    rfoot_des_ini[0] = (slow_mpc_gait(9));
                                    rfoot_des_ini[1] = (slow_mpc_gait(10));
                                    rfoot_des_ini[2] = (slow_mpc_gait(11));
                                    lfoot_des_ini[0] = (slow_mpc_gait(6));
                                    lfoot_des_ini[1] = (slow_mpc_gait(7));
                                    lfoot_des_ini[2] = (slow_mpc_gait(8));
                                }  

                                /// MPC force: now should be replaced by 
                                Grf_sub_filter[0] = butterworthLPF41.filter(fast_mpc_gait(0+15));
                                Grf_sub_filter[1] = butterworthLPF42.filter(fast_mpc_gait(1+15));
                                Grf_sub_filter[3-1] = butterworthLPF43.filter(fast_mpc_gait(2+15));
                                Grf_sub_filter[4-1] = butterworthLPF44.filter(fast_mpc_gait(3+15));
                                Grf_sub_filter[5-1] = butterworthLPF45.filter(fast_mpc_gait(4+15));
                                Grf_sub_filter[6-1] = butterworthLPF46.filter(fast_mpc_gait(5+15));
                                Grf_sub_filter[7-1] = butterworthLPF47.filter(fast_mpc_gait(6+15));
                                Grf_sub_filter[8-1] = butterworthLPF48.filter(fast_mpc_gait(7+15));
                                Grf_sub_filter[9-1] = butterworthLPF49.filter(fast_mpc_gait(8+15));
                                Grf_sub_filter[10-1] = butterworthLPF50.filter(fast_mpc_gait(9+15));
                                Grf_sub_filter[11-1] = butterworthLPF51.filter(fast_mpc_gait(10+15));
                                Grf_sub_filter[12-1] = butterworthLPF52.filter(fast_mpc_gait(11+15));   

                                FR_GRF = Grf_sub_filter.block<3,1>(0,0); 
                                FL_GRF = Grf_sub_filter.block<3,1>(3,0); 
                                RR_GRF = Grf_sub_filter.block<3,1>(6,0); 
                                RL_GRF = Grf_sub_filter.block<3,1>(9,0);                           
                                
                                
                                if (dynamic_count * dtx >= initial_number*tstep)
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
                                            body_p_des[0] = com_des[0] - com_des_ini[0] + body_p_Homing_Retarget(0,0);
                                            body_p_des[1] = com_des[1] - com_des_ini[1] + body_p_Homing_Retarget(1,0);
                                            body_p_des[2] = com_des[2];

                                            body_r_des[0] = theta_des[0] - theta_des_ini[0]  + body_r_Homing_Retarget(0,0);
                                            body_r_des[1] = theta_des[1] - theta_des_ini[1]  + body_r_Homing_Retarget(1,0);
                                            body_r_des[2] = theta_des[2] - theta_des_ini[2]  + body_r_Homing_Retarget(2,0);
                

                                            //// directly generation from the gait planner
                                            FR_foot_des[0] = FR_foot_desx + lfoot_des[0] - lfoot_des_ini[0];
                                            FR_foot_des[1] = FR_foot_desy + lfoot_des[1] - lfoot_des_ini[1];
                                            FR_foot_des[2] = FR_foot_desz + lfoot_des[2] - lfoot_des_ini[2];
                                            RL_foot_des[0] = RL_foot_desx + lfoot_des[0] - lfoot_des_ini[0];
                                            RL_foot_des[1] = RL_foot_desy + lfoot_des[1] - lfoot_des_ini[1];
                                            RL_foot_des[2] = RL_foot_desz + lfoot_des[2] - lfoot_des_ini[2];
                                            FL_foot_des[0] = FL_foot_desx + rfoot_des[0] - rfoot_des_ini[0];
                                            FL_foot_des[1] = FL_foot_desy + rfoot_des[1] - rfoot_des_ini[1];
                                            FL_foot_des[2] = FL_foot_desz + rfoot_des[2] - rfoot_des_ini[2];
                                            RR_foot_des[0] = RR_foot_desx + rfoot_des[0] - rfoot_des_ini[0];
                                            RR_foot_des[1] = RR_foot_desy + rfoot_des[1] - rfoot_des_ini[1];
                                            RR_foot_des[2] = RR_foot_desz + rfoot_des[2] - rfoot_des_ini[2]; 
                                            // if(count_in_rt_loop * dtx == 1.5*tstep)
                                            // {
                                            //     cout<<"FR_foot_des:"<<FR_foot_des<<endl;
                                            //     cout<<"FL_foot_des:"<<FL_foot_des<<endl;
                                            //     cout<<"RR_foot_des:"<<RR_foot_des<<endl;
                                            //     cout<<"RL_foot_des:"<<RL_foot_des<<endl;
                                            // } 
                                            

                                            // //// FR, RL two legs move synchronous
                                            // FR_foot_des[0] = FR_foot_Homing[0] + lfoot_des[0];
                                            // FR_foot_des[1] = FR_foot_Homing[1] + lfoot_des[1];
                                            // FR_foot_des[2] = FR_foot_Homing[2] + lfoot_des[2];
                                            // RL_foot_des[0] = RL_foot_Homing[0] + lfoot_des[0];
                                            // RL_foot_des[1] = RL_foot_Homing[1] + lfoot_des[1];
                                            // RL_foot_des[2] = RL_foot_Homing[2] + lfoot_des[2];
                                            // //// FL, RR two legs move synchronous
                                            // FL_foot_des[0] = FL_foot_Homing[0] + rfoot_des[0];
                                            // FL_foot_des[1] = FL_foot_Homing[1] + rfoot_des[1];
                                            // FL_foot_des[2] = FL_foot_Homing[2] + rfoot_des[2];
                                            // RR_foot_des[0] = RR_foot_Homing[0] + rfoot_des[0];
                                            // RR_foot_des[1] = RR_foot_Homing[1] + rfoot_des[1];
                                            // RR_foot_des[2] = RR_foot_Homing[2] + rfoot_des[2];                                                    
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
                                    body_p_des[0] = body_p_Homing_Retarget[0];
                                    body_p_des[1] = body_p_Homing_Retarget[1];
                                    body_p_des[2] = body_p_Homing_Retarget[2];  
                                    body_r_des[0] = body_r_Homing_Retarget[0];
                                    body_r_des[1] = body_r_Homing_Retarget[1];
                                    body_r_des[2] = body_r_Homing_Retarget[2]; 

                                }
                            }


                            ////=========  ///////////////////////////
                            ////// body pose feedback!!! set swing_flag according to the reference
                            base_pos_fb_controler();          
                            comv_des[0] = (body_p_des[0] - com_des_pre[0]) /dtx;
                            comv_des[1] = (body_p_des[1] - com_des_pre[1]) /dtx;
                            comv_des[2] = (body_p_des[2] - com_des_pre[2]) /dtx;
                            thetav_des[0] = (body_r_des[0] - theta_des_pre[0]) /dtx;
                            thetav_des[1] = (body_r_des[1] - theta_des_pre[1]) /dtx;
                            thetav_des[2] = (body_r_des[2] - theta_des_pre[2]) /dtx;                           

                            foot_earlier_contact_flag.setZero();  
                            //////////////judge if earlier contact
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
                            foot_contact_position.block<3, 1>(0, 0)
                                    << recent_contact_x_filter[0].CalculateAverage(FR_foot_mea(0, 0)),
                                    recent_contact_y_filter[0].CalculateAverage(FR_foot_mea(1, 0)),
                                    recent_contact_z_filter[0].CalculateAverage(FR_foot_mea(2, 0));

                            foot_contact_position.block<3, 1>(0, 1)
                                    << recent_contact_x_filter[1].CalculateAverage(FL_foot_mea(0, 0)),
                                    recent_contact_y_filter[1].CalculateAverage(FL_foot_mea(1, 0)),
                                    recent_contact_z_filter[1].CalculateAverage(FL_foot_mea(2, 0));

                            foot_contact_position.block<3, 1>(0, 2)
                                    << recent_contact_x_filter[2].CalculateAverage(RR_foot_mea(0, 0)),
                                    recent_contact_y_filter[2].CalculateAverage(RR_foot_mea(1, 0)),
                                    recent_contact_z_filter[2].CalculateAverage(RR_foot_mea(2, 0));

                            foot_contact_position.block<3, 1>(0, 3)
                                    << recent_contact_x_filter[3].CalculateAverage(RL_foot_mea(0, 0)),
                                    recent_contact_y_filter[3].CalculateAverage(RL_foot_mea(1, 0)),
                                    recent_contact_z_filter[3].CalculateAverage(RL_foot_mea(2, 0));

                            //ground_angle = state_est_kine.compute_ground_inclination(body_r_des, body_r_est, body_p_est,foot_contact_position,right_support); 
                            if (use_terrain_adapt) {
                                body_r_des(0,0) += ground_angle(0,0);
                                body_r_des(1,0) += ground_angle(1,0);
                            }

                            foot_earlier_contact_flag_old = foot_earlier_contact_flag;

                            // ///////////////===========================================///////////////////////////
                            // ///======= adjust the com acceleration in real-time!!!!!!!!!!!!!!!!!!!
                            base_acc_ff_controler(); 
                            // //////=================== QP-based force distribution ================//////////////////
                            F_sum(0) = gait::mass * coma_des[0];
                            F_sum(1) = gait::mass * coma_des[1];
                            F_sum(2) = gait::mass * (gait::_g + coma_des[2]);
                            ///////==== momentum check !!!!!!!!!!!!!!!!!!!!!!!! =================
                            F_sum(3,0) = Momentum_global(0,0) * theta_acc_des[0] + Momentum_global(0,1) * theta_acc_des[1] + Momentum_global(0,2) * theta_acc_des[2];
                            F_sum(4,0) = Momentum_global(1,0) * theta_acc_des[0] + Momentum_global(1,1) * theta_acc_des[1] + Momentum_global(1,2) * theta_acc_des[2];
                            F_sum(5,0) = Momentum_global(2,0) * theta_acc_des[0] + Momentum_global(2,1) * theta_acc_des[1] + Momentum_global(2,2) * theta_acc_des[2];        


                            Dynam.force_opt(body_p_des,FR_foot_des, FL_foot_des, RR_foot_des, RL_foot_des,
                                            F_sum, gait_mode, right_support, y_offset,foot_contact_flag);


                            root_rot_mat_ref =  Eigen::AngleAxisd(body_r_des(2,0), Eigen::Vector3d::UnitZ())
                                            *Eigen::AngleAxisd(body_r_des(1,0), Eigen::Vector3d::UnitY())
                                            *Eigen::AngleAxisd(body_r_des(0,0), Eigen::Vector3d::UnitX());
                            double alpha = 0.2;
                            double alpha_inter = 0;
                            alpha_inter = std::max(std::min(((dynamic_count * dtx - initial_number*tstep) / (tstep)),1.0),0.0);
                            alpha *= alpha_inter;
                            if(using_real_rotz>0.5)
                            {
                                FR_GRF_opt = (alpha * root_rot_mat.transpose() *Dynam.grf_opt.block<3,1>(0,0)) + (1-alpha) * root_rot_mat.transpose()*FR_GRF;
                                FL_GRF_opt = (alpha * root_rot_mat.transpose() *Dynam.grf_opt.block<3,1>(3,0)) + (1-alpha) * root_rot_mat.transpose()*FL_GRF;
                                RR_GRF_opt = (alpha * root_rot_mat.transpose() *Dynam.grf_opt.block<3,1>(6,0)) + (1-alpha) * root_rot_mat.transpose()*RR_GRF;
                                RL_GRF_opt = (alpha * root_rot_mat.transpose() *Dynam.grf_opt.block<3,1>(9,0)) + (1-alpha) * root_rot_mat.transpose()*RL_GRF; 
                            }
                            else
                            {
                                FR_GRF_opt = (alpha * root_rot_mat_ref.transpose() *Dynam.grf_opt.block<3,1>(0,0)) + (1-alpha) * root_rot_mat_ref.transpose()*FR_GRF;
                                FL_GRF_opt = (alpha * root_rot_mat_ref.transpose() *Dynam.grf_opt.block<3,1>(3,0)) + (1-alpha) * root_rot_mat_ref.transpose()*FL_GRF;
                                RR_GRF_opt = (alpha * root_rot_mat_ref.transpose() *Dynam.grf_opt.block<3,1>(6,0)) + (1-alpha) * root_rot_mat_ref.transpose()*RR_GRF;
                                RL_GRF_opt = (alpha * root_rot_mat_ref.transpose() *Dynam.grf_opt.block<3,1>(9,0)) + (1-alpha) * root_rot_mat_ref.transpose()*RL_GRF;                                 
                            }

                        }        
                        
                        ////// smoothing the joint trajectory when starting moving
                        leg_kinematic(); 

                        if(test_stepping_in_place>0.5) 
                        {
                            ratex = std::min(pow(dynamic_count/1000.0,2),1.0);
                            for(int j=0; j<12;j++)
                            {
                                qDes[j] = jointLinearInterpolation(sin_mid_q[j], qDes[j], ratex, 0);
                            }
                        }
                        else
                        {
                            if(dynamic_count * dtx >= initial_number*tstep)
                            {
                                ratex = std::min(pow((dynamic_count * dtx- initial_number*tstep)/(2*tstep),2),1.0);
                                for(int j=0; j<12;j++)
                                {
                                    qDes[j] = jointLinearInterpolation(sin_mid_q[j], qDes[j], ratex, 0);
                                }
                            }
                        }

                        if(using_real_jaco>0.5)
                        {
                            Torque_ff_GRF.block<3,1>(0,0) = - FR_Jaco_est.transpose() *  FR_GRF_opt;
                            Torque_ff_GRF.block<3,1>(3,0) = - FL_Jaco_est.transpose() *  FL_GRF_opt;
                            Torque_ff_GRF.block<3,1>(6,0) = - RR_Jaco_est.transpose() *  RR_GRF_opt;
                            Torque_ff_GRF.block<3,1>(9,0) = - RL_Jaco_est.transpose() *  RL_GRF_opt;                             
                        }
                        else
                        {
                            Torque_ff_GRF.block<3,1>(0,0) = - FR_Jaco.transpose() *  FR_GRF_opt;
                            Torque_ff_GRF.block<3,1>(3,0) = - FL_Jaco.transpose() *  FL_GRF_opt;
                            Torque_ff_GRF.block<3,1>(6,0) = - RR_Jaco.transpose() *  RR_GRF_opt;
                            Torque_ff_GRF.block<3,1>(9,0) = - RL_Jaco.transpose() *  RL_GRF_opt; 
                        }

                        for(int j=0; j<12;j++)
                        {
                            if(j % 3 == 0)
                            {
                                Torque_ff_GRF(j,0) = std::min(std::max(Torque_ff_GRF(j,0),-15.0),15.0);
                            }
                            if(j % 3 == 1)
                            {
                                Torque_ff_GRF(j,0) = std::min(std::max(Torque_ff_GRF(j,0),-23.0),23.0);
                            }
                            if(j % 3 == 2)
                            {
                                Torque_ff_GRF(j,0) = std::min(std::max(Torque_ff_GRF(j,0),-30.0),30.0);
                            }
                        }
                        
                        rate_stand_up = std::min(pow(dynamic_count *dtx /(initial_number*tstep),2),1.0); 
                        for(int j=0; j<12;j++)
                        {
                            if(rate_stand_up<1.0)
                            {   
                                cout<<rate_stand_up<<endl;
                                cout<<Torque_ff_GRF_standing[0]<<endl;

                                Torque_ff_GRF_error = Torque_ff_GRF - Torque_ff_GRF_standing;
                                cout<<Torque_ff_GRF_error[0]<<endl;
                                Torque_ff_GRF_error_inter[j] =  jointLinearInterpolation(0, Torque_ff_GRF_error(j,0), rate_stand_up, 0);
                                Torque_ff_GRF[j] = Torque_ff_GRF_standing[j] + Torque_ff_GRF_error_inter[j];
                            }
                            // else
                            // {
                            //     Torque_ff_GRF[j] = jointLinearInterpolation(0, Torque_ff_GRF(j,0), rate_stand_up, 0);
                            // }
                            
                        }




                        break;
                    }
                default:
                    FR_swing = false; 
                    FL_swing = false;
                    RR_swing = false;
                    RL_swing = false;
                    break;
            }

            //////////////////// compute the feedforward torque: folowing the idea form convexMPC2018MIT ////////////////
            if(1.0>0)
            {
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
                
                // ////// clamp the joint angular velocity////
                dqdes_raw.setZero();
                for(int j=0; j<12;j++)
                {
                    
                    if(motiontime>=1)
                    {
                        dqdes_raw(j,0) = (qDes[j] - qDes_pre[j]) /dtx;
                    }                    
                }
                dqDes[0] = butterworthLPF53.filter(dqdes_raw(0,0));
                dqDes[1] = butterworthLPF54.filter(dqdes_raw(1,0));
                dqDes[2] = butterworthLPF55.filter(dqdes_raw(2,0));
                dqDes[3] = butterworthLPF56.filter(dqdes_raw(3,0));
                dqDes[4] = butterworthLPF57.filter(dqdes_raw(4,0));
                dqDes[5] = butterworthLPF58.filter(dqdes_raw(5,0));
                dqDes[6] = butterworthLPF59.filter(dqdes_raw(6,0));
                dqDes[7] = butterworthLPF60.filter(dqdes_raw(7,0));
                dqDes[8] = butterworthLPF61.filter(dqdes_raw(8,0));
                dqDes[9] = butterworthLPF62.filter(dqdes_raw(9,0));
                dqDes[10] = butterworthLPF63.filter(dqdes_raw(10,0));
                dqDes[11] = butterworthLPF64.filter(dqdes_raw(11,0));

                if(gait_status == STAND_INIT_STATUS)
                {
                    for(int j=0; j<12;j++)
                    {
                        dqDes[j] = 0.0;      
                                    
                    }
                    // cout<<"xxx1"<<dqDes[0]<<endl;  
                }
                else
                {
                    for(int j=0; j<12;j++)
                    {
                        dqDes[j] = std::min(std::max(dqDes[j],-6.0),6.0);                    
                    }
                }
                // cout<<"xxx"<<dqDes[1]<<endl; 


                
                
                //====== PID joint tracking feedback + spring softplus feedfoward (when enable spring)====////////
                for(int j=0; j<12;j++)
                {
                    if(j % 3 == 0)
                    { 
                        //// hip joint tracking
                        qDes[j] = std::min(std::max(qDes[j],go1_Hip_min),go1_Hip_max);
                        torq_kp_hip = 8 * hip_kp_scale;
                        torq_kd_hip = 0.3 * hip_kd_scale;
                        k_p_rest_hip = sin_mid_q[j];   
                        k_spring_hip = FR_k_spring_hip;                                               
                        
                        torque_err.block<torque_err_row-1,1>(0,j) = torque_err.block<torque_err_row-1,1>(1,j);
                        torque_err(torque_err_row-1,j) = qDes[j] - RecvLowROS.motorState[j].q;
                        torque_err_intergration.setZero();
                        for(int ij=0; ij<torque_err_row; ij++)
                        {
                            torque_err_intergration(j,0) += torque_err(ij,j);
                        }                 
                        //torque(j,0) = (qDes[j] - RecvLowROS.motorState[j].q)*torq_kp_hip + (0 - RecvLowROS.motorState[j].dq)*torq_kd_hip + torque_err_intergration(j,0)*torq_ki_hip;                      
                        // // ff control
                        ///// tuning the parameters carefully
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
                                    //Torque_ff_spring(j,0) = 0;
                                }                                                    
                            }
                        }     

                        torque(j,0) += Torque_ff_spring(j,0);                     

                    }
                    else
                    {
                        
                        if(j % 3 ==1)
                        {
                            //// thigh joint tracking 
                            qDes[j] = std::min(std::max(qDes[j],go1_Thigh_min),go1_Thigh_max);
                            //// support leg
                            torq_kp_thigh = 7 * thigh_kp_scale;
                            torq_kd_thigh = 0.3 * thigh_kd_scale;
                            torq_ki_thigh = 0.01*1; 
                            if(j>6)/// front legs
                            {
                               k_spring_thigh = FR_k_spring_thigh;
                            }
                            else /// real legs
                            {
                               k_spring_thigh = RR_k_spring_thigh;
                            }
                            
                            k_p_rest_thigh = sin_mid_q[j];  
                            
                            //// fb control
                            torque_err.block<torque_err_row-1,1>(0,j) = torque_err.block<torque_err_row-1,1>(1,j);
                            torque_err(torque_err_row-1,j) = qDes[j] - RecvLowROS.motorState[j].q;
                            torque_err_intergration.setZero();
                            for(int ij=0; ij<torque_err_row; ij++)
                            {
                            torque_err_intergration(j,0) += torque_err(ij,j);
                            }                             
                            // torque(j,0) = (qDes[j] - RecvLowROS.motorState[j].q)*torq_kp_thigh + (0 - RecvLowROS.motorState[j].dq)*torq_kd_thigh + torque_err_intergration(j,0)*torq_ki_thigh;                           
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
                            qDes[j] = std::min(std::max(qDes[j],go1_Calf_min),go1_Calf_max);
                            torq_kp_calf = 9 * calf_kp_scale;
                            torq_kd_calf = 0.31 * calf_kd_scale;
                            torq_ki_calf = 0.01*1;  
                            k_spring_calf = FR_k_spring_calf;                              
                            // k_p_rest_calf = -1.3;
                            //k_p_rest_calf = -1.38;    
                            k_p_rest_calf = sin_mid_q[j];  
                            //// calf joint tracking
                            torque_err.block<torque_err_row-1,1>(0,j) = torque_err.block<torque_err_row-1,1>(1,j);
                            torque_err(torque_err_row-1,j) = qDes[j] - RecvLowROS.motorState[j].q;
                            torque_err_intergration.setZero();
                            for(int ij=0; ij<torque_err_row; ij++)
                            {
                                torque_err_intergration(j,0) += torque_err(ij,j);
                            }                             
                            // torque(j,0) = (qDes[j] - RecvLowROS.motorState[j].q)*torq_kp_calf + (0 - RecvLowROS.motorState[j].dq)*torq_kd_calf + torque_err_intergration(j,0)*torq_ki_calf;                           
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
    
                if(test_stepping_in_place<0.5)
                {
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
                            if(using_real_jaco>0.5)
                            {
                            FR_torque_impedance = (FR_Jaco_est.transpose() * (swing_kp*(FR_foot_relative_des - FR_foot_relative_mea) 
                                                                                + swing_kd*(FR_v_relative - FR_v_est_relative)));
                            }
                            else
                            {
                            FR_torque_impedance = (FR_Jaco.transpose() * (swing_kp*(FR_foot_relative_des - FR_foot_relative_mea) 
                                                                                + swing_kd*(FR_v_relative - FR_v_est_relative)));                                
                            }
                        }
                        if(!FL_swing)
                        {
                            FL_torque_impedance.setZero();  
                        }
                        else
                        {      
                            if(using_real_jaco>0.5)
                            {
                            FL_torque_impedance = (FL_Jaco_est.transpose() * (swing_kp*(FL_foot_relative_des - FL_foot_relative_mea) 
                                                                                + swing_kd*(FL_v_relative - FL_v_est_relative)));
                            }
                            else
                            {
                            FL_torque_impedance = (FL_Jaco.transpose() * (swing_kp*(FL_foot_relative_des - FL_foot_relative_mea) 
                                                                                + swing_kd*(FL_v_relative - FL_v_est_relative)));                                
                            }
                        }
                        if(!RR_swing)
                        {
                            RR_torque_impedance.setZero();  
                        }
                        else
                        { 
                            if(using_real_jaco>0.5)
                            {                            
                            RR_torque_impedance = (RR_Jaco_est.transpose() * (swing_kp*(RR_foot_relative_des - RR_foot_relative_mea) 
                                                                                + swing_kd*(RR_v_relative - RR_v_est_relative)));  
                            }
                            else
                            {
                            RR_torque_impedance = (RR_Jaco.transpose() * (swing_kp*(RR_foot_relative_des - RR_foot_relative_mea) 
                                                                                + swing_kd*(RR_v_relative - RR_v_est_relative)));                                  
                            }                  
                        }
                        if(!RL_swing)
                        {
                            RL_torque_impedance.setZero();  
                        }
                        else
                        { 
                            if(using_real_jaco>0.5)
                            {                              
                            RL_torque_impedance = (RL_Jaco_est.transpose() * (swing_kp*(RL_foot_relative_des - RL_foot_relative_mea) 
                                                                                + swing_kd*(RL_v_relative - RL_v_est_relative)));
                            }
                            else
                            {
                            RL_torque_impedance = (RL_Jaco.transpose() * (swing_kp*(RL_foot_relative_des - RL_foot_relative_mea) 
                                                                                + swing_kd*(RL_v_relative - RL_v_est_relative)));                                
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
                
                
                }

                // ////// Gravity+coriolis force////////
                // torque.block<3,1>(0,0) +=  (torque_bias.block<3,1>(3,0));
                // torque.block<3,1>(3,0) +=  (torque_bias.block<3,1>(0,0));
                // torque.block<3,1>(6,0) +=  (torque_bias.block<3,1>(9,0));
                // torque.block<3,1>(9,0) +=  (torque_bias.block<3,1>(6,0));


                torque(0,0) +=  -0.9; ///FR
                torque(3,0) +=  0.9; /// FL
                torque(6,0) +=  -0.9; // RR
                torque(9,0) +=  0.9; //RL

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
                        torque_ff(j,0) = std::min(std::max(torque_ff(j,0),-15.0),15.0);
                    }
                    if(j % 3 == 1)
                    {
                        torque_ff(j,0) = std::min(std::max(torque_ff(j,0),-23.0),23.0);
                    }
                    if(j % 3 == 2)
                    {
                        torque_ff(j,0) = std::min(std::max(torque_ff(j,0),-30.0),30.0);
                    }
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
                
                if (gait_status == DYNAMIC_STATUS)
                {
                    energy_cost += (abs(RecvLowROS.motorState[j].dq * RecvLowROS.motorState[j].tauEst) * dtx); 
                    
                }
                
                           
            }
            state_gait_ekf(60,0) = energy_cost;
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
    ////// clamp the joint ////
    for(int j=0; j<12;j++)
    {
        qDes_pre[j] = qDes[j];
    }    
    

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

    /// desired com velocity /////
    for(int j=0; j<3; j++)
    {
        joint2simulation.position[96+j] = comv_des[j];
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
        ///std::cout<<RecvLowROS.footForce[j]<<endl;
    }

        for(int j=0; j<3; j++)
    {
        joint2simulationx.position[52+j] = Go1_ctrl_states.imu_acc[j];
    }

    for(int j=0; j<3; j++)
    {
        joint2simulationx.position[55+j] = ground_angle[j];
    }  
    

    joint2simulationx.position[58] = RecvLowROS.Car_position.x;
    joint2simulationx.position[59] = RecvLowROS.Car_position.y;
    joint2simulationx.position[60] = RecvLowROS.Car_position.z;
    joint2simulationx.position[61] = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
         
    for(int j=0; j<12; j++)
    {
        joint2simulationx.position[62+j] = RecvLowROS.motorState[j].dq;
    }                  

    // robot data publisher
    for(int j=0;j<100;j++)
    {
        leg2sim.position[j] = state_gait_ekf[j];
    }
    leg2sim.position[99] = 1000*dt_rt_loop.toSec();
    leg2sim.position[98] = ms_double.count();


    // /// publisher to wbc::

    date2wbc.header.stamp = ros::Time::now();

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
    for(int j=0; j<12; j++)
    {
        date2wbc.position[48+j] = dqDes[j];   // dq_des;
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
    double dt = 0.001; // Change the operational period (in sec).
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