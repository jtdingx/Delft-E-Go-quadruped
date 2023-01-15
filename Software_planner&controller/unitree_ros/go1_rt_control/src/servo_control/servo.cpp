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
#include "servo.h"
#include <Eigen/Dense>
#include "kinematics/Kinematics.h"
#include "sensor_msgs/JointState.h"
#include "/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/go1_rt_control/src/Robotpara/robot_const_para_config.h"
#include <geometry_msgs/Twist.h>  
#include "yaml.h"




using namespace std;
using namespace unitree_model;

bool start_up = true;

#define PI 3.1415926

sensor_msgs::JointState joint2simulation;
sensor_msgs::JointState leg2sim;

Eigen::Matrix<double,5,1> footforce_fr, footforce_fl, footforce_rr,footforce_rl;

double base_offset_x = 0;
double base_offset_y = 0; 
double base_offset_z = 0;
double base_offset_roll = 0;
double base_offset_pitch =0;
double base_offset_yaw = 0;

///// gains variation
double  kpp0_det;
double  kdp0_det;
double  kpp1_det;
double  kdp1_det;          
double  kpp2_det;
double  kdp2_det;





class multiThread
{
public:
    multiThread(string rname){
        robot_name = rname;
        imu_sub = nm.subscribe("/trunk_imu", 1, &multiThread::imuCallback, this);
        footForce_sub[0] = nm.subscribe("/visual/FR_foot_contact/the_force", 1, &multiThread::FRfootCallback, this);
        footForce_sub[1] = nm.subscribe("/visual/FL_foot_contact/the_force", 1, &multiThread::FLfootCallback, this);
        footForce_sub[2] = nm.subscribe("/visual/RR_foot_contact/the_force", 1, &multiThread::RRfootCallback, this);
        footForce_sub[3] = nm.subscribe("/visual/RL_foot_contact/the_force", 1, &multiThread::RLfootCallback, this);
        servo_sub[0] = nm.subscribe("/" + robot_name + "_gazebo/FR_hip_controller/state", 1, &multiThread::FRhipCallback, this);
        servo_sub[1] = nm.subscribe("/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, &multiThread::FRthighCallback, this);
        servo_sub[2] = nm.subscribe("/" + robot_name + "_gazebo/FR_calf_controller/state", 1, &multiThread::FRcalfCallback, this);
        servo_sub[3] = nm.subscribe("/" + robot_name + "_gazebo/FL_hip_controller/state", 1, &multiThread::FLhipCallback, this);
        servo_sub[4] = nm.subscribe("/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, &multiThread::FLthighCallback, this);
        servo_sub[5] = nm.subscribe("/" + robot_name + "_gazebo/FL_calf_controller/state", 1, &multiThread::FLcalfCallback, this);
        servo_sub[6] = nm.subscribe("/" + robot_name + "_gazebo/RR_hip_controller/state", 1, &multiThread::RRhipCallback, this);
        servo_sub[7] = nm.subscribe("/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, &multiThread::RRthighCallback, this);
        servo_sub[8] = nm.subscribe("/" + robot_name + "_gazebo/RR_calf_controller/state", 1, &multiThread::RRcalfCallback, this);
        servo_sub[9] = nm.subscribe("/" + robot_name + "_gazebo/RL_hip_controller/state", 1, &multiThread::RLhipCallback, this);
        servo_sub[10] = nm.subscribe("/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, &multiThread::RLthighCallback, this);
        servo_sub[11] = nm.subscribe("/" + robot_name + "_gazebo/RL_calf_controller/state", 1, &multiThread::RLcalfCallback, this);
    
        //// topic with first-layer MPC
        nrt_mpc_gait_subscribe_ = nm.subscribe("/MPC/Gait", 10, &multiThread::nrt_gait_sub_operation, this);
        gait_des_sub_ = nm.subscribe("/rtMPC/traj", 10,&multiThread::rt_gait_sub_operation, this);

        robot_mode_sub_ = nm.subscribe("/Robot_mode", 10,&multiThread::robot_mode, this);
    }

    void imuCallback(const sensor_msgs::Imu & msg)
    { 
        lowState.imu.quaternion[0] = msg.orientation.w;
        lowState.imu.quaternion[1] = msg.orientation.x;
        lowState.imu.quaternion[2] = msg.orientation.y;
        lowState.imu.quaternion[3] = msg.orientation.z;

        lowState.imu.gyroscope[0] = msg.angular_velocity.x;
        lowState.imu.gyroscope[1] = msg.angular_velocity.y;
        lowState.imu.gyroscope[2] = msg.angular_velocity.z;
        
        lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
        lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
        lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
        
    }

    void FRhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[0].mode = msg.mode;
        lowState.motorState[0].q = msg.q;
        lowState.motorState[0].dq = msg.dq;
        lowState.motorState[0].tauEst = msg.tauEst;
        
    }

    void FRthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[1].mode = msg.mode;
        lowState.motorState[1].q = msg.q;
        lowState.motorState[1].dq = msg.dq;
        lowState.motorState[1].tauEst = msg.tauEst;

    }

    void FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[2].mode = msg.mode;
        lowState.motorState[2].q = msg.q;
        lowState.motorState[2].dq = msg.dq;
        lowState.motorState[2].tauEst = msg.tauEst;

    }

    void FLhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[3].mode = msg.mode;
        lowState.motorState[3].q = msg.q;
        lowState.motorState[3].dq = msg.dq;
        lowState.motorState[3].tauEst = msg.tauEst;

    }

    void FLthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[4].mode = msg.mode;
        lowState.motorState[4].q = msg.q;
        lowState.motorState[4].dq = msg.dq;
        lowState.motorState[4].tauEst = msg.tauEst;

    }

    void FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[5].mode = msg.mode;
        lowState.motorState[5].q = msg.q;
        lowState.motorState[5].dq = msg.dq;
        lowState.motorState[5].tauEst = msg.tauEst;

    }

    void RRhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[6].mode = msg.mode;
        lowState.motorState[6].q = msg.q;
        lowState.motorState[6].dq = msg.dq;
        lowState.motorState[6].tauEst = msg.tauEst;

    }

    void RRthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[7].mode = msg.mode;
        lowState.motorState[7].q = msg.q;
        lowState.motorState[7].dq = msg.dq;
        lowState.motorState[7].tauEst = msg.tauEst;

    }

    void RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[8].mode = msg.mode;
        lowState.motorState[8].q = msg.q;
        lowState.motorState[8].dq = msg.dq;
        lowState.motorState[8].tauEst = msg.tauEst;

    }

    void RLhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[9].mode = msg.mode;
        lowState.motorState[9].q = msg.q;
        lowState.motorState[9].dq = msg.dq;
        lowState.motorState[9].tauEst = msg.tauEst;
    }

    void RLthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[10].mode = msg.mode;
        lowState.motorState[10].q = msg.q;
        lowState.motorState[10].dq = msg.dq;
        lowState.motorState[10].tauEst = msg.tauEst;
    }

    void RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[11].mode = msg.mode;
        lowState.motorState[11].q = msg.q;
        lowState.motorState[11].dq = msg.dq;
        lowState.motorState[11].tauEst = msg.tauEst;
    }

    void FRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[0].x = msg.wrench.force.x;
        lowState.eeForce[0].y = msg.wrench.force.y;
        lowState.eeForce[0].z = msg.wrench.force.z;
        lowState.footForce[0] = msg.wrench.force.z;
    }

    void FLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[1].x = msg.wrench.force.x;
        lowState.eeForce[1].y = msg.wrench.force.y;
        lowState.eeForce[1].z = msg.wrench.force.z;
        lowState.footForce[1] = msg.wrench.force.z;
    }

    void RRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[2].x = msg.wrench.force.x;
        lowState.eeForce[2].y = msg.wrench.force.y;
        lowState.eeForce[2].z = msg.wrench.force.z;
        lowState.footForce[2] = msg.wrench.force.z;
    }

    void RLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[3].x = msg.wrench.force.x;
        lowState.eeForce[3].y = msg.wrench.force.y;
        lowState.eeForce[3].z = msg.wrench.force.z;
        lowState.footForce[3] = msg.wrench.force.z;
    }


    //// ====================================== real-time mpc control //////////
    void nrt_gait_sub_operation(const sensor_msgs::JointState::ConstPtr &msg)
    {
	for (int jx = 0; jx<100; jx++)
	{
	    slow_mpc_gait(jx) = msg->position[jx]; 
	}
	    // mpc_gait_flag = slow_mpc_gait(99);
    }
    
    
    void rt_gait_sub_operation(const sensor_msgs::JointState::ConstPtr &msg)
    {
	for (int jx = 0; jx<51; jx++)
	{
	    fast_mpc_gait(jx) = msg->position[36+jx]; 
	}
	// fast_mpc_gait_flag = msg->position[99];
	count_in_mpc_max = msg->position[98];
	
    }    
    
    void robot_mode(const geometry_msgs::Twist::ConstPtr &msg)
    {
      printf("linear x: %f\n",msg->linear.x);
	
    }      
  
  


private:
    ros::NodeHandle nm;
    ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub, nrt_mpc_gait_subscribe_, gait_des_sub_,robot_mode_sub_;


    string robot_name;
};





void gait_pose_callback()
{
    /// FL, FR, RL, RR;
    a1_ctrl_states.joint_pos[0] = lowState.motorState[3].q;
    a1_ctrl_states.joint_vel[0] = lowState.motorState[3].dq;
    a1_ctrl_states.joint_pos[1] = lowState.motorState[4].q;
    a1_ctrl_states.joint_vel[1] = lowState.motorState[4].dq;
    a1_ctrl_states.joint_pos[2] = lowState.motorState[5].q;
    a1_ctrl_states.joint_vel[2] = lowState.motorState[5].dq;

    a1_ctrl_states.joint_pos[3] = lowState.motorState[0].q;
    a1_ctrl_states.joint_vel[3] = lowState.motorState[0].dq;
    a1_ctrl_states.joint_pos[4] = lowState.motorState[1].q;
    a1_ctrl_states.joint_vel[4] = lowState.motorState[1].dq;
    a1_ctrl_states.joint_pos[5] = lowState.motorState[2].q;
    a1_ctrl_states.joint_vel[5] = lowState.motorState[2].dq;

    a1_ctrl_states.joint_pos[6] = lowState.motorState[9].q;
    a1_ctrl_states.joint_vel[6] = lowState.motorState[9].dq;   
    a1_ctrl_states.joint_pos[7] = lowState.motorState[10].q;
    a1_ctrl_states.joint_vel[7] = lowState.motorState[10].dq;
    a1_ctrl_states.joint_pos[8] = lowState.motorState[11].q;
    a1_ctrl_states.joint_vel[8] = lowState.motorState[11].dq;

    a1_ctrl_states.joint_pos[9] = lowState.motorState[6].q;
    a1_ctrl_states.joint_vel[9] = lowState.motorState[6].dq;
    a1_ctrl_states.joint_pos[10] = lowState.motorState[7].q;
    a1_ctrl_states.joint_vel[10] = lowState.motorState[7].dq;
    a1_ctrl_states.joint_pos[11] = lowState.motorState[8].q;
    a1_ctrl_states.joint_vel[11] = lowState.motorState[8].dq;

    // a1_ctrl_states.foot_force[0] = lowState.footForce[1]; 
    // a1_ctrl_states.foot_force[1] = lowState.footForce[0]; 
    // a1_ctrl_states.foot_force[2] = lowState.footForce[3]; 
    // a1_ctrl_states.foot_force[3] = lowState.footForce[2]; 
    a1_ctrl_states.foot_force[0] = (footforce_fl.mean()); 
    a1_ctrl_states.foot_force[1] = (footforce_fr.mean()); 
    a1_ctrl_states.foot_force[2] = (footforce_rl.mean()); 
    a1_ctrl_states.foot_force[3] = (footforce_rr.mean()); 

    a1_ctrl_states.root_quat = root_quat;


    a1_ctrl_states.imu_acc = Eigen::Vector3d(
            acc_x.CalculateAverage(lowState.imu.accelerometer[0]),
            acc_y.CalculateAverage(lowState.imu.accelerometer[1]),
            acc_z.CalculateAverage(lowState.imu.accelerometer[2])
    );

    // a1_ctrl_states.imu_acc = Eigen::Vector3d(
    //         (lowState.imu.accelerometer[0]),
    //         (lowState.imu.accelerometer[1]),
    //         (lowState.imu.accelerometer[2])
    // );    

    a1_ctrl_states.imu_ang_vel = Eigen::Vector3d(
            gyro_x.CalculateAverage(lowState.imu.gyroscope[0]),
            gyro_y.CalculateAverage(lowState.imu.gyroscope[1]),
            gyro_z.CalculateAverage(lowState.imu.gyroscope[2])
    );

    // calculate several useful variables
    // euler should be roll pitch yaw
    a1_ctrl_states.root_rot_mat = a1_ctrl_states.root_quat.toRotationMatrix();
    a1_ctrl_states.root_euler = Utils::quat_to_euler(a1_ctrl_states.root_quat);
    double yaw_angle = a1_ctrl_states.root_euler[2];

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


Eigen::Matrix<double, 100,1>  state_est_main_update(double dt) {

    Eigen::Matrix<double,100,1> state;
    state.setZero();
    // state estimation
    if (!go1_estimate.is_inited()) {
        go1_estimate.init_state(a1_ctrl_states);
    } else {
        go1_estimate.update_estimation(a1_ctrl_states, dt);
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



void keyboard_model_callback(const geometry_msgs::Twist::ConstPtr &msgIn) 
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
    std::cout<<"msgIn->linear.x:"<<msgIn->linear.x<<std::endl;  
}

void Base_offset_callback(const geometry_msgs::Twist::ConstPtr &msgIn) {
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

        kpp[0] +=  kpp0_det;
        kdp[0] +=  kdp0_det;

        kpp[1] +=  kpp1_det;
        kdp[1] +=  kdp1_det;          

        kpp[2] +=  kpp2_det;
        kdp[2] +=  kdp2_det;    
        std::cout<<"kpp[1]:"<<kpp[1]<<endl; 
        std::cout<<"kdp[1]:"<<kdp[1]<<endl;
    }
 

}


void base_pos_fb_controler()
{
    // ////// PD-type CoM acceleration control: for Grf compensation//////
    // coma_des[0] = kpp[0] * (body_p_des[0] - com_sensor[0]) 
    //               + kdp[0]*(comv_des[0] - comv_sensor[0]);

    // coma_des[1] = kpp[1] * (body_p_des[1] - com_sensor[1]) 
    //               + kdp[1]*(comv_des[1] - comv_sensor[1]);

    // coma_des[2] = kpp[2] * (body_p_des[2] - com_sensor[2]) 
    //               + kdp[2]*(comv_des[2] - comv_sensor[2]);  


    body_r_des[0] = kpw[0] * (body_r_des[0] - a1_ctrl_states.root_euler[0]) 
                     + kdw[0]*((body_r_des[0]-theta_des_pre[0])/dtx - a1_ctrl_states.imu_ang_vel[0]);

    body_r_des[1] = kpw[1] * (body_r_des[1] - a1_ctrl_states.root_euler[1]) 
                     + kdw[1]*((body_r_des[1]-theta_des_pre[1])/dtx - a1_ctrl_states.imu_ang_vel[1]);

    body_r_des[2] = kpw[2] * (body_r_des[2] - a1_ctrl_states.root_euler[2]) 
                     + kdw[2]*((body_r_des[2]-theta_des_pre[2])/dtx - a1_ctrl_states.imu_ang_vel[2]);                  


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


void config_set()
{   
    /////load default parameter from the yaml.file
    ///////////////////  yaml code . ///////// 
    // YAML::Node config = YAML::LoadFile("/home/pi/go1_catkin/src/unitree_ros_to_real/unitree_legged_real/config/config.yaml");
    YAML::Node config = YAML::LoadFile("/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/go1_rt_control/config/config.yaml");

    dt_slow_mpc =  config["dt_slow_mpc"].as<double>();
    z_c =  config["body_p_Homing_Retarget2"].as<double>();
    tstep =  config["t_period"].as<double>();
    dtx =  config["dt_rt_loop1"].as<double>();
    using_hie_using = config["using_hie_using"].as<double>();


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unitree_gazebo_servo");

    string robot_name;
    ros::param::get("/robot_name", robot_name);
    string control_mode;
    ros::param::get("/rctr", control_mode);
    cout << "robot_name: " << robot_name << endl;
    cout << "control_model: " << control_mode << endl;
    
    config_set();

    gait_mode = 102;
    x_offset = 0.01;

    if (gait_mode ==101) ///bipedal
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


    //desired joint angles
    FR_angle_des.setZero(); FL_angle_des.setZero(); RR_angle_des.setZero(); RL_angle_des.setZero(); 
    //measure angles
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




    // desired foot location reatlive to body center;
    FR_foot_relative_des.setZero(); FL_foot_relative_des.setZero();
    RR_foot_relative_des.setZero(); RL_foot_relative_des.setZero();

    FR_foot_relative_des_old.setZero(); FL_foot_relative_des_old.setZero();
    RR_foot_relative_des_old.setZero(); RL_foot_relative_des_old.setZero();    

    // measured foot location reatlive to body center;
    FR_foot_relative_mea.setZero(); FL_foot_relative_mea.setZero();
    RR_foot_relative_mea.setZero(); RL_foot_relative_mea.setZero(); 
    FR_foot_relative_mea_old.setZero(); FL_foot_relative_mea_old.setZero();
    RR_foot_relative_mea_old.setZero(); RL_foot_relative_mea_old.setZero(); 


    // desired body posioin and rotation
    body_p_Homing.setZero();
    FR_foot_Homing.setZero();
    FL_foot_Homing.setZero();
    RR_foot_Homing.setZero();
    RL_foot_Homing.setZero();

    FR_foot_Homing_retarget<<0,
                             -0.12,
                             0; 
    FL_foot_Homing_retarget<<0,
                             0.12,
                             0;
    RR_foot_Homing_retarget<<0,
                             -0.12,
                             0; 
    RL_foot_Homing_retarget<<0,
                             0.12,
                             0;

    body_p_Homing_Retarget<< -0.01,
                             -0.0,
                             z_c;
                           
    body_r_homing.setZero();   



    body_p_des.setZero();
    body_r_des.setZero();
    FR_foot_des.setZero(); 
    FL_foot_des.setZero(); 
    RR_foot_des.setZero(); 
    RL_foot_des.setZero();

    body_p_est.setZero();
    body_r_est.setZero();

    root_pos.setZero();
    root_quat.setIdentity();
    root_euler.setZero();
    root_rot_mat.setZero();
    root_rot_mat_z.setZero();
    root_lin_vel.setZero();
    root_ang_vel.setZero();
    root_acc.setZero();

    rate = 0;
    ratex = 0;
    rate_stand_up = 0;    

    ////// Force distribution 
    kpp<< 60,
          60,
          0;
    kdp<< 10,
          15,
          0;

    kpw<< 0.1,
          0.1,
          0.5;

    kdw<< 0.0001,
          0.0001,
          0.001;


    F_sum.setZero();
    Momentum_sum << 0.0168352186, 0.0004636141, 0.0002367952,
                    0.0004636141, 0.0656071082, 3.6671e-05, 
                    0.0002367952, 3.6671e-05,   0.0742720659;


    rleg_com = 0; 
    lleg_com= 0;
    F_lr_predict.setZero();   
    Force_L_R.setZero();   
    bjx1 = 1;
    right_support = 1;

    FR_torque.setZero(); 
    FL_torque.setZero(); 
    RR_torque.setZero(); 
    RL_torque.setZero();  
    Legs_torque.setZero();   

    FR_swing = false; 
    FL_swing = false;
    RR_swing = false;
    RL_swing = false;              

    ////////////////////////// state estimation///////////////////////
    state_kine.setZero();

    support_flag = 0; /////left: 0; right: 1; double: 2
    omega_sensor = sqrt(gait::_g/z_c);

        
    support_pos_sensor[0] = 0; ///left support by default
    support_pos_sensor[1] = gait::RobotParaClass_HALF_HIP_WIDTH; ///left support by default
    support_pos_sensor[2] = 0; ///left support by default
    com_sensor[0] = 0; /////assuming com is 10cm above the pelvis
    com_sensor[1] = 0;
    com_sensor[2] = (z_c);
    com_sensor_hip[0] = 0; /////assuming com is 10cm above the pelvis
    com_sensor_hip[1] = 0;
    com_sensor_hip[2] = (z_c);	


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
        thetaacc_des[i] = 0;
        theta_des_pre[i] = 0;
        rfoot_theta_des[i] = 0;
        lfoot_theta_des[i] = 0;

        comv_sensor[i] = 0;
        coma_sensor[i] = 0;
        zmp_sensor[i] = 0;
        zmp_ref[i] = 0;
        dcm_ref[i]= 0;
        dcm_sensor[i] = 0;        

        frfoot_pose_sensor[i] = 0;
        flfoot_pose_sensor[i] = 0;
        rrfoot_pose_sensor[i] = 0;
        rlfoot_pose_sensor[i] = 0;
        com_des_ini[i] = 0;;
        rfoot_des_ini[i] = 0;;
        lfoot_des_ini[i] = 0;;
        theta_des_ini[i] = 0;
        body_r_Homing_dynamic[i] = 0;
        body_p_Homing_dynamic[i] = 0;
    }

    com_des[2] = z_c;
    
    if (gait_mode ==102)
    {}
    else
    {
        rfoot_des[1] = -gait::RobotParaClass_HALF_HIP_WIDTH;
        lfoot_des[1] = gait::RobotParaClass_HALF_HIP_WIDTH;
    }


    comav_butterworth.setZero();
    theta_default.setZero();


    L_com.setZero();
    com_estkine.setZero();
    cop_estkine.setZero();
    theta_estkine.setZero();
    theta_estkine_pre.setZero();    
    thetaa_estkine.setZero();
    Fr_estkine = 0;
    Fl_estkine = 0;
    comv_estkine.setZero();
    dob_result.setZero();
    J_ini_xx_est = 0;
    J_ini_yy_est = 0;
    thetav_estkine.setZero();




    ////////////////////// real-time mpc loop /////////////////////////	
    count_in_mpc_max = 1000;
    count_in_rt_loop = 0;

	joint2simulationx.position.resize(100);
	state_to_MPC.position.resize(25);
	state_feedback.setZero();
	slow_mpc_gait.setZero();   
	mpc_gait_flag = 0;
	mpc_gait_flag_old = 0;
	slow_mpc_gait_inte.setZero(); 
	
	fast_mpc_gait_flag = 0;
	fast_mpc_gait_flag_old = 0;
	fast_mpc_gait.setZero();	
	
	
	COM_in1.setZero(); 
	COM_in2.setZero(); 
	COMxyz_ref.setZero(); 
	COM_ref2.setZero(); 
	COM_in1(2) = COM_in2(2) = COMxyz_ref(2) = COM_ref2(2) = z_c; 

	COMv_ref.setZero(); 

	COMacc_in1.setZero();  
	COMacc_in2.setZero();  
	COMacc_ref.setZero();  
	COMaccv_ref.setZero();  
	COMacc_ref2.setZero();  

	FootL_in1.setZero();  
	FootL_in2.setZero();  
	FootL_ref.setZero();  
	FootL_ref2.setZero();  
	FootL_in1(1) = FootL_in2(1)= FootL_ref(1) = FootL_ref2(1) = gait::RobotParaClass_HALF_HIP_WIDTH; 
	FootLv_ref.setZero(); 

	FootR_in1.setZero();  
	FootR_in2.setZero();  
	FootR_ref.setZero();  
	FootR_in1(1) = FootR_in2(1)= FootR_ref(1) = FootR_ref2(1) = -gait::RobotParaClass_HALF_HIP_WIDTH; 
	FootRv_ref.setZero(); 

	zmp_in1.setZero(); zmp_in2.setZero(); zmpxyz_ref.setZero(); zmpv_ref.setZero(); zmp_ref2.setZero();
	dcm_in1.setZero(); dcm_in2.setZero(); dcmxyz_ref.setZero(); dcmv_ref.setZero(); dcm_ref2.setZero();

	body_in1.setZero(); body_in2.setZero(); body_ref.setZero(); bodyv_ref.setZero(); body_ref2;
	rfootrpy_in1.setZero(); rfootrpy_in2.setZero(); rfootrpy_ref.setZero(); rfootrpyv_ref.setZero(); rfootrpy_ref2.setZero();
	lfootrpy_in1.setZero(); lfootrpy_in2.setZero(); lfootrpy_ref.setZero(); lfootrpyv_ref.setZero(); lfootrpy_ref2.setZero();



	PelvisPos.setZero();  
	body_thetax.setZero();  
	LeftFootPosx.setZero(); 
	RightFootPosx.setZero();  
	dcmxyz_ref.setZero();  
	F_L.setZero();  
	F_R.setZero(); 
	M_L.setZero();  
	M_R.setZero(); 
	LeftFootRPY.setZero();  
	RightFootRPY.setZero(); 
	
	rpy_mpc_body.setZero();
	rpy_mpc_body(2) = COM_ref2(2);

	rfoot_inter.setZero();
	rfoot_inter(1) = FootR_ref2(1);

	lfoot_inter.setZero();
	lfoot_inter(1) = FootL_ref2(1);

	bodytheta_inter.setZero();

	rftheta_inter.setZero(); 
	lftheta_inter.setZero();

	zmp_inter.setZero(); 
	dcm_inter.setZero();

	zmp_mpc_ref.setZero(); rfoot_mpc_ref.setZero(); lfoot_mpc_ref.setZero(); bodyangle_mpc_ref.setZero(); comacc_mpc_ref.setZero();
	bodyangle_mpc.setZero();
	bodyangle_state.setZero();

	count_in_rt_ros = 0;

	count_inteplotation = 0;
	count_inteplotation_fast = 0;
	t_int = 0;  
	n_t_int = (int) round(dt_slow_mpc /dtx); 
	n_t_int_fast = (int) round(gait::dt_mpc_fast /dtx); 


	_mass = gait::mass;
	_j_ini = gait::J_ini;

	_Zsc = 0;
	_ggg = gait::g;	
	
	//// gait filter
    f_sample_comx1 = 1/dtx;
	fcutoff_comx1 = 5;
	fcutoff_comx2 = 10;
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
	butterworthLPF13.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF14.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF15.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF16.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF17.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF18.init(f_sample_comx1,fcutoff_comx2);
    fcutoff_comx3 = 100;	
	butterworthLPF19.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF20.init(f_sample_comx1,fcutoff_comx3);
	butterworthLPF21.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF22.init(f_sample_comx1,fcutoff_comx3);
    butterworthLPF23.init(f_sample_comx1,fcutoff_comx3);    
	butterworthLPF24.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF25.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF26.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF27.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF28.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF29.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF30.init(f_sample_comx1,fcutoff_comx3);
    
    pitch_angle_W = 0;

    n_period = round(tstep / dtx); 




  
    multiThread listen_publish_obj(robot_name);
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(300000); // must wait 300ms, to get first state

    ros::NodeHandle n;
    ros::Publisher lowState_pub; //for rviz visualization
    ros::Publisher gait_data_pub; // for data_analysis
    ros::Publisher gait_data_pubx;
    ros::Publisher control_to_rtmpc_pub_; /// state feedback to rtmpc
    ros::Publisher joint2sim_pub_;
    ros::Subscriber robot_mode_cmd;
    ros::Subscriber gain_cmd;
    
    rt_frequency = 1000; /// frequency of lower_level control
    time_programming = 1.0/rt_frequency;
    ros::Rate loop_rate(rt_frequency);

    joint2simulation.position.resize(100);
    leg2sim.position.resize(100);

    n_count = 0;
    stand_duration = 3; /// stand up: 2s
    
    

    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<unitree_legged_msgs::LowState>("/" + robot_name + "_gazebo/lowState/state", 1);
    servo_pub[0] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);
    gait_data_pub = n.advertise<sensor_msgs::JointState>("go1_gait_data",10);
    gait_data_pubx = n.advertise<sensor_msgs::JointState>("go1_gait_datax",10);
    control_to_rtmpc_pub_ = n.advertise<sensor_msgs::JointState>("/control2rtmpc/state", 10);
    
    robot_mode_cmd = n.subscribe("/Robot_mode", 10, keyboard_model_callback); //// for robot mode selection
    gain_cmd = n.subscribe("/Base_offset", 10, Base_offset_callback);

    cmd_gait = STAND_INIT;

    state_estimate_ekf_pub_ = n.advertise<sensor_msgs::JointState>("go1_state_estmate",10);
    state_gait_ekf.setZero();
    stand_count = 0;
    stand_up_count = 0;
    dynamic_count = 0;
    
    ///// a must : parameter initailization for robot control
    paramInit();

    ////////////////////======================================///////////////////
    ////////////////--------state estimation-----------//////////////////////////////    
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

    rfoot_sensor.setZero();
    lfoot_sensor.setZero();    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////*************************************************** main control loop ********************************////
    while (ros::ok()){

        count_in_rt_ros += 1;
        // publisher measured state:
        lowState_pub.publish(lowState);/// for data analysis
        
    
        ////////////////////////// kinematic-based state estimation ///////////////////////////////////////////////
        // Forward kinematics: for kinematic-based state estimation //////////////////////////////////////////////
        FR_angle_mea(0,0) = lowState.motorState[0].q;
        FR_angle_mea(1,0) = lowState.motorState[1].q;
        FR_angle_mea(2,0) = lowState.motorState[2].q;
        FL_angle_mea(0,0) = lowState.motorState[3].q;
        FL_angle_mea(1,0) = lowState.motorState[4].q;
        FL_angle_mea(2,0) = lowState.motorState[5].q;
        RR_angle_mea(0,0) = lowState.motorState[6].q;
        RR_angle_mea(1,0) = lowState.motorState[7].q;
        RR_angle_mea(2,0) = lowState.motorState[8].q;
        RL_angle_mea(0,0) = lowState.motorState[9].q;
        RL_angle_mea(1,0) = lowState.motorState[10].q;
        RL_angle_mea(2,0) = lowState.motorState[11].q;

        FR_dq_mea(0,0) = lowState.motorState[0].dq;
        FR_dq_mea(1,0) = lowState.motorState[1].dq;
        FR_dq_mea(2,0) = lowState.motorState[2].dq;
        FL_dq_mea(0,0) = lowState.motorState[3].dq;
        FL_dq_mea(1,0) = lowState.motorState[4].dq;
        FL_dq_mea(2,0) = lowState.motorState[5].dq;
        RR_dq_mea(0,0) = lowState.motorState[6].dq;
        RR_dq_mea(1,0) = lowState.motorState[7].dq;
        RR_dq_mea(2,0) = lowState.motorState[8].dq;
        RL_dq_mea(0,0) = lowState.motorState[9].dq;
        RL_dq_mea(1,0) = lowState.motorState[10].dq;
        RL_dq_mea(2,0) = lowState.motorState[11].dq;


        //// IMU quaternion
        root_quat = Eigen::Quaterniond(quat_w.CalculateAverage(lowState.imu.quaternion[0]),
                                       quat_x.CalculateAverage(lowState.imu.quaternion[1]),
                                       quat_y.CalculateAverage(lowState.imu.quaternion[2]),
                                       quat_z.CalculateAverage(lowState.imu.quaternion[3]));         

        // euler angle: roll pitch yaw
        root_rot_mat = root_quat.toRotationMatrix();
        root_euler = Utils::quat_to_euler(root_quat);
        body_r_est(0,0) = root_euler(0);
        body_r_est(1,0) = root_euler(1);
        body_r_est(2,0) = root_euler(2);

        double yaw_angle = body_r_est(2,0);
        root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

        /// relative to body center. global framework;
        body_p_est.setZero();
        FR_foot_relative_mea = Kine.Forward_kinematics_g(body_p_est, body_r_est,FR_angle_mea, 0);
        FR_Jaco_est = Kine.Jacobian_kin;
        FL_foot_relative_mea = Kine.Forward_kinematics_g(body_p_est, body_r_est,FL_angle_mea, 1);
        FL_Jaco_est = Kine.Jacobian_kin;
        RR_foot_relative_mea = Kine.Forward_kinematics_g(body_p_est, body_r_est,RR_angle_mea, 2);
        RR_Jaco_est = Kine.Jacobian_kin;
        RL_foot_relative_mea = Kine.Forward_kinematics_g(body_p_est, body_r_est,RL_angle_mea, 3);
        RL_Jaco_est = Kine.Jacobian_kin;
        
        //// relative velocity in global framework: another way is to use differential of measured retlative position;
        // FR_v_est_relative = FR_Jaco_est * FR_dq_mea;
        // FL_v_est_relative = FL_Jaco_est * FL_dq_mea; 
        // RR_v_est_relative = RR_Jaco_est * RR_dq_mea;
        // RL_v_est_relative = RL_Jaco_est * RL_dq_mea; 
        if (count_in_rt_ros > 1)
        {
            FR_v_est_relative = (FR_foot_relative_mea - FR_foot_relative_mea_old)/dtx;
            FL_v_est_relative = (FL_foot_relative_mea - FL_foot_relative_mea_old)/dtx;
            RR_v_est_relative = (RR_foot_relative_mea - RR_foot_relative_mea_old)/dtx;
            RL_v_est_relative = (RL_foot_relative_mea - RL_foot_relative_mea_old)/dtx;            
        }
        //// feet force generation
        if(count_in_rt_ros<5)
        {
            footforce_fr(count_in_rt_ros,0) = lowState.footForce[0];
            footforce_fl(count_in_rt_ros,0) = lowState.footForce[1];
            footforce_rr(count_in_rt_ros,0) = lowState.footForce[2];
            footforce_rl(count_in_rt_ros,0) = lowState.footForce[3];
        }
        else
        {
            footforce_fr.block<4,1>(0,0) = footforce_fr.block<4,1>(1,0);
            footforce_fl.block<4,1>(0,0) = footforce_fl.block<4,1>(1,0);
            footforce_rr.block<4,1>(0,0) = footforce_rr.block<4,1>(1,0);
            footforce_rl.block<4,1>(0,0) = footforce_rl.block<4,1>(1,0);
            footforce_fr(4,0) = lowState.footForce[0];
            footforce_fl(4,0) = lowState.footForce[1];
            footforce_rr(4,0) = lowState.footForce[2];
            footforce_rl(4,0) = lowState.footForce[3];
        } 
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
            }
        }
        else
        {
            ///// kinematic-based open-loop state estimation //////////
            comav_butterworth = state_est_kine.state_estimator(gait_mode, support_flag, FR_foot_relative_mea, FL_foot_relative_mea,RR_foot_relative_mea, RL_foot_relative_mea,
                                                    footforce_fr, footforce_fl,footforce_rr,footforce_rl,frfoot_pose_sensor, flfoot_pose_sensor, rrfoot_pose_sensor, rlfoot_pose_sensor, 
                                                    support_pos_sensor, com_sensor, dcm_sensor, omega_sensor);        
            // std::cout<<"comx:"<<com_sensor[0]<<std::endl;
            // std::cout<<"count_in_rt_ros:"<<count_in_rt_ros<<std::endl;
            /////////// global state: EKF based estination //////////
            gait_pose_callback();
            state_gait_ekf = state_est_main_update(dtx);
            for(int j=0; j<3;j++)
            {
                // com_sensor[j] = (a1_ctrl_states.estimated_root_pos(j,0) + com_sensor[j])/2;
                com_sensor[j] = (a1_ctrl_states.estimated_root_pos(j,0))/1;
                comv_sensor[j] = a1_ctrl_states.estimated_root_vel(j,0); 
            }
        }
        theta_estkine = body_r_est;
        if (count_in_rt_ros > 1)
        {
            // comv_sensor[0] = (com_sensor[0] - com_sensor_pre[0])/dtx; 
            // comv_sensor[1] = (com_sensor[1] - com_sensor_pre[1])/dtx; 
            // comv_sensor[2] = (com_sensor[2] - com_sensor_pre[2])/dtx; 
            thetav_estkine = (theta_estkine - theta_estkine_pre)/dtx; 
        }

        /// send data feedback: t_mpc, 3d comp, 3d comv, 3d compa, 3d theta, 3d thetav,
        state_feedback.block<3,1>(1,0) =  a1_ctrl_states.estimated_root_pos;
        state_feedback.block<3,1>(4,0) =  a1_ctrl_states.estimated_root_vel;
        state_feedback.block<3,1>(10,0) =  theta_estkine;   
        state_feedback.block<3,1>(13,0) =  thetav_estkine;   
        if(gait_status = DYNAMIC_STATUS) /// measured lfoot and rfoot location
        {
            // //// FR, RL two legs move synchronous
            // FR_foot_des[0] = FR_foot_Homing[0] + lfoot_des[0] - lfoot_des_ini[0];
            // FR_foot_des[1] = FR_foot_Homing[1] + lfoot_des[1] - lfoot_des_ini[1];
            // FR_foot_des[2] = FR_foot_Homing[2] + lfoot_des[2] - lfoot_des_ini[2];
            // RL_foot_des[0] = RL_foot_Homing[0] + lfoot_des[0] - lfoot_des_ini[0];
            // RL_foot_des[1] = RL_foot_Homing[1] + lfoot_des[1] - lfoot_des_ini[1];
            // RL_foot_des[2] = RL_foot_Homing[2] + lfoot_des[2] - lfoot_des_ini[2];
            // //// FL, RR two legs move synchronous
            // FL_foot_des[0] = FL_foot_Homing[0] + rfoot_des[0] - rfoot_des_ini[0];
            // FL_foot_des[1] = FL_foot_Homing[1] + rfoot_des[1] - rfoot_des_ini[1];
            // FL_foot_des[2] = FL_foot_Homing[2] + rfoot_des[2] - rfoot_des_ini[2];
            // RR_foot_des[0] = RR_foot_Homing[0] + rfoot_des[0] - rfoot_des_ini[0];
            // RR_foot_des[1] = RR_foot_Homing[1] + rfoot_des[1] - rfoot_des_ini[1];
            // RR_foot_des[2] = RR_foot_Homing[2] + rfoot_des[2] - rfoot_des_ini[2];
            state_feedback(19,0) =  (a1_ctrl_states.foot_pos_world(0,1) - FR_foot_Homing[0] + lfoot_des_ini[0] 
                                   + a1_ctrl_states.foot_pos_world(0,2) - RL_foot_Homing[0] + lfoot_des_ini[0])/2;
            state_feedback(20,0) =  (a1_ctrl_states.foot_pos_world(1,1) - FR_foot_Homing[1] + lfoot_des_ini[1] 
                                   + a1_ctrl_states.foot_pos_world(1,2) - RL_foot_Homing[1] + lfoot_des_ini[1])/2;
            state_feedback(21,0) =  (a1_ctrl_states.foot_pos_world(2,1) - FR_foot_Homing[2] + lfoot_des_ini[2] 
                                   + a1_ctrl_states.foot_pos_world(2,2) - RL_foot_Homing[2] + lfoot_des_ini[2])/2;
            state_feedback(22,0) =  (a1_ctrl_states.foot_pos_world(0,0) - FL_foot_Homing[0] + rfoot_des_ini[0] 
                                   + a1_ctrl_states.foot_pos_world(0,3) - RR_foot_Homing[0] + rfoot_des_ini[0])/2;
            state_feedback(23,0) =  (a1_ctrl_states.foot_pos_world(1,0) - FL_foot_Homing[1] + rfoot_des_ini[1] 
                                   + a1_ctrl_states.foot_pos_world(1,3) - RR_foot_Homing[1] + rfoot_des_ini[1])/2;
            state_feedback(24,0) =  (a1_ctrl_states.foot_pos_world(2,0) - FL_foot_Homing[2] + rfoot_des_ini[2] 
                                   + a1_ctrl_states.foot_pos_world(2,3) - RR_foot_Homing[2] + rfoot_des_ini[2])/2; 
        }
        else /// stepping in place: not moving
        {
            state_feedback.block<3,1>(19,0) =  a1_ctrl_states.estimated_root_pos;
            state_feedback.block<3,1>(22,0) =  a1_ctrl_states.estimated_root_pos;          
        }


        /////////////////////////////////MPC gait generation //////////////////
        for (int jx = 0; jx<25; jx++)
        {
          state_to_MPC.position[jx] = state_feedback(jx,0);
        }        
        control_to_rtmpc_pub_.publish(state_to_MPC); //// state feedback to MPC; 1+5*body p,v,a+3*3*foot position;
        
        
        //////////////////// gait control loop/////////////////////////////////////////////////////
        /// *****************joint cmd generation*******************////
        double targetPos[12]  = {0, 0.87, -1.5, 0, 0.87, -1.5, 0, 0.87, -1.5, 0, 0.87, -1.5};  
        // reference angle generation: a simple test


        switch (cmd_gait){        
            case STAND_INIT:
                gait_status = STAND_INIT_STATUS;

                //****************Homing_pose*******************
                for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;

                percent = std::min(1.0, pow((n_count*time_programming)/stand_duration,2));
                for(int j=0; j<12; j++){
                    lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
                }

                FR_angle_des(0,0) = lowCmd.motorCmd[0].q;
                FR_angle_des(1,0) = lowCmd.motorCmd[1].q;
                FR_angle_des(2,0) = lowCmd.motorCmd[2].q;
                FL_angle_des(0,0) = lowCmd.motorCmd[3].q;
                FL_angle_des(1,0) = lowCmd.motorCmd[4].q;
                FL_angle_des(2,0) = lowCmd.motorCmd[5].q;
                RR_angle_des(0,0) = lowCmd.motorCmd[6].q;
                RR_angle_des(1,0) = lowCmd.motorCmd[7].q;
                RR_angle_des(2,0) = lowCmd.motorCmd[8].q;
                RL_angle_des(0,0) = lowCmd.motorCmd[9].q;
                RL_angle_des(1,0) = lowCmd.motorCmd[10].q;
                RL_angle_des(2,0) = lowCmd.motorCmd[11].q;   

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

                if (n_count*time_programming==stand_duration)
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
                
                /////// for normal walking: preparing for stand up
                ratex = pow(stand_count/1000.0,2); 
                if(ratex<=1000)
                {
                    for(int j=0; j<3;j++)
                    {
                        body_p_des[j] = jointLinearInterpolation(body_p_Homing(j,0), body_p_Homing_Retarget(j,0), ratex, 0); 
                        if(j==1)
                        {
                            FR_foot_des[j] = jointLinearInterpolation(FR_foot_des(j,0), FR_foot_Homing_retarget(j,0), ratex, 0);
                            FL_foot_des[j] = jointLinearInterpolation(FL_foot_des(j,0), FL_foot_Homing_retarget(j,0), ratex, 0);
                            RR_foot_des[j] = jointLinearInterpolation(RR_foot_des(j,0), RR_foot_Homing_retarget(j,0), ratex, 0);
                            RL_foot_des[j] = jointLinearInterpolation(RL_foot_des(j,0), RL_foot_Homing_retarget(j,0), ratex, 0);                            
                        }                   
                    }
                    body_p_Homing = body_p_des;
                    body_r_homing = body_r_des;
                    FR_foot_Homing = FR_foot_des;
                    FL_foot_Homing = FL_foot_des;
                    RR_foot_Homing = RR_foot_des;
                    RL_foot_Homing = RL_foot_des;                    
                }
                // else
                // {   
                //     body_p_des[0] = body_p_Homing[0] + base_offset_x;
                //     body_p_des[1] = body_p_Homing[1] + base_offset_y;
                //     body_p_des[2] = body_p_Homing[2] + base_offset_z;
                //     body_r_des[0] = body_r_homing[0] + base_offset_roll;
                //     body_r_des[1] = body_r_homing[1] + base_offset_pitch;
                //     body_r_des[2] = body_r_homing[2] + base_offset_yaw;                            
                // }


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

                break;
            case STAND_UP:
                gait_status = STAND_UP_STATUS;
                stand_up_count++;
                if(stand_up_count==1)
                {
                    printf("===== STAND UP GRF compensation======\n");
                }
                /// hand tuned gait: global inverse kinematics test://///////////////////
                // ******************** body movement test *********************////
                // body_p_des(2,0) = body_p_Homing(2,0) + (0.05 * (sin(2*gait::pi*(stand_up_count*time_programming))));
                // body_p_des(1,0) = body_p_Homing(1,0) + (0.05 * (cos(gait::pi*stand_up_count*time_programming+ PI/2)));
                // body_r_des(0,0) = (0.1 * (sin(stand_up_count*time_programming)));
                // body_r_des(1,0) = (0.1 * (sin(stand_up_count*time_programming))); 
                
                comv_des[0] = (body_p_des[0] - com_des_pre[0]) /dtx;
                comv_des[1] = (body_p_des[1] - com_des_pre[1]) /dtx;
                comv_des[2] = (body_p_des[2] - com_des_pre[2]) /dtx;     
                right_support = 2;           
                
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
                // cout<<"xxx"<<endl;


                // //// prediction force
                // F_sum(0) = gait::mass * coma_des[0];
                // F_sum(1) = gait::mass * coma_des[1];
                // F_sum(2) = gait::mass * gait::_g + gait::mass * coma_des[2];
                // F_sum(3,0) = Momentum_sum(0,0) * thetaacc_des[0] + Momentum_sum(0,1) * thetaacc_des[1] + Momentum_sum(0,2) * thetaacc_des[2];
                // F_sum(4,0) = Momentum_sum(1,0) * thetaacc_des[0] + Momentum_sum(1,1) * thetaacc_des[1] + Momentum_sum(1,2) * thetaacc_des[2];
                // F_sum(5,0) = Momentum_sum(2,0) * thetaacc_des[0] + Momentum_sum(2,1) * thetaacc_des[1] + Momentum_sum(2,2) * thetaacc_des[2];
                // Dynam.force_opt(body_p_des,FR_foot_des, FL_foot_des, RR_foot_des, RL_foot_des,
                //                 F_sum, gait_mode, right_support, y_offset);
                // FR_torque = Dynam.compute_joint_torques(FR_Jaco,FR_swing,FR_foot_relative_des,FR_foot_relative_mea,
                //                                         FR_v_relative,FR_v_est_relative,0);
                // FL_torque = Dynam.compute_joint_torques(FL_Jaco,FL_swing,FL_foot_relative_des,FL_foot_relative_mea,
                //                                         FL_v_relative,FL_v_est_relative,1);            
                // RR_torque = Dynam.compute_joint_torques(RR_Jaco,RR_swing,RR_foot_relative_des,RR_foot_relative_mea,
                //                                         RR_v_relative,RR_v_est_relative,2);
                // RL_torque = Dynam.compute_joint_torques(RL_Jaco,RL_swing,RL_foot_relative_des,RL_foot_relative_mea,
                //                                         RL_v_relative,RL_v_est_relative,3);
                // ///cout << "xxxxxxxxxxxxxxxxx"<<endl;                                        
                // Legs_torque.block<3,1>(0,0) = FR_torque;
                // Legs_torque.block<3,1>(3,0) = FL_torque;
                // Legs_torque.block<3,1>(6,0) = RR_torque;
                // Legs_torque.block<3,1>(9,0) = RL_torque; 
                // Legs_torque(0,0) = butterworthLPF19.filter(FR_torque(0,0));
                // Legs_torque(1,0) = butterworthLPF20.filter(FR_torque(1,0));
                // Legs_torque(2,0) = butterworthLPF21.filter(FR_torque(2,0));
                // Legs_torque(3,0) = butterworthLPF22.filter(FL_torque(0,0));
                // Legs_torque(4,0) = butterworthLPF23.filter(FL_torque(1,0));
                // Legs_torque(5,0) = butterworthLPF24.filter(FL_torque(2,0));
                // Legs_torque(6,0) = butterworthLPF25.filter(RR_torque(0,0));
                // Legs_torque(7,0) = butterworthLPF26.filter(RR_torque(1,0));
                // Legs_torque(8,0) = butterworthLPF27.filter(RR_torque(2,0));
                // Legs_torque(9,0) = butterworthLPF28.filter(RL_torque(0,0));
                // Legs_torque(10,0) = butterworthLPF29.filter(RL_torque(1,0));
                // Legs_torque(11,0) = butterworthLPF30.filter(RL_torque(2,0));



                break;
            
            case DYNAMIC:
                gait_status = DYNAMIC_STATUS;
                dynamic_count++;
                if(dynamic_count==1)
                {
                    body_p_Homing_dynamic[0] = body_p_des[0];
                    body_p_Homing_dynamic[1] = body_p_des[1];
                    body_p_Homing_dynamic[2] = body_p_des[2];
                    body_r_Homing_dynamic[0] = body_r_des[0];
                    body_r_Homing_dynamic[1] = body_r_des[1];
                    body_r_Homing_dynamic[2] = body_r_des[2];                               
                    printf("===PERFORMING DYNAMIC WALKING===\n");
                    cout<<"body_p_Homing_dynamic"<<body_p_Homing_dynamic<<endl;
                    cout<<"body_r_Homing_dynamic"<<body_r_Homing_dynamic<<endl;
                } 
                a1_ctrl_states.movement_mode = 1;


                        
                ///////////////////////////////////////// NLP-based gait planner //////////////////////////
                // {
                /////// rt loop counter 
                count_in_rt_loop += 1;  
                t_int = ((int) round(count_in_rt_loop/n_t_int));
                state_feedback(0,0) = t_int;

                // //////======= test ===========//////////////
                // body_p_des[0] = body_p_Homing_dynamic[0] + 0.00 * sin(8*M_PI*dynamic_count/1000.0);
                // body_p_des[1] = body_p_Homing_dynamic[1] + 0.015 * sin(8*M_PI*dynamic_count/1000.0);
                // body_p_des[2] = body_p_Homing_dynamic[2] + 0.00 *(sin(8*M_PI*dynamic_count/1000.0));
                // body_r_des[0] = body_r_Homing_dynamic[0] + 0.0 * sin(8*M_PI*dynamic_count/1000.0);
                // body_r_des[1] = body_r_Homing_dynamic[1] + 0.0 * sin(8*M_PI*dynamic_count/1000.0);
                // body_r_des[2] = body_r_Homing_dynamic[2] + 0.0 * sin(8*M_PI*dynamic_count/1000.0); 
                
                // comv_des[0] = -0.00 * cos(8*M_PI*dynamic_count/1000.0) * (8*M_PI/1000.0);
                // comv_des[1] = -0.15 * cos(8*M_PI*dynamic_count/1000.0) * (8*M_PI/1000.0);
                // comv_des[2] = -0.00 * cos(8*M_PI*dynamic_count/1000.0) * (8*M_PI/1000.0);

                // thetav_des[0] = -0.00 * cos(8*M_PI*dynamic_count/1000.0) * (8*M_PI/1000.0);
                // thetav_des[1] = -0.00 * cos(8*M_PI*dynamic_count/1000.0) * (8*M_PI/1000.0);
                // thetav_des[2] = -0.00 * cos(8*M_PI*dynamic_count/1000.0) * (8*M_PI/1000.0);   
                // right_support = 2;




                //// non -real-time intepoloation: data filter: from hierachical convex optimization:
                bjx1 = slow_mpc_gait(27);
                right_support = slow_mpc_gait(99);
                // if(right_support>1)
                // {
                //     cout<<"right_support:"<<right_support<<endl;
                //     cout<<count_in_rt_ros<<endl;
                // }  
                if(using_hie_using<0.5)
                {
                    com_des[0] = butterworthLPF1.filter(slow_mpc_gait(0,0));
                    com_des[1] = butterworthLPF2.filter(slow_mpc_gait(1,0));                
                    com_des[2] = butterworthLPF3.filter(slow_mpc_gait(2,0));
                    // theta_des[0] = butterworthLPF4.filter(slow_mpc_gait(3));
                    // theta_des[1] = butterworthLPF5.filter(slow_mpc_gait(4));	    
                    // theta_des[2] = butterworthLPF6.filter(slow_mpc_gait(5));
                    // theta_des[0] = 0;
                    // // theta_des[1] = 0;	    
                    // theta_des[2] = 0; 
                    theta_des[0] = butterworthLPF4.filter(fast_mpc_gait(3));
                    theta_des[1] = butterworthLPF5.filter(fast_mpc_gait(4));	    
                    theta_des[2] = butterworthLPF6.filter(fast_mpc_gait(5));                                          
                    rfoot_des[0] = butterworthLPF7.filter(slow_mpc_gait(9));
                    rfoot_des[1] = butterworthLPF8.filter(slow_mpc_gait(10));
                    rfoot_des[2] = butterworthLPF9.filter(slow_mpc_gait(11));
                    lfoot_des[0] = butterworthLPF10.filter(slow_mpc_gait(6));
                    lfoot_des[1] = butterworthLPF11.filter(slow_mpc_gait(7));
                    lfoot_des[2] = butterworthLPF12.filter(slow_mpc_gait(8));
                } 
                else
                {
                    com_des[0] = butterworthLPF1.filter(fast_mpc_gait(0,0));
                    com_des[1] = butterworthLPF2.filter(fast_mpc_gait(1,0));                
                    com_des[2] = butterworthLPF3.filter(fast_mpc_gait(2,0));
                    theta_des[0] = butterworthLPF4.filter(fast_mpc_gait(3));
                    theta_des[1] = butterworthLPF5.filter(fast_mpc_gait(4));	    
                    theta_des[2] = butterworthLPF6.filter(fast_mpc_gait(5));
                    // theta_des[0] = 0;
                    // // theta_des[1] = 0;	    
                    // theta_des[2] = 0;                       
                    rfoot_des[0] = butterworthLPF7.filter(fast_mpc_gait(9));
                    rfoot_des[1] = butterworthLPF8.filter(fast_mpc_gait(10));
                    rfoot_des[2] = butterworthLPF9.filter(fast_mpc_gait(11));
                    lfoot_des[0] = butterworthLPF10.filter(fast_mpc_gait(6));
                    lfoot_des[1] = butterworthLPF11.filter(fast_mpc_gait(7));
                    lfoot_des[2] = butterworthLPF12.filter(fast_mpc_gait(8));                    

                }             




                 
                if (dynamic_count==1)
                {
                    for(int j=0;j<3;j++)
                    {
                        com_des_ini[j] = com_des[j];
                        rfoot_des_ini[j] = rfoot_des[j];
                        lfoot_des_ini[j] = lfoot_des[j];
                        theta_des_ini[j] = theta_des[j];
                    }    
                }
                // coma_des[0] = butterworthLPF13.filter(slow_mpc_gait(39));
                // coma_des[1] = butterworthLPF14.filter(slow_mpc_gait(40));
                // coma_des[2] = butterworthLPF15.filter(slow_mpc_gait(41));		
                // thetaacc_des[0] = butterworthLPF16.filter(slow_mpc_gait(73));
                // thetaacc_des[1] = butterworthLPF17.filter(slow_mpc_gait(74));
                // thetaacc_des[2] = butterworthLPF18.filter(slow_mpc_gait(75));	

                if (count_in_rt_loop * dtx >= 0.6)
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
                        body_p_des[2] = com_des[2] - com_des_ini[2];

                        body_r_des[0] = theta_des[0] - theta_des_ini[0];
                        body_r_des[1] = theta_des[1] - theta_des_ini[1];
                        body_r_des[2] = theta_des[2] - theta_des_ini[2];
              
                        //// FR, RL two legs move synchronous
                        FR_foot_des[0] = FR_foot_Homing[0] + lfoot_des[0] - lfoot_des_ini[0];
                        FR_foot_des[1] = FR_foot_Homing[1] + lfoot_des[1] - lfoot_des_ini[1];
                        FR_foot_des[2] = FR_foot_Homing[2] + lfoot_des[2] - lfoot_des_ini[2];
                        RL_foot_des[0] = RL_foot_Homing[0] + lfoot_des[0] - lfoot_des_ini[0];
                        RL_foot_des[1] = RL_foot_Homing[1] + lfoot_des[1] - lfoot_des_ini[1];
                        RL_foot_des[2] = RL_foot_Homing[2] + lfoot_des[2] - lfoot_des_ini[2];

                        //// FL, RR two legs move synchronous
                        FL_foot_des[0] = FL_foot_Homing[0] + rfoot_des[0] - rfoot_des_ini[0];
                        FL_foot_des[1] = FL_foot_Homing[1] + rfoot_des[1] - rfoot_des_ini[1];
                        FL_foot_des[2] = FL_foot_Homing[2] + rfoot_des[2] - rfoot_des_ini[2];
                        RR_foot_des[0] = RR_foot_Homing[0] + rfoot_des[0] - rfoot_des_ini[0];
                        RR_foot_des[1] = RR_foot_Homing[1] + rfoot_des[1] - rfoot_des_ini[1];
                        RR_foot_des[2] = RR_foot_Homing[2] + rfoot_des[2] - rfoot_des_ini[2]; 

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

                
                comv_des[0] = (body_p_des[0] - com_des_pre[0]) /dtx;
                comv_des[1] = (body_p_des[1] - com_des_pre[1]) /dtx;
                comv_des[2] = (body_p_des[2] - com_des_pre[2]) /dtx;  

                base_pos_fb_controler();
                ///cout << "xyyyy"<<endl;
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

                ///// Force distribute/// Grf generation         

                leg_position.block<3,1>(0,0) = FR_foot_des;
                leg_position.block<3,1>(3,0) = FL_foot_des; 
                leg_position.block<3,1>(6,0) = RR_foot_des;
                leg_position.block<3,1>(9,0) = RL_foot_des; 

                /////////////////////////////// desired foot position and foot velocity: relative to  body framework
                FR_foot_relative_des = (FR_foot_des - body_p_des);
                FL_foot_relative_des = (FL_foot_des - body_p_des);
                RR_foot_relative_des = (RR_foot_des - body_p_des);
                RL_foot_relative_des = (RL_foot_des - body_p_des);            
                if (count_in_rt_loop>0)
                {
                    FR_v_relative = (FR_foot_relative_des - FR_foot_relative_des_old) /dtx;
                    FL_v_relative = (FL_foot_relative_des - FL_foot_relative_des_old) /dtx;
                    RR_v_relative = (RR_foot_relative_des - RR_foot_relative_des_old) /dtx;
                    RL_v_relative = (RL_foot_relative_des - RL_foot_relative_des_old) /dtx;
                }

                
                nt_slow_mpc = (( count_in_rt_loop % n_t_int )* dtx) / (dt_slow_mpc);
                

                // //// prediction force
                // // F_sum(0) = gait::mass * slow_mpc_gait(80);
                // // F_sum(1) = gait::mass * slow_mpc_gait(81);
                // // F_sum(2) = gait::mass * gait::_g - gait::mass * slow_mpc_gait(82);
                // F_sum(0) = gait::mass * coma_des[0];
                // F_sum(1) = gait::mass * coma_des[1];
                // F_sum(2) = gait::mass * gait::_g + gait::mass * coma_des[2];
                // F_sum(3,0) = Momentum_sum(0,0) * thetaacc_des[0] + Momentum_sum(0,1) * thetaacc_des[1] + Momentum_sum(0,2) * thetaacc_des[2];
                // F_sum(4,0) = Momentum_sum(1,0) * thetaacc_des[0] + Momentum_sum(1,1) * thetaacc_des[1] + Momentum_sum(1,2) * thetaacc_des[2];
                // F_sum(5,0) = Momentum_sum(2,0) * thetaacc_des[0] + Momentum_sum(2,1) * thetaacc_des[1] + Momentum_sum(2,2) * thetaacc_des[2];
                // // // double com_rleg_dis = sqrt(pow(com_des[0]-rfoot_des[0], 2) + pow(com_des[1]-rfoot_des[1], 2) + pow(com_des[2]-rfoot_des[2], 2));
                // // // double com_lleg_dis = sqrt(pow(com_des[0]-lfoot_des[0], 2) + pow(com_des[1]-lfoot_des[1], 2) + pow(com_des[2]-lfoot_des[2], 2));
                // // vec_foot_rl << lfoot_des[0] - rfoot_des[0],
                // //                 lfoot_des[1] - rfoot_des[1],
                // //                 lfoot_des[2] - rfoot_des[2];
                // // vec_com_rfoot << com_des[0] - rfoot_des[0],
                // //                 com_des[1] - rfoot_des[1],
                // //                 com_des[2] - rfoot_des[2];                          
                // // rlleg_dis = sqrt(pow(vec_foot_rl[0], 2) + pow(vec_foot_rl[1], 2) + pow(vec_foot_rl[2], 2));
                // // com_rleg_dis = vec_foot_rl[0]*vec_com_rfoot[0] + vec_foot_rl[1]*vec_com_rfoot[1] + vec_foot_rl[2]*vec_com_rfoot[2];
                // // rleg_com_raw = com_rleg_dis /rlleg_dis;
                // // rleg_com_raw1 = std::min(rleg_com_raw,1.0);
                // // rleg_com = std::max(rleg_com_raw1,0.0);
                // // lleg_com = 1 - rleg_com;            
                // // // double com_rleg_dis = sqrt(pow(com_des[0]-rfoot_des[0], 2) + pow(com_des[1]-rfoot_des[1], 2) + pow(com_des[2]-rfoot_des[2], 2));
                // // // double com_lleg_dis = sqrt(pow(com_des[0]-lfoot_des[0], 2) + pow(com_des[1]-lfoot_des[1], 2) + pow(com_des[2]-lfoot_des[2], 2));
                // // // rleg_com = com_rleg_dis/(com_rleg_dis + com_lleg_dis);
                // // // lleg_com = 1 - rleg_com;
                // // if(right_support == 0) ////left support
                // // {
                // //     F_lr_predict(0) = F_sum(0);
                // //     F_lr_predict(1) = F_sum(1);
                // //     F_lr_predict(2) = F_sum(2);
                // //     F_lr_predict(3) = 0;
                // //     F_lr_predict(4) = 0;
                // //     F_lr_predict(5) = 0;
                // //     switch (gait_mode)
                // //         {
                // //         case 101:  ////biped walking
                // //             FR_swing = true;
                // //             RR_swing = true;
                // //             FL_swing = false;
                // //             RL_swing = false;
                // //             break;
                // //         case 102:  ///troting
                // //             FR_swing = false;
                // //             RL_swing = false;
                // //             FL_swing = true;
                // //             RR_swing = true;
                // //             break;
                // //         case 103:  ///gallop: alter the  x-y direction
                // //             FR_swing = true;
                // //             FL_swing = true;
                // //             RR_swing = false;
                // //             RL_swing = false;                    
                // //             break;            
                // //         default:
                // //             break;
                // //         } 
                // // }
                // // else
                // // {
                // //     if (right_support == 1)
                // //     {
                // //         F_lr_predict(0) = 0;
                // //         F_lr_predict(1) = 0;
                // //         F_lr_predict(2) = 0;
                // //         F_lr_predict(3) = F_sum(0);
                // //         F_lr_predict(4) = F_sum(1);
                // //         F_lr_predict(5) = F_sum(2);
                // //         switch (gait_mode)
                // //             {
                // //             case 101:  ////biped walking
                // //                 FR_swing = false;
                // //                 RR_swing = false;
                // //                 FL_swing = true;
                // //                 RL_swing = true;
                // //                 break;
                // //             case 102:  ///troting
                // //                 FR_swing = true;
                // //                 RL_swing = true;
                // //                 FL_swing = false;
                // //                 RR_swing = false;                            
                // //                 break;
                // //             case 103:  ///gallop: alter the  x-y direction
                // //                 FR_swing = false;
                // //                 FL_swing = false;
                // //                 RR_swing = true;
                // //                 RL_swing = true;                    
                // //                 break;            
                // //             default:
                // //                 break;
                // //             }
                // //     }
                // //     else
                // //     {
                // //         F_lr_predict(0) = F_sum(0) * rleg_com;
                // //         F_lr_predict(3) = F_sum(0) - F_lr_predict(0);
                // //         F_lr_predict(1) = F_sum(1) * rleg_com;
                // //         F_lr_predict(4) = F_sum(1) - F_lr_predict(1);
                // //         F_lr_predict(2) = F_sum(2) * rleg_com;
                // //         F_lr_predict(5) = F_sum(2) - F_lr_predict(2);
                // //         FR_swing = false;
                // //         FL_swing = false;
                // //         RR_swing = false;
                // //         RL_swing = false;  
                // //     }                
                // // }
                // // ////force:  left-right-leg   
                // // for(int j=0; j<6; j++)
                // // {
                // //     //Force_L_R(j)= slow_mpc_gait(15+j) + nt_slow_mpc * (F_lr_predict(j) - slow_mpc_gait(15+j));  //continuous force profile
                // //     //Force_L_R(j)= slow_mpc_gait(15+j);  //discontinuous force profile
                // //     Force_L_R(j) = F_lr_predict(j);
                // // }                                
                // // Dynam.force_distribution(body_p_des,leg_position, Force_L_R, gait_mode, y_offset,rfoot_des,lfoot_des);
                // // cout<<"right_support:"<<right_support<<endl;
                // Dynam.force_opt(body_p_des,FR_foot_des, FL_foot_des, RR_foot_des, RL_foot_des,
                //                 F_sum, gait_mode, right_support, y_offset);
                // FR_torque = Dynam.compute_joint_torques(FR_Jaco,FR_swing,FR_foot_relative_des,FR_foot_relative_mea,
                //                                         FR_v_relative,FR_v_est_relative,0);
                // FL_torque = Dynam.compute_joint_torques(FL_Jaco,FL_swing,FL_foot_relative_des,FL_foot_relative_mea,
                //                                         FL_v_relative,FL_v_est_relative,1);
                // RR_torque = Dynam.compute_joint_torques(RR_Jaco,RR_swing,RR_foot_relative_des,RR_foot_relative_mea,
                //                                         RR_v_relative,RR_v_est_relative,2);
                // RL_torque = Dynam.compute_joint_torques(RL_Jaco,RL_swing,RL_foot_relative_des,RL_foot_relative_mea,
                //                                         RL_v_relative,RL_v_est_relative,3);
                // ///cout << "xxxxxxxxxxxxxxxxx"<<endl;                                        
                // Legs_torque.block<3,1>(0,0) = FR_torque;
                // Legs_torque.block<3,1>(3,0) = FL_torque;
                // Legs_torque.block<3,1>(6,0) = RR_torque;
                // Legs_torque.block<3,1>(9,0) = RL_torque;    
                // Legs_torque(0,0) = butterworthLPF19.filter(FR_torque(0,0));
                // Legs_torque(1,0) = butterworthLPF20.filter(FR_torque(1,0));
                // Legs_torque(2,0) = butterworthLPF21.filter(FR_torque(2,0));
                // Legs_torque(3,0) = butterworthLPF22.filter(FL_torque(0,0));
                // Legs_torque(4,0) = butterworthLPF23.filter(FL_torque(1,0));
                // Legs_torque(5,0) = butterworthLPF24.filter(FL_torque(2,0));
                // Legs_torque(6,0) = butterworthLPF25.filter(RR_torque(0,0));
                // Legs_torque(7,0) = butterworthLPF26.filter(RR_torque(1,0));
                // Legs_torque(8,0) = butterworthLPF27.filter(RR_torque(2,0));
                // Legs_torque(9,0) = butterworthLPF28.filter(RL_torque(0,0));
                // Legs_torque(10,0) = butterworthLPF29.filter(RL_torque(1,0));
                // Legs_torque(11,0) = butterworthLPF30.filter(RL_torque(2,0));
                // // }

                break;
            default:
                FR_swing = false;
                FL_swing = false;
                RR_swing = false;
                RL_swing = false;

                break;                    




            for (int i = 0; i < 12; i++) 
            {
                lowCmd.motorCmd[i].tau = Legs_torque(i);
            }  
        }

        // desired angle: generatee by inverse kinematics
        lowCmd.motorCmd[0].q = FR_angle_des(0,0);
        lowCmd.motorCmd[1].q = FR_angle_des(1,0);
        lowCmd.motorCmd[2].q = FR_angle_des(2,0);
        lowCmd.motorCmd[3].q = FL_angle_des(0,0);
        lowCmd.motorCmd[4].q = FL_angle_des(1,0);
        lowCmd.motorCmd[5].q = FL_angle_des(2,0);
        lowCmd.motorCmd[6].q = RR_angle_des(0,0);
        lowCmd.motorCmd[7].q = RR_angle_des(1,0);
        lowCmd.motorCmd[8].q = RR_angle_des(2,0); 
        lowCmd.motorCmd[9].q = RL_angle_des(0,0);
        lowCmd.motorCmd[10].q = RL_angle_des(1,0);
        lowCmd.motorCmd[11].q = RL_angle_des(2,0); 

        ////////////////////////////////////////////////////////////////////////
        // joint command pub to servors:
        for(int m=0; m<12; m++){
            servo_pub[m].publish(lowCmd.motorCmd[m]);
        } 


        FR_foot_relative_des_old = FR_foot_relative_des;
        FL_foot_relative_des_old = FL_foot_relative_des;
        RR_foot_relative_des_old = RR_foot_relative_des;
        RL_foot_relative_des_old = RL_foot_relative_des;

        FR_foot_relative_mea_old = FR_foot_relative_mea;
        FL_foot_relative_mea_old = FL_foot_relative_mea;
        RR_foot_relative_mea_old = RR_foot_relative_mea;
        RL_foot_relative_mea_old = RL_foot_relative_mea;

        com_des_pre[0] = body_p_des[0];
        com_des_pre[1] = body_p_des[1];
        com_des_pre[2] = body_p_des[2];


        com_sensor_pre[0] = com_sensor[0];
        com_sensor_pre[1] = com_sensor[1];
        com_sensor_pre[2] = com_sensor[2];
        
        theta_estkine_pre = theta_estkine;   
        theta_des_pre[0] = body_r_des[0];
        theta_des_pre[1] = body_r_des[1];
        theta_des_pre[2] = body_r_des[2];

         ///********************* data saving & publisher***********************************///////
        joint2simulation.header.stamp = ros::Time::now();
        joint2simulationx.header.stamp = ros::Time::now();
        leg2sim.header.stamp = ros::Time::now();


        for(int j=0; j<12; j++){
                joint2simulation.position[j] = lowCmd.motorCmd[j].q; // desired joint angles; 
        }
        for(int j=0; j<12; j++)
        {
            joint2simulation.position[12+j] = lowState.motorState[j].q;   // measured joint angles;
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

        /// force and torque distribution//// calculated by qp or MPC, this work proposes close-form solution
        for(int j=0; j<6; j++)
        {
            joint2simulation.position[54+j] = Force_L_R(j);  // desired force;
        }


        // FR force: 
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

        // joint torque desired 
        for(int j=0; j<12; j++)
        {
            joint2simulation.position[78+j] = Legs_torque(j);
        }
        
        ////////
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[90+j] = body_r_est(j);
        } 
        // cout<<"bddy_r_est"<<body_r_est.transpose()<<endl;
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[93+j] = com_sensor[j];
        } 




        ////////////////////////////  torque_measurment /////////////////////
        for(int j=0; j<12; j++)
        {
            joint2simulationx.position[j] = lowState.motorState[j].tauEst;
        } 
        for(int j=0; j<12; j++)
        {
            joint2simulationx.position[12+j] = Dynam.grf_opt(j,0);
        }    




        for(int j=0; j<4; j++)
        {
            joint2simulationx.position[48+j] = lowState.footForce[j];
        } 



        // robot data publisher
        for(int j=0;j<100;j++)
        {
            leg2sim.position[j] = state_gait_ekf[j];
        }

        gait_data_pub.publish(joint2simulation);
        gait_data_pubx.publish(joint2simulationx);
        state_estimate_ekf_pub_.publish(leg2sim);        








        
        ros::spinOnce();

        loop_rate.sleep();
        n_count++;   
        
    }
    
    return 0;
}
