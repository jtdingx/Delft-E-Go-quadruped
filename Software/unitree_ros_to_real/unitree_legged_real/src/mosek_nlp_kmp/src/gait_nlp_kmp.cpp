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
// #include "NLPRTControl/NLPRTControlClass.h"
//#include "/home/pi/go1_catkin/src/unitree_ros_to_real/unitree_legged_real/src/go1_rt_control/src/Robotpara/robot_const_para_config.h"
#include "/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/src/go1_rt_control/src/Robotpara/robot_const_para_config.h"
#include <geometry_msgs/Twist.h>  
#include "NLPRTControl/MpcRTControlClass.h"
#include "yaml.h"
#include <ros/package.h> 
#include <geometry_msgs/Twist.h>  


//////GRF force MPC class from MIT
// #include "FORCEMPC_mit/ConvexMpc.h"
#include "FORCEMPC_mit/Go1CtrlStates.h"
#include "FORCEMPC_mit/Go1Params.h"
////GRF force from home made
/////#### extern c for cvxpy
extern "C"{
#include "cpg_workspace.h"
#include "cpg_solve.h"
}
#include "utils/Utils.h"
const int mpc_horizon = 10;


using namespace Eigen;
using namespace std;
using namespace gait;


sensor_msgs::JointState joint2simulation;
sensor_msgs::JointState jointnlp_state;
sensor_msgs::JointState jointmpc_grf;

// NLPRTControlClass nlp_planner;
MpcRTControlClass tvo_nlp_planner;


bool mpc_start = false;

Eigen::Vector3d Rfoot_location_feedback;

Eigen::Vector3d Lfoot_location_feedback;

Eigen::Matrix<double,18,1> state_est;
Eigen::Matrix<double,37,1> state_feedback_receieved;
double step_length;
double step_width;
double step_yaw;
double step_yaw_old;
double state_alpha;

Matrix<double,100,1> nlp_gait_result;

extern double count;
extern double count_old;
double dt_mpc;
double test_mpc_grf;

// double footstepnumber;
int currentstep = 0;
int key_board_period = 0;

Vector3d yaw_ref_ones;

Eigen::Matrix<double,12,1> support_leg_est;



/// prepare for MPC grf solver//////
double mpc_gait_flag, mpc_gait_flag_old;
bool mpc_solver_declar = false;
Go1CtrlStates state; 



///// ========= home-made MPC for ground reaction force adjustment ====

Eigen::Matrix<double,4,3> foot_poses;
//foot_poses.setZero(); // 4x3 matrix, where each row contains the 3D position of the corresponding foot wrt. the body.
Eigen::Matrix<double,6,12> A;
Eigen::Matrix<double,6,1> bd;
// A.setZero();                // A is a matrix containing Identity matrices and cross product of relative foot position
//                             // later represented as a skew-symmetric matrix 
// bd.setZero();               // bd is a vector containing the external force and torque on the body.
Eigen::Matrix<double,4,1> no_contact;
// no_contact.setZero();
Eigen::Matrix<double,4,1> F_contact_force;
// F_contact_force.setZero();

Eigen::Matrix<double,mpc_horizon*6,12> A_predicted;
// A_predicted.setZero();

Eigen::Matrix<double,mpc_horizon,4> no_contact_predicted;
// no_contact_predicted.setZero();

Eigen::Matrix<double, 12,mpc_horizon> X_ref;
Eigen::Matrix<double, 12, 1> x_init;

Eigen::Matrix<double, 4,3> foot_poses_tem_ref;	

double _hcom;
Eigen::Vector3d body_p_Homing_Retarget;
Eigen::Vector3d body_r_Homing_Retarget;

Eigen::Matrix<double, 12,1> force_leg;
int right_support;
Matrix<double,25,1> control_gait;
Eigen::Matrix<double,53,1> state_feedback;
Eigen::Matrix<double, 4, 1 > bodyangle_state;
Eigen::Matrix<double, 11, 1> Nrtfoorpr_gen;

Eigen::Matrix<double, 3, 10> com_mpc_ref;
Eigen::Matrix<double, 3, 10> comv_mpc_ref;


///// mpc future state generated by planner
Eigen::Matrix<double, 3, 10> com_nlp_ref, comv_nlp_ref, rfoot_mpc_ref, lfoot_mpc_ref;
Eigen::Matrix<double, 1,10> support_prediction;
Eigen::Matrix<double, 30,1> yaw_mpc_ref;
Eigen::Matrix<double, 120, 1> support_position_mpc_ref;


///// home_made mpc

void mpc_compute_dynamics(Go1CtrlStates &state)
{
    // foot_poses contains the position of each foot relative to the body, i.e. foot_poses(i,j) is the rel position of
    // foot i in the j coordinate.
    Eigen::MatrixXd pos(3,12);
    pos << (Eigen::Matrix3d() << 0, -foot_poses(0, 2), foot_poses(0, 1), 
                          foot_poses(0, 2), 0, -foot_poses(0, 0), 
                          -foot_poses(0,1), foot_poses(0, 0), 0).finished(),
        (Eigen::Matrix3d() << 0, -foot_poses(1, 2), foot_poses(1, 1), 
                          foot_poses(1, 2), 0, -foot_poses(1, 0), 
                          -foot_poses(1,1), foot_poses(1, 0), 0).finished(),
        (Eigen::Matrix3d() << 0, -foot_poses(2, 2), foot_poses(2, 1), 
                           foot_poses(2, 2), 0, -foot_poses(2, 0), 
                           -foot_poses(2,1), foot_poses(2, 0), 0).finished(),
        (Eigen::Matrix3d() << 0, -foot_poses(3, 2), foot_poses(3, 1), 
                           foot_poses(3, 2), 0, -foot_poses(3, 0), 
                          -foot_poses(3,1), foot_poses(3, 0), 0).finished();
    state.root_rot_mat = Utils::eulerAnglesToRotationMatrix(state.root_euler);    
    // computeInertiaMatrix(q_mea, base_pos, base_quat);
    Eigen::Matrix3d Ig = state.root_rot_mat * state.Go1_trunk_inertia * state.root_rot_mat.transpose();
    Eigen::MatrixXd Ig_inv = Ig.inverse();   
    A.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3)/state.robot_mass;
    A.block<3,3>(0,3) = Eigen::MatrixXd::Identity(3,3)/state.robot_mass;
    A.block<3,3>(0,6) = Eigen::MatrixXd::Identity(3,3)/state.robot_mass;
    A.block<3,3>(0,9) = Eigen::MatrixXd::Identity(3,3)/state.robot_mass;
    A.block<3,12>(3,0) = Ig_inv * pos;
}

void mpc_update_A(Eigen::MatrixXd A){
    // This function updates the A matrix within the solver with the input A matrix.
    // No need to explicitly call this, just call update_parameters() with the corresponding inputs.
    int n = A.rows();
    int m = A.cols();
    for (int i=0; i<n; i++){
        for (int j=0; j<m; j++){
            cpg_update_A(i+j*n, A(i,j));
            // cpg_update_A(i*n+j, A(i,j));
        }
    }
};


void mpc_update_X_ref(Eigen::MatrixXd X_ref){
    // This function updates the contact forces F_contact within the solver with the input contact forces.
    // No need to explicitly call this, just call update_parameters() with the corresponding inputs.
    int n = X_ref.rows();
    int m = X_ref.cols();
    for (int i=0; i<n; i++){
        for (int j=0; j<m; j++){
            cpg_update_X_ref(i + j*n, X_ref(i,j));
            // cpg_update_X_ref(i*n+j, X_ref(i,j));
        }
    }
};

void mpc_update_x_init(Eigen::MatrixXd x_init){
    // This function updates the contact forces F_contact within the solver with the input contact forces.
    // No need to explicitly call this, just call update_parameters() with the corresponding inputs.
    int n = x_init.rows();
    int m = x_init.cols();
    for (int i=0; i<n; i++){
        for (int j=0; j<m; j++){
            cpg_update_x_init(i + j*n, x_init(i,j));
            // cpg_update_x_init(i*n+j, x_init(i,j));
        }
    }
};


void mpc_update_contact_state(Eigen::MatrixXd no_contact){
    // This function updates the contact forces F_contact within the solver with the input contact forces.
    // No need to explicitly call this, just call update_parameters() with the corresponding inputs.
    int n = no_contact.rows();
    int m = no_contact.cols();
    for (int i=0; i<n; i++){
        for (int j=0; j<m; j++){
            cpg_update_no_contact(i + j*n, no_contact(i,j));
            // cpg_update_no_contact(i*n+j, no_contact(i,j));
        }
    }
};

void mpc_update_parameters(Go1CtrlStates &state, Eigen::Matrix<double,3,10> com_mpc_ref, Eigen::Matrix<double,3,10> comv_mpc_ref, Eigen::Matrix<double,120,1> support_position, Eigen::Matrix<double,30,1> yaw_ref, Eigen::Matrix<double,3,10> rfoot, Eigen::Matrix<double,3,10> lfoot, Eigen::Matrix<double,10,1> support_prediction)
{ // MIGHT NOT NEED INPUTS Eigen::Matrix<double,6,12> A, Eigen::Matrix<double,6,1> bd, Eigen::Matrix<double,4,1> F_contact){
    // This function updates the internal parameters of the solver (dynamics of the system and contact forces)
    // based on the current dynamics and contact forces.
    Eigen::Matrix<double, mpc_horizon*6, 12> A_pred_mpc;
    Eigen::Matrix<double, mpc_horizon, 4> no_contact_mpc;
    // Eigen::Matrix<double, 12,mpc_horizon> X_ref;
    // Eigen::Matrix<double, 12, 1> x_init;
    // ////update the com_ref and comv_ref
    // for (int i = 0; i < mpc_horizon; i++)
    // {
    //   com_mpc_ref(0,i) = state.root_pos(0,0) + dt_mpc * (i+1) * state.root_lin_vel_d(0,0);
    //   com_mpc_ref(1,i) = state.root_pos(1,0) + dt_mpc * (i+1) * state.root_lin_vel_d(1,0);
    //   comv_mpc_ref(0,i) = state.root_lin_vel_d(0,0);
    //   comv_mpc_ref(1,i) = state.root_lin_vel_d(1,0);
    //   comv_mpc_ref(2,i) = 0;
    // }
    // ////// X_ref: base_pos, base_rpy, base_linear_velo, base_angular_velo
    X_ref.block<3,mpc_horizon>(0,0) = com_mpc_ref.block<3,mpc_horizon>(0,0);
    X_ref.block(3,0,3,mpc_horizon).setZero();
    X_ref.block(6,0,3,mpc_horizon) = comv_mpc_ref.block(0,0,3,mpc_horizon);
    X_ref.block(9,0,3,mpc_horizon).setZero();
    for (int i = 0; i < mpc_horizon; i++)
    {
      X_ref(3,i) = body_r_Homing_Retarget(0,0);
      X_ref(4,i) = body_r_Homing_Retarget(1,0);
      X_ref(5,i) = yaw_ref(3*i,0) + body_r_Homing_Retarget(2,0);
      X_ref(11,i) = yaw_ref(3*i+1,0);
    }
    ////// update A_predicted in the rt loop
    Eigen::Matrix<double,PLAN_HORIZON,1> yaw_ref_angle;
    for(int i=0; i<PLAN_HORIZON;i++)
    {
      yaw_ref_angle(i) = yaw_ref(3*(i));
    }    
    ////calculate the average yaw motion/////
    Eigen::Vector3d avg_root_euler_in_horizon;
    avg_root_euler_in_horizon
            <<
            0,
            0,
            (state.root_euler[2] + yaw_ref_angle.sum()) / (PLAN_HORIZON + 1);
    double cos_yaw = cos(avg_root_euler_in_horizon[2]);
    double sin_yaw = sin(avg_root_euler_in_horizon[2]);
      ////// update Rz
    Eigen::Matrix3d root_rot_mat_aver; 
    root_rot_mat_aver << cos_yaw, -sin_yaw, 0,
                          sin_yaw, cos_yaw, 0,
                                0,       0, 1;                                                   
    // ////calculate the average yaw motion/////
    // Eigen::Vector3d avg_root_euler_in_horizon;
    // avg_root_euler_in_horizon
    //         <<
    //         (state.root_euler[0] + 0) / (PLAN_HORIZON + 1),
    //         (state.root_euler[1] + 0) / (PLAN_HORIZON + 1),
    //         (state.root_euler[2] + yaw_ref_angle.sum()) / (PLAN_HORIZON + 1);
    //   //////// update Rz
    // Eigen::Matrix3d root_rot_mat_aver; 
    // root_rot_mat_aver = Utils::eulerAnglesToRotationMatrix(avg_root_euler_in_horizon); 
    Eigen::Matrix3d Ig_avg = root_rot_mat_aver * state.Go1_trunk_inertia * root_rot_mat_aver.transpose();
    Eigen::Matrix3d Ig_inv = Ig_avg.inverse();
    //state.root_rot_mat = root_rot_mat_aver;
    Eigen::Vector3d foot_position_to_com;
    Eigen::Matrix<double, 4,3> foot_poses_tem;
    Eigen::Matrix<double, 6, 12> A_average;
    for (int i = 0; i < mpc_horizon; i++)
    {
      //FR2COM
      foot_position_to_com = support_position.block<3,1>(12*(i),0) + lfoot.block<3,1>(0,i) - com_mpc_ref.block<3,1>(0,i) - body_p_Homing_Retarget;
      foot_poses_tem.row(0) = foot_position_to_com.transpose();
      ///FL2COM
      foot_position_to_com = support_position.block<3,1>(12*(i)+3,0) + rfoot.block<3,1>(0,i) - com_mpc_ref.block<3,1>(0,i) - body_p_Homing_Retarget;
      foot_poses_tem.row(1)  = foot_position_to_com.transpose();
      ///RR2COM
      foot_position_to_com = support_position.block<3,1>(12*(i)+6,0) + rfoot.block<3,1>(0,i) - com_mpc_ref.block<3,1>(0,i) - body_p_Homing_Retarget;
      foot_poses_tem.row(2)  = foot_position_to_com.transpose();
      ///RL2COM
      foot_position_to_com = support_position.block<3,1>(12*(i)+9,0) + lfoot.block<3,1>(0,i) - com_mpc_ref.block<3,1>(0,i) - body_p_Homing_Retarget;
      foot_poses_tem.row(3)  = foot_position_to_com.transpose(); 
      Eigen::MatrixXd pos(3,12);
      pos << (Eigen::Matrix3d() << 0, -foot_poses_tem(0, 2), foot_poses_tem(0, 1), 
                          foot_poses_tem(0, 2), 0, -foot_poses_tem(0, 0), 
                          -foot_poses_tem(0,1), foot_poses_tem(0, 0), 0).finished(),
          (Eigen::Matrix3d() << 0, -foot_poses_tem(1, 2), foot_poses_tem(1, 1), 
                          foot_poses_tem(1, 2), 0, -foot_poses_tem(1, 0), 
                          -foot_poses_tem(1,1), foot_poses_tem(1, 0), 0).finished(),
          (Eigen::Matrix3d() << 0, -foot_poses_tem(2, 2), foot_poses_tem(2, 1), 
                          foot_poses_tem(2, 2), 0, -foot_poses_tem(2, 0), 
                          -foot_poses_tem(2,1), foot_poses_tem(2, 0), 0).finished(),
          (Eigen::Matrix3d() << 0, -foot_poses_tem(3, 2), foot_poses_tem(3, 1), 
                          foot_poses_tem(3, 2), 0, -foot_poses_tem(3, 0), 
                          -foot_poses_tem(3,1), foot_poses_tem(3, 0), 0).finished();     
      A_average.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3)/state.robot_mass;
      A_average.block<3,3>(0,3) = Eigen::MatrixXd::Identity(3,3)/state.robot_mass;
      A_average.block<3,3>(0,6) = Eigen::MatrixXd::Identity(3,3)/state.robot_mass;
      A_average.block<3,3>(0,9) = Eigen::MatrixXd::Identity(3,3)/state.robot_mass;
      A_average.block<3,12>(3,0) = Ig_inv * pos;           
      A_predicted.block(i*6, 0, 6, 12) = A_average;
      int right_support_pre = (int) support_prediction(i,0);
      if(right_support_pre==0) ///left support:
      {
        no_contact_predicted(i,1) = 1;
        no_contact_predicted(i,2) = 1; 
        no_contact_predicted(i,0) = 0;
        no_contact_predicted(i,3) = 0;     
      }
      else
      {
        ////
        if(right_support_pre==1) ///right support
        {
          no_contact_predicted(i,0) = 1;
          no_contact_predicted(i,3) = 1; 
          no_contact_predicted(i,1) = 0;
          no_contact_predicted(i,2) = 0;     
        } 
        else
        {
          no_contact_predicted(i,0) = 0;
          no_contact_predicted(i,3) = 0; 
          no_contact_predicted(i,1) = 0;
          no_contact_predicted(i,2) = 0; 
        }   
      }
      if(i==0) //// for display
      {
        foot_poses_tem_ref = foot_poses_tem;
      }
    }
    A_pred_mpc = A_predicted; 
    // A_pred_mpc.block(0,0,6,12) = A; //// current A matrix; ????????????????????
    /// current state initialization ????????????? or global state? ///// 
    x_init << state.root_pos, state.root_euler, state.root_lin_vel, state.root_ang_vel;
    
    ///the com to the current support center help to increase the MPC stability, but may degrade the tracking performance///
    for (int i = 0; i < mpc_horizon; i++)
    {
      X_ref.block<3,1>(0,i) += body_p_Homing_Retarget;
      // X_ref.block<3,1>(0,i) -= (support_position.block<3,1>(0,0)+lfoot.block<3,1>(0,0)+support_position.block<3,1>(3,0)+rfoot.block<3,1>(0,0)+support_position.block<3,1>(6,0)+lfoot.block<3,1>(0,0)+support_position.block<3,1>(9,0)+rfoot.block<3,1>(0,0))/4;
    }
    ////// update contact_predicted in the rt loop 
    // contact prediction predicted;
    no_contact_mpc = no_contact_predicted; 
    /// ==== TBD judge if in the early contact ///
    // int right_support_pre = (int) support_prediction(0,0);
    // if(right_support_pre==1) ///left support:
    // {
    //   no_contact(1) = (1 and state_feedback(54,0));
    //   no_contact(2) = (1 and state_feedback(55,0)); 
    //   no_contact(0) = 0;
    //   no_contact(3) = 0;     
    // }
    // else
    // {
    // ////
    //   if(right_support_pre==0) ///right support
    //   {
    //     no_contact(0) = (1 and state_feedback(53,0));
    //     no_contact(3) = (1 and state_feedback(56,0)); 
    //     no_contact(1) = 0;
    //     no_contact(2) = 0;     
    //   } 
    //   else
    //   {
    //     no_contact(0) = 0;
    //     no_contact(3) = 0; 
    //     no_contact(1) = 0;
    //     no_contact(2) = 0; 
    //   }   
    // }
    // // cout<<"no_contact:"<<no_contact.transpose()<< endl;
    // for (size_t i = 0; i < 1; i++)
    // {
    //   no_contact_mpc.block(i,0,1,4) = no_contact.transpose();
    // }
    mpc_update_A(A_pred_mpc);
    mpc_update_contact_state(no_contact_mpc.transpose());
    mpc_update_X_ref(X_ref);
    mpc_update_x_init(x_init);
}

void mpc_solve(Eigen::Matrix<double, 12,1>& F_GRF){
  // Solve the problem instance
  cpg_set_solver_max_iter(200);
  cpg_solve();
  // Print objective function value
//   printf("obj = %f\n", CPG_Result.info->obj_val);
  // Print primal solution
  for(int i=0; i<12; i++) {
    if(! isnan(CPG_Result.prim->F[i]))
    {
      F_GRF(i,0) =  CPG_Result.prim->F[i];
    }
    else
    {
      if(i % 3 ==2)
      {
        F_GRF(i,0) = 30;
      }
      else
      {
        F_GRF(i,0) = 0;
      }    
    }
  } 
  // for (int i = 0; i < count; i++)
  // {
  //   /* code */
  // }
  // const double mu_friction = 0.7;
  F_GRF(2,0) =  std::min(std::max(F_GRF(2,0),0.0),160.0);
  F_GRF(0,0) =  std::min(std::max(F_GRF(0,0), -0.7* F_GRF(2,0)),0.7* F_GRF(2,0));
  F_GRF(1,0) =  std::min(std::max(F_GRF(1,0), -0.7* F_GRF(2,0)),0.7* F_GRF(2,0));
  F_GRF(5,0) =  std::min(std::max(F_GRF(5,0),0.0),160.0);
  F_GRF(3,0) =  std::min(std::max(F_GRF(3,0), -0.7* F_GRF(5,0)),0.7* F_GRF(5,0));
  F_GRF(4,0) =  std::min(std::max(F_GRF(4,0), -0.7* F_GRF(5,0)),0.7* F_GRF(5,0));
  F_GRF(8,0) =  std::min(std::max(F_GRF(8,0),0.0),160.0);
  F_GRF(6,0) =  std::min(std::max(F_GRF(6,0), -0.7* F_GRF(8,0)),0.7* F_GRF(8,0));
  F_GRF(7,0) =  std::min(std::max(F_GRF(7,0), -0.7* F_GRF(8,0)),0.7* F_GRF(8,0));
  F_GRF(11,0) =  std::min(std::max(F_GRF(11,0),0.0),160.0);
  F_GRF(9,0) =  std::min(std::max(F_GRF(9,0), -0.7* F_GRF(11,0)),0.7* F_GRF(11,0));
  F_GRF(10,0) =  std::min(std::max(F_GRF(10,0), -0.7* F_GRF(11,0)),0.7* F_GRF(11,0));  
}

void control_gait_sub_operation(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int jx = 0; jx<25; jx++)
    {
        control_gait(jx) = msg->position[jx]; 
    }
    
    /// state feedback
    for (int jx = 0; jx<53; jx++)
    {
        state_feedback(jx) = msg->position[jx]; 
    }  
    
    bodyangle_state(0) = state_feedback(10);
    bodyangle_state(1) = state_feedback(11);
    bodyangle_state(2) = state_feedback(13);
    bodyangle_state(3) = state_feedback(14);  

    for (int jx = 0; jx<37; jx++)
    {
        
        state_feedback_receieved(jx) = msg->position[jx]; 
        if (jx<18)
        {
            state_est(jx) = msg->position[jx+1];
        }

    }
    
    Lfoot_location_feedback(0) = state_feedback(19);
    Lfoot_location_feedback(1) = state_feedback(20);
    Lfoot_location_feedback(2) = state_feedback(21);
    
    Rfoot_location_feedback(0) = state_feedback(22);
    Rfoot_location_feedback(1) = state_feedback(23);
    Rfoot_location_feedback(2) = state_feedback(24); 

    support_leg_est = state_feedback_receieved.block<12,1>(25,0);
    
}


///////////////// keyboard controlller 
void step_parameters_callback(const geometry_msgs::Twist::ConstPtr &msgIn) {
    if(mpc_start =true)
    {
      if(currentstep>key_board_period+2)
      {
        step_length +=  0.01*msgIn->linear.x;   /// W: forward, S: backward
        step_width +=  0.01*msgIn->linear.y;    ///A: leftward, D:rightward       
        step_yaw += 0.05*M_PI*msgIn->linear.z;  ////Q: anti-clockwise, E:clockwise
        
      }


      if(abs(step_yaw-step_yaw_old)>0.001) ////update the yaw motion ////
      {
        step_length = 0;
        step_width = 0;

        key_board_period = currentstep; //// update the period
      }
      step_yaw_old = step_yaw;

      step_length = std::max(std::min(step_length,0.07), -0.07);
      step_width = std::max(std::min(step_width,0.05), -0.05);

    }

}





void keyboard_subscribe_sub_operation(const geometry_msgs::Twist::ConstPtr &msg)
{
    // printf("linear x: %f\n",msg->linear.x); 
    // printf("linear y: %f\n",msg->linear.y); 
}


void config_set()
{   
    /////load default parameter from the yaml.file
    ///////////////////  yaml code . ///////// 
    // YAML::Node config = YAML::LoadFile("/home/pi/go1_catkin/src/unitree_ros_to_real/unitree_legged_real/config/config.yaml");
    YAML::Node config = YAML::LoadFile("/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/config/config.yaml");

    dt_mpc =  config["dt_slow_mpc"].as<double>();
    test_mpc_grf = config["test_mpc_grf"].as<double>();
    //std::cout<<"dt_mpc:"<<dt_mpc<<std::endl;
    // footstepnumber = config["footnumber"].as<double>();
    _hcom = config["body_p_Homing_Retarget2"].as<double>();

    body_p_Homing_Retarget.setZero();
    body_p_Homing_Retarget(0,0)  = config["body_p_Homing_Retarget0"].as<double>();
    body_p_Homing_Retarget(1,0)  = config["body_p_Homing_Retarget1"].as<double>();
    //body_p_Homing_Retarget(2,0)  = config["body_p_Homing_Retarget2"].as<double>();
    
    body_r_Homing_Retarget.setZero();
    // body_r_Homing_Retarget(0,0)  = config["body_r_Homing_Retarget0"].as<double>();
    // body_r_Homing_Retarget(1,0)  = config["body_r_Homing_Retarget1"].as<double>();
    // body_r_Homing_Retarget(2,0)  = config["body_r_Homing_Retarget2"].as<double>();

}



int main(int argc, char *argv[])
{

    ros::init(argc,argv,"mpc");
    ros::NodeHandle nh;

    joint2simulation.position.resize(100);
    jointnlp_state.position.resize(100);
    jointmpc_grf.position.resize(100);
    state_est.setZero();
    state_feedback_receieved.setZero();
    nlp_gait_result.setZero();
    config_set();
    int count = 0;
    int count_old = 0;   
    double mpc_stop = 0;
    yaw_ref_ones.setConstant(1);

    mpc_gait_flag = 0;
    mpc_gait_flag_old = 0;
    com_mpc_ref.setZero();

    force_leg.setZero();
    force_leg(2,0) = force_leg(5,0) = force_leg(8,0) = force_leg(11,0) = gait::mass * gait::g /4;


    ros::Subscriber keyboard_subscribe_ = nh.subscribe<geometry_msgs::Twist>("/Robot_mode", 10,keyboard_subscribe_sub_operation);
    ros::Subscriber step_parameters_cmd = nh.subscribe("/Base_offset", 1, step_parameters_callback);    
    
    ros::Subscriber control_gait_subscribe_ = nh.subscribe<sensor_msgs::JointState>("/control2rtmpc/state", 10,control_gait_sub_operation); 

    ros::Publisher mpc_gait_pub_ = nh.advertise<sensor_msgs::JointState>("/MPC/Gait", 10);
    ros::Publisher grf_mpc_state_pub_ = nh.advertise<sensor_msgs::JointState>("/rtMPC/grfmpc", 10);
    ros::Publisher grf_mpc_grf_pub_ = nh.advertise<sensor_msgs::JointState>("/rtMPC/traj", 10);

    int pub_rate  = (int) (round(1/dt_mpc));

    ros::Rate nrt_rate(pub_rate);

    
    ros::Duration duratione_des(dt_mpc);
    while (ros::ok())
    {   
        ros::Time start = ros::Time::now(); 
        
        if (state_feedback_receieved(0) > 0) //////
        {
            
            mpc_start =true;
            count += 1;
            // cout<<"xxxxxxxxxxxx"<<endl;
        }

        ///// input: obtained from the ros::subscriber
        /////////// time interval
        /////////// estimated_statex
        /////////// _Rfoot_location_feedback / _Lfoot_location_feedback
        if ((mpc_stop <1))
        {
            //   nlp_gait_result = nlp_planner.WalkingReactStepping(count,mpc_start,state_est,Rfoot_location_feedback, Lfoot_location_feedback);
            nlp_gait_result = tvo_nlp_planner.WalkingReactStepping(count,mpc_start,state_est,Rfoot_location_feedback, Lfoot_location_feedback,step_length,step_width, step_yaw, support_leg_est); 
            mpc_gait_flag = nlp_gait_result(26,0);

            Nrtfoorpr_gen = nlp_gait_result.block<11,1>(86, 0);

            //////   ********************************************************************************* ////
            //// =============== Force MPC ============================///
            ////// reference trajectory from motion planner
            com_nlp_ref = tvo_nlp_planner.com_nlp_ref;
            comv_nlp_ref = tvo_nlp_planner.comv_nlp_ref;
            rfoot_mpc_ref = tvo_nlp_planner.rfoot_mpc_ref;
            lfoot_mpc_ref = tvo_nlp_planner.lfoot_mpc_ref;
            support_prediction = tvo_nlp_planner.support_prediction;

            yaw_mpc_ref = tvo_nlp_planner.yaw_mpc_ref;
            support_position_mpc_ref = tvo_nlp_planner.support_position_mpc_ref;
            right_support = (int) nlp_gait_result(99,0); ///// should be updated

            // cout<<"right_support:"<<right_support<<endl;
            /// desired average linear velocity;
            state.root_lin_vel_d[0] = (Nrtfoorpr_gen(2,0) - Nrtfoorpr_gen(1,0))/Nrtfoorpr_gen(8,0);
            state.root_lin_vel_d[1] = (Nrtfoorpr_gen(4,0) - Nrtfoorpr_gen(3,0))/Nrtfoorpr_gen(8,0);  
            state.root_lin_vel_d[2] = 0;
            com_mpc_ref = com_nlp_ref;
            comv_mpc_ref = comv_nlp_ref;

            if(test_mpc_grf > 0.5)////using the reference state. In this way, MPC works in the forward manner without feedback control
            {
              state.root_pos = com_nlp_ref.col(0);
              state.root_lin_vel = comv_nlp_ref.col(0);
              state.root_euler(2) = yaw_mpc_ref(0,0);
              state.root_ang_vel(2) = yaw_mpc_ref(1,0);
              state.root_lin_vel_d_world = state.root_lin_vel_d;
              
              Eigen::Vector3d foot_position_to_com_ref;
              //FR2COM
              foot_position_to_com_ref = support_position_mpc_ref.block<3,1>(12*(0),0) + lfoot_mpc_ref.block<3,1>(0,0) - com_mpc_ref.block<3,1>(0,0) - body_p_Homing_Retarget;
              foot_poses.row(0) = foot_position_to_com_ref.transpose();
              ///FL2COM
              foot_position_to_com_ref = support_position_mpc_ref.block<3,1>(12*(0)+3,0) + rfoot_mpc_ref.block<3,1>(0,0) - com_mpc_ref.block<3,1>(0,0) - body_p_Homing_Retarget;
              foot_poses.row(1)  = foot_position_to_com_ref.transpose();
              ///RR2COM
              foot_position_to_com_ref = support_position_mpc_ref.block<3,1>(12*(0)+6,0) + rfoot_mpc_ref.block<3,1>(0,0) - com_mpc_ref.block<3,1>(0,0) - body_p_Homing_Retarget;
              foot_poses.row(2)  = foot_position_to_com_ref.transpose();
              ///RL2COM
              foot_position_to_com_ref = support_position_mpc_ref.block<3,1>(12*(0)+9,0) + lfoot_mpc_ref.block<3,1>(0,0) - com_mpc_ref.block<3,1>(0,0) - body_p_Homing_Retarget;
              foot_poses.row(3)  = foot_position_to_com_ref.transpose(); 

              state_feedback.block<3,1>(37,0) = support_position_mpc_ref.block<3,1>(12*(0),0) + lfoot_mpc_ref.block<3,1>(0,0);
              state_feedback.block<3,1>(40,0) = support_position_mpc_ref.block<3,1>(12*(0)+3,0) + rfoot_mpc_ref.block<3,1>(0,0);
              state_feedback.block<3,1>(43,0) = support_position_mpc_ref.block<3,1>(12*(0)+6,0) + rfoot_mpc_ref.block<3,1>(0,0);
              state_feedback.block<3,1>(46,0) = support_position_mpc_ref.block<3,1>(12*(0)+9,0) + lfoot_mpc_ref.block<3,1>(0,0);



            }
            else
            {
              //// update state, using the weigaht state: notice that the relative com motion to the support center is used to here;              
              state_alpha= 0.8;

              state.root_pos[0] = ((1 - state_alpha)*state_feedback[1] + (state_alpha)*com_nlp_ref(0,0));
              state.root_pos[1] = ((1 - state_alpha)*state_feedback[4] + (state_alpha)*com_nlp_ref(1,0));
              state.root_pos[2] = ((1 - state_alpha)*state_feedback[7] + (state_alpha)*com_nlp_ref(2,0));
              state.root_lin_vel[0] = ((1 - state_alpha)*state_feedback[2] + (state_alpha)*comv_nlp_ref(0,0));
              state.root_lin_vel[1] = ((1 - state_alpha)*state_feedback[5] + (state_alpha)*comv_nlp_ref(1,0));
              state.root_lin_vel[2] = ((1 - state_alpha)*state_feedback[8] + (state_alpha)*comv_nlp_ref(2,0));
              state.root_euler = (1 - state_alpha)*state_feedback.block<3,1>(10,0);
              state.root_euler[2] += (state_alpha*yaw_mpc_ref(0,0));

              state.root_ang_vel = (1 - state_alpha)*state_feedback.block<3,1>(13,0);
              state.root_ang_vel[2] += (state_alpha*yaw_mpc_ref(1,0));

              state.root_lin_vel_d_world = state.root_lin_vel_d;

              ////current leg position to current current com /////
              Eigen::Vector3d foot_position_to_com_ref;
              //FR2COM
              foot_position_to_com_ref = support_position_mpc_ref.block<3,1>(12*(0),0) + lfoot_mpc_ref.block<3,1>(0,0) - com_mpc_ref.block<3,1>(0,0) - body_p_Homing_Retarget;
              foot_poses.row(0) = (state_alpha)*foot_position_to_com_ref.transpose();

              foot_poses.row(0) += (1 - state_alpha)*(state_feedback.block<3,1>(37,0) - state.root_pos).transpose(); //FRtoCoM
              ///FL2COM
              foot_position_to_com_ref = support_position_mpc_ref.block<3,1>(12*(0)+3,0) + rfoot_mpc_ref.block<3,1>(0,0) - com_mpc_ref.block<3,1>(0,0) - body_p_Homing_Retarget;
              foot_poses.row(1)  = (state_alpha)*foot_position_to_com_ref.transpose();
             
              foot_poses.row(1) += (1 - state_alpha)*(state_feedback.block<3,1>(40,0) - state.root_pos).transpose(); //FLtoCoM 
              ///RR2COM
              foot_position_to_com_ref = support_position_mpc_ref.block<3,1>(12*(0)+6,0) + rfoot_mpc_ref.block<3,1>(0,0) - com_mpc_ref.block<3,1>(0,0) - body_p_Homing_Retarget;
              foot_poses.row(2)  = (state_alpha)*foot_position_to_com_ref.transpose();
              
              foot_poses.row(2) += (1 - state_alpha)*(state_feedback.block<3,1>(43,0) - state.root_pos).transpose(); //RRtoCoM 
              ///RL2COM
              foot_position_to_com_ref = support_position_mpc_ref.block<3,1>(12*(0)+9,0) + lfoot_mpc_ref.block<3,1>(0,0) - com_mpc_ref.block<3,1>(0,0) - body_p_Homing_Retarget;
              foot_poses.row(3)  = (state_alpha)*foot_position_to_com_ref.transpose();                
              foot_poses.row(3) += (1 - state_alpha)*(state_feedback.block<3,1>(46,0) - state.root_pos).transpose(); //RLtoCoM 
                 
            }
            // --------------------- GRF COMPUTATION ----------------------------------
            ///////////// home-made MPC 
            // Compute the new dynamics based on the new states of the robot and save them internally to the landing controller
            mpc_compute_dynamics(state);
            // Update the parameters of the mpc solver///
            /////// -----------------
            mpc_update_parameters(state, com_mpc_ref, comv_mpc_ref, support_position_mpc_ref, yaw_mpc_ref, rfoot_mpc_ref, lfoot_mpc_ref, support_prediction);
            // Re-solve for the F vector
            // ros::Time t0_qp = ros::Time::now();
            //auto t0_qp = std::chrono::high_resolution_clock::now();
            mpc_solve(force_leg);
            ///// std::cout << force_leg(2,0) + force_leg(5,0) + force_leg(8,0) + force_leg(11,0) << std::endl;
        }

 
        




        
        currentstep = nlp_gait_result(93,0);
        mpc_stop = nlp_gait_result(97,0);

        ros::Time end = ros::Time::now();
	
        ros::Duration duration = end -start;

        double t_slow_mpc = duration.toSec() * 1000;	

        nlp_gait_result(98,0) = t_slow_mpc;

        for (int jx = 0; jx<100; jx++)
        {
          joint2simulation.position[jx] = nlp_gait_result(jx,0);
        }


        ///// save data for MPC computation /////////// 
        for(int jx =0; jx <3; jx++)
        {
          jointnlp_state.position[jx] = foot_poses(0,jx); /// FR
        }
        for(int jx =3; jx <6; jx++)
        {
          jointnlp_state.position[jx] = foot_poses(1,jx-3); /// FL
        }
        for(int jx =6; jx <9; jx++)
        {
          jointnlp_state.position[jx] = foot_poses(2,jx-6); /// RR
        }
        for(int jx =9; jx <12; jx++)
        {
          jointnlp_state.position[jx] = foot_poses(3,jx-9); /// RL
        }           

        for(int jx =12; jx <24; jx++)
        {
          jointnlp_state.position[jx] = x_init(jx-12,0); /// X_init
        } 

        for(int jx =24; jx <36; jx++)
        {
          jointnlp_state.position[jx] = X_ref(jx-24,0); /// X_ref:
        } 

        for(int jx =36; jx <39; jx++)
        {
          jointnlp_state.position[jx] = foot_poses_tem_ref(0,jx-36); /// FR
        }
        for(int jx =39; jx <42; jx++)
        {
          jointnlp_state.position[jx] = foot_poses_tem_ref(1,jx-39); /// FL
        }
        for(int jx =42; jx <45; jx++)
        {
          jointnlp_state.position[jx] = foot_poses_tem_ref(2,jx-42); /// RR
        }
        for(int jx =45; jx <48; jx++)
        {
          jointnlp_state.position[jx] = foot_poses_tem_ref(3,jx-45); /// RL
        } 


        /////// output reference gait trajectory in the prediction window/////
        jointnlp_state.position[48] = com_nlp_ref(0,2);
        jointnlp_state.position[49] = com_nlp_ref(1,2);
        jointnlp_state.position[50] = com_nlp_ref(2,2);
        jointnlp_state.position[51] = com_nlp_ref(0,9);
        jointnlp_state.position[52] = com_nlp_ref(1,9);
        jointnlp_state.position[53] = com_nlp_ref(2,9);        
        jointnlp_state.position[54] = com_nlp_ref(0,0);
        jointnlp_state.position[55] = com_nlp_ref(1,0);
        jointnlp_state.position[56] = com_nlp_ref(2,0);  

        jointnlp_state.position[48+9] = rfoot_mpc_ref(0,2);
        jointnlp_state.position[49+9] = rfoot_mpc_ref(1,2);
        jointnlp_state.position[50+9] = rfoot_mpc_ref(2,2);
        jointnlp_state.position[51+9] = rfoot_mpc_ref(0,9);
        jointnlp_state.position[52+9] = rfoot_mpc_ref(1,9);
        jointnlp_state.position[53+9] = rfoot_mpc_ref(2,9);        
        jointnlp_state.position[54+9] = rfoot_mpc_ref(0,0);
        jointnlp_state.position[55+9] = rfoot_mpc_ref(1,0);
        jointnlp_state.position[56+9] = rfoot_mpc_ref(2,0);  

        jointnlp_state.position[48+18] = lfoot_mpc_ref(0,2);
        jointnlp_state.position[49+18] = lfoot_mpc_ref(1,2);
        jointnlp_state.position[50+18] = lfoot_mpc_ref(2,2);
        jointnlp_state.position[51+18] = lfoot_mpc_ref(0,9);
        jointnlp_state.position[52+18] = lfoot_mpc_ref(1,9);
        jointnlp_state.position[53+18] = lfoot_mpc_ref(2,9);        
        jointnlp_state.position[54+18] = lfoot_mpc_ref(0,0);
        jointnlp_state.position[55+18] = lfoot_mpc_ref(1,0);
        jointnlp_state.position[56+18] = lfoot_mpc_ref(2,0);  

        jointnlp_state.position[48+27] = yaw_mpc_ref(3,0);
        jointnlp_state.position[49+27] = yaw_mpc_ref(27,0);
        jointnlp_state.position[50+27] = yaw_mpc_ref(0,0);
        // jointnlp_state.position[51+27] = yaw_mpc_ref(27,0);
        // jointnlp_state.position[52+27] = yaw_mpc_ref(28,0);
        // jointnlp_state.position[53+27] = yaw_mpc_ref(29,0);        
        // jointnlp_state.position[54+27] = yaw_mpc_ref(0,0);
        // jointnlp_state.position[55+27] = yaw_mpc_ref(1,0);
        // jointnlp_state.position[56+27] = yaw_mpc_ref(2,0);  


        for (int ji = 0; ji < 12; ji++)
        {
          jointmpc_grf.position[51+ji] = force_leg(ji,0); 
        }
        jointmpc_grf.position[63] = tvo_nlp_planner._bjx1_nlp;
        jointmpc_grf.position[64] = tvo_nlp_planner._bjx1_nlp_flag;
        jointmpc_grf.position[65] = tvo_nlp_planner._bjxx_nlp;
        jointmpc_grf.position[66] = tvo_nlp_planner._bjxx_nlp_flag;
        jointmpc_grf.position[67] = tvo_nlp_planner._period_i_nlp;
        jointmpc_grf.position[68] = tvo_nlp_planner._period_i_nlp_flag;
        

        
        joint2simulation.header.stamp = ros::Time::now();
        jointnlp_state.header.stamp = ros::Time::now();  
        jointmpc_grf.header.stamp = ros::Time::now(); 
  
        mpc_gait_pub_.publish(joint2simulation);
        grf_mpc_state_pub_.publish(jointnlp_state);
        grf_mpc_grf_pub_.publish(jointmpc_grf);
        //count_old = (int) state_feedback_receieved(0,0);
 
        ros::spinOnce(); 
        
	
	
	    //// for simulation /////
        nrt_rate.sleep();
        /* code */
    }

    


    return 0;
}