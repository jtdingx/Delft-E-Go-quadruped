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
#include "dynmics_compute.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


Dynamiccclass::Dynamiccclass()	: QPBaseClass()
{
    mass = 12;
    initial_matrix = 0;
    joint_torque.setZero();

    stance_kp = 10; 
    stance_kd = 1;
    swing_kp = 100;
    swing_kd = 5;
    gravity_compensate<< -0.80,   0.80, -0.80, 0.80, 
                                0,      0,     0,    0,
                                0,      0,     0,    0; 

    F_leg_ref.setZero();  
    F_leg_guess.setZero();
    ///QP initiallize
    int nVars = 12;
    int nEqCon = 12; 
    int nIneqCon = 24;
    resizeQP(nVars, nEqCon, nIneqCon);	
    
    
    qp_solution = true;    
    grf_opt.setZero();
    Q_goal.setZero();
    Q_goal1.setZero();
    q_goal.setZero();

    A_unit.setIdentity();
    B_unit.setIdentity();

    qp_alpha = 100000;
    qp_beta = 10;
    qp_gama = 100;
    fz_max = 160;
    mu = 0.25;
    x_offset = 0.000;    

    AA.setZero();
    bb.setZero();

    qp_H.setZero();
    qp_h.setZero();


    ///  0<=fz<=fz_max
    for(int i =0; i<4;i++)
    {
        qp_H(2*i,3*i+2) = -1;
        qp_H(2*i+1,3*i+2) = 1;
        qp_h(2*i+1,0) = fz_max;
    }

        // print(qp_L)
    //  -u f_z =< f_x <= u f_z
    for(int i =0; i<4;i++)
    {
        qp_H(8+2*i,3*i) = -1;
        qp_H(8+2*i,3*i+2) = -mu;
        qp_H(8+2*i+1,3*i) = 1;
        qp_H(8+2*i+1,3*i+2) = -mu;
    }
    for(int i =0; i<4;i++)
    {
        qp_H(16+2*i,3*i+1) = -1;
        qp_H(16+2*i,3*i+2) = -mu;
        qp_H(16+2*i+1,3*i+1) = 1;
        qp_H(16+2*i+1,3*i+2) = -mu; 
    }

}


Dynamiccclass::~Dynamiccclass()
{
}



Eigen::Matrix<double, 3,1> Dynamiccclass::compute_joint_torques(Eigen::Matrix<double, 3,3> Jaco, 
                                                                bool support_flag, 
                                                                Eigen::Matrix<double, 3,1> p_des,
                                                                Eigen::Matrix<double, 3,1> p_est,
                                                                Eigen::Matrix<double, 3,1> pv_des,
                                                                Eigen::Matrix<double, 3,1> pv_est,
                                                                int leg_number)
{   
    Eigen::Matrix<double, 3,1>  joint_torque_des;

    if (support_flag) //// swing foot
    {
       joint_torque_des = Jaco.transpose() * (swing_kp*(p_des - p_est) + swing_kd*(pv_des - pv_est)) + gravity_compensate.col(leg_number); 
    }
    else
    {
       //cout<<"xxxx"<<endl;
       joint_torque_des = -Jaco.transpose() * grf_opt.block<3,1>(leg_number*3,0) + gravity_compensate.col(leg_number);
       //cout<<"yyyy"<<endl;
    //    if (leg_number==0)
    //    {
    //       cout<<"stance F_leg_ref:" << F_leg_ref.col(leg_number).transpose()<<endl;
    //    } 
    }
    
    
    
    


    return joint_torque_des;
}


void Dynamiccclass::force_distribution(Eigen::Matrix<double, 3,1> com_des, 
                                       Eigen::Matrix<double, 12,1> leg_des, 
                                       Eigen::Matrix<double, 6,1> F_force_des, 
                                       int mode, double y_coefficient, double rfoot_des[3],double lfoot_des[3])
{
    double body_FR_dis = sqrt(pow(com_des(0)-leg_des(0),2) + pow(com_des(1)-leg_des(1),2) + pow(com_des(2)-leg_des(2),2));
    double body_FL_dis = sqrt(pow(com_des(0)-leg_des(3),2) + pow(com_des(1)-leg_des(4),2) + pow(com_des(2)-leg_des(5),2));
    double body_RR_dis = sqrt(pow(com_des(0)-leg_des(6),2) + pow(com_des(1)-leg_des(7),2) + pow(com_des(2)-leg_des(8),2));
    double body_RL_dis = sqrt(pow(com_des(0)-leg_des(9),2) + pow(com_des(1)-leg_des(10),2) + pow(com_des(2)-leg_des(11),2));

    double f_double;
    

    ///// F_leg_ref:  FR, FL, RR, RL ////////
    if (mode == 101) ///// bipedal 
    {
         /////// right two legs is the right leg, left two legs is the left leg
         f_double = F_force_des(0)* body_FL_dis/(body_FL_dis + body_RL_dis); /// L X;
         F_leg_ref(0,3) = f_double;
         F_leg_ref(0,1) = F_force_des(0) - f_double;
         
         f_double = F_force_des(1)* body_FL_dis/(body_FL_dis + body_RL_dis) *y_coefficient; /// L Y;
         F_leg_ref(1,3) = f_double;
         F_leg_ref(1,1) = F_force_des(1)*y_coefficient - f_double;        

         f_double = F_force_des(2)* body_FL_dis/(body_FL_dis + body_RL_dis); /// L Z;
         F_leg_ref(2,3) = f_double;
         F_leg_ref(2,1) = F_force_des(2) - f_double;


         f_double = F_force_des(3)* body_FR_dis/(body_FR_dis + body_RR_dis); /// R X;
         F_leg_ref(0,2) = f_double;
         F_leg_ref(0,0) = F_force_des(3) - f_double;          

         f_double = F_force_des(4)* body_FR_dis/(body_FR_dis + body_RR_dis) *y_coefficient; /// R y;
         F_leg_ref(1,2) = f_double;
         F_leg_ref(1,0) = F_force_des(4)*y_coefficient- f_double;  

         f_double = F_force_des(5)* body_FR_dis/(body_FR_dis + body_RR_dis); /// R Z;
         F_leg_ref(2,2) = f_double;
         F_leg_ref(2,0) = F_force_des(5) - f_double;           
    }
    else
    {
        if (mode == 102) ////troting   ///////// F_leg_ref:  FR, FL, RR, RL ////////
        {
            ///// FL,RR is the right leg, FR,RL is the left leg /////

            Eigen::Vector3d vec_foot_rl;
            vec_foot_rl << leg_des[9] - leg_des[0],
                           leg_des[10] - leg_des[1],
                           leg_des[11] - leg_des[2];

            Eigen::Vector3d vec_com_rfoot;
            vec_com_rfoot << lfoot_des[0] - leg_des[0],
                           lfoot_des[1] - leg_des[1],
                           lfoot_des[2] - leg_des[2];                          

            double rlleg_dis = sqrt(pow(vec_foot_rl[0], 2) + pow(vec_foot_rl[1], 2) + pow(vec_foot_rl[2], 2));
            double com_rleg_dis = vec_foot_rl[0]*vec_com_rfoot[0] + vec_foot_rl[1]*vec_com_rfoot[1] + vec_foot_rl[2]*vec_com_rfoot[2];
            double rleg_com_raw = com_rleg_dis /rlleg_dis;
            double rleg_com_raw1 = std::min(rleg_com_raw,1.0);
            double rleg_com = std::max(rleg_com_raw1,0.0);        


            f_double = F_force_des(0)* rleg_com; /// L X;
            F_leg_ref(0,3) = f_double;
            F_leg_ref(0,0) = F_force_des(0) - f_double;
            
            f_double = F_force_des(1)* rleg_com*y_coefficient; /// L Y;
            F_leg_ref(1,3) = f_double;
            F_leg_ref(1,0) = F_force_des(1)*y_coefficient - f_double;        

            f_double = F_force_des(2)* rleg_com; /// L Z;
            F_leg_ref(2,3) = f_double;
            F_leg_ref(2,0) = F_force_des(2) - f_double;

            //////////////////////////
            Eigen::Vector3d vec_foot_rlx;
            vec_foot_rlx << leg_des[6] - leg_des[3],
                           leg_des[7] - leg_des[4],
                           leg_des[8] - leg_des[5];

            Eigen::Vector3d vec_com_rfootx;
            vec_com_rfootx << rfoot_des[0] - leg_des[3],
                           rfoot_des[1] - leg_des[4],
                           rfoot_des[2] - leg_des[5];                          

            double rlleg_disx = sqrt(pow(vec_foot_rlx[0], 2) + pow(vec_foot_rlx[1], 2) + pow(vec_foot_rlx[2], 2));
            double com_rleg_disx = vec_foot_rlx[0]*vec_com_rfootx[0] + vec_foot_rlx[1]*vec_com_rfootx[1] + vec_foot_rlx[2]*vec_com_rfootx[2];
            double rleg_com_rawx = com_rleg_disx /rlleg_disx;
            double rleg_com_raw1x = std::min(rleg_com_rawx,1.0);
            double rleg_comx = std::max(rleg_com_raw1x,0.0); 

            f_double = F_force_des(3)* rleg_comx; /// R x;
            F_leg_ref(0,2) = f_double;
            F_leg_ref(0,1) = F_force_des(3) - f_double;          

            f_double = F_force_des(4)* rleg_comx*y_coefficient; /// R y;
            F_leg_ref(1,2) = f_double;
            F_leg_ref(1,1) = F_force_des(4)*y_coefficient - f_double;  

            f_double = F_force_des(5)* rleg_comx; /// R z;
            F_leg_ref(2,2) = f_double;
            F_leg_ref(2,1) = F_force_des(5) - f_double;           
        } 
        else  ////bounding
        {
           ////// not testing now ///////////////////
        } 


           
    }

    F_leg_guess.block<3,1>(0,0) = F_leg_ref.col(0);
    F_leg_guess.block<3,1>(3,0) = F_leg_ref.col(1);
    F_leg_guess.block<3,1>(6,0) = F_leg_ref.col(2);    
    F_leg_guess.block<3,1>(9,0) = F_leg_ref.col(3); 

}



void Dynamiccclass::force_opt(Eigen::Matrix<double, 3,1> base_p,
                              Eigen::Matrix<double, 3,1> FR_p, 
                              Eigen::Matrix<double, 3,1> FL_p, 
                              Eigen::Matrix<double, 3,1> RR_p, 
                              Eigen::Matrix<double, 3,1> RL_p, 
                              Eigen::Matrix<double, 6,1> FT_total_des, 
                              int mode, int right_support, double y_coefficient)
{
    ///note: AF = d_total
    Eigen::Matrix<double, 6,12> A;
    A.setZero();

    F_leg_guess.setZero();
    
    
    
    A.block<3,3>(0,0) = A_unit;
    A.block<3,3>(0,3) = A_unit;
    A.block<3,3>(0,6) = A_unit;
    A.block<3,3>(0,9) = A_unit;

    Eigen::Matrix<double, 3,1> com_fr = base_p - FR_p;
    Eigen::Matrix3d w_hat = skew_hat(com_fr);
    A.block<3,3>(3,0) = w_hat;

    Eigen::Matrix<double, 3,1> com_fl = base_p - FL_p;
    w_hat = skew_hat(com_fl);
    A.block<3,3>(3,3) = w_hat;

    Eigen::Matrix<double, 3,1> com_rr = base_p - RR_p;
    w_hat = skew_hat(com_rr);
    A.block<3,3>(3,6) = w_hat;

    Eigen::Matrix<double, 3,1> com_rl = base_p - RL_p;
    w_hat = skew_hat(com_rl);
    A.block<3,3>(3,9) = w_hat;

    Q_goal = 2 * (qp_alpha * A.transpose() * A + (qp_beta + qp_gama) * B_unit);
    Q_goal1 = (Q_goal.transpose() + Q_goal) /2.0;


    ///
    q_goal = -2 * (qp_alpha * A.transpose() * FT_total_des + qp_beta * F_leg_guess + qp_gama * grf_opt);
    


    ////////////////////////////// feasibility constraints ////////////////////////////////
    AA.setZero();
    bb.setZero();

    // cout<<right_support<<endl;

    if (mode==102) //troting gait pair:FRRL-left; pair:FLRR-right
    {
        if(right_support==0) // left support, right two legs zeros force
        {
            AA.block<3,3>(3,3) = A_unit;
            AA.block<3,3>(6,6) = A_unit;
        }
        else
        {
            if(right_support==1) // right support
            {
                AA.block<3,3>(0,0) = A_unit;
                AA.block<3,3>(9,9) = A_unit;
            }            
        }            
    }
    else
    {
        if(mode==101) /// pace gait pair:FL-RL-left; pair:FR-RR-right
        {
            if(right_support==0) // left support, right two legs zeros force
            {
                AA.block<3,3>(0,0) = A_unit;
                AA.block<3,3>(6,6) = A_unit;
            }
            else
            {
                if(right_support==1) // right support
                {
                    AA.block<3,3>(3,3) = A_unit;
                    AA.block<3,3>(9,9) = A_unit;
                }                
            }
  
        }
    }











    solve_grf_opt();

    if(!qp_solution)
    {
       grf_opt = F_leg_guess;
    }

    // return grf_opt;
    


} 

Eigen::Matrix3d Dynamiccclass::skew_hat(Eigen::Matrix<double, 3,1> vec_w)
{
   Eigen::Matrix3d w_hat;
   w_hat.setZero();
   w_hat << 0,           -vec_w[2,0],  vec_w[1,0],
            vec_w[2,0],            0, -vec_w[0,0],
            -vec_w[1,0],  vec_w[0,0],          0;

   return w_hat;
}

///// three model MPC solution :modified================================================================
void Dynamiccclass::solve_grf_opt()
{
  _G = Q_goal1;
  _g0 = q_goal;
  _X = grf_opt;

//   _CI.block<_Nt,_nh>(0,0) = _q_upx.transpose() * (-1);
//   _CI.block<_Nt,_nh>(0,_nh) = _q_lowx.transpose() * (-1);
//   _CI.block<_Nt,_nh>(0,2*_nh) = _q_upy.transpose() * (-1);
//   _CI.block<_Nt,_nh>(0,3*_nh) = _q_lowy.transpose() * (-1); 
//   _CI.block<_Nt,_nh>(0,4*_nh) = _t_upx.transpose() * (-1);
//   _CI.block<_Nt,_nh>(0,5*_nh) = _t_lowx.transpose() * (-1);
//   _CI.block<_Nt,_nh>(0,6*_nh) = _t_upy.transpose() * (-1);
//   _CI.block<_Nt,_nh>(0,7*_nh) = _t_lowy.transpose() * (-1);
// //   _CI.block<_Nt,_nh>(0,8*_nh) = _z_upx.transpose() * (-1);  ///results non-smooth ZMPy
// //   _CI.block<_Nt,_nh>(0,9*_nh) = _z_lowx.transpose() * (-1);
// //   _CI.block<_Nt,_nh>(0,10*_nh) = _z_upy.transpose() * (-1);
// //   _CI.block<_Nt,_nh>(0,11*_nh) = _z_lowy.transpose() * (-1);  

//   _ci0.block(0, 0,_nh,1) = _qq_upx;
//   _ci0.block(_nh, 0,_nh,1) = _qq_lowx;
//   _ci0.block(2*_nh, 0,_nh,1) = _qq_upy;
//   _ci0.block(3*_nh, 0,_nh,1) = _qq_lowy;
//   _ci0.block(4*_nh, 0,_nh,1) = _tt_upx; 
//   _ci0.block(5*_nh, 0,_nh,1) = _tt_lowx; 
//   _ci0.block(6*_nh, 0,_nh,1) = _tt_upy;
//   _ci0.block(7*_nh, 0,_nh,1) = _tt_lowy;  
// /*  _ci0.block(8*_nh, 0,_nh,1) = _zz_upx; 
//   _ci0.block(9*_nh, 0,_nh,1) = _zz_lowx; 
//   _ci0.block(10*_nh, 0,_nh,1) = _zz_upy;
//   _ci0.block(11*_nh, 0,_nh,1) = _zz_lowy; */ 
  
  _CI = - qp_H.transpose();
  _ci0 = qp_h;
  _CE = AA;
  _ce0 = bb;


  Solve();  

}




void Dynamiccclass::Solve()
{
// // min 0.5 * x G x + g0^T x
// // _s.t.
// // 		CE^T x + ce0 = 0
// // 		CI^T x + ci0 >= 0

		qp_solution = solveQP();
		if (qp_solution)
		{
		  grf_opt = _X;
		}

}




