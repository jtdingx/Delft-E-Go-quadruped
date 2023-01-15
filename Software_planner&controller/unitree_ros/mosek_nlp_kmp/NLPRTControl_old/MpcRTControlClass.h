/*****************************************************************************
MpcRTControlClass.h

Description:	Header file of MpcRTControlClass

@Version:	1.0
@Author:	Jiatao ding (jtdingx@gmail.com)
@Release:	Tue 27 Jun 2017 09:31:28 AM CEST
@Update:	Tue 27 Jun 2017 09:31:24 AM CEST
*****************************************************************************/
#pragma once
#include "NLP/MPCClass.h"
#include "/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/go1_rt_control/src/Robotpara/robot_const_para_config.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>

const double dt_mpc = 0.01;   // definition of sampling time of MPC solover

using namespace Eigen;
using namespace std;

class MpcRTControlClass
{
	public:
		MpcRTControlClass();
		~MpcRTControlClass(){};

		MPCClass mpc;

		Eigen::Matrix<double,100,1> WalkingReactStepping(int walkdtime, bool start_mpc ,Eigen::Matrix<double,18,1> estimated_statex,
															Eigen::Vector3d _Rfoot_location_feedbackx, Eigen::Vector3d _Lfoot_location_feedbackx);
	

		int _walkdtime_max, _wal_max;	
		int _refer_t_max;

		Eigen::Vector3d PelvisPos,LeftFootPosx,RightFootPosx, comxyzx, thetaxyx, Lfootxyzx, Rfootxyzx;
		Vector3d PelvisPosv,PelvisPosa;
		Matrix<double,9,1> body_thetax;  
		Vector3d zmp_ref;

		int j_count, bjx1;
        double tx, td;

		Matrix<double,18,1> mpc_rlfoot_traj; 
		Matrix<double,38,1> mpc_body; 
		Eigen::Matrix<double,9,1> CoM_squat;  

		int right_support;  		 

	private:
		void StartWalking();
		void StopWalking();
		
		/// step parameters reference
		double stepwidthinput,steplengthinput,stepheightinput;


		int _t_int;
		double _dtx;
		
		int _t_walkdtime_flag, _t_walkdtime_restart_flag;
		bool _stop_walking, _start_walking_again;
		double _ppx, _pix, _pdx,  _ppy,_piy, _pdy, _ppz,_piz,  _pdz, _ppthetax, _pdthetax, _pithetax,  _ppthetay, _pithetay,  _pdthetay,  _ppthetaz, _pithetaz,  _pdthetaz; 
		Eigen::VectorXd _error_com_position, _error_torso_angle;
		
		
		Eigen::VectorXd _flag_walkdtime;
		Eigen::VectorXd _stop_flag_walkdtime;
	
		Eigen::Vector3d optCoM;
		

		int _walkdtime1;	
		
		Eigen::MatrixXd _COM_IN, _COM_est;
		Eigen::MatrixXd _body_IN;	
		Eigen::MatrixXd _FootR_IN;	
		Eigen::MatrixXd _FootL_IN;	

		Eigen::Matrix<double,18,1> _estimated_state;	
		Eigen::MatrixXd _estimated_state_global;
		Eigen::Vector3d _Rfoot_location_feedback,_Lfoot_location_feedback;

		Eigen::MatrixXd _state_generate_interpo;




	protected:
		int t_walkdtime_flag;
		int dt_sample;
		
		Vector3d _torso_angle;
		void torso_angle(int arg1);
		
		double _feedback_lamda;
		bool IsStartWalk;

		double RobotPara_totalmass;
		double RobotPara_HALF_HIP_WIDTH;
		double RobotPara_dt;
		double RobotPara_Tstep;
		double RobotPara_Z_C;
		double RobotPara_g;
		double RobotPara_FOOT_WIDTH;


		Eigen::Vector3d COM_in1, COM_in2, COM_in3;
		Eigen::Vector3d body_in1, body_in2, body_in3;
		Eigen::Vector3d FootL_in1, FootL_in2, FootL_in3;
		Eigen::Vector3d FootR_in1, FootR_in2, FootR_in3;


	/****KMP based trajectory***********************/
	Eigen::Matrix<double,6,1> _kmp_leg_traje;	
	
    Vector3d LeftFootRPY,RightFootRPY;
    Eigen::Matrix<double,18,1> leg_rpy;
	
    Vector3d dcm_ref;	
    
    double mpc_stop;				
};




