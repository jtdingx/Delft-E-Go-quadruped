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
// #include "/home/pi/go1_catkin/src/unitree_ros_to_real/unitree_legged_real/src/go1_rt_control/src/Robotpara/robot_const_para_config.h"
#include "/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/go1_rt_control/src/Robotpara/robot_const_para_config.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>


using namespace Eigen;
using namespace std;

class MpcRTControlClass
{
	public:
		MpcRTControlClass();
		~MpcRTControlClass(){};

		MPCClass mpc;

		Eigen::Matrix<double,100,1> WalkingReactStepping(int walkdtime, bool start_mpc ,Eigen::Matrix<double,18,1> estimated_statex,
														Eigen::Vector3d _Rfoot_location_feedbackx, Eigen::Vector3d _Lfoot_location_feedbackx,
														double step_length, double step_width);
	

		int _walkdtime_max, _wal_max;	
		int _refer_t_max;
		int n_time_set;

		Eigen::Vector3d PelvisPos,LeftFootPosx,RightFootPosx, comxyzx, thetaxyx, Lfootxyzx, Rfootxyzx;
		Eigen::Vector3d PelvisPos_old,LeftFootPosx_old,RightFootPosx_old;
		Eigen::Vector3d zmp_old, dcm_old;
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
		void config_set();
		
		/// step parameters reference
		double stepwidthinput,steplengthinput,stepheightinput;


		int _t_int;
		double _dtx;
		
		int _t_walkdtime_flag, _t_walkdtime_restart_flag;
		bool _stop_walking, _start_walking_again;
		double _ppx, _pix, _pdx,  _ppy,_piy, _pdy, _ppz,_piz,  _pdz, _ppthetax, _pdthetax, _pithetax,  _ppthetay, _pithetay,  _pdthetay,  _ppthetaz, _pithetaz,  _pdthetaz; 
	
		Eigen::Vector3d optCoM;
		

		int _walkdtime1;	


		Eigen::Matrix<double,18,1> _estimated_state;	
		Eigen::Vector3d _Rfoot_location_feedback,_Lfoot_location_feedback;

		Eigen::Matrix<double,100,1> _state_generate_interpo;




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
		int gait_mode;
        double dt_mpc;   // definition of sampling time of MPC solover
        double clear_height;

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




