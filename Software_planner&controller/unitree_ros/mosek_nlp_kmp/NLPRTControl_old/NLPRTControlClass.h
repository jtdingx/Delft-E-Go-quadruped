/*****************************************************************************
NLPRTControlClass.h

Description:	Header file of NLPRTControlClass

*****************************************************************************/
#pragma once
// #include "NLP/NLPClass.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>
#include "/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/go1_rt_control/src/Robotpara/robot_const_para_config.h"

const double dt_nlp = 0.01;   // 
const double _height_offsetx =  0.00;  
const double _height_offset_time = 1;  

using namespace Eigen;
using namespace std;

class NLPRTControlClass
{
public:
	NLPRTControlClass();
	~NLPRTControlClass(){};

	NLPClass nlp;


	void StartWalking();
	void StopWalking();
	
	void rt_nlp_gait(Eigen::Matrix<double,18,1> estimated_statex, Eigen::Vector3d _Rfoot_location_feedbackx, 
                     Eigen::Vector3d _Lfoot_location_feedbackx);

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
	
	int _walkdtime_max, _wal_max;	
	int _walkdtime1;	
	
	Eigen::MatrixXd _COM_IN;
	Eigen::MatrixXd _body_IN;	
	Eigen::MatrixXd _FootR_IN;	
	Eigen::MatrixXd _FootL_IN;	

	Eigen::Matrix<double,18,1> _estimated_state;	
	Eigen::MatrixXd _estimated_state_global;
	Eigen::Vector3d _Rfoot_location_feedback,_Lfoot_location_feedback;
	
	
	
	
	Eigen::Matrix<double,100,1> _state_generate_interpo;
	

	Eigen::Matrix<double,100,1> WalkingReactStepping(int walkdtime, bool start_mpc ,Eigen::Matrix<double,18,1> estimated_statex, 
                                                     Eigen::Vector3d _Rfoot_location_feedback, Eigen::Vector3d _Lfoot_location_feedback);

	
	Eigen::Vector3d COM_in1, COM_in2, COM_in3;
	Eigen::Vector3d body_in1, body_in2, body_in3;
	Eigen::Vector3d FootL_in1, FootL_in2, FootL_in3;
	Eigen::Vector3d FootR_in1, FootR_in2, FootR_in3;	
    
    
    bool IsStartWalk;
    Eigen::Vector3d PelvisPos,LeftFootPosx,RightFootPosx, comxyzx, thetaxyx, Lfootxyzx, Rfootxyzx;
    Vector3d PelvisPosv,PelvisPosa;
    Matrix<double,9,1> body_thetax;   
    
    
    double RobotPara_totalmass;
    double RobotPara_HALF_HIP_WIDTH;
    double RobotPara_dt;
    double RobotPara_Tstep;
    double RobotPara_Z_C;
    double RobotPara_g;
    double RobotPara_FOOT_WIDTH;    
    Eigen::Vector3d F_R,F_L, M_R,M_L, zmp_ref,ZMPxy_realx;
    int j_count, bjx1;
    double tx, td;
    
    Matrix<double,18,1> mpc_rlfoot_traj; 
    Matrix<double,38,1> mpc_body; 
    Eigen::Matrix<double,9,1> CoM_squat;  

	int right_support;  
    
    
protected:
	
	double _feedback_lamda;	
	
	
	Eigen::Vector3d _F_r_nlp, _F_l_nlp,_M_r_nlp,_M_l_nlp;
	Eigen::Vector4d _ZMP_relax_nlp;
	
	Eigen::Vector3d dob_comx, dob_comy, dob_comz,dob_thetax,dob_theaty, dob_hip_pos_ac;
	Eigen::Matrix<double,5,1> dob_dist;


	Eigen::Vector3d ekf_comx, ekf_comy, ekf_comz, ekf_dist;	
	/****KMP based trajectory***********************/
	Eigen::Matrix<double,6,1> _kmp_leg_traje;	
	
    Vector3d LeftFootRPY,RightFootRPY;
    Eigen::Matrix<double,18,1> leg_rpy;
	
    Vector3d dcm_ref;	
    
    double mpc_stop;
};

