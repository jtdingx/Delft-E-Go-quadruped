/**
PRMPCClass.h

Description:	Header file of PRMPCClass
*/
#pragma once

#include "/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/go1_rt_control/src/QP/QPBaseClass.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>
#include <vector>  

#include <vector> 
#include "/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/go1_rt_control/src/Robotpara/robot_const_para_config.h"

#include "KMP/kmp.h"
#include <armadillo>



using namespace Eigen;
using namespace std;
using namespace arma;


/// constant variable defintion
const int _footstepsnumber = 30;       //  number of _footstepnumber: for demo generation// walking on the flat ground

const int _nh = 4;                     /// length of the prediction window  	
const int _nsum = _nh;                   
const int _Nt = 2*_nh;                 /// the number of the variable of the optimization problem
const double _height_offset_time = gait::height_offset_time; 

class PRMPCClass : public QPBaseClass
{
	public:
		PRMPCClass();
		~PRMPCClass(){};

	
		/******************* KMP class preparation **************/
		kmp kmp_leg_L;
		kmp kmp_leg_R;

		int    _inDim_kmp; 	      		    //input dimension
		int    _outDim_kmp; 	      		    //output dimension
		int    _pvFlag_kmp;			    // output: pos (and vel)
		///////////// adjust KMP parameters
		double _lamda_kmp, _kh_kmp;	    	    //set kmp parameters 
		vec    _query_kmp;            	    // input
		vec    _mean_kmp;  	            // output:mean
		mat    _data_kmp;
			  
		
		void config_set();		
		//// robot parameters
		double _RobotPara_Z_C, _RobotPara_G, _RobotPara_FOOT_LENGTH, _RobotPara_FOOT_WIDTH, _RobotPara_HIP_TO_ANKLE_X_OFFSET, _RobotParaClass_HALF_HIP_WIDTH;
                std::string Robot_name;


		void Initialize();
		
		Eigen::Matrix<double, 14, 1> body_theta_mpc(int i, Eigen::Matrix<double,4,1> bodyangle_state, Eigen::Matrix<double,2,5> zmp_mpc_refx, 
		Eigen::Matrix<double,2,5> bodyangle_mpc_refx, Eigen::Matrix<double,2,5> rfoot_mpc_refx, Eigen::Matrix<double,2,5> lfoot_mpc_refx,
		Eigen::Matrix<double,3,5> comacc_mpc_refx, Eigen::Matrix<double, 9, 1> Nrtfoorpr_gen);
		
		void solve_body_rotation(); 	
		
		void Indexfind(double goalvari, int xyz);

		Eigen::MatrixXd Matrix_ps(Eigen::Matrix<double,2,2> a, int nh, Eigen::RowVector2d cxps);
		Eigen::MatrixXd Matrix_pu(Eigen::Matrix<double,2,2> a, Eigen::Matrix<double,2,1> b, int nh, Eigen::RowVector2d cxpu);		
		void Solve();

		// current state based on the past one and two actual sampling time;
		Eigen::Matrix<double, 9, 1> XGetSolution_position(const int walktime, const double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_ref, Eigen::Vector3d bodyv_ref);
		Eigen::Matrix<double, 9, 1> XGetSolution_position_mod(const int walktime, const double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_ref, Eigen::Vector3d body_ref2);
		Eigen::Matrix<double, 21, 1> XGetSolution_position_mod1(const int walktime, const double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_ref, Eigen::Vector3d bodyv_ref);
		Eigen::Matrix<double, 9, 1> XGetSolution_position_mod2(const int walktime, const double dt_sample, Eigen::Vector3d body_in2, Eigen::Vector3d body_ref);
        Eigen::Matrix<double, 21, 1> XGetSolution_position_mod3(const int walktime, const double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_ref, Eigen::Vector3d body_ref2);

		
		std::string _robot_name;
		double _robot_mass;

		//////////////////////////////// substitute the 
		int _j_period;

		Eigen::Matrix<double,_footstepsnumber,1> _tx;	
		Eigen::Matrix<double,_footstepsnumber,1> _ts, _td;
		double _tx_total;
		double _tdsp_ratio;

	////////// generate the COM trajectory during the DSP
		int _nstepx, _nsum_mpc;
		int _bjx1, _bjx2;
		
		//// foot trajectory interpolaton
		Eigen::Matrix<double, 30, 1> Foot_trajectory_solve_mod1(int j_index,bool _stopwalking,Eigen::Matrix<double, 9, 1> Nrtfoorpr_gen);
		Eigen::Matrix<double, 30, 1> Foot_trajectory_solve_mod2(int j_index,bool _stopwalking,Eigen::Matrix<double, 9, 1> Nrtfoorpr_gen);
		Eigen::Matrix<double, 30, 1> XGetSolution_Foot_rotation(int walktime, double dt_sample);
		Eigen::Matrix<double,3,_footstepsnumber> _footxyz_real;
		//parameters declaration  	
		Eigen::Matrix<double,_footstepsnumber,1> _steplength, _stepwidth, _stepheight,_lift_height_ref;	
		Eigen::Matrix<double,_footstepsnumber,1> _footx_ref, _footy_ref, _footz_ref;        
		void FootStepInputs(double stepwidth, double steplengthx, double stepheight, double _lift_height);
		double _bjxx;
		int _t_end_footstep;
		Eigen::Matrix<double, 1,10> _Lfootx, _Lfooty,_Lfootz, _Lfootvx, _Lfootvy,_Lfootvz, _Lfootax, _Lfootay,_Lfootaz;
		Eigen::Matrix<double, 1,10> _Rfootx, _Rfooty,_Rfootz, _Rfootvx, _Rfootvy,_Rfootvz, _Rfootax, _Rfootay,_Rfootaz; 
		double _ry_left_right;
		
		Eigen::Matrix<double, 4, 4> solve_AAA_inv2(Eigen::Vector3d t_plan);  
		Eigen::Matrix<double, 5, 5> solve_AAA_inv_x_coma(Eigen::Vector3d t_plan);
		
		Eigen::Matrix<double, 3, 5> _Lfoot_r, _Rfoot_r,_Lfoot_rv, _Rfoot_rv,_Lfoot_ra, _Rfoot_ra;
		double _footx_max, _footx_min,_footy_max, _footy_min;		
		
		Eigen::Matrix<double,6,1> XGetSolution_Foot_position_KMP(int walktime, double dt_sample);
		Eigen::Matrix<double,6,1> XGetSolution_Foot_position_KMP_faster(int walktime, double dt_sample);		
			
		  
		/////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		//// for kMP swing generation: private variable for leg status storage	
		/////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		
		Eigen::Matrix<double, 1,1> _Lfootx_kmp, _Lfooty_kmp,_Lfootz_kmp, _Lfootvx_kmp, _Lfootvy_kmp,_Lfootvz_kmp;
		Eigen::Matrix<double, 1,1> _Rfootx_kmp, _Rfooty_kmp,_Rfootz_kmp, _Rfootvx_kmp, _Rfootvy_kmp,_Rfootvz_kmp;

			  


	protected:
		Eigen::Matrix<double,1,_nsum> _zmpx_real, _zmpy_real;
		Eigen::Matrix<double,1,_nsum> _thetax, _thetavx, _thetaax;
		Eigen::Matrix<double,1,_nsum> _thetay, _thetavy, _thetaay;	
		Eigen::Matrix<double,1,_nsum> _thetaz, _thetavz, _thetaaz;	
		Eigen::Matrix<double,1,_nsum> _torquex_real, _torquey_real;
        Eigen::Matrix<double,1,_nsum> _thetax_real, _thetay_real;


		// initial parameters for MPC
		Eigen::Matrix<double,1,1> _ggg;	
		
		// CoM+angular momentum state and contro input
		Eigen::Matrix<double,2,1> _xk,_yk,_zk,_thetaxk,_thetayk;
		Eigen::Matrix<double,1,1> _x_vacc_k,_y_vacc_k,_z_vacc_k,_thetax_vacc_k,_thetay_vacc_k;
		
		
		Eigen::Matrix<double,2,2> _a;
		Eigen::Matrix<double,2,1> _b;
		Eigen::RowVector2d _cp,_cv;
			
		//predictive model
		Eigen::Matrix<double,_nh,2> _pps,_pvs;	
		Eigen::Matrix<double,_nh,_nh> _ppu,_pvu;		
		Eigen::Matrix<double,_nh,_nh> _ppu_2, _pvu_2;
		Eigen::Matrix<double,_nh,_nh> _pthetax,_pthetay;

		Eigen::Matrix<double,_nh,1> _det_px_ref,_det_py_ref;
		
		int xyz1;  //flag for find function 
		int xyz2;

		double _mass,  _rad,  _j_ini;
		
		// angle range
		double _thetax_max,  _thetax_min,  _thetay_max, _thetay_min;
		Eigen::Matrix<double, _nh,1> _thetax_max_vec,  _thetax_min_vec,  _thetay_max_vec, _thetay_min_vec;	
		// torque range
		double _torquex_max, _torquex_min, _torquey_max, _torquey_min; 
		Eigen::Matrix<double, _nh,1> _torquex_max_vec, _torquex_min_vec, _torquey_max_vec, _torquey_min_vec;
		

		// zmp range
		double _zmpx_max, _zmpx_min, _zmpy_max, _zmpy_min; 
		Eigen::Matrix<double, _nh,1> _zmpx_max_vec, _zmpx_min_vec, _zmpy_max_vec, _zmpy_min_vec;

		// solution preparation
		Eigen::Matrix<double, _Nt,1> _V_ini;		
	
		Eigen::Matrix<double, _nh,1> _copx_center_ref, _copy_center_ref,_comz_center_ref, _thetax_center_ref, _thetay_center_ref; 	
		
		
		// weight coefficients
		double _Rthetax, _Rthetay;
		double _alphathetax, _alphathetay;
		double _beltathetax, _beltathetay;
		double _gama_zmpx, _gama_zmpy;
		

		Eigen::Matrix<double,_nh,_nh> A_unit;	

		Eigen::Matrix<double,_nh,1> _t_f;
		
		int _n_vis, xxx, xxx1,xxx2;

		// qp model	
		Eigen::Matrix<double,_nh,_nh> _WX, _WY, _WZ, _WthetaX, _WthetaY;
		Eigen::Matrix<double,_Nt,_Nt> _Q_goal;
		Eigen::Matrix<double,_Nt,1> _q_goal;	

		Eigen::Matrix<double,_nh,_Nt> _Sjthetax,_Sjthetay;
		

		// zmp constraints
		Eigen::Matrix<double,_nh,_Nt> _z_upx,_z_lowx,_z_upy,_z_lowy;
		Eigen::Matrix<double,_nh,1> _zz_upx,_zz_lowx,_zz_upy,_zz_lowy;

		// angle constraints
		Eigen::Matrix<double,_nh,_Nt> _q_upx,_q_lowx,_q_upy,_q_lowy;
		Eigen::Matrix<double,_nh,1> _qq_upx,_qq_lowx,_qq_upy,_qq_lowy;

		
		Eigen::Matrix<double,_nh,_Nt> _t_upx,_t_lowx,_t_upy,_t_lowy;
		Eigen::Matrix<double,_nh,1> _tt_upx,_tt_lowx,_tt_upy,_tt_lowy;
		
		///////////////for polynomial intepolation
		Eigen::Matrix<double, 4,4>  _AAA_inv,_AAA_inv_mod;
		
		Eigen::Matrix<double, 3, 3> solve_AAA_inv(Eigen::Matrix<double, 4, 1> t_plana);
		void solve_AAA_inv1();
		Eigen::Matrix<double, 3, 3> solve_AAA_inv_mod(Eigen::Matrix<double, 4, 1> t_plan);
		void solve_AAA_inv_mod1();

		Eigen::Matrix<double, 7, 7> solve_AAA_inv_x(Eigen::Vector3d t_plan); 

		Eigen::Matrix<double, 6, 6> solve_AAA_inv_x_mod(Eigen::Vector3d t_plan); 
        
        bool qp_solution;

		double _dt;  //sampling time
		double _dt_mpc = gait::dt_mpc_fast;  //sampling time
        double _tstep;   ///time duratio ot the previous MPC

		double stepwidth;
		double steplengthx;
		double stepheight;
		double _lift_height; //// for bipedal walking and troting

		int sign_function(double x);

};
