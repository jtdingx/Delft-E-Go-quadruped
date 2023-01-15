/**
MPCClass.h

Description:	Header file of MPCClass

@Version:	1.0
@Author:	Jiatao ding (jtdingx@gmail.com)
@Release:	Thu 02 Aug 2018 11:53:47 AM CEST
@Update:	Thu 02 Aug 2018 11:53:41 AM CEST
*/
#pragma once

//#include "QP/QPBaseClass.h"
#include "/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/src/go1_rt_control/src/QP/QPBaseClass.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>
#include "/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/src/go1_rt_control/src/Robotpara/robot_const_para_config.h"
// #include "/home/jiatao/unitree_loco_catk/src/unitree_ros/go1_rt_control/src/Robotpara/robot_const_para_config.h"

using namespace Eigen;
using namespace std;

// constant variable defintion
// const int _footstepsnumber = 30;       //  number of _footstepnumber
// const int _nh = 30;                    /// =PreviewT/_dt: number of sampling time for predictive window: <= 2*_nT; (_dt defined in MpcRTControlClass.h: dt_mpc)  	
// const int _nT = round(_tstep/_dt);      /// _tstep/_dt)  the number of one step cycle
// const int _nstep = 2;                   /// maximal footstep locations where the predictive windwo covers
// const int _Nt = 5*_nh + 3*_nstep;       /// _Nt = 5*_nh + 3*_nstep;  the number of the variable of the optimization problem
// const int _nsum = (_footstepsnumber-1)*_nT;
const int _nsum = 1800;


class MPCClass : public QPBaseClass
{
public:
	MPCClass();
	virtual ~MPCClass() {};

	bool solve_true;
	int nsum_x;
	
	//void FootStepNumberInputs(int footstepsnumber);
	void FootStepInputs(double stepwidth, double steplength, double stepheight);	
	void Initialize();
	int _gait_mode;
	void Re_Initialize(double step_length_ref, double step_width_ref,Eigen::Matrix<double,18,1> estimated_state);
	double sx_mod, sy_mod;
    void config_set();
	////////////////=================================================//////////////////
	/// for step timing optimization
	Eigen::Matrix<double, 40, 1> step_timing_opti_loop(int i,Eigen::Matrix<double,18,1> estimated_state, 
													   Eigen::Vector3d _Rfoot_location_feedback, Eigen::Vector3d _Lfoot_location_feedback,
													   double lamda, bool _stopwalking, int _t_walkdtime,int _t_walkdtime_old,
													   double step_length_ref, double step_width_ref);
	void Indexfind(double goalvari, int xyz);
	void solve_stepping_timing(); 		

	double _lamda_comx;
	double _lamda_comvx;
	double _lamda_comy;
	double _lamda_comvy; 
	
	// /// for height +angular momentum + step optimization
	// Eigen::MatrixXd Matrix_ps(Eigen::MatrixXd a, int nh, Eigen::MatrixXd cxps);
	// Eigen::MatrixXd Matrix_pu(Eigen::MatrixXd a, Eigen::MatrixXd b, int nh, Eigen::MatrixXd cxpu);
	
	///////////////=================================================/////////////////////////////
	
	void Solve();
	
	////// for trajectory intepolation
	Eigen::Matrix<double, 18, 1> Foot_trajectory_solve(int j_index, bool _stopwalking);
	int Get_maximal_number(double dtx);	
	int Get_maximal_number_reference();

	void CoM_height_solve(int j_index, bool _stopwalking, int ntdx);
	Eigen::Matrix<double, 18, 1> XGetSolution_Foot_rotation(const int walktime, const double dt_sample,int j_index);
	int right_support;
	
	// current state based on the past one and two actual sampling time;
	Vector3d XGetSolution_CoM_position(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3);
	Vector3d XGetSolution_Foot_positionR(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3);
	Vector3d XGetSolution_Foot_positionL(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3);	
	Vector3d XGetSolution_body_inclination(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3);		
	
	
	void File_wl_steptiming();	
	
	
	int _n_loop_omit;	
	int _n_loop_stop;	
	int _n_sum_stop;
	int _j_period;
	int _j_count;
	// Eigen::Matrix<double,_footstepsnumber,1> _tx;		///store the the start time of each walkng cycle
	Eigen::VectorXd _tx;		///store the the start time of each walkng cycle
	
	
	int _method_flag;
	std::string _robot_name;
	double _robot_mass;
	double _lift_height;	

    int _nTdx;

// private:    
        /// update objective function and constraints matrices for step timing optimization
	void step_timing_object_function(int i);  
  
	void step_timing_constraints(int i);  
	
	
	//parameters declaration  
	// all the variable for step timing optimization
	//Eigen::Matrix<double,_footstepsnumber,1> _steplength, _stepwidth, _stepheight,_lift_height_ref;
	//Eigen::Matrix<double,_footstepsnumber,1> _Lxx_ref, _Lyy_ref;  ///reference steplengh and stepwidth
	//Eigen::Matrix<double,_footstepsnumber,1> _footx_ref, _footy_ref, _footz_ref;    //footstep locations generation
	//Eigen::Matrix<double,_footstepsnumber,1> _footx_offline, _footy_offline, _footz_offline;    //footstep locations generation
	Eigen::VectorXd _steplength, _stepwidth, _stepheight,_lift_height_ref;
	Eigen::VectorXd _Lxx_ref, _Lyy_ref;  ///reference steplengh and stepwidth
	Eigen::VectorXd _footx_ref, _footy_ref, _footz_ref;    //footstep locations generation
	Eigen::VectorXd _footx_offline, _footy_offline, _footz_offline;    //footstep locations generation

        
	//Eigen::Matrix<double,_footstepsnumber,1> _ts, _td;  //time period of single support and double support
	Eigen::VectorXd _ts, _td;  //time period of single support and double support		 
	Eigen::Matrix<double,_nsum,1> _t;	            ///whole sampling time sequence

	Eigen::Matrix<double,1,_nsum> _comx, _comvx, _comax;
	Eigen::Matrix<double,1,_nsum> _comy, _comvy, _comay;
	Eigen::Matrix<double,1,_nsum> _comz, _comvz, _comaz;
	Eigen::Matrix<double,1,_nsum> _Lxx_ref_real, _Lyy_ref_real,_Ts_ref_real; 
	
	/// para
	double _hcom;
	double _g;	
	double _Wn;
	double _Wndt;
	
	Eigen::Matrix<double,1,_nsum> _px, _py, _pz;
	Eigen::Matrix<double,1,_nsum> _zmpvx, _zmpvy;
	// Eigen::Matrix<double,1,_footstepsnumber> _COMx_is, _COMx_es, _COMvx_is;
	// Eigen::Matrix<double,1,_footstepsnumber> _COMy_is, _COMy_es, _COMvy_is;
	Eigen::RowVectorXd _COMx_is, _COMx_es, _COMvx_is;
	Eigen::RowVectorXd _COMy_is, _COMy_es, _COMvy_is;
	Eigen::Matrix<double,1,_nsum> _comx_feed, _comvx_feed,_comax_feed;
	Eigen::Matrix<double,1,_nsum> _comy_feed, _comvy_feed,_comay_feed;
        
	//optimal variables
	Eigen::Matrix<double,8,1> _Vari_ini;
	Eigen::Matrix<double,8,1> _vari_ini;

	// weight coefficient	
	double _aax,_aay,_aaxv,_aayv,_bbx,_bby,_rr1,_rr2;
	double _aax1,_aay1,_bbx1,_bby1,_rr11,_rr21;
	
	//constraints on step timing parameters
	double _t_min, _t_max;
	
	// swing foot velocity constraints  parameters
	double _footx_vmax, _footx_vmin,_footy_vmax,_footy_vmin;	
	
	// swing foot velocity constraints  parameters
	double _comax_max, _comax_min,_comay_max, _comay_min;	


	
	// %%% physical	
	double _mass,  _rad,  _j_ini;	
	
	//external force
    double _FX, _FY;	
	double _t_last;
	double _det_xa,_det_ya;
	double _det_xv,_det_yv;
	double _det_xp,_det_yp;
	
	Eigen::Matrix<double,1,_nsum> _tcpu;	
	
	
	

	
	int xyz0; //flag for find function 
	int xyz1;  
	int xyz2;	
	
	int _period_i; ///
	
	int _ki, _k_yu;
	double _Tk;
	
	double _Lxx_refx,_Lyy_refy,_Lxx_refx1,_Lyy_refy1;
	double _tr1_ref,_tr2_ref,_tr1_ref1,_tr2_ref1;
	
	double _tr1_min,_tr2_min,_tr1_max,_tr2_max,_tr11_min,_tr21_min,_tr11_max,_tr21_max;
	
	Eigen::Matrix<double,1,8> _SS1,_SS2,_SS3,_SS4,_SS5,_SS6,_SS7,_SS8;
	
	Eigen::Matrix<double,1,1> _comvx_endref,_comvy_endref;
	
	Eigen::Matrix<double,1,1> _AxO,_BxO,_Cx, _Axv,_Bxv,_Cxv;
	Eigen::Matrix<double,1,1> _AyO,_ByO,_Cy, _Ayv,_Byv,_Cyv;
	
	Eigen::Matrix<double,8,8> _SQ_goal0,_SQ_goal,_SQ_goal1,_SQ_goal20,_SQ_goal2,_SQ_goal3;
	Eigen::Matrix<double,8,1> _Sq_goal,_Sq_goal1,_Sq_goal2,_Sq_goal3;
	Eigen::Matrix<double,8,8> _Ax,_Ay;
	Eigen::Matrix<double,1,1> _Bx,_By;
	Eigen::Matrix<double,1,1> _ixi,_iyi;	
	
	//constraints on step timing parameters
	Eigen::Matrix<double,1,8> _trx1_up, _trx1_lp,_trx2_up, _trx2_lp,_trx3_up, _trx3_lp,_trx4_up, _trx4_lp;	
	Eigen::Matrix<double,1,1> _det_trx1_up, _det_trx1_lp,_det_trx2_up,_det_trx2_lp,_det_trx3_up, _det_trx3_lp,_det_trx4_up, _det_trx4_lp;		
	Eigen::Matrix<double,8,8> _trx;
	Eigen::Matrix<double,8,1> _det_trx;
	
	// tr1 & tr2: equation constraints
	Eigen::Matrix<double,1,8> _trx12, _trx121;	
	Eigen::Matrix<double,1,1> _det_trx12, _det_trx121;		
	Eigen::Matrix<double,2,8> _trxx;
	Eigen::Matrix<double,2,1> _det_trxx;	
	
	///foot location constraints 
	double _footx_max, _footx_min,_footy_max,_footy_min;	
	Eigen::Matrix<double,1,8> _h_lx_up,  _h_lx_lp, _h_ly_up, _h_ly_lp,_h_lx_up1, _h_lx_lp1, _h_ly_up1, _h_ly_lp1;
	Eigen::Matrix<double,1,1> _det_h_lx_up,_det_h_lx_lp,_det_h_ly_up,_det_h_ly_lp,_det_h_lx_up1,_det_h_lx_lp1,_det_h_ly_up1,_det_h_ly_lp1;
	Eigen::Matrix<double,8,8> _h_lx_upx;
	Eigen::Matrix<double,8,1> _det_h_lx_upx;


	///foot location constraints 	
	Eigen::Matrix<double,1,8> _h_lvx_up,  _h_lvx_lp, _h_lvy_up, _h_lvy_lp,_h_lvx_up1, _h_lvx_lp1, _h_lvy_up1, _h_lvy_lp1;
	Eigen::Matrix<double,1,1> _det_h_lvx_up,_det_h_lvx_lp,_det_h_lvy_up,_det_h_lvy_lp,_det_h_lvx_up1,_det_h_lvx_lp1,_det_h_lvy_up1,_det_h_lvy_lp1;
	Eigen::Matrix<double,8,8> _h_lvx_upx;
	Eigen::Matrix<double,8,1> _det_h_lvx_upx;	
	
	/// CoM accelearation boundary
	double _AA, _CCx, _BBx, _CCy, _BBy,_AA1x,_AA2x,_AA3x,_AA1y,_AA2y,_AA3y;
	Eigen::Matrix<double,1,8> _CoM_lax_up,  _CoM_lax_lp,  _CoM_lay_up,  _CoM_lay_lp;
	Eigen::Matrix<double,1,1> _det_CoM_lax_up,  _det_CoM_lax_lp,  _det_CoM_lay_up,  _det_CoM_lay_lp;
	Eigen::Matrix<double,4,8> _CoM_lax_upx;
	Eigen::Matrix<double,4,1> _det_CoM_lax_upx;	
	
	
	
	/// CoM velocity_inremental boundary
	double _VAA, _VCCx, _VBBx, _VCCy, _VBBy,_VAA1x,_VAA2x,_VAA3x,_VAA1y,_VAA2y,_VAA3y;
	Eigen::Matrix<double,1,8> _CoM_lvx_up,  _CoM_lvx_lp,  _CoM_lvy_up,  _CoM_lvy_lp;
	Eigen::Matrix<double,1,1> _det_CoM_lvx_up,  _det_CoM_lvx_lp,  _det_CoM_lvy_up,  _det_CoM_lvy_lp;
	Eigen::Matrix<double,4,8> _CoM_lvx_upx;
	Eigen::Matrix<double,4,1> _det_CoM_lvx_upx;	
	
	/// CoM initial velocity_ boundary
	double _VAA1x1,_VAA2x1,_VAA3x1,_VAA1y1,_VAA2y1,_VAA3y1;
	Eigen::Matrix<double,1,8> _CoM_lvx_up1,  _CoM_lvx_lp1,  _CoM_lvy_up1,  _CoM_lvy_lp1;
	Eigen::Matrix<double,1,1> _det_CoM_lvx_up1,  _det_CoM_lvx_lp1,  _det_CoM_lvy_up1,  _det_CoM_lvy_lp1;
	Eigen::Matrix<double,4,8> _CoM_lvx_upx1;
	Eigen::Matrix<double,4,1> _det_CoM_lvx_upx1;	
	
	

	
	
	
	/// for foot trajectory generation	
	Eigen::VectorXd _t_f;

	int _bjxx;	
	int _bjx1;
	int _reinit_count;
	//Eigen::Matrix<double, 3,_footstepsnumber> _footxyz_real;
	Eigen::MatrixXd _footxyz_real;
	
	Eigen::Matrix<double, 1,_nsum> _Lfootx, _Lfooty,_Lfootz, _Lfootvx, _Lfootvy,_Lfootvz, _Lfootax, _Lfootay,_Lfootaz;
	Eigen::Matrix<double, 1,_nsum> _Rfootx, _Rfooty,_Rfootz, _Rfootvx, _Rfootvy,_Rfootvz, _Rfootax, _Rfootay,_Rfootaz;	
    double _ry_left_right;	
	
    Eigen::Matrix<double,1,1> _ggg;
	Eigen::Matrix<double,_nsum,1> _Zsc;

	///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
	// other variables for step height+angular optimization
	///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
 	Eigen::Matrix<double,1,2> _zmpx_real, _zmpy_real;
    Eigen::Matrix<double,1,2> _dcmx_real, _dcmy_real;

	int _t_end_footstep;
	Eigen::Vector3d _Lfoot_r, _Rfoot_r,_Lfoot_rv, _Rfoot_rv,_Lfoot_ra, _Rfoot_ra;		

    double _tstep;              ///step period
	double _dt;                //sampling time
	int _nh = 30;
	int _footstepsnumber = 30;       //  number of _footstepnumber

protected:
	void Rfooty_plan(int arg1);
// 	footx_ref_comx_feed(int i);
};
