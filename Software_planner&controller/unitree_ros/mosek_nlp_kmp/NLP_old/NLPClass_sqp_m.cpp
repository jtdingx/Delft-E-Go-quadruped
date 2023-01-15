/*****************************************************************************
NLPClass.cpp

Description:    source file of NLPClass
*****************************************************************************/
#include "NLP/NLPClass.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>
#include <vector>

#include "KMP/kmp.h"
#include <armadillo>
#include "/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/go1_rt_control/src/Robotpara/robot_const_para_config.h"


using namespace Eigen;
using namespace std;
using namespace arma;

#include "fusion.h"
using namespace mosek::fusion;
using namespace monty;


std::shared_ptr<ndarray<int,1>>    nint(const std::vector<int> &X)    { return new_array_ptr<int>(X); }
std::shared_ptr<ndarray<double,1>> ndou(const std::vector<double> &X) { return new_array_ptr<double>(X); }


NLPClass::NLPClass()
    : QPBaseClass()
    ,_robot_name("")
    , _robot_mass(12)
    , _lift_height(0.03)
    , _method_flag(0)
    , RobotPara_totalmass(12)
    , RobotPara_HALF_HIP_WIDTH(0.12675)
    , RobotPara_dt(0.0)
    , RobotPara_Tstep(0.0)
    , RobotPara_Z_C(0.309) 
    , RobotPara_g(9.8)
    , RobotPara_FOOT_WIDTH(0.02)  
    , _pvFlag_kmp(1)
{
  
}


void NLPClass::FootStepInputs( double stepwidth, double steplengthx, double stepheight)
{	
    cout << steplengthx<<endl;
    
    _steplength.setConstant(steplengthx);
    _steplength(_footstepsnumber-1) = 0;
    _steplength(_footstepsnumber-2) = 0;
    _steplength(_footstepsnumber-3) = 0;
    _steplength(_footstepsnumber-4) = 0;	
    _steplength(_footstepsnumber-5) = 0;
    _steplength(0) = 0;
    _steplength(1) = 0;
    _steplength(2) = 0;
    _steplength(3) = steplengthx/2; 
    
    _stepwidth.setConstant(stepwidth);
    _stepwidth(0) = _stepwidth(0)/2;
    
    _stepheight.setConstant(stepheight);     

    _lift_height_ref.setConstant(_lift_height);
    _lift_height_ref(_footstepsnumber-1) = 0;
    _lift_height_ref(_footstepsnumber-2) = 0;	
    _lift_height_ref(_footstepsnumber-3) = _lift_height/2; 
    _lift_height_ref(_footstepsnumber-4) = _lift_height; 	
	
}


void NLPClass::Initialize()
{
  ///// pvFlag=0:model position, pvFlag=1, model position and velocity
	_pvFlag_kmp = 1;			    // output: pos (and vel)
    
  _inDim_kmp  = 1; 	      		    //input dimension
	_outDim_kmp = 3*(_pvFlag_kmp+1); 	      		    //output dimension
	
	///////////// adjust KMP parameters
	if (_pvFlag_kmp > 0)
	{
	  _lamda_kmp  = 5, _kh_kmp = 0.75;	    	    //set kmp parameters 
	}
	else
	{
	  _lamda_kmp  = 5, _kh_kmp = 0.1;	    	    //set kmp parameters
	}
	
	
  ///initialize: input: time; output: pos+vel
	_query_kmp = zeros<vec>(1);
	_mean_kmp = zeros<vec>(_outDim_kmp);

	
  if (_pvFlag_kmp > 0)
	{
	  static char fileName1[]="/home/jiatao/unitree/catkin_ws/src/unitree_ros/mosek_nlp_kmp/src/KMP/referdata_swing_faster.txt";
	
	  _data_kmp.load( fileName1 );         	    // load original data
	  kmp_leg_L.kmp_initialize(_data_kmp, _inDim_kmp, _outDim_kmp, _pvFlag_kmp,_lamda_kmp, _kh_kmp); // initialize kmp
	  kmp_leg_R.kmp_initialize(_data_kmp, _inDim_kmp, _outDim_kmp, _pvFlag_kmp,_lamda_kmp, _kh_kmp); // initialize kmp	  
	}
	else
	{
	  static char fileName1[]="/home/jiatao/unitree/catkin_ws/src/unitree_ros/mosek_nlp_kmp/src/KMP/referdata_swing_pos_faster.txt";
	
	  _data_kmp.load( fileName1 );         	    // load original data
	  kmp_leg_L.kmp_initialize(_data_kmp, _inDim_kmp, _outDim_kmp, _pvFlag_kmp,_lamda_kmp, _kh_kmp); // initialize kmp
	  kmp_leg_R.kmp_initialize(_data_kmp, _inDim_kmp, _outDim_kmp, _pvFlag_kmp,_lamda_kmp, _kh_kmp); // initialize kmp	  
	}
	
  int nVars = 4;
  int nEqCon = 1;
  int nIneqCon = 24;
  resizeQP(nVars, nEqCon, nIneqCon);    

  cout << "finish!!!!!!!!!!! initial for nlp_KMP parameters"<<endl;



//using !!! para variance: bipedal mode	and troting mode	 	
	_steplength(14) = 0;
 	_steplength(15) *= -1;
	_steplength(16) *= -1;
 	_steplength(17) *= -1;
	_steplength(18) *= -1;
 	_steplength(19) *= -1;
	_steplength(20) *= -1;
 	_steplength(21) *= -1;  

// // using !!! para variance: lateral bounding mode	 	
//   _stepwidth(4) -= 0.02; 
//   _stepwidth(5) += 0.01;
//   _stepwidth(6) -= 0.02;
//   _stepwidth(7) += 0.01;
//   _stepwidth(8) -= 0.02;
//   _stepwidth(9) += 0.01;
//   _stepwidth(10) -= 0.02;
//   _stepwidth(11) += 0.01;
// 	_stepwidth(12) -= 0;  
// 	_stepwidth(13) += 0;    
// 	_stepwidth(14) -= 0;
//  	_stepwidth(15) -= 0.02;
// 	_stepwidth(16) += 0.01;
//   _stepwidth(17) -= 0.02;
// 	_stepwidth(18) += 0.01;
//   _stepwidth(19) -= 0.02; 
// 	_stepwidth(20) += 0.01;


  // ==step parameters initialize==: given by the inputs
  ////NP initialization for step timing optimization
  //// reference steplength and stepwidth for timing optimization  
 	// ==step loctions setup==
  _Lxx_ref = _steplength;
  _Lyy_ref = _stepwidth;	   
  // local coordinate
	_Lyy_ref(0) = 0;
	for (int j =0; j<_footstepsnumber;j++)
	{
	  _Lyy_ref(j) = (int)pow(-1,j)*_stepwidth(j);
	}
  
// 	reference footstep locations setup
  _footx_ref.setZero();
	_footy_ref.setZero();
	_footz_ref.setZero();		
  	for (int i = 1; i < _footstepsnumber; i++) {
 	  _footx_ref(i) = _footx_ref(i-1) + _steplength(i-1);
	  _footy_ref(i) = _footy_ref(i-1) + (int)pow(-1,i-1)*_stepwidth(i-1);   
	  _footz_ref(i) = _footz_ref(i-1) + _stepheight(i-1);
	}
	cout <<  "_footx_ref: "<< _footx_ref.transpose()<<endl;
	cout <<  "_footy_ref: "<< _footy_ref.transpose()<<endl;
	cout <<  "_footz_ref: "<< _footz_ref.transpose()<<endl;
    
	_footx_offline = _footx_ref;
	_footy_offline = _footy_ref;
	_footz_offline = _footz_ref;
	
	/// support foot location feedbacke for step time optimization
	_footx_real_feed = _footx_ref;  
	_footy_real_feed = _footy_ref;	

	// == step cycle setup
	_ts.setConstant(_tstep);
	
// 	/// step variance
// 	_ts(9)=1;
// 	_ts(11)=0.6;
	
	_td = 0.2*_ts;
	_tx.setZero();
  	for (int i = 1; i < _footstepsnumber; i++) {
 	  _tx(i) = _tx(i-1) + _ts(i-1);
	  _tx(i) = round(_tx(i)/_dt)*_dt -0.000001;	  
	}	

	//whole sampling time sequnece       
	_t.setLinSpaced(round(_tx(_footstepsnumber-1)/_dt),_dt,_tx(_footstepsnumber-1));

	//parameters
  _hcom = RobotPara_Z_C-_height_offset;	
	_g = RobotPara_g;	
	_Wn = sqrt(_g/_hcom);
	
	_Wndt = _Wn*_dt;	
	
        // COM state
	_comx.setZero(); _comvx.setZero(); _comax.setZero();
	_comy.setZero(); _comvy.setZero(); _comay.setZero();
	_comz.setConstant(_hcom); 
	_comvz.setZero(); 
	_comaz.setZero();
        ///actual steplengh,stepwidth and walking period
	_Lxx_ref_real.setZero(); 
	_Lyy_ref_real.setZero(); 
	_Ts_ref_real.setZero();
	
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// %% parameters for first MPC-step timing adjustment and next one step location
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%	
	_px.setZero(); _py.setZero(); _pz.setZero();
	_zmpvx.setZero(); _zmpvy.setZero(); 
	_COMx_is.setZero(); _COMx_es.setZero(); _COMvx_is.setZero(); 
	_COMy_is.setZero(); _COMy_es.setZero(); _COMvy_is.setZero();
	_comx_feed.setZero(); _comvx_feed.setZero(); _comax_feed.setZero();
	_comy_feed.setZero(); _comvy_feed.setZero(); _comay_feed.setZero();
	
	_Vari_ini.setZero(); //Lxx,Lyy,Tr1,Tr2;
	_vari_ini.setZero();

	
	//step timing constraints:
	_t_min = 0.5; _t_max = 1;
	

/// for dt =0.025: push when walking forward
	// swing foot velocity constraints	
	_footx_vmax = 3;
	_footx_vmin = -2.875;
	_footy_vmax = 2;
	_footy_vmin = -1;		
	
	// CoM acceleration velocity:
	_comax_max = 5;
	_comax_min = -5;
	_comay_max = 6;
	_comay_min = -6;	
	

	if(_robot_name == "coman"){
        _rad = 0.1; 	    	  
        //weight coefficient:  _rr1 and _rr2 should be small to enhance the  
        _aax = 50000;            _aay =50000;
        _aaxv =100;              _aayv=50;
        _bbx = 80000000;         _bby =80000000;
        _rr1 = 500000;           _rr2 =500000; 

            //  foot location constraints 
        _footx_max=0.25;
        _footx_min=-0.1;
	}
	else if (_robot_name == "go1")
	{
        _rad = 0.2;    	

        //weight coefficient: ball hit
        _aax = 50000;            _aay = 50000;
       _aaxv = 1000;            _aayv = 500;
        _bbx = 2000000;          _bby = 10000000;
        _rr1 = 1000000;          _rr2 = 1000000;  	

            //  foot location constraints 
        _footx_max = 0.15;
        _footx_min = -0.05;	
	}
	else if (_robot_name == "cogimon"){	  
        _rad = 0.2; 	 
        
        //weight coefficient
        _aax = 50000;            _aay =5000;
        _aaxv =100;              _aayv=10;
        _bbx = 80000000;         _bby =80000000;
        _rr1 = 500000;           _rr2 =500000; 
            //  foot location constraints 
        _footx_max=0.3;
        _footx_min=-0.15;	  
  } 
	
	
	
	_footy_max=2*RobotPara_HALF_HIP_WIDTH + 0.03; 
	_footy_min=RobotPara_HALF_HIP_WIDTH - 0.03; 	
	
	_mass = _robot_mass; 
	_j_ini = _mass* pow(_rad,2);	
	

	_tcpu.setZero();
	
	_n_loop_omit = 2*round(_tstep/_dt);
	
	xyz0 = -1; //flag for find function 
	xyz1 = 0;  
	xyz2 = 1;

    
  _t_whole.setLinSpaced(_nsum,_dt,_tx(_footstepsnumber-1));  ///sampling time sequence for entire step period	
  
  _j_period = 0; // the number for step cycle indefind
  
  //footz refer: 
  _Zsc.setZero();		
  for (int i = 0; i < _nsum-1; i++) {	  
    Indexfind(_t_whole(i),xyz1);	  
    _Zsc(i) = _footz_ref(_j_period);   
    _j_period = 0; 
  }    
    
    
    
		
	_periond_i = 0;  /// period number falls into which cycle 
	_ki = 0;
	_k_yu = 0;
	_Tk = 0; 
	
	/// remaining time boundaries
	_tr1_min=0;   _tr2_min=0;  _tr1_max=0;  _tr2_max=0;
	_tr11_min=0;  _tr21_min=0; _tr11_max=0; _tr21_max=0;	
	
	/// selection matrix for variables
  _SS1.setZero(); _SS1(0) =1;
	_SS2.setZero(); _SS2(1) =1;
	_SS3.setZero(); _SS3(2) =1;
	_SS4.setZero(); _SS4(3) =1;

	
	_comvx_endref.setZero();
	_comvy_endref.setZero();
	
	_AxO.setZero();_BxO.setZero();_Cx.setZero(); 
	_Axv.setZero();_Bxv.setZero();_Cxv.setZero();
	_AyO.setZero();_ByO.setZero();_Cy.setZero(); 
	_Ayv.setZero();_Byv.setZero();_Cyv.setZero();	

	_SQ_goal0.setZero();	
	_SQ_goal.setZero();
	_SQ_goal1.setZero();
	_SQ_goal20.setZero();
	_SQ_goal2.setZero();	
	_SQ_goal3.setZero();	
	_Sq_goal.setZero();
	_Sq_goal1.setZero();	
	_Sq_goal2.setZero();
	_Sq_goal3.setZero();	
	_Ax.setZero(); 	_Ay.setZero();
	_Bx.setZero();  _By.setZero();
	_ixi.setZero(); _iyi.setZero();
	
	/// remaining time constraints: inequality constraints
	_trx1_up.setZero();  _trx1_lp.setZero();
	_trx2_up.setZero();  _trx2_lp.setZero();	
	_trx3_up.setZero();  _trx3_lp.setZero();	
	_trx4_up.setZero();  _trx4_lp.setZero();	
	_det_trx1_up.setZero();  _det_trx1_lp.setZero();
	_det_trx2_up.setZero();  _det_trx2_lp.setZero();	
	_det_trx3_up.setZero();  _det_trx3_lp.setZero();	
	_det_trx4_up.setZero();  _det_trx4_lp.setZero();	
	
	_trx.setZero();  _det_trx.setZero();
	
	//// tr1&tr2:equation constraints
	_trx12.setZero();     _trx121.setZero();	
	_det_trx12.setZero(); _det_trx121.setZero();		
	_trxx.setZero();      _det_trxx.setZero();	

	_trx12_up2.setZero(); _trx12_lp2.setZero();
	_trx12_up1.setZero(); _trx12_lp1.setZero();
	_det_trx12_up.setZero(); _det_trx12_lp.setZero();

	

	
	_h_lx_up.setZero();  _h_lx_lp.setZero(); _h_ly_up.setZero(); _h_ly_lp.setZero();
	_h_lx_up1.setZero(); _h_lx_lp1.setZero(); _h_ly_up1.setZero(); _h_ly_lp1.setZero();
	_det_h_lx_up.setZero();_det_h_lx_lp.setZero();_det_h_ly_up.setZero();_det_h_ly_lp.setZero();
	_det_h_lx_up1.setZero();_det_h_lx_lp1.setZero();_det_h_ly_up1.setZero();_det_h_ly_lp1.setZero();
	_h_lx_upx.setZero(); _det_h_lx_upx.setZero();
	
	// swing foot velocity constraints
	_h_lvx_up.setZero();  _h_lvx_lp.setZero(); _h_lvy_up.setZero(); _h_lvy_lp.setZero();
	_h_lvx_up1.setZero(); _h_lvx_lp1.setZero(); _h_lvy_up1.setZero(); _h_lvy_lp1.setZero();
	_det_h_lvx_up.setZero();_det_h_lvx_lp.setZero();_det_h_lvy_up.setZero();_det_h_lvy_lp.setZero();
	_det_h_lvx_up1.setZero();_det_h_lvx_lp1.setZero();_det_h_lvy_up1.setZero();_det_h_lvy_lp1.setZero();	
	_h_lvx_upx.setZero(); _det_h_lvx_upx.setZero();	
	
	// CoM acceleration boundary
	_AA = 0; _CCx=0; _BBx=0; _CCy=0; _BBy=0;_AA1x=0;_AA2x=0;_AA3x=0;_AA1y=0;_AA2y=0;_AA3y=0;
	_CoM_lax_up.setZero();  _CoM_lax_lp.setZero();  _CoM_lay_up.setZero();  _CoM_lay_lp.setZero();
	_det_CoM_lax_up.setZero();  _det_CoM_lax_lp.setZero();  _det_CoM_lay_up.setZero();  _det_CoM_lay_lp.setZero();
	_CoM_lax_upx.setZero();
	_det_CoM_lax_upx.setZero();		
	
	
	
	/// CoM velocity_inremental boundary
	_VAA=0; _VCCx=0; _VBBx=0; _VCCy=0; _VBBy=0;_VAA1x=0;_VAA2x=0;_VAA3x=0;_VAA1y=0;_VAA2y=0;_VAA3y=0;
	_CoM_lvx_up.setZero();  _CoM_lvx_lp.setZero();  _CoM_lvy_up.setZero();  _CoM_lvy_lp.setZero();
	_det_CoM_lvx_up.setZero();  _det_CoM_lvx_lp.setZero();  _det_CoM_lvy_up.setZero();  _det_CoM_lvy_lp.setZero();
	_CoM_lvx_upx.setZero();
	_det_CoM_lvx_upx.setZero();	
	
	
	
	/// CoM initial velocity_ boundary
	_VAA1x1=0;_VAA2x1=0;_VAA3x1=0;_VAA1y1=0;_VAA2y1=0;_VAA3y1=0;
	_CoM_lvx_up1.setZero();  _CoM_lvx_lp1.setZero();  _CoM_lvy_up1.setZero();  _CoM_lvy_lp1.setZero();
	_det_CoM_lvx_up1.setZero();  _det_CoM_lvx_lp1.setZero();  _det_CoM_lvy_up1.setZero();  _det_CoM_lvy_lp1.setZero();
	_CoM_lvx_upx1.setZero();
	_det_CoM_lvx_upx1.setZero();	
	
	
	//DCM constraints
	_cpx_max = 0;  _cpx_min = 0; _cpy_max = 0; _cpy_min = 0; _lp = 0; _W_max = 0; _W_min = 0;
	_cp_x_up.setZero();  _cp_x_min.setZero();  _cp_y_up.setZero();  _cp_y_min.setZero();
	_det_cp_x_up.setZero();  _det_cp_x_min.setZero();  _det_cp_y_up.setZero();  _det_cp_y_min.setZero();
	_h_cp.setZero();         _det_cp.setZero();
        _VCCx_matrix.setZero(); _VCCy_matrix.setZero();

	_cp_x_up2.setZero();  _cp_x_lp2.setZero();  _cp_y_up2.setZero();  _cp_y_lp2.setZero();
	_cp_x_up1.setZero();  _cp_x_lp1.setZero();  _cp_y_up1.setZero();  _cp_y_lp1.setZero();
	_det_cp_x_up1.setZero();  _det_cp_x_lp1.setZero();  _det_cp_y_up1.setZero();  _det_cp_y_lp1.setZero();

	
	
	//// Constraint transformation
	H1_q_xy.setZero();
	F_quadratci.setZero();	
	
	vector <Eigen::MatrixXd> H2_q_xy_offline(8);	
	H2_q_xy = H2_q_xy_offline;
	
	for(int jxx=1; jxx<=8; jxx++)
	{	     		 
         H2_q_xy[jxx-1] = _cp_x_up2;
	}	
	
	
	_W_goal.setZero();
	
	vector <Eigen::MatrixXd> cin_quad_offline(8);

	cin_quad = cin_quad_offline;
	for(int jxx=1; jxx<=8; jxx++)
	{	     		 
         cin_quad[jxx-1] = _W_goal;
	}
	
	_A_q1.setZero();
	_b_q1.setZero();
	
	vector <Eigen::MatrixXd> cin_aff_offline(24);	
	cin_aff = cin_aff_offline;
	for(int jxx=1; jxx<=24; jxx++)
	{	     		 
         cin_aff[jxx-1] = _W_goal;
	}	
	
	
	
	

	///////////////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	///%%%%%%%%%%%%% foot trajectory geneartion
	//////////////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  _t_f.setZero();	
	_bjx1 = 0;
	_bjxx = 0;
	_footxyz_real.setZero();
	

  right_support = 0;

	
	_Lfootx.setZero(); _Lfooty.setConstant(_stepwidth(0));_Lfootz.setZero(); _Lfootvx.setZero(); _Lfootvy.setZero();_Lfootvz.setZero(); 
	_Lfootax.setZero(); _Lfootay.setZero();_Lfootaz.setZero();
	_Rfootx.setZero(); _Rfooty.setConstant(-_stepwidth(0));_Rfootz.setZero(); _Rfootvx.setZero(); _Rfootvy.setZero();_Rfootvz.setZero(); 
	_Rfootax.setZero(); _Rfootay.setZero();_Rfootaz.setZero();	
	_ry_left_right = 0;	
	
	
	 _CoM_position_optimal.setZero();
	 _torso_angle_optimal.setZero();
	 _L_foot_optition_optimal.setZero();
	 _R_foot_optition_optimal.setZero();
	 _foot_location_optimal.setZero();	
	
	
	
	
	/////// swing leg trajectory generation using kmp_initialize
	_Lfootx_kmp.setZero();  _Lfooty_kmp.setConstant(_stepwidth(0)); _Lfootz_kmp.setZero(); 
	_Lfootvx_kmp.setZero(); _Lfootvy_kmp.setZero();                 _Lfootvz_kmp.setZero(); 

	_Rfootx_kmp.setZero();  _Rfooty_kmp.setConstant(-_stepwidth(0));_Rfootz_kmp.setZero(); 
	_Rfootvx_kmp.setZero(); _Rfootvy_kmp.setZero();                 _Rfootvz_kmp.setZero(); 

	
	///////////////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//////////////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// %% pamameters for second MPC- ZMP movement, height variance and body inclination
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// %% robot parameters		
	_zmpx_real.setZero(_nsum); _zmpy_real.setZero(_nsum);	
	_thetax.setZero(_nsum); _thetavx.setZero(_nsum); _thetaax.setZero(_nsum);
	_thetay.setZero(_nsum); _thetavy.setZero(_nsum); _thetaay.setZero(_nsum);
	_thetaz.setZero(_nsum); _thetavz.setZero(_nsum); _thetaaz.setZero(_nsum);

	
	_torquex_real.setZero(_nsum); _torquey_real.setZero(_nsum);
    
  _dcmx_real.setZero(_nsum); _dcmy_real.setZero(_nsum);	
    

        // ==initial parameters for MPC==
	_ggg.setConstant(1, 9.8);

	_Hcom1.setConstant(_nh,1,_hcom);
	
			


	
	
	
	
	_footx_real.setZero(_footstepsnumber);  _footy_real.setZero(_footstepsnumber); _footz_real.setZero(_footstepsnumber);
	_footx_real_next.setZero(_nsum);  _footy_real_next.setZero(_nsum); _footz_real_next.setZero(_nsum);
	_footx_real_next1.setZero(_nsum);  _footy_real_next1.setZero(_nsum); _footz_real_next1.setZero(_nsum);


	
////////////// adimittance control
	//////polynomial intepolation for lower level interpolation
	_AAA_inv.setZero();

    _nTdx = 0;
    _F_R.setZero(); 
	_F_L.setZero();
	_M_R.setZero();
	_M_L.setZero();
	
	/////////////for ZMP distribution
	_F_R(2) = _F_L(2) = 0.5 * _mass * RobotPara_g;
	_Co_L.setZero(); _Co_R.setZero();
	
	_comxyzx.setZero(); _comvxyzx.setZero(); _comaxyzx.setZero(); 
	_thetaxyx.setZero(); _thetavxyx.setZero(); _thetaaxyx.setZero();
	_Lfootxyzx.setZero(); _Rfootxyzx.setZero();
	_ZMPxy_realx.setZero();
	
	_zmp_rela_vari.setZero();
	
	_comxyzx(2) = RobotPara_Z_C-_height_offset;	  
	
	//CoMMM_ZMP_foot.setZero();
	
        _ZMP_ratio = 0;
	_t_end_walking = 0;
	
	_comy_matrix_inv.setZero();

	
	_comx_ref_lip.setZero();
	_comy_ref_lip.setZero(); 
	_comvx_ref_lip.setZero(); 
	_comvy_ref_lip.setZero(); 
    
    _t_end_walking = _tx(_footstepsnumber-1)- 8*_tstep/4;  // ending time when normal walking ends
    _n_end_walking = round(_t_end_walking/_dt);	
    _t_end_footstep = round((_tx(_footstepsnumber-1)- 2*_tstep)/_dt); 
    
    
    _Lfoot_r.setZero();
    _Rfoot_r.setZero();   
    _Lfoot_rv.setZero();
    _Rfoot_rv.setZero();     
    _Lfoot_ra.setZero();
    _Rfoot_ra.setZero();    
    
    cout <<  "initialization success "<<endl;
	
} 


////////////////////// LIPM com reference generation/////////////////
void NLPClass::Ref_com_lip_offline()
{
  
  _footx_real = _footx_ref;  
  _footy_real = _footy_ref; 
  _footz_real = _footz_ref;
  
  for (int i =1; i<_footstepsnumber; i++)
  {

    _ki = round(_tx(i-1,0)/_dt) - 1;
    if (i==1)
    {
      _COMx_is(i-1) = 0; 
      _COMy_is(i-1) = 0;     
    }
    else
    {
      _COMx_is(i-1) = (_footx_real(i-1)+_footx_real(i-2))/2 - _footx_real(i-1);
      _COMy_is(i-1) = (_footy_real(i-1)+_footy_real(i-2))/2 - _footy_real(i-1);        
    }
    _COMx_es(i-1) = (_footx_real(i)+_footx_real(i-1))/2 - _footx_real(i-1);
    _COMvx_is(i-1) = (_COMx_es(i-1)-_COMx_is(i-1)*cosh(_Wn*_ts(i-1)))/(1/_Wn *sinh(_Wn*_ts(i-1)));
    
    _COMy_es(i-1) = (_footy_real(i)+_footy_real(i-1))/2 - _footy_real(i-1);
    _COMvy_is(i-1) = (_COMy_es(i-1)-_COMy_is(i-1)*cosh(_Wn*_ts(i-1)))/(1/_Wn *sinh(_Wn*_ts(i-1)));      
    
    for (int j=1; j<=_nT;j++)
    {
      double tj = _dt*j;
      _px(j+_ki-1) =  _footx_real(i-1);
      _py(j+_ki-1) =  _footy_real(i-1);
      
      _comx_ref_lip(j+_ki-1) = _COMx_is(i-1)*cosh(_Wn*tj) + _COMvx_is(i-1)*1/_Wn*sinh(_Wn*tj)+_px(j+_ki-1);
      _comy_ref_lip(j+_ki-1) = _COMy_is(i-1)*cosh(_Wn*tj) + _COMvy_is(i-1)*1/_Wn*sinh(_Wn*tj)+_py(j+_ki-1);
      _comvx_ref_lip(j+_ki-1) = _Wn*_COMx_is(i-1)*sinh(_Wn*tj) + _COMvx_is(i-1)*cosh(_Wn*tj);
      _comvy_ref_lip(j+_ki-1) = _Wn*_COMy_is(i-1)*sinh(_Wn*tj) + _COMvy_is(i-1)*cosh(_Wn*tj);      
    } 
  }

  File_wl();
  
}

void NLPClass::Ref_com_lip_update(int j_current)
{
  ////// not used in real time since tracking the global step location is the goal
  for (int i_period =_bjxx; i_period<_bjxx+2; i_period++)
  {    
    _ki = round(_tx(i_period-1,0)/_dt) - 1;
    if (i_period==1)
    {
      _COMx_is(i_period-1) = 0; 
      _COMy_is(i_period-1) = 0;     
    }
    else
    {
      _COMx_is(i_period-1) = (_footx_real(i_period-1)+_footx_real(i_period-2))/2 - _footx_real(i_period-1);
      _COMy_is(i_period-1) = (_footy_real(i_period-1)+_footy_real(i_period-2))/2 - _footy_real(i_period-1);        
    }
    _COMx_es(i_period-1) = (_footx_real(i_period)+_footx_real(i_period-1))/2 - _footx_real(i_period-1);
    _COMvx_is(i_period-1) = (_COMx_es(i_period-1)-_COMx_is(i_period-1)*cosh(_Wn*_ts(i_period-1)))/(1/_Wn *sinh(_Wn*_ts(i_period-1)));
    
    _COMy_es(i_period-1) = (_footy_real(i_period)+_footy_real(i_period-1))/2 - _footy_real(i_period-1);
    _COMvy_is(i_period-1) = (_COMy_es(i_period-1)-_COMy_is(i_period-1)*cosh(_Wn*_ts(i_period-1)))/(1/_Wn *sinh(_Wn*_ts(i_period-1)));      
    
    for (int j=1; j<=_nT;j++)
    {
      double tj = _dt*j;
      _px(j+_ki-1) =  _footx_real(i_period-1);
      _py(j+_ki-1) =  _footy_real(i_period-1);
      
      _comx_ref_lip(j+_ki-1) = _COMx_is(i_period-1)*cosh(_Wn*tj) + _COMvx_is(i_period-1)*1/_Wn*sinh(_Wn*tj)+_px(j+_ki-1);
      _comy_ref_lip(j+_ki-1) = _COMy_is(i_period-1)*cosh(_Wn*tj) + _COMvy_is(i_period-1)*1/_Wn*sinh(_Wn*tj)+_py(j+_ki-1);
      _comvx_ref_lip(j+_ki-1) = _Wn*_COMx_is(i_period-1)*sinh(_Wn*tj) + _COMvx_is(i_period-1)*cosh(_Wn*tj);
      _comvy_ref_lip(j+_ki-1) = _Wn*_COMy_is(i_period-1)*sinh(_Wn*tj) + _COMvy_is(i_period-1)*cosh(_Wn*tj);      
    } 
  }  
}




///////////////////// NLP based com generation with step parameters adjustment
Eigen::Matrix<double, 38, 1> NLPClass::step_timing_opti_loop(int i,Eigen::Matrix<double,18,1> estimated_state, Eigen::Vector3d _Rfoot_location_feedback,
                                                             Eigen::Vector3d _Lfoot_location_feedback,double lamda, bool _stopwalking)
{
  Eigen::Matrix<double, 38, 1> com_traj;
  
  clock_t _t_start,_t_finish;
  _t_start = clock();
  
  //// step cycle number when (i+1)*dt fall into: attention that _tstep+1 falls ionto the next cycle      
  Indexfind((i+1)*_dt,xyz0);	   /// next one sampling time
  _periond_i = _j_period+1;      ///coincident with Matlab
  _j_period = 0;  


  //ZMP & ZMPv
  _px(0,i) = _footx_ref(_periond_i-1,0); _zmpvx(0,i) = 0; 
  _py(0,i) = _footy_ref(_periond_i-1,0); _zmpvy(0,i) = 0; 
  
//   cout<<"_py:"<<_py(0,i)<<endl;  
  
  ///remaining step timing for the next stepping
  _ki = round(_tx(_periond_i-1,0)/_dt);
  _k_yu = i-_ki;                
  _Tk = _ts(_periond_i-1) - _k_yu*_dt; 
  
  //// reference remaining time and step length and step width
//    position tracking 
//   _Lxx_refx = _footx_offline(_periond_i)-_footx_ref(_periond_i-1);        
//   _Lyy_refy = _footy_offline(_periond_i)-_footy_ref(_periond_i-1); //%% tracking the step location

  //    velocity tracking 
   _Lxx_refx = _Lxx_ref(_periond_i-1);        
   _Lyy_refy = _Lyy_ref(_periond_i-1); //%% tracking relative location

  _tr1_ref = cosh(_Wn*_Tk);        _tr2_ref = sinh(_Wn*_Tk);

  // warm start
  if(i==1)
  {
    _vari_ini << _Lxx_refx,
                 _Lyy_refy,
                _tr1_ref,
                _tr2_ref;
  }
  else
  {
    _vari_ini = _Vari_ini.col(i-1);
  }


 		 
// step timing upper&lower boundaries  modification
  if ((_t_min -_k_yu*_dt)>=0.001)
  {
    _tr1_min = cosh(_Wn*(_t_min-_k_yu*_dt));
    _tr2_min = sinh(_Wn*(_t_min-_k_yu*_dt));    
  }
  else
  {
    _tr1_min = cosh(_Wn*(0.001));
    _tr2_min = sinh(_Wn*(0.001));      
  }
 
  _tr1_max = cosh(_Wn*(_t_max-_k_yu*_dt));
  _tr2_max = sinh(_Wn*(_t_max-_k_yu*_dt));
  
 

  if (i==1)
  {
    _COMx_is(_periond_i-1) = _comx_feed(i-1)-_footx_ref(_periond_i-1);  
    _COMx_es.col(_periond_i-1) = _SS1*_vari_ini*0.5;  
    _COMvx_is(_periond_i-1)= (_COMx_es(_periond_i-1)-_COMx_is(_periond_i-1)*_SS3*_vari_ini)/(1/_Wn *_SS4*_vari_ini);
    _COMy_is(_periond_i-1) = _comy_feed(i-1)-_footy_ref(_periond_i-1);  
    _COMy_es.col(_periond_i-1) = _SS2*_vari_ini*0.5;  
    _COMvy_is(_periond_i-1)= (_COMy_es(_periond_i-1)-_COMy_is(_periond_i-1)*_SS3*_vari_ini)/(1/_Wn *_SS4*_vari_ini);  
    _comvx_endref= _Wn*_COMx_is(_periond_i-1)*_SS4*_vari_ini + _COMvx_is(_periond_i-1)*_SS3*_vari_ini;
    _comvy_endref= _Wn*_COMy_is(_periond_i-1)*_SS4*_vari_ini + _COMvy_is(_periond_i-1)*_SS3*_vari_ini;     
  }


  //cout <<  "ffffffffff "<<endl;
// SEQUENCE QUADARTIC PROGRAMMING-step timing &step location optimization
  for (int xxxx=1; xxxx<=3; xxxx++)
  { 
    //// be careful the  divide / (one of the factor should be double: type)
// // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// // %% optimal programme formulation/OBJECTIVE FUNCTION:
// // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
// // %%%%% Lx0 = Lxk(:,i);Ly0 = Lyk(:,i);tr10 = Tr1k(:,i); tr20 = Tr2k(:,i);   
    step_timing_object_function(i);

// // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// // %% constraints
// // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    step_timing_constraints(i);
    
    ///// generated CoM final position
    if(_Tk >= 0.1*_ts(_periond_i-1))
    {
      solve_stepping_timing();

      if (_X.rows() == 4)
      {
	      _vari_ini += _X;
      }          
    }
    else
    {
      _vari_ini << _Lxx_refx,
                  _Lyy_refy,
                  _tr1_ref,
                  _tr2_ref;      
    }

  
    
  }
  
  
// // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// // %% results postprocession
// // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
  
  _Vari_ini.col(i) = _vari_ini;  

// update the optimal parameter in the real-time: at the result, the effect of optimization reduced gradually
// //////post procession//////////////////////
//   if (_vari_ini(0,0)>_footx_max)
//   {
//    _vari_ini(0,0)  = _footx_max;
//   }
//   else
//   {
//     if (_vari_ini(0,0)<_footx_min)
//     {
//     _vari_ini(0,0)  = _footx_min;
//     }    
    
//   }

//   if ((abs(_vari_ini(0,0)-_Lxx_refx)>0.005)&&((_vari_ini(0,0)-_Lxx_refx)*_comvx_feed(i-1)<0))
//   {
//     _vari_ini(0,0) *= (-1);
//   }
  
  
//   if (_vari_ini(1,0)>_footy_max)
//   {
//    _vari_ini(1,0)  = _footy_max;
//   }
//   else
//   {
//     if (_vari_ini(1,0)<_footy_min)
//     {
//     _vari_ini(1,0)  = _footy_min;
//     }    
    
//   }  
  

//   if (_vari_ini(2,0)>_tr1_max)
//   {
//    _vari_ini(2,0)  = _tr1_max;
//    _vari_ini(3,0)  = _tr2_max;
//   }
//   else
//   {
//     if (_vari_ini(2,0)<_tr1_min)
//     {
//       _vari_ini(2,0)  = _tr1_min;
//       _vari_ini(3,0)  = _tr2_min;
//     }    
    
//   }  
  
//   if (_vari_ini(3,0)>_tr2_max)
//   {
//    _vari_ini(2,0)  = _tr1_max;
//    _vari_ini(3,0)  = _tr2_max;
//   }
//   else
//   {
//     if (_vari_ini(3,0)<_tr2_min)
//     {
//       _vari_ini(2,0)  = _tr1_min;
//       _vari_ini(3,0)  = _tr2_min;
//     }
//   }
  


// update the optimal parameter in the real-time: at the result, the effect of optimization reduced gradually
  _Lxx_ref(_periond_i-1) = _SS1*_vari_ini;
  _Lyy_ref(_periond_i-1) = _SS2*_vari_ini;
  _ts(_periond_i-1) = _k_yu*_dt+ log((_SS3+_SS4)*_vari_ini)/_Wn;   //check log function  
  
  
  _Lxx_ref_real(i) = _Lxx_ref(_periond_i-1);
  _Lyy_ref_real(i) = _Lyy_ref(_periond_i-1);
  _Ts_ref_real(i) = _ts(_periond_i-1);  


  _COMx_is(_periond_i-1) = _comx_feed(i-1)-_footx_ref(_periond_i-1);  
  _COMx_es.col(_periond_i-1) = _SS1*_vari_ini*0.5;  
  _COMvx_is(_periond_i-1)= (_COMx_es(_periond_i-1)-_COMx_is(_periond_i-1)*_SS3*_vari_ini)/(1/_Wn *_SS4*_vari_ini);
  _COMy_is(_periond_i-1) = _comy_feed(i-1)-_footy_ref(_periond_i-1);  
  _COMy_es.col(_periond_i-1) = _SS2*_vari_ini*0.5;  
  _COMvy_is(_periond_i-1)= (_COMy_es(_periond_i-1)-_COMy_is(_periond_i-1)*_SS3*_vari_ini)/(1/_Wn *_SS4*_vari_ini);    


  
// update walking period and step location 
  for (int jxx = _periond_i+1; jxx<=_footstepsnumber; jxx++)
  {
    _tx(jxx-1) = _tx(jxx-2)+_ts(jxx-2);
  }
  
//   the real-time calcuated next one step location: similar with the the footx_real calulated in the NMPC ):
  _footx_ref(_periond_i)=_footx_ref(_periond_i-1)+_SS1*_vari_ini;
  _footy_ref(_periond_i)=_footy_ref(_periond_i-1)+_SS2*_vari_ini;    

  _comvx_endref= _Wn*_COMx_is(_periond_i-1)*_SS4*_vari_ini + _COMvx_is(_periond_i-1)*_SS3*_vari_ini;
  _comvy_endref= _Wn*_COMy_is(_periond_i-1)*_SS4*_vari_ini + _COMvy_is(_periond_i-1)*_SS3*_vari_ini;     
      
// update CoM state  
//   _comx(i)= _COMx_is(_periond_i-1)*cosh(_Wndt) + _COMvx_is(_periond_i-1)*1/_Wn*sinh(_Wndt)+_px(i);
//   _comy(i)= _COMy_is(_periond_i-1)*cosh(_Wndt) + _COMvy_is(_periond_i-1)*1/_Wn*sinh(_Wndt)+_py(i);           
//   _comvx(i)= _Wn*_COMx_is(_periond_i-1)*sinh(_Wndt) + _COMvx_is(_periond_i-1)*cosh(_Wndt);
//   _comvy(i)= _Wn*_COMy_is(_periond_i-1)*sinh(_Wndt) + _COMvy_is(_periond_i-1)*cosh(_Wndt);     
//   _comax(i)= pow(_Wn,2)*_COMx_is(_periond_i-1)*cosh(_Wndt) + _COMvx_is(_periond_i-1)*_Wn*sinh(_Wndt);
//   _comay(i)= pow(_Wn,2)*_COMy_is(_periond_i-1)*cosh(_Wndt) + _COMvy_is(_periond_i-1)*_Wn*sinh(_Wndt);   
//   
//    _comz(i) = _hcom; 
//    _comvz(i) = 0; 
//    _comaz(i) = 0;
   
   
   
    /////  generate the trajectory during the double support phase'
    _nTdx = round(_td(1)/_dt)+1;
    
    //////CoM height variation
    CoM_height_solve(i, _stopwalking,_nTdx);
   
    for (int jxx=1; jxx <=_nTdx; jxx++)
    {
        double _wndtx = _Wn*_dt*jxx;

        _comx(i+jxx-1)= _COMx_is(_periond_i-1)*cosh(_wndtx) + _COMvx_is(_periond_i-1)*1/_Wn*sinh(_wndtx)+_px(i);
        _comy(i+jxx-1)= _COMy_is(_periond_i-1)*cosh(_wndtx) + _COMvy_is(_periond_i-1)*1/_Wn*sinh(_wndtx)+_py(i);           
        _comvx(i+jxx-1)= _Wn*_COMx_is(_periond_i-1)*sinh(_wndtx) + _COMvx_is(_periond_i-1)*cosh(_wndtx);
        _comvy(i+jxx-1)= _Wn*_COMy_is(_periond_i-1)*sinh(_wndtx) + _COMvy_is(_periond_i-1)*cosh(_wndtx);     
        _comax(i+jxx-1)= pow(_Wn,2)*_COMx_is(_periond_i-1)*cosh(_wndtx) + _COMvx_is(_periond_i-1)*_Wn*sinh(_wndtx);
        _comay(i+jxx-1)= pow(_Wn,2)*_COMy_is(_periond_i-1)*cosh(_wndtx) + _COMvy_is(_periond_i-1)*_Wn*sinh(_wndtx);   
            

        _zmpx_real(0,i+jxx-1) = _comx(0,i+jxx-1) - (_comz(0,i+jxx-1) - _Zsc(i+jxx-1))/(_comaz(0,i+jxx-1)+_ggg(0))*_comax(0,i+jxx-1);
        _zmpy_real(0,i+jxx-1) = _comy(0,i+jxx-1) - (_comz(0,i+jxx-1) - _Zsc(i+jxx-1))/(_comaz(0,i+jxx-1)+_ggg(0))*_comay(0,i+jxx-1);

        _dcmx_real(0,i+jxx-1) = _comx(0,i+jxx-1) + _comvx(0,i+jxx-1) * sqrt((_comz(0,i+jxx-1) - _Zsc(i+jxx-1))/(_comaz(0,i+jxx-1)+_ggg(0)));
        _dcmy_real(0,i+jxx-1) = _comy(0,i+jxx-1) + _comvy(0,i+jxx-1) * sqrt((_comz(0,i+jxx-1) - _Zsc(i+jxx-1))/(_comaz(0,i+jxx-1)+_ggg(0)));      
    }

    _ZMPxy_realx(0) = _zmpx_real(0,i);
    _ZMPxy_realx(1) = _zmpy_real(0,i);
   
   
   

//  external disturbances!!! using feedback data:
 
  /// /// relative state to the actual foot lcoation: very good
  if (_periond_i % 2 == 0)  // odd : left support
  {
    estimated_state(0,0) =  estimated_state(0,0) - _Lfoot_location_feedback(0); //relative comx
    estimated_state(3,0) =  estimated_state(3,0) - _Lfoot_location_feedback(1);	// relative comy 	    
  }
  else
  {
    estimated_state(0,0) =  estimated_state(0,0) - _Rfoot_location_feedback(0);
    estimated_state(3,0) =  estimated_state(3,0) - _Rfoot_location_feedback(1);	  
  }  
     
     

/////
  double _lamda_comx = 0;
  double _lamda_comvx = 0;
  double _lamda_comy = 0;
  double _lamda_comvy = 0;   


//   /// for dt = 0.025: for multi-pushes when stepping in place
//   if (_periond_i>2)   
//   {
//     _lamda_comx = 0.25;   
//     _lamda_comvx = 0.001;        
//     _lamda_comy = 0.025;   
//     _lamda_comvy = 0.001;        
//   }    


//   /// for dt = 0.025: step variance
//   if (_periond_i>2)   
//   {
//     _lamda_comx = 0.00;   
//     _lamda_comvx = 0.000;        
//     _lamda_comy = 0.00;
//     _lamda_comvy = 0.000;           
//   } 
  

//   /// for dt = 0.025: lateral push recovery
//   if (_periond_i>2)   
//   {
//     _lamda_comx = 0.001;     
//     _lamda_comvx = 0.0001;           
//     _lamda_comy = 0.035;
//     _lamda_comvy = 0.001;            
//   }  
  

  
  _comx_feed(i)  = ((1-_lamda_comx)*(_comx(i)-_px(i))+(_lamda_comx)*estimated_state(0,0))+_px(i);    
  _comvx_feed(i) =  (1-_lamda_comvx)*_comvx(i) + (_lamda_comvx)*estimated_state(1,0);
  _comax_feed(i) = (1-_lamda_comx)*_comax(i) + _lamda_comx*estimated_state(2,0);
  _comy_feed(i) =  ((1-_lamda_comy)*(_comy(i)-_py(i))+(_lamda_comy)*estimated_state(3,0))+_py(i);    
  _comvy_feed(i) =  (1-_lamda_comvy)*_comvy(i) + (_lamda_comvy)*estimated_state(4,0);  
  _comay_feed(i) = (1-_lamda_comy)*_comay(i) + _lamda_comy*estimated_state(5,0);  
     

   
  
  _t_finish = clock();  
  _tcpu(0,i-1) = (double)(_t_finish - _t_start)/CLOCKS_PER_SEC;

  
  _t_f.setLinSpaced(_nh,(i+1)*_dt, (i+_nh)*_dt);
  
  Indexfind(i*_dt,xyz1);                   //// step cycle number when (i)*dt fall into : current sampling time
  _bjxx = _j_period+1;  //coincidence with matlab 
  _j_period = 0;  
  
  Indexfind(_t_f(0),xyz1);                /// step cycle number when (i+1)*dt fall into : current sampling time
  _bjx1 = _j_period+1;
  _j_period = 0;  
  
  _td = 0.2* _ts;
  
  _footxyz_real.row(0) = _footx_ref.transpose();
  _footxyz_real.row(1) = _footy_ref.transpose();  
  _footxyz_real.row(2) = _footz_ref.transpose();    
  _footxyz_real(1,0) = -_stepwidth(0);
  
  com_traj(0) = _comx(0,i);
  com_traj(1) = _comy(0,i);
  com_traj(2) = _comz(0,i);
  com_traj(3) = _comvx(0,i);
  com_traj(4) = _comvy(0,i);
  com_traj(5) = _comvz(0,i);
  com_traj(6) = _comax(0,i);
  com_traj(7) = _comay(0,i);
  com_traj(8) = _comaz(0,i); 

  com_traj(9) = _zmpx_real(0,i);
  com_traj(10) = _zmpy_real(0,i);
  com_traj(11) = _dcmx_real(0,i);
  com_traj(12) = _dcmy_real(0,i); 
  com_traj(13) = _zmpx_real(0,i+1);
  com_traj(14) = _zmpy_real(0,i+1);
  com_traj(15) = _dcmx_real(0,i+1);
  com_traj(16) = _dcmy_real(0,i+1); 

  com_traj(17) = _zmpx_real(0,i+2);
  com_traj(18) = _zmpy_real(0,i+2);
  com_traj(19) = _dcmx_real(0,i+2);
  com_traj(20) = _dcmy_real(0,i+2);

  com_traj(21) = _comax(0,i+1);
  com_traj(22) = _comay(0,i+1);
  com_traj(23) = _comaz(0,i+1); 
  com_traj(24) = _comax(0,i+2);
  com_traj(25) = _comay(0,i+2);
  com_traj(26) = _comaz(0,i+2);   
  
  com_traj(27) = _bjxx; 
  com_traj(28) = _footx_ref(_bjxx); 
  com_traj(29) = _footx_ref(_bjxx+1);  
  com_traj(30) = _footy_ref(_bjxx); 
  com_traj(31) = _footy_ref(_bjxx+1); 
  com_traj(32) = _footz_ref(_bjxx); 
  com_traj(33) = _footz_ref(_bjxx+1);  
  com_traj(34) = _periond_i-1; 
  com_traj(35) = _ts(_periond_i-1);  
  
  com_traj(36) = _Lxx_ref_real(i);
  com_traj(37) = _Lyy_ref_real(i);


  _comxyzx(0) = _comx(0,i);
  _comxyzx(1) = _comy(0,i);
  _comxyzx(2) = _comz(0,i);

  _comaxyzx(0) = _comax(0,i);
  _comaxyzx(1) = _comay(0,i);
  _comaxyzx(2) = _comaz(0,i);  
  
  return com_traj;
}


int NLPClass::Indexfind(double goalvari, int xyz)
{
  _j_period = 0;
  
  if (xyz<-0.5)
  {
      while (goalvari > (_tx(_j_period))+0.0001)
      {
	_j_period++;
      }    
      _j_period = _j_period-1;	    
  }
  else
  {
  
    if (xyz<0.05)
    {
      while (goalvari >= _tx(_j_period))
      {
	_j_period++;
      }    
      _j_period = _j_period-1;	  
    }
    else
    {
      while ( fabs(goalvari - _t_f(_j_period)) >0.0001 )
      {
	_j_period++;
      }	    
    }	  

      
  }

  return 0;

}


void NLPClass::step_timing_object_function(int i)
{
    _AxO(0) = _comx_feed(i-1)-_footx_ref(_periond_i-1); _BxO(0) = _comvx_feed(i-1)/_Wn; _Cx(0,0) =-0.5*_Lxx_refx; 
    _Axv = _Wn*_BxO;  _Bxv = _Wn*_AxO; _Cxv= -_comvx_endref; 
    _AyO(0) = _comy_feed(i-1)-_footy_ref(_periond_i-1); _ByO(0) = _comvy_feed(i-1)/_Wn; _Cy(0,0) =-0.5*_Lyy_refy; 
    _Ayv = _Wn*_ByO;  _Byv = _Wn*_AyO; _Cyv= - _comvy_endref;    
    
    _SQ_goal0(0,0)=0.5*_bbx;
    _SQ_goal0(1,1)=0.5*_bby;
    _SQ_goal0(2,2)=0.5*(_rr1+_aax*_AxO(0,0)*_AxO(0,0)+_aay*_AyO(0,0)*_AyO(0,0)+_aaxv*_Axv(0,0)*_Axv(0,0)+_aayv*_Ayv(0,0)*_Ayv(0,0));
    _SQ_goal0(2,3)=0.5*(     _aax*_AxO(0,0)*_BxO(0,0)+_aay*_AyO(0,0)*_ByO(0,0)+_aaxv*_Axv(0,0)*_Bxv(0,0)+_aayv*_Ayv(0,0)*_Byv(0,0));
    _SQ_goal0(3,2)=0.5*(     _aax*_BxO(0,0)*_AxO(0,0)+_aay*_ByO(0,0)*_AyO(0,0)+_aaxv*_Bxv(0,0)*_Axv(0,0)+_aayv*_Byv(0,0)*_Ayv(0,0));
    _SQ_goal0(3,3)=0.5*(_rr2+_aax*_BxO(0,0)*_BxO(0,0)+_aay*_ByO(0,0)*_ByO(0,0)+_aaxv*_Bxv(0,0)*_Bxv(0,0)+_aayv*_Byv(0,0)*_Byv(0,0));     
    
    _SQ_goal = (_SQ_goal0+_SQ_goal0.transpose())/2.0;  
    _Sq_goal << -_bbx*_Lxx_refx,
	      -_bby*_Lyy_refy,
	      -_rr1*_tr1_ref+_aax*_AxO(0,0)*_Cx(0,0)+_aay*_AyO(0,0)*_Cy(0,0)+_aaxv*_Axv(0,0)*_Cxv(0,0)+_aayv*_Ayv(0,0)*_Cyv(0,0),
	      -_rr2*_tr2_ref+_aax*_BxO(0,0)*_Cx(0,0)+_aay*_ByO(0,0)*_Cy(0,0)+_aaxv*_Bxv(0,0)*_Cxv(0,0)+_aayv*_Byv(0,0)*_Cyv(0,0);   
	      
    _SQ_goal1 = 2 * _SQ_goal;
    _Sq_goal1 = 2 * _SQ_goal * _vari_ini + _Sq_goal;
    
    // Matrix preparation
    _W_goal.block<4,4>(0,0) = _SQ_goal;
    _W_goal.block<1,4>(4,0) = 0.5 * _Sq_goal.transpose();
    _W_goal.block<4,1>(0,4) = 0.5 * _Sq_goal;
//    cout<<"_W_goal:"<<_W_goal<<endl;
    
}

void NLPClass::step_timing_constraints(int i)
{ 
// // %% constraints 
// // tr1 & tr2: equation constraints: transformed into inequality constraints     
//     _trx12_up2 = _SS3.transpose()*_SS3-_SS4.transpose()*_SS4; 
//     _trx12_up1.setZero();
//     _det_trx12_up(0,0) = 1;
    
//     _trx12_lp2 = -_trx12_up2;
//     _trx12_lp1.setZero();
//     _det_trx12_lp(0,0) = -1;    

   // tr1 & tr2: equation constraints:   
    _trx12 = (2*(_SS3.transpose()*_SS3-_SS4.transpose()*_SS4)*_vari_ini).transpose();
    _det_trx12 = -(_vari_ini.transpose()*(_SS3.transpose()*_SS3-_SS4.transpose()*_SS4)*_vari_ini);
    _det_trx12(0,0) +=1;
  
// // %% remaining time constraints: inequality constraints    
    _trx1_up = _SS3;
    _det_trx1_up = -(_SS3*_vari_ini);
    _det_trx1_up(0,0) +=  _tr1_max; 

    _trx1_lp = -_SS3; 
    _det_trx1_lp = -(-_SS3*_vari_ini); 
    _det_trx1_lp(0,0) -= _tr1_min;  	

    _trx2_up = _SS4;
    _det_trx2_up = -(_SS4*_vari_ini);
    _det_trx2_up(0,0) += _tr2_max;	

    _trx2_lp = -_SS4;
    _det_trx2_lp = -(-_SS4*_vari_ini);  
    _det_trx2_lp(0,0) -= _tr2_min;   	 
    
    _trx.row(0) = _trx1_up; _trx.row(1) = _trx1_lp; _trx.row(2) = _trx2_up; _trx.row(3) = _trx2_lp;	
    
    _det_trx.row(0) = _det_trx1_up; _det_trx.row(1) = _det_trx1_lp; _det_trx.row(2) = _det_trx2_up; _det_trx.row(3) = _det_trx2_lp;

    

//  foot location constraints      
    if (_periond_i % 2 == 0)
    {
      
	  if (i>=(round(2*_ts(1)/_dt))+1) ///update the footy_limit
	  {
	      
      _footy_min=-(2*RobotPara_HALF_HIP_WIDTH + 0.03); 
// 	    _footy_max=-(RobotPara_HALF_HIP_WIDTH - 0.03); 
	    _footy_max = -(RobotPara_FOOT_WIDTH+0.01);
	  }
	  else
	  {
	    _footy_min=-(2*RobotPara_HALF_HIP_WIDTH + 0.03); 
	    _footy_max=-(RobotPara_HALF_HIP_WIDTH - 0.03); 	    
	  }  
    }
    else
    { 
       if (i>=(round(2*_ts(1)/_dt))+1)
       {
        _footy_max=2*RobotPara_HALF_HIP_WIDTH + 0.03; 
      // 	_footy_min=RobotPara_HALF_HIP_WIDTH - 0.03; 
        _footy_min =RobotPara_FOOT_WIDTH+0.01;
      }
       else
       {
        _footy_max=2*RobotPara_HALF_HIP_WIDTH + 0.03; 
        _footy_min=  RobotPara_HALF_HIP_WIDTH - 0.03; 	 
      }     
      
    }
    
// DCM constraints
    if ((_t_min -_k_yu*_dt)>=0.001)
    {
      _cpx_max = _footx_max / (exp(_Wn * _t_min) - 1);
      _cpx_min = _footx_min / (exp(_Wn * _t_min) - 1);    
    }
    else
    {
      _cpx_max = _footx_max / (exp(_Wn * (0.001 + _k_yu*_dt)) - 1);
      _cpx_min = _footx_min / (exp(_Wn * (0.001 + _k_yu*_dt)) - 1);      
    }
    
    if (_periond_i % 2 ==1)
    {
      _lp = 2*RobotPara_HALF_HIP_WIDTH;
      _W_max = _footy_max - _lp;
      _W_min = _footy_min - _lp;
      if ((_t_min -_k_yu*_dt)>=0.001)
      {
        _cpy_max = -_lp / (exp(_Wn * _t_max)+1) + _W_max / (exp(2* _Wn*_t_min) - 1);
        _cpy_min = -_lp / (exp(_Wn * _t_min)+1) + _W_max / (exp(2* _Wn*_t_min) - 1);
      }
      else
      {
        _cpy_max = -_lp / (exp(_Wn * _t_max)+1) + _W_max / (exp(2* _Wn*(0.001 + _k_yu * _dt)) - 1);
        _cpy_min = -_lp / (exp(_Wn * (0.001 + _k_yu * _dt))+1) + _W_max / (exp(2* _Wn*(0.001 + _k_yu * _dt)) - 1);	
      }
    }
    else
    {
      _lp = -2*RobotPara_HALF_HIP_WIDTH;
      _W_max = _footy_max - _lp;
      _W_min = _footy_min - _lp;
      if ((_t_min -_k_yu*_dt)>=0.001)
      {
        _cpy_max = -_lp / (exp(_Wn * _t_min)+1) + _W_max / (exp(2* _Wn*_t_min) - 1);
        _cpy_min = -_lp / (exp(_Wn * _t_max)+1) + _W_max / (exp(2* _Wn*_t_min) - 1);
      }
      else
      {
        _cpy_max = -_lp / (exp(_Wn * (0.001 + _k_yu * _dt))+1) + _W_max / (exp(2* _Wn*(0.001 + _k_yu * _dt)) - 1);
        _cpy_min = -_lp / (exp(_Wn * _t_max)+1) + _W_max / (exp(2* _Wn*(0.001 + _k_yu * _dt)) - 1);	
      }      
    }

    // only the next one step
    _h_lx_up = _SS1;
    _det_h_lx_up = -(_SS1*_vari_ini);
    _det_h_lx_up(0,0) += _footx_max;

    _h_lx_lp = -_SS1;
    _det_h_lx_lp = -(-_SS1*_vari_ini); 
    _det_h_lx_lp(0,0) -= _footx_min;

    _h_ly_up = _SS2;
    _det_h_ly_up = -(_SS2*_vari_ini);
    _det_h_ly_up(0,0) += _footy_max;

    _h_ly_lp = -_SS2;
    _det_h_ly_lp = -(-_SS2*_vari_ini);
    _det_h_ly_lp(0,0) -= _footy_min;        

    _h_lx_upx.row(0)= _h_lx_up;    _h_lx_upx.row(1)= _h_lx_lp;   _h_lx_upx.row(2)= _h_ly_up;     _h_lx_upx.row(3)= _h_ly_lp;
    _det_h_lx_upx.row(0)=_det_h_lx_up;  _det_h_lx_upx.row(1)=_det_h_lx_lp;  _det_h_lx_upx.row(2)=_det_h_ly_up;  _det_h_lx_upx.row(3)=_det_h_ly_lp;   
    
// swing foot velocity boundary
    if (_k_yu ==0)
    {
      _h_lvx_up.setZero();  _h_lvx_lp.setZero(); _h_lvy_up.setZero(); _h_lvy_lp.setZero();
      _h_lvx_up1.setZero(); _h_lvx_lp1.setZero(); _h_lvy_up1.setZero(); _h_lvy_lp1.setZero();
      _det_h_lvx_up.setZero();_det_h_lvx_lp.setZero();_det_h_lvy_up.setZero();_det_h_lvy_lp.setZero();
      _det_h_lvx_up1.setZero();_det_h_lvx_lp1.setZero();_det_h_lvy_up1.setZero();_det_h_lvy_lp1.setZero();    
    }                
    else
    {   
      _h_lvx_up = _SS1;
      _det_h_lvx_up(0,0) = -(_SS1*_vari_ini-_Lxx_ref(_periond_i-1)- _footx_vmax*_dt);

      _h_lvx_lp = -_SS1;
      _det_h_lvx_lp(0,0) = _SS1*_vari_ini-_Lxx_ref(_periond_i-1)-_footx_vmin*_dt; 

      _h_lvy_up = _SS2;
      _det_h_lvy_up(0,0) = -(_SS2*_vari_ini-_Lyy_ref(_periond_i-1)- _footy_vmax*_dt);

      _h_lvy_lp = -_SS2;
      _det_h_lvy_lp(0,0) = _SS2*_vari_ini-_Lyy_ref(_periond_i-1)-_footy_vmin*_dt;  
    }                                   
    _h_lvx_upx.row(0)= _h_lvx_up;    _h_lvx_upx.row(1)= _h_lvx_lp;  _h_lvx_upx.row(2)= _h_lvy_up;   _h_lvx_upx.row(3)= _h_lvy_lp;
    _det_h_lvx_upx.row(0)=_det_h_lvx_up; _det_h_lvx_upx.row(1)=_det_h_lvx_lp; _det_h_lvx_upx.row(2)=_det_h_lvy_up; _det_h_lvx_upx.row(3)=_det_h_lvy_lp;    

////////////////////////// CoM position relative to the current support center
// CoM accelearation boundary    
    
    _AA= _Wn*sinh(_Wn*_dt); _CCx = _comx_feed(0,i-1)-_footx_ref(_periond_i-1,0); _BBx = pow(_Wn,2)*_CCx*cosh(_Wn*_dt); 
		            _CCy = _comy_feed(0,i-1)-_footy_ref(_periond_i-1,0); _BBy = pow(_Wn,2)*_CCy*cosh(_Wn*_dt);

    _AA1x = _AA*_Wn; _AA2x = -2* _AA*_CCx*_Wn;  _AA3x = 2*_BBx; 
    _AA1y = _AA*_Wn; _AA2y = -2* _AA*_CCy*_Wn;  _AA3y = 2*_BBy;


    _CoM_lax_up = _AA1x*_SS1+_AA2x*_SS3+(_AA3x-2*_comax_max)*_SS4;
    _det_CoM_lax_up = -(_AA1x*_SS1+_AA2x*_SS3+(_AA3x-2*_comax_max)*_SS4)*_vari_ini;

    _CoM_lax_lp = -_AA1x*_SS1-_AA2x*_SS3-(_AA3x-2*_comax_min)*_SS4;
    _det_CoM_lax_lp = -(-_AA1x*_SS1-_AA2x*_SS3-(_AA3x-2*_comax_min)*_SS4)*_vari_ini; 

    _CoM_lay_up = _AA1y*_SS2+_AA2y*_SS3+(_AA3y-2*_comay_max)*_SS4;
    _det_CoM_lay_up = -(_AA1y*_SS2+_AA2y*_SS3+(_AA3y-2*_comay_max)*_SS4)*_vari_ini;

    _CoM_lay_lp = -_AA1y*_SS2-_AA2y*_SS3-(_AA3y-2*_comay_min)*_SS4;
    _det_CoM_lay_lp = -(-_AA1y*_SS2-_AA2y*_SS3-(_AA3y-2*_comay_min)*_SS4)*_vari_ini;      
    
    _CoM_lax_upx.row(0) = _CoM_lax_up; _CoM_lax_upx.row(1) = _CoM_lax_lp; 
    _CoM_lax_upx.row(2) = _CoM_lay_up; _CoM_lax_upx.row(3) = _CoM_lay_lp;
    _det_CoM_lax_upx.row(0) = _det_CoM_lax_up;  _det_CoM_lax_upx.row(1) = _det_CoM_lax_lp; 
    _det_CoM_lax_upx.row(2) = _det_CoM_lay_up;  _det_CoM_lax_upx.row(3) = _det_CoM_lay_lp;
       
//   CoM velocity_inremental boundary  
    _VAA= cosh(_Wn*_dt); _VCCx = _comx_feed(0,i-1)-_footx_ref(_periond_i-1,0); _VBBx = _Wn*_VCCx*sinh(_Wn*_dt); 
		         _VCCy = _comy_feed(0,i-1)-_footy_ref(_periond_i-1,0); _VBBy = _Wn*_VCCy*sinh(_Wn*_dt);

    _VAA1x = _VAA*_Wn; _VAA2x = -2* _VAA*_VCCx*_Wn; _VAA3x = 2*_VBBx - 2*_comvx_feed(0,i-1); 
    _VAA1y = _VAA*_Wn; _VAA2y = -2* _VAA*_VCCy*_Wn; _VAA3y = 2*_VBBy - 2*_comvy_feed(0,i-1);

    _CoM_lvx_up = _VAA1x*_SS1+_VAA2x*_SS3+(_VAA3x-2*_comax_max*_dt)*_SS4;
    _det_CoM_lvx_up = -(_VAA1x*_SS1+_VAA2x*_SS3+(_VAA3x-2*_comax_max*_dt)*_SS4)*_vari_ini;

    _CoM_lvx_lp = -_VAA1x*_SS1-_VAA2x*_SS3-(_VAA3x-2*_comax_min*_dt)*_SS4;
    _det_CoM_lvx_lp = -(-_VAA1x*_SS1-_VAA2x*_SS3-(_VAA3x-2*_comax_min*_dt)*_SS4)*_vari_ini; 

    _CoM_lvy_up = _VAA1y*_SS2+_VAA2y*_SS3+(_VAA3y-2*_comay_max*_dt)*_SS4;
    _det_CoM_lvy_up = -(_VAA1y*_SS2+_VAA2y*_SS3+(_VAA3y-2*_comay_max*_dt)*_SS4)*_vari_ini;

    _CoM_lvy_lp = -_VAA1y*_SS2-_VAA2y*_SS3-(_VAA3y-2*_comay_min*_dt)*_SS4;
    _det_CoM_lvy_lp = -(-_VAA1y*_SS2-_VAA2y*_SS3-(_VAA3y-2*_comay_min*_dt)*_SS4)*_vari_ini;      
    
    _CoM_lvx_upx.row(0) = _CoM_lvx_up; _CoM_lvx_upx.row(1) = _CoM_lvx_lp; 
    _CoM_lvx_upx.row(2) = _CoM_lvy_up; _CoM_lvx_upx.row(3) = _CoM_lvy_lp;
    _det_CoM_lvx_upx.row(0) = _det_CoM_lvx_up;  _det_CoM_lvx_upx.row(1) = _det_CoM_lvx_lp; 
    _det_CoM_lvx_upx.row(2) = _det_CoM_lvy_up;  _det_CoM_lvx_upx.row(3) = _det_CoM_lvy_lp;  
    
    
    
//   CoM intial velocity boundary: check   
    _VAA1x1 = _Wn; _VAA2x1 = -2*_VCCx*_Wn; _VAA3x1 = - 2*_comvx_feed(0,i-1); 
    _VAA1y1 = _Wn; _VAA2y1 = -2*_VCCy*_Wn; _VAA3y1 = - 2*_comvy_feed(0,i-1);

    /// modified!!!
    _CoM_lvx_up1 = _VAA1x1*_SS1+_VAA2x1*_SS3+(_VAA3x1-2*_comax_max*_dt)*_SS4;
    _det_CoM_lvx_up1 = -(_VAA1x1*_SS1+_VAA2x1*_SS3+(_VAA3x1-2*_comax_max*_dt/2.0)*_SS4)*_vari_ini;

    _CoM_lvx_lp1 = -_VAA1x1*_SS1-_VAA2x1*_SS3-(_VAA3x1-2*_comax_min*_dt)*_SS4;
    _det_CoM_lvx_lp1 = -(-_VAA1x1*_SS1-_VAA2x1*_SS3-(_VAA3x1-2*_comax_min*_dt/2.0)*_SS4)*_vari_ini; 

    _CoM_lvy_up1 = _VAA1y1*_SS2+_VAA2y1*_SS3+(_VAA3y1-2*_comay_max*_dt)*_SS4;
    _det_CoM_lvy_up1 = -(_VAA1y1*_SS2+_VAA2y1*_SS3+(_VAA3y1-2*_comay_max*_dt/2.0)*_SS4)*_vari_ini;

    _CoM_lvy_lp1 = -_VAA1y1*_SS2-_VAA2y1*_SS3-(_VAA3y1-2*_comay_min*_dt)*_SS4;
    _det_CoM_lvy_lp1 = -(-_VAA1y1*_SS2-_VAA2y1*_SS3-(_VAA3y1-2*_comay_min*_dt/2.0)*_SS4)*_vari_ini;      
    
    _CoM_lvx_upx1.row(0) = _CoM_lvx_up1; _CoM_lvx_upx1.row(1) = _CoM_lvx_lp1; 
    _CoM_lvx_upx1.row(2) = _CoM_lvy_up1; _CoM_lvx_upx1.row(3) = _CoM_lvy_lp1;
    _det_CoM_lvx_upx1.row(0) = _det_CoM_lvx_up1;  _det_CoM_lvx_upx1.row(1) = _det_CoM_lvx_lp1; 
    _det_CoM_lvx_upx1.row(2) = _det_CoM_lvy_up1;  _det_CoM_lvx_upx1.row(3) = _det_CoM_lvy_lp1;         
    
  
    // DCM offset constraints:!!!!! not using
    _cp_x_up2 = 0.5 * _SS1.transpose() * (_SS3 - _SS4);  
    _cp_x_up1 = -_cpx_max * _SS4.transpose();  
    _det_cp_x_up1(0,0) = _VCCx;  
    
    _cp_x_lp2 = -_cp_x_up2;  
    _cp_x_lp1 = _cpx_min * _SS4.transpose();  
    _det_cp_x_lp1(0,0) = -_VCCx;
    
    _cp_y_up2 = 0.5 * _SS2.transpose() * (_SS3 - _SS4);  
    _cp_y_up1 = -_cpy_max * _SS4.transpose();  
    _det_cp_y_up1(0,0) = _VCCy;  
    
    _cp_y_lp2 = -_cp_y_up2;  
    _cp_y_lp1 = _cpy_min * _SS4.transpose();  
    _det_cp_y_lp1(0,0) = -_VCCy;     
    
    /////constraint transforms
    ///// quadratic constraints
    H2_q_xy[0] = _trx12_up2;  H1_q_xy.row(0) = _trx12_up1.transpose(); F_quadratci.row(0) = _det_trx12_up;
    H2_q_xy[1] = _trx12_lp2;  H1_q_xy.row(1) = _trx12_lp1.transpose(); F_quadratci.row(1) = _det_trx12_lp;
/*    H2_q_xy[2] = _cp_x_up2;   H1_q_xy.row(2) = _cp_x_up1.transpose();  F_quadratci.row(2) = _det_cp_x_up1;
    H2_q_xy[3] = _cp_x_lp2;   H1_q_xy.row(3) = _cp_x_lp1.transpose();  F_quadratci.row(3) = _det_cp_x_lp1;  */  
//    H2_q_xy[4] = _cp_y_up2;   H1_q_xy.row(4) = _cp_y_up1.transpose();  F_quadratci.row(4) = _det_cp_y_up1;
//     H2_q_xy[5] = _cp_y_lp2;   H1_q_xy.row(5) = _cp_y_lp1.transpose();  F_quadratci.row(5) = _det_cp_y_lp1; 
								       F_quadratci(6,0) = 1;
								       F_quadratci(7,0) = -1;        
    //// affine constratins
								       
  _A_q1.block<4,4>(0, 0) = _trx ;
  _A_q1.block<4,4>(4, 0) = _h_lx_upx ;
  _A_q1.block<4,4>(8, 0) = _h_lvx_upx ;
  _A_q1.block<4,4>(12, 0) = _CoM_lax_upx ;
  _A_q1.block<4,4>(16, 0) = _CoM_lvx_upx ;
  _A_q1.block<4,4>(20, 0) = _CoM_lvx_upx1 ;
								       
  _b_q1.block<4,1>(0, 0) = _det_trx;
  _b_q1.block<4,1>(4, 0) = _det_h_lx_upx;
  _b_q1.block<4,1>(8, 0) = _det_h_lvx_upx;
  _b_q1.block<4,1>(12, 0) = _det_CoM_lax_upx;
  _b_q1.block<4,1>(16, 0) = _det_CoM_lvx_upx;
  _b_q1.block<4,1>(20, 0) = _det_CoM_lvx_upx1;								       
    
    
}


void NLPClass::fun_sdr_qudratic_constraints()
{
  Eigen::Matrix<double,5,5> A_temp;
  
  for (int j = 1; j <= 8; j++)
  {
    if (j<=6)
    {
      A_temp.setZero();
      A_temp.block<4,4>(0,0) = H2_q_xy[j-1];
      A_temp.block<1,4>(4,0) = 0.5 * H1_q_xy.row(j-1);
      A_temp.block<4,1>(0,4) = 0.5 * (H1_q_xy.row(j-1)).transpose();
      cin_quad[j-1] = A_temp;	
    }
    else
    {
      if (j==7)
      {
	A_temp.setZero();
	A_temp(4,4) = 1;
	cin_quad[j-1] = A_temp;	  
      }
      else
      {
	A_temp.setZero();
	A_temp(4,4) = -1;
	cin_quad[j-1] = A_temp;	  
      }	
    }
  }  

}

void NLPClass::fun_sdr_affine_constraints()
{
  Eigen::Matrix<double,5,5> A_temp;
  
  for (int j = 1; j <= 24; j++)
  {
    A_temp.setZero();
    A_temp.block<1,4>(4,0) = 0.5 * _A_q1.row(j-1);
    A_temp.block<4,1>(0,4) = 0.5 * (_A_q1.row(j-1)).transpose();
    cin_aff[j-1] = A_temp;
  }
}


void NLPClass::solve_stepping_timing()
{
  
// ///// Mosek solution preparation
    	  
//   fun_sdr_qudratic_constraints();
  
//   fun_sdr_affine_constraints();  
  
//   // Objective function: min: tr<C,X>
//   //// Constraints: tr<A_i, X> =< B_i for i=1.....k; with A \in R(d*d), X \in R(d*d);  
//   int d_vari = 5, k_constraint = 32;
//   std::vector< std::shared_ptr<ndarray<double,2>> > A; 
//   std::vector< std::shared_ptr<ndarray<double,2>> > AX;
  
  
//   Model::t M = new Model(); auto _M = finally([&]() { M->dispose(); });
  
//   Variable::t X_mosek = M->variable(Domain::inPSDCone(d_vari));
  
  
//   /////// obj
//   auto A_goali = std::make_shared<ndarray<double,2>>(shape(d_vari,d_vari));
//       for(int s1=0; s1<d_vari; s1++)
// 	  for(int s2=0; s2<=s1; s2++)
// 	      (*A_goali)(s1,s2) = (*A_goali)(s2,s1) = _W_goal(s1,s2);
  
//   AX.push_back(A_goali);	  
//   M->objective(ObjectiveSense::Minimize, Expr::sum(Expr::mulElm(AX[0], X_mosek)));  
  
//   /// Constraints
//   for(int i=0; i<k_constraint; i++) {
//     Eigen::Matrix<double,5,5> A_temp;
//     A_temp.setZero();
//     if (i<8)
//     {
//       A_temp = cin_quad[i];
//     }
//     else
//     {
//       A_temp = cin_aff[i-8];
//     }
    
//     auto Ai = std::make_shared<ndarray<double,2>>(shape(d_vari,d_vari));
//     for(int s1=0; s1<d_vari; s1++)
// 	for(int s2=0; s2<=s1; s2++)
// 	    (*Ai)(s1,s2) = (*Ai)(s2,s1) = A_temp(s1,s2);
//     A.push_back(Ai);
//   }
  
  
//   for(int i=0; i<k_constraint; i++) {
//     if (i<8)
//     {
//       M->constraint(Expr::dot(A[i],X_mosek), Domain::lessThan(F_quadratci(i,0)));  
//     }
//     else
//     {
//       M->constraint(Expr::dot(A[i],X_mosek), Domain::lessThan(_b_q1(i-8,0)));  
//     }
//   }
  
   
//   /////
//   try{
    
//     M->solve();
    
//     auto Xj = *(X_mosek->level());
//     auto xsize = X_mosek->getSize();
//     std::cout<< "X_mosek successful "<<endl;
// //     std::cout<< "X_mosek size: "<<Xj[23]<<endl;
    
//     _vari_ini(0,0) = (double) Xj[20];
//     _vari_ini(1,0) = (double) Xj[21];
//     _vari_ini(2,0) = (double) Xj[22];
//     _vari_ini(3,0) = (double) Xj[23];    
//   }
//   catch(const OptimizeError& e)
//   { 
//     std::cout<< "optimization failed!!! Using the inital values"<<endl;
//     _vari_ini << _Lxx_refx,
//                  _Lyy_refy,
// 		 _tr1_ref,
// 		 _tr2_ref;    
//   }
//   catch(const SolutionError& e)
//   { 
//     std::cout<< "solution was not available!!! Using the inital values"<<endl;
//     _vari_ini << _Lxx_refx,
//                  _Lyy_refy,
// 		 _tr1_ref,
// 		 _tr2_ref;    
//   }
//   catch(const std::exception& e)
//   { 
//     std::cout<< "Unexpected error!!! Using the inital values"<<endl;
//     _vari_ini << _Lxx_refx,
//                  _Lyy_refy,
// 		 _tr1_ref,
// 		 _tr2_ref;    
//   }
// //   std::cout << "  _vari_ini = " << _vari_ini << std::endl;  


//////// SQP /////////////////////////////////////////////////////////////
/*  int nVars = 4;
  int nEqCon = 1;
  int nIneqCon = 24;
  resizeQP(nVars, nEqCon, nIneqCon);*/	    

  _G = _SQ_goal1;
  _g0 = _Sq_goal1;
  _X = _vari_ini;

// min 0.5 * x G x + g0^T x
// _s.t.
// 		CE^T x + ce0 = 0   ///// equality constraints
// 		CI^T x + ci0 >= 0  //// inequality constraints
  _CI = _A_q1.transpose() * (-1);
  
  _ci0 = _b_q1;

  
  _CE = _trx12.transpose()*(-1);

  _ce0 = _det_trx12;


 
  Solve(); 
}





void NLPClass::Solve()
{
// min 0.5 * x G x + g0^T x
// _s.t.
// 		CE^T x + ce0 = 0
// 		CI^T x + ci0 >= 0
		solveQP();

}


//// each time: planer for foot_trajectory:
void NLPClass::Foot_trajectory_solve(int j_index, bool _stopwalking)
{
  

	    
//   double  Footz_ref = 0.05;
     double  Footz_ref = 0.03;    //0.05m 
//    double  Footz_ref = 0.11;    //0.07m 
    
  _footxyz_real(1,0) = -_stepwidth(0);
  
  
//// judge if stop  
  if(_stopwalking)  
  {
    
    for (int i_t = _bjx1+1; i_t < _footstepsnumber; i_t++) {	  
      _lift_height_ref(i_t) = 0;  
    }	  

  }    
  
  
//   foot trajectory generation:
  if (_bjx1 >= 2)
  {
//     cout << "_bjx1 >= 2"<<endl;
    if (_bjx1 % 2 == 0)           //odd:left support
    {
      right_support = 0; 
//       cout << "left support"<<endl;
      _Lfootx(j_index) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
      _Lfooty(j_index) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
      _Lfootz(j_index) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);
      
      _Lfootx(j_index+1) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
      _Lfooty(j_index+1) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
      _Lfootz(j_index+1) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);   

        
      
      if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab: double support
      {
        right_support = 2; 
      // 	cout << "dsp"<<endl;
        _Rfootx(j_index) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
        _Rfooty(j_index) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
        _Rfootz(j_index) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);	
        
        _Rfootx(j_index+1) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
        _Rfooty(j_index+1) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
        _Rfootz(j_index+1) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);	
	
	      
      }
      else
      {
	
        // 	cout << "ssp"<<endl;
        //initial state and final state and the middle state
        double t_des = (j_index +1 - round(_tx(_bjx1-1)/_dt) +1)*_dt;
        Eigen::Vector3d t_plan;
        t_plan(0) = t_des - _dt;
        t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.0001;
        t_plan(2) = _ts(_bjx1-1) + 0.0001;
        
        if (abs(t_des - _ts(_bjx1-1)) <= (_dt + 0.0005))
        {
          _Rfootx(j_index) = _footxyz_real(0,_bjxx); 
          _Rfooty(j_index) = _footxyz_real(1,_bjxx);
          _Rfootz(j_index) = _footxyz_real(2,_bjxx); 	
          
          _Rfootx(j_index+1) = _footxyz_real(0,_bjxx); 
          _Rfooty(j_index+1) = _footxyz_real(1,_bjxx);
          _Rfootz(j_index+1) = _footxyz_real(2,_bjxx); 	  
          
        }
        else
        {
          Eigen::Matrix<double,7,7> AAA;
          AAA.setZero();
          Eigen::Matrix<double, 1, 7> aaaa;
          aaaa.setZero();
          
          aaaa(0) = 6*pow(t_plan(0), 5);   aaaa(1) =  5*pow(t_plan(0), 4);  aaaa(2) =  4*pow(t_plan(0), 3);   aaaa(3) =  3*pow(t_plan(0), 2);
          aaaa(4) = 2*pow(t_plan(0), 1);   aaaa(5) = 1;                     aaaa(6) =  0;
          AAA.row(0) = aaaa;
          
          aaaa(0) = 30*pow(t_plan(0), 4);  aaaa(1) =  20*pow(t_plan(0), 3);  aaaa(2) =  12*pow(t_plan(0), 2);   aaaa(3) =  6*pow(t_plan(0), 1);
          aaaa(4) = 2;                     aaaa(5) = 0;                      aaaa(6) =  0;
          AAA.row(1) = aaaa;
          
          aaaa(0) = pow(t_plan(0), 6);     aaaa(1) =  pow(t_plan(0), 5);     aaaa(2) =  pow(t_plan(0), 4);   aaaa(3) =  pow(t_plan(0), 3);
          aaaa(4) = pow(t_plan(0), 2);     aaaa(5) = pow(t_plan(0), 1);      aaaa(6) =  1;	  
          AAA.row(2) = aaaa;
          
          aaaa(0) = pow(t_plan(1), 6);     aaaa(1) =  pow(t_plan(1), 5);     aaaa(2) =  pow(t_plan(1), 4);   aaaa(3) =  pow(t_plan(1), 3);
          aaaa(4) = pow(t_plan(1), 2);     aaaa(5) = pow(t_plan(1), 1);      aaaa(6) =  1;
          AAA.row(3) = aaaa;
          
          aaaa(0) = pow(t_plan(2), 6);     aaaa(1) =  pow(t_plan(2), 5);     aaaa(2) =  pow(t_plan(2), 4);   aaaa(3) =  pow(t_plan(2), 3);
          aaaa(4) = pow(t_plan(2), 2);     aaaa(5) = pow(t_plan(2), 1);      aaaa(6) =  1;
          AAA.row(4) = aaaa;	  
          
          aaaa(0) = 6*pow(t_plan(2), 5);   aaaa(1) =  5*pow(t_plan(2), 4);  aaaa(2) =  4*pow(t_plan(2), 3);   aaaa(3) =  3*pow(t_plan(2), 2);
          aaaa(4) = 2*pow(t_plan(2), 1);   aaaa(5) = 1;                     aaaa(6) =  0;
          AAA.row(5) = aaaa;
          
          aaaa(0) = 30*pow(t_plan(2), 4);  aaaa(1) =  20*pow(t_plan(2), 3);  aaaa(2) =  12*pow(t_plan(2), 2);   aaaa(3) =  6*pow(t_plan(2), 1);
          aaaa(4) = 2;                     aaaa(5) = 0;                      aaaa(6) =  0;
          AAA.row(6) = aaaa;
          
          Eigen::Matrix<double,7,7> AAA_inv;
          AAA_inv = AAA.inverse();	  
          
          
                
            
          Eigen::Matrix<double, 1, 7> t_a_plan;
          t_a_plan.setZero();
          t_a_plan(0) = pow(t_des, 6);   t_a_plan(1) = pow(t_des, 5);   t_a_plan(2) = pow(t_des, 4);  t_a_plan(3) = pow(t_des, 3);
          t_a_plan(4) = pow(t_des, 2);   t_a_plan(5) = pow(t_des, 1);   t_a_plan(6) = 1;
          

          Eigen::Matrix<double, 1, 7> t_a_planv;
          t_a_planv.setZero();
          t_a_planv(0) = 6*pow(t_des, 5);   t_a_planv(1) = 5*pow(t_des, 4);   t_a_planv(2) = 4*pow(t_des, 3);  t_a_planv(3) = 3*pow(t_des, 2);
          t_a_planv(4) = 2*pow(t_des, 1);   t_a_planv(5) = 1;                 t_a_planv(6) = 0;
          
          
          Eigen::Matrix<double, 1, 7> t_a_plana;
          t_a_plana.setZero(7);
          t_a_plana(0) = 30*pow(t_des, 4);   t_a_plana(1) = 20*pow(t_des, 3);   t_a_plana(2) = 12*pow(t_des, 2);  t_a_plana(3) = 6*pow(t_des, 1);
          t_a_plana(4) = 2;                  t_a_plana(5) = 0;                  t_a_plana(6) = 0;
          
      // 	  cout <<"AAA="<<endl<<AAA<<endl;
      // 	  cout <<"AAA_inverse="<<endl<<AAA.inverse()<<endl;	  
      // 	  cout <<"t_des="<<endl<<t_des<<endl;
      // 	  cout <<"t_plan="<<endl<<t_plan<<endl;
          
          ////////////////////////////////////////////////////////////////////////////
          Eigen::Matrix<double, 7, 1> Rfootx_plan;
          Rfootx_plan.setZero();	
          Rfootx_plan(0) = _Rfootvx(j_index-1);     Rfootx_plan(1) = _Rfootax(j_index-1); Rfootx_plan(2) = _Rfootx(j_index-1); Rfootx_plan(3) = _Lfootx(j_index);
          Rfootx_plan(4) = _footxyz_real(0,_bjxx);  Rfootx_plan(5) = 0;                   Rfootx_plan(6) = 0;
          
          
          Eigen::Matrix<double, 7, 1> Rfootx_co;
          Rfootx_co.setZero();
          Rfootx_co = AAA_inv * Rfootx_plan;
          
          _Rfootx(j_index) = t_a_plan * Rfootx_co;
          _Rfootvx(j_index) = t_a_planv * Rfootx_co;
          _Rfootax(j_index) = t_a_plana * Rfootx_co;
          
          /////////////////////////////////////////////////////////////////////////////
          if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1)+_dt)
          {
            _ry_left_right = (_footxyz_real(1,_bjxx) + _footxyz_real(1,_bjxx-2))/2;
          }
          
          Eigen::Matrix<double, 7, 1> Rfooty_plan;
          Rfooty_plan.setZero();	
          Rfooty_plan(0) = _Rfootvy(j_index-1);     Rfooty_plan(1) = _Rfootay(j_index-1); Rfooty_plan(2) = _Rfooty(j_index-1); Rfooty_plan(3) = _ry_left_right;
          Rfooty_plan(4) = _footxyz_real(1,_bjxx);  Rfooty_plan(5) = 0;                   Rfooty_plan(6) = 0;	    
          
          Eigen::Matrix<double, 7, 1> Rfooty_co;
          Rfooty_co.setZero();
          Rfooty_co = AAA_inv * Rfooty_plan;
          
          _Rfooty(j_index) = t_a_plan * Rfooty_co;
          _Rfootvy(j_index) = t_a_planv * Rfooty_co;
          _Rfootay(j_index) = t_a_plana * Rfooty_co;	
          
          
          //////////////////////////////////////////////////////////
          Eigen::Matrix<double, 7, 1> Rfootz_plan;
          Rfootz_plan.setZero();	
          Rfootz_plan(0) = _Rfootvz(j_index-1);     Rfootz_plan(1) = _Rfootaz(j_index-1); Rfootz_plan(2) = _Rfootz(j_index-1); Rfootz_plan(3) = _Lfootz(j_index)+_lift_height_ref(_bjx1-1);
          Rfootz_plan(4) = _footxyz_real(2,_bjxx);  Rfootz_plan(5) = 0;                   Rfootz_plan(6) = 0;	
          
          Eigen::Matrix<double, 7, 1> Rfootz_co;
          Rfootz_co.setZero();
          Rfootz_co = AAA_inv * Rfootz_plan;
          
          _Rfootz(j_index) = t_a_plan * Rfootz_co;
          _Rfootvz(j_index) = t_a_planv * Rfootz_co;
          _Rfootaz(j_index) = t_a_plana * Rfootz_co;	
        
          
          _Rfootx(j_index+1) = _Rfootx(j_index)+_dt * _Rfootvx(j_index);
          _Rfooty(j_index+1) = _Rfooty(j_index)+_dt * _Rfootvy(j_index);
          _Rfootz(j_index+1) = _Rfootz(j_index)+_dt * _Rfootvz(j_index);
          
        }
      }   
    }
    
    else                       //right support
    {
      right_support = 1; 
//       cout << "right support"<<endl;
      _Rfootx(j_index) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
      _Rfooty(j_index) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
      _Rfootz(j_index) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);
      
      _Rfootx(j_index+1) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
      _Rfooty(j_index+1) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
      _Rfootz(j_index+1) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);      
      
      if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab: double suppot
      {
        right_support = 2; 
      // 	cout << "dsp"<<endl;
        _Lfootx(j_index) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
        _Lfooty(j_index) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
        _Lfootz(j_index) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);

        _Lfootx(j_index+1) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
        _Lfooty(j_index+1) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
        _Lfootz(j_index+1) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);
	
      }
      else
      {
            // 	cout << "ssp"<<endl;
        //initial state and final state and the middle state
        double t_des = (j_index +1 - round(_tx(_bjx1-1)/_dt) +1)*_dt;
        Eigen::Vector3d t_plan;
        t_plan(0) = t_des - _dt;
        t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.0001;
        t_plan(2) = _ts(_bjx1-1) + 0.0001;
        
        if (abs(t_des - _ts(_bjx1-1)) <= (_dt + 0.0005))
        {
          
          _Lfootx(j_index) = _footxyz_real(0,_bjxx); 
          _Lfooty(j_index) = _footxyz_real(1,_bjxx);
          _Lfootz(j_index) = _footxyz_real(2,_bjxx); 

          _Lfootx(j_index+1) = _footxyz_real(0,_bjxx); 
          _Lfooty(j_index+1) = _footxyz_real(1,_bjxx);
          _Lfootz(j_index+1) = _footxyz_real(2,_bjxx); 
          
        }
        else
        {
          Eigen::Matrix<double, 7, 7> AAA;
          AAA.setZero();
          Eigen::Matrix<double, 1, 7> aaaa;
          aaaa.setZero();
          
          aaaa(0) = 6*pow(t_plan(0), 5);   aaaa(1) =  5*pow(t_plan(0), 4);  aaaa(2) =  4*pow(t_plan(0), 3);   aaaa(3) =  3*pow(t_plan(0), 2);
          aaaa(4) = 2*pow(t_plan(0), 1);   aaaa(5) = 1;                     aaaa(6) =  0;
          AAA.row(0) = aaaa;
          
          aaaa(0) = 30*pow(t_plan(0), 4);  aaaa(1) =  20*pow(t_plan(0), 3);  aaaa(2) =  12*pow(t_plan(0), 2);   aaaa(3) =  6*pow(t_plan(0), 1);
          aaaa(4) = 2;                     aaaa(5) = 0;                      aaaa(6) =  0;
          AAA.row(1) = aaaa;
          
          aaaa(0) = pow(t_plan(0), 6);     aaaa(1) =  pow(t_plan(0), 5);     aaaa(2) =  pow(t_plan(0), 4);   aaaa(3) =  pow(t_plan(0), 3);
          aaaa(4) = pow(t_plan(0), 2);     aaaa(5) = pow(t_plan(0), 1);      aaaa(6) =  1;	  
          AAA.row(2) = aaaa;
          
          aaaa(0) = pow(t_plan(1), 6);     aaaa(1) =  pow(t_plan(1), 5);     aaaa(2) =  pow(t_plan(1), 4);   aaaa(3) =  pow(t_plan(1), 3);
          aaaa(4) = pow(t_plan(1), 2);     aaaa(5) = pow(t_plan(1), 1);      aaaa(6) =  1;
          AAA.row(3) = aaaa;
          
          aaaa(0) = pow(t_plan(2), 6);     aaaa(1) =  pow(t_plan(2), 5);     aaaa(2) =  pow(t_plan(2), 4);   aaaa(3) =  pow(t_plan(2), 3);
          aaaa(4) = pow(t_plan(2), 2);     aaaa(5) = pow(t_plan(2), 1);      aaaa(6) =  1;
          AAA.row(4) = aaaa;	  
          
          aaaa(0) = 6*pow(t_plan(2), 5);   aaaa(1) =  5*pow(t_plan(2), 4);  aaaa(2) =  4*pow(t_plan(2), 3);   aaaa(3) =  3*pow(t_plan(2), 2);
          aaaa(4) = 2*pow(t_plan(2), 1);   aaaa(5) = 1;                     aaaa(6) =  0;
          AAA.row(5) = aaaa;
          
          aaaa(0) = 30*pow(t_plan(2), 4);  aaaa(1) =  20*pow(t_plan(2), 3);  aaaa(2) =  12*pow(t_plan(2), 2);   aaaa(3) =  6*pow(t_plan(2), 1);
          aaaa(4) = 2;                     aaaa(5) = 0;                      aaaa(6) =  0;
          AAA.row(6) = aaaa;
          
          Eigen::Matrix<double,7,7> AAA_inv;
      // 	  AAA_inv.setZero(); 
          AAA_inv = AAA.inverse();          
            
          Eigen::Matrix<double, 1, 7> t_a_plan;
          t_a_plan.setZero();
          t_a_plan(0) = pow(t_des, 6);   t_a_plan(1) = pow(t_des, 5);   t_a_plan(2) = pow(t_des, 4);  t_a_plan(3) = pow(t_des, 3);
          t_a_plan(4) = pow(t_des, 2);   t_a_plan(5) = pow(t_des, 1);   t_a_plan(6) = 1;
          

          Eigen::Matrix<double, 1, 7> t_a_planv;
          t_a_planv.setZero();
          t_a_planv(0) = 6*pow(t_des, 5);   t_a_planv(1) = 5*pow(t_des, 4);   t_a_planv(2) = 4*pow(t_des, 3);  t_a_planv(3) = 3*pow(t_des, 2);
          t_a_planv(4) = 2*pow(t_des, 1);   t_a_planv(5) = 1;                 t_a_planv(6) = 0;
          
          
          Eigen::Matrix<double, 1, 7> t_a_plana;
          t_a_plana.setZero();
          t_a_plana(0) = 30*pow(t_des, 4);   t_a_plana(1) = 20*pow(t_des, 3);   t_a_plana(2) = 12*pow(t_des, 2);  t_a_plana(3) = 6*pow(t_des, 1);
          t_a_plana(4) = 2;                  t_a_plana(5) = 0;                  t_a_plana(6) = 0;
          
      // 	  cout <<"AAA="<<endl<<AAA<<endl;
      // 	  cout <<"AAA_inverse="<<endl<<AAA.inverse()<<endl;	  
      // 	  cout <<"t_des="<<endl<<t_des<<endl;
      // 	  cout <<"t_plan="<<endl<<t_plan<<endl;
          
          ////////////////////////////////////////////////////////////////////////////
          Eigen::Matrix<double, 7, 1> Lfootx_plan;
          Lfootx_plan.setZero();	
          Lfootx_plan(0) = _Lfootvx(j_index-1);     Lfootx_plan(1) = _Lfootax(j_index-1); Lfootx_plan(2) = _Lfootx(j_index-1); Lfootx_plan(3) = _Rfootx(j_index);
          Lfootx_plan(4) = _footxyz_real(0,_bjxx);  Lfootx_plan(5) = 0;                   Lfootx_plan(6) = 0;	  
          
          
          Eigen::Matrix<double, 7, 1> Lfootx_co;
          Lfootx_co.setZero();
          Lfootx_co = AAA_inv * Lfootx_plan;
          
          _Lfootx(j_index) = t_a_plan * Lfootx_co;
          _Lfootvx(j_index) = t_a_planv * Lfootx_co;
          _Lfootax(j_index) = t_a_plana * Lfootx_co;
          
          /////////////////////////////////////////////////////////////////////////////
          if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1)+_dt)
          {
            _ry_left_right = (_footxyz_real(1,_bjxx) + _footxyz_real(1,_bjxx-2))/2;
          }
          
          Eigen::Matrix<double, 7, 1> Lfooty_plan;
          Lfooty_plan.setZero();	
          Lfooty_plan(0) = _Lfootvy(j_index-1);     Lfooty_plan(1) = _Lfootay(j_index-1); Lfooty_plan(2) = _Lfooty(j_index-1); Lfooty_plan(3) = _ry_left_right;
          Lfooty_plan(4) = _footxyz_real(1,_bjxx);  Lfooty_plan(5) = 0;                   Lfooty_plan(6) = 0;		  
          
          
          Eigen::Matrix<double, 7, 1> Lfooty_co;
          Lfooty_co.setZero();
          Lfooty_co = AAA_inv * Lfooty_plan;
          
          _Lfooty(j_index) = t_a_plan * Lfooty_co;
          _Lfootvy(j_index) = t_a_planv * Lfooty_co;
          _Lfootay(j_index) = t_a_plana * Lfooty_co;	
          
          
          //////////////////////////////////////////////////////////
          Eigen::Matrix<double, 7, 1> Lfootz_plan;
          Lfootz_plan.setZero();			
          Lfootz_plan(0) = _Lfootvz(j_index-1);     Lfootz_plan(1) = _Lfootaz(j_index-1); Lfootz_plan(2) = _Lfootz(j_index-1); Lfootz_plan(3) = _Rfootz(j_index)+Footz_ref;
          Lfootz_plan(4) = _footxyz_real(2,_bjxx);  Lfootz_plan(5) = 0;                   Lfootz_plan(6) = 0;		  
          
          
          Eigen::VectorXd Lfootz_co;
          Lfootz_co.setZero(7);
          Lfootz_co = AAA_inv * Lfootz_plan;
          
          _Lfootz(j_index) = t_a_plan * Lfootz_co;
          _Lfootvz(j_index) = t_a_planv * Lfootz_co;
          _Lfootaz(j_index) = t_a_plana * Lfootz_co;
          
          
          _Lfootx(j_index+1) = _Lfootx(j_index)+_dt * _Lfootvx(j_index);
          _Lfooty(j_index+1) = _Lfooty(j_index)+_dt * _Lfootvy(j_index);
          _Lfootz(j_index+1) = _Lfootz(j_index)+_dt * _Lfootvz(j_index);
          
          
        }
      }

    }
      
  }
  else
  {
    right_support = 2; 
    _Rfooty(j_index) = -_stepwidth(0);
    _Lfooty(j_index) = _stepwidth(0);
  }
    

  
}


//// foot trajectory solve--------polynomial: to reduce the overfitting using 4 data ================================================
Eigen::Matrix<double, 18, 1> NLPClass::Foot_trajectory_solve_mod2(int j_index,bool _stopwalking)
{
  Eigen::Matrix<double, 18, 1> rffoot_traj;
//// judge if stop  
  if ((_stopwalking) ||(j_index > _t_end_footstep)) 
  {
    
    for (int i_t = _bjx1+1; i_t < _footstepsnumber; i_t++) {	  
      _lift_height_ref(i_t) = 0;  
    }	  

  }  

  _footxyz_real(1,0) = -_stepwidth(0);
  
//     /////// for stair climbing////
//     //0.05-0.06m    
//     if((_bjx1>=6)&&(_bjx1<=15))
//     {
//         _lift_height_ref(_bjx1-1) = 0.08;
//     }
//     else
//     {
//         if (_bjx1>15)
//         {
//             _lift_height_ref(_bjx1-1) = 0.1; 
//         }
//         else
//         {
//             _lift_height_ref(_bjx1-1) = 0.05; 
//         }
// 
//     }  
  
  
  
//   foot trajectory generation:
  if ((_bjx1 >= 2)&&(j_index <= _t_end_footstep))
  {
      if (_bjx1 % 2 == 0)           //odd:left support
      {
        right_support = 0; 
    //     no change on the left support location
        _Lfootx(j_index) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
        _Lfooty(j_index) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
        _Lfootz(j_index) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);
        
        _Lfootx(j_index+1) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
        _Lfooty(j_index+1) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
        _Lfootz(j_index+1) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);    
        
        /// right swing
        if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab: double support
        {
          right_support = 2; 
          _Rfootx(j_index) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
          _Rfooty(j_index) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
          _Rfootz(j_index) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);	
          
          _Rfootx(j_index+1) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
          _Rfooty(j_index+1) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
          _Rfootz(j_index+1) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);		  
        }
        else
        {
            double t_des = (j_index +1 - round(_tx(_bjx1-1)/_dt) +1)*_dt;
	          //cout<< "t_des"<<t_des<<endl;
            Eigen::Vector3d t_plan(0,0,0);
            t_plan(0) = t_des - _dt;
//             t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.0001;
//             t_plan(2) = _ts(_bjx1-1) + 0.0045;
            t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.0001;
            t_plan(2) = _ts(_bjx1-1) ;	    
            
            if (abs(t_des - _ts(_bjx1-1)) <= ( + 0.0005))
            {
                _Rfootx(j_index) = _footxyz_real(0,_bjxx); 
                _Rfooty(j_index) = _footxyz_real(1,_bjxx);
                _Rfootz(j_index) = _footxyz_real(2,_bjxx); 	
                
                _Rfootx(j_index+1) = _footxyz_real(0,_bjxx); 
                _Rfooty(j_index+1) = _footxyz_real(1,_bjxx);
                _Rfootz(j_index+1) = _footxyz_real(2,_bjxx); 	  	    
            }
            else
            {
                Eigen::Matrix<double,4,4> AAA_inv = solve_AAA_inv2(t_plan);
                    
                Eigen::Matrix<double, 1, 4> t_a_plan;
                t_a_plan << pow(t_des, 3), pow(t_des, 2), pow(t_des, 1), 1;
                
                Eigen::Matrix<double, 1, 4> t_a_planv;
                t_a_planv << 3*pow(t_des, 2), 2*pow(t_des, 1), 1, 0;

                Eigen::Matrix<double, 1, 4> t_a_plana;
                t_a_plana << 6*pow(t_des, 1), 2, 0, 0;
                
                ////////////////////////////////////////////////////////////////////////////
                Eigen::Matrix<double, 4, 1> Rfootx_plan;
                //Rfootx_plan << _Rfootx(j_index-1), _Lfootx(j_index), _footxyz_real(0,_bjxx), 0;
                Rfootx_plan << _Rfootx(j_index-1), (_footxyz_real(0,_bjxx-2)+ _footxyz_real(0,_bjxx))/2, _footxyz_real(0,_bjxx), 0;
                
                Eigen::Matrix<double, 4, 1> Rfootx_co;
                Rfootx_co.setZero();
                Rfootx_co = AAA_inv * Rfootx_plan;
                
                _Rfootx(j_index) = t_a_plan * Rfootx_co;
                _Rfootvx(j_index) = t_a_planv * Rfootx_co;
                _Rfootax(j_index) = t_a_plana * Rfootx_co;
                
                /////////////////////////////////////////////////////////////////////////////
                if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1)+_dt)
                {
                _ry_left_right = (_footxyz_real(1,_bjxx) + _footxyz_real(1,_bjxx-2))/2;
                }
                
                Eigen::Matrix<double, 4, 1> Rfooty_plan;
                Rfooty_plan << _Rfooty(j_index-1), _ry_left_right, _footxyz_real(1,_bjxx), 0;	    
                
                Eigen::Matrix<double, 4, 1> Rfooty_co;
                Rfooty_co.setZero();
                Rfooty_co = AAA_inv * Rfooty_plan;
                
                _Rfooty(j_index) = t_a_plan * Rfooty_co;
                _Rfootvy(j_index) = t_a_planv * Rfooty_co;
                _Rfootay(j_index) = t_a_plana * Rfooty_co;	
                
                
                //////////////////////////////////////////////////////////
                Eigen::Matrix<double, 4, 1> Rfootz_plan;
                // Rfootz_plan << _Rfootz(j_index-1), _Rfootvz(j_index-1), _Lfootz(j_index)+_lift_height_ref(_bjx1-1), _footxyz_real(2,_bjxx), 0;	
                Rfootz_plan << _Rfootz(j_index-1), max(_footxyz_real(2,_bjxx-2),_footxyz_real(2,_bjxx))+_lift_height_ref(_bjx1-1), _footxyz_real(2,_bjxx), 0;
                
                Eigen::Matrix<double, 4, 1> Rfootz_co;
                Rfootz_co.setZero();
                Rfootz_co = AAA_inv * Rfootz_plan;
                
                _Rfootz(j_index) = t_a_plan * Rfootz_co;
                _Rfootvz(j_index) = t_a_planv * Rfootz_co;
                _Rfootaz(j_index) = t_a_plana * Rfootz_co;	
                    
                
                _Rfootx(j_index+1) = _Rfootx(j_index)+_dt * _Rfootvx(j_index);
                _Rfooty(j_index+1) = _Rfooty(j_index)+_dt * _Rfootvy(j_index);
                _Rfootz(j_index+1) = _Rfootz(j_index)+_dt * _Rfootvz(j_index);	    
            }
        }
      }
      else                       //right support
      {
        right_support = 1; 
    //       no change on right support
        _Rfootx(j_index) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
        _Rfooty(j_index) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
        _Rfootz(j_index) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);
        
        _Rfootx(j_index+1) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
        _Rfooty(j_index+1) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
        _Rfootz(j_index+1) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);      
        
        /// left swing
        if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab: double suppot
        {
          right_support = 2; 
        // 	cout << "dsp"<<endl;
            _Lfootx(j_index) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
            _Lfooty(j_index) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
            _Lfootz(j_index) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);

            _Lfootx(j_index+1) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
            _Lfooty(j_index+1) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
            _Lfootz(j_index+1) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);
        
        }
        else
        {
        // 	cout << "ssp"<<endl;
            //initial state and final state and the middle state
            double t_des = (j_index +1 - round(_tx(_bjx1-1)/_dt) +1)*_dt;
	    //cout<< "t_des"<<t_des<<endl;
            Eigen::Vector3d t_plan(0,0,0);
            t_plan(0) = t_des - _dt;
//             t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.0001;
//             t_plan(2) = _ts(_bjx1-1) + 0.0045;
            t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.0001;
            t_plan(2) = _ts(_bjx1-1) ;	
            
            if (abs(t_des - _ts(_bjx1-1)) <= ( + 0.0005))
            {
                
                _Lfootx(j_index) = _footxyz_real(0,_bjxx); 
                _Lfooty(j_index) = _footxyz_real(1,_bjxx);
                _Lfootz(j_index) = _footxyz_real(2,_bjxx); 

                _Lfootx(j_index+1) = _footxyz_real(0,_bjxx); 
                _Lfooty(j_index+1) = _footxyz_real(1,_bjxx);
                _Lfootz(j_index+1) = _footxyz_real(2,_bjxx); 
                
            }
            else
            {
                Eigen::Matrix<double,4,4> AAA_inv = solve_AAA_inv2(t_plan);;
                    
                Eigen::Matrix<double, 1, 4> t_a_plan;
                t_a_plan << pow(t_des, 3), pow(t_des, 2), pow(t_des, 1), 1;
                

                Eigen::Matrix<double, 1, 4> t_a_planv;
                t_a_planv << 3*pow(t_des, 2), 2*pow(t_des, 1), 1, 0;
                
                
                Eigen::Matrix<double, 1, 4> t_a_plana;
                t_a_plana << 6*pow(t_des, 1), 2, 0, 0;
                
                
                ////////////////////////////////////////////////////////////////////////////
                Eigen::Matrix<double, 4, 1> Lfootx_plan;
                //Lfootx_plan << _Lfootx(j_index-1), _Rfootx(j_index), _footxyz_real(0,_bjxx), 0;	  
                Lfootx_plan << _Lfootx(j_index-1), (_footxyz_real(0,_bjxx-2)+ _footxyz_real(0,_bjxx))/2, _footxyz_real(0,_bjxx), 0;
                
                Eigen::Matrix<double, 4, 1> Lfootx_co;
                Lfootx_co.setZero();
                Lfootx_co = AAA_inv * Lfootx_plan;
                
                _Lfootx(j_index) = t_a_plan * Lfootx_co;
                _Lfootvx(j_index) = t_a_planv * Lfootx_co;
                _Lfootax(j_index) = t_a_plana * Lfootx_co;
                
                /////////////////////////////////////////////////////////////////////////////
                if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1)+_dt)
                {
                _ry_left_right = (_footxyz_real(1,_bjxx) + _footxyz_real(1,_bjxx-2))/2;
                }
                
                Eigen::Matrix<double, 4, 1> Lfooty_plan;
                Lfooty_plan << _Lfooty(j_index-1), _ry_left_right, _footxyz_real(1,_bjxx), 0;		  
                
                
                Eigen::Matrix<double, 4, 1> Lfooty_co;
                Lfooty_co.setZero();
                Lfooty_co = AAA_inv * Lfooty_plan;
                
                _Lfooty(j_index) = t_a_plan * Lfooty_co;
                _Lfootvy(j_index) = t_a_planv * Lfooty_co;
                _Lfootay(j_index) = t_a_plana * Lfooty_co;	
                
                
                //////////////////////////////////////////////////////////
                Eigen::Matrix<double, 4, 1> Lfootz_plan;
                // Lfootz_plan << _Lfootz(j_index-1), _Lfootvz(j_index-1), _Rfootz(j_index)+_lift_height_ref(_bjx1-1), _footxyz_real(2,_bjxx), 0;
                Lfootz_plan << _Lfootz(j_index-1), max(_footxyz_real(2,_bjxx-2),_footxyz_real(2,_bjxx))+_lift_height_ref(_bjx1-1), _footxyz_real(2,_bjxx), 0;		  
                
                
                Eigen::Matrix<double, 4, 1> Lfootz_co;
                Lfootz_co.setZero();
                Lfootz_co = AAA_inv * Lfootz_plan;
                
                _Lfootz(j_index) = t_a_plan * Lfootz_co;
                _Lfootvz(j_index) = t_a_planv * Lfootz_co;
                _Lfootaz(j_index) = t_a_plana * Lfootz_co;
                
            
                _Lfootx(j_index+1) = _Lfootx(j_index)+_dt * _Lfootvx(j_index);
                _Lfooty(j_index+1) = _Lfooty(j_index)+_dt * _Lfootvy(j_index);
                _Lfootz(j_index+1) = _Lfootz(j_index)+_dt * _Lfootvz(j_index);

            }
        }

      }

  }
  else
  {
    right_support = 2;
    if ((j_index > _t_end_footstep))
    {
      _Rfootx(j_index) = _Rfootx(j_index-1);
      _Rfooty(j_index) = _Rfooty(j_index-1);
      _Rfootz(j_index) = _Rfootz(j_index-1);
      _Lfootx(j_index) = _Lfootx(j_index-1);
      _Lfooty(j_index) = _Lfooty(j_index-1);
      _Lfootz(j_index) = _Lfootz(j_index-1);
    }
    else
    {
      _Rfooty(j_index) = -_stepwidth(0);
      _Lfooty(j_index) = _stepwidth(0);
    }

  }

  rffoot_traj(0,0) = _Rfootx(j_index);
  rffoot_traj(1,0) = _Rfooty(j_index);
  rffoot_traj(2,0) = _Rfootz(j_index);

  rffoot_traj(3,0) = _Lfootx(j_index);
  rffoot_traj(4,0) = _Lfooty(j_index);
  rffoot_traj(5,0) = _Lfootz(j_index);

  rffoot_traj(6,0) = _Rfootvx(j_index);
  rffoot_traj(7,0) = _Rfootvy(j_index);
  rffoot_traj(8,0) = _Rfootvz(j_index);

  rffoot_traj(9,0) = _Lfootvx(j_index);
  rffoot_traj(10,0) = _Lfootvy(j_index);
  rffoot_traj(11,0) = _Lfootvz(j_index);


  rffoot_traj(12,0) = _Rfootax(j_index);
  rffoot_traj(13,0) = _Rfootay(j_index);
  rffoot_traj(14,0) = _Rfootaz(j_index);

  rffoot_traj(15,0) = _Lfootax(j_index);
  rffoot_traj(16,0) = _Lfootay(j_index);
  rffoot_traj(17,0) = _Lfootaz(j_index);  

  return rffoot_traj;

}

//// each time: planer for CoM_height_trajectory: only the CoM_height_generation
void NLPClass::CoM_height_solve(int j_index, bool _stopwalking, int ntdx)
{     
  
//   CoM trajectory generation:
  if (_bjx1 >= 2)
  {      
//     if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt >= _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab: double suppot
//     {
//       _comz(j_index) = _footxyz_real(2,_bjx1-1)+_hcom;
//       _comz(j_index+1) = _footxyz_real(2,_bjx1-1)+_hcom;
//            
//     }
//     else
//     {
      //initial state and final state and the middle state: whole-cycle
      Eigen::Vector3d t_plan;
      t_plan(0) = 0.0001;
      t_plan(1) = _ts(_bjx1-1)/2 + 0.0001;
      t_plan(2) = _ts(_bjx1-1) + 0.0001;
      
      Eigen::Matrix<double, 7, 7> AAA;
      AAA.setZero();
      Eigen::Matrix<double, 1, 7> aaaa;
      aaaa.setZero();
      
      aaaa(0) = 6*pow(t_plan(0), 5);   aaaa(1) =  5*pow(t_plan(0), 4);  aaaa(2) =  4*pow(t_plan(0), 3);   aaaa(3) =  3*pow(t_plan(0), 2);
      aaaa(4) = 2*pow(t_plan(0), 1);   aaaa(5) = 1;                     aaaa(6) =  0;
      AAA.row(0) = aaaa;
      
      aaaa(0) = 30*pow(t_plan(0), 4);  aaaa(1) =  20*pow(t_plan(0), 3);  aaaa(2) =  12*pow(t_plan(0), 2);   aaaa(3) =  6*pow(t_plan(0), 1);
      aaaa(4) = 2;                     aaaa(5) = 0;                      aaaa(6) =  0;
      AAA.row(1) = aaaa;
      
      aaaa(0) = pow(t_plan(0), 6);     aaaa(1) =  pow(t_plan(0), 5);     aaaa(2) =  pow(t_plan(0), 4);   aaaa(3) =  pow(t_plan(0), 3);
      aaaa(4) = pow(t_plan(0), 2);     aaaa(5) = pow(t_plan(0), 1);      aaaa(6) =  1;	  
      AAA.row(2) = aaaa;
      
      aaaa(0) = pow(t_plan(1), 6);     aaaa(1) =  pow(t_plan(1), 5);     aaaa(2) =  pow(t_plan(1), 4);   aaaa(3) =  pow(t_plan(1), 3);
      aaaa(4) = pow(t_plan(1), 2);     aaaa(5) = pow(t_plan(1), 1);      aaaa(6) =  1;
      AAA.row(3) = aaaa;
      
      aaaa(0) = pow(t_plan(2), 6);     aaaa(1) =  pow(t_plan(2), 5);     aaaa(2) =  pow(t_plan(2), 4);   aaaa(3) =  pow(t_plan(2), 3);
      aaaa(4) = pow(t_plan(2), 2);     aaaa(5) = pow(t_plan(2), 1);      aaaa(6) =  1;
      AAA.row(4) = aaaa;	  
      
      aaaa(0) = 6*pow(t_plan(2), 5);   aaaa(1) =  5*pow(t_plan(2), 4);  aaaa(2) =  4*pow(t_plan(2), 3);   aaaa(3) =  3*pow(t_plan(2), 2);
      aaaa(4) = 2*pow(t_plan(2), 1);   aaaa(5) = 1;                     aaaa(6) =  0;
      AAA.row(5) = aaaa;
      
      aaaa(0) = 30*pow(t_plan(2), 4);  aaaa(1) =  20*pow(t_plan(2), 3);  aaaa(2) =  12*pow(t_plan(2), 2);   aaaa(3) =  6*pow(t_plan(2), 1);
      aaaa(4) = 2;                     aaaa(5) = 0;                      aaaa(6) =  0;
      AAA.row(6) = aaaa;
      
      Eigen::Matrix<double,7,7> AAA_inv;
      AAA_inv = AAA.inverse(); 
      

      Eigen::Matrix<double, 7, 1> comz_plan;
      comz_plan.setZero();			
      comz_plan(0) = 0;                 comz_plan(1) = 0;                 
      comz_plan(2) = _footxyz_real(2,_bjx1-2)+_hcom; 
      comz_plan(3) = (_footxyz_real(2,_bjx1-2)+_footxyz_real(2,_bjx1-1))/2+_hcom;      
      comz_plan(4) = _footxyz_real(2,_bjx1-1)+_hcom;  
      comz_plan(5) = 0;                 comz_plan(6) = 0;		  
      
      
      Eigen::VectorXd Lfootz_co;
      Lfootz_co.setZero(7);
      Lfootz_co = AAA_inv * comz_plan;      
      
      ////////////////////////////////////////////////////////////////////////////
      for (int jxx=1; jxx <=ntdx; jxx++)
      {
        double t_des = (j_index +jxx - round(_tx(_bjx1-1)/_dt) )*_dt;
        Eigen::Matrix<double, 1, 7> t_a_plan;
        t_a_plan.setZero();
        t_a_plan(0) = pow(t_des, 6);   t_a_plan(1) = pow(t_des, 5);   t_a_plan(2) = pow(t_des, 4);  t_a_plan(3) = pow(t_des, 3);
        t_a_plan(4) = pow(t_des, 2);   t_a_plan(5) = pow(t_des, 1);   t_a_plan(6) = 1;

        Eigen::Matrix<double, 1, 7> t_a_planv;
        t_a_planv.setZero();
        t_a_planv(0) = 6*pow(t_des, 5);   t_a_planv(1) = 5*pow(t_des, 4);   t_a_planv(2) = 4*pow(t_des, 3);  t_a_planv(3) = 3*pow(t_des, 2);
        t_a_planv(4) = 2*pow(t_des, 1);   t_a_planv(5) = 1;                 t_a_planv(6) = 0;
        
        Eigen::Matrix<double, 1, 7> t_a_plana;
        t_a_plana.setZero();
        t_a_plana(0) = 30*pow(t_des, 4);   t_a_plana(1) = 20*pow(t_des, 3);   t_a_plana(2) = 12*pow(t_des, 2);  t_a_plana(3) = 6*pow(t_des, 1);
        t_a_plana(4) = 2;                  t_a_plana(5) = 0;                  t_a_plana(6) = 0;
        
        _comz(j_index+jxx-1) = t_a_plan * Lfootz_co;
        _comvz(j_index+jxx-1) = t_a_planv * Lfootz_co;
        _comaz(j_index+jxx-1) = t_a_plana * Lfootz_co;          
       }
     
//     }
  }
  else
  {
    _comz(j_index) = _hcom;
    _comz(j_index+1) = _hcom;
    _comz(j_index+2) = _hcom;
    _comvz(j_index) = 0;
    _comvz(j_index+1) = 0;
    _comvz(j_index+2) = 0;  
    _comaz(j_index) = 0;
    _comaz(j_index+1) = 0;
    _comaz(j_index+2) = 0;     
    
  }
    

  
}

////////////////////////////////////////////////////// KMP : pos+vel generation
Eigen::Matrix<double,6,1> NLPClass::XGetSolution_Foot_position_KMP(int walktime, double dt_sample,int j_index)
{
  
  ///////walktime=====>ij;   int j_index====>i;  dt_sample========>dtx;   
   Eigen::Matrix<double,6,1> com_inte;	
 
  double  Footz_ref = _lift_height;    //0.05m 
 
//   
  //// three via_points: time, mean, sigma 
  vec via_point1 =  zeros<vec>(43);
  vec via_point2 =  zeros<vec>(43);
  vec via_point3 =  zeros<vec>(43);
  
  via_point1(7) =0.00000000001; via_point1(14)=0.00000000001; via_point1(21)=0.00000000001;	
  via_point1(28)=0.00000000001; via_point1(35)=0.00000000001; via_point1(42)=0.00000000001;  
  
  via_point2(0) =0.65/2;
  via_point2(7) =0.00000000001; via_point2(14)=0.00000000001; via_point2(21)=0.00000000001;	
  via_point2(28)=0.00000000001; via_point2(35)=0.00000000001; via_point2(42)=0.00000000001;
  
  via_point3(0) =0.65;
  via_point3(7) =0.00000000001; via_point3(14)=0.00000000001; via_point3(21)=0.00000000001;	
  via_point3(28)=0.00000000001; via_point3(35)=0.00000000001; via_point3(42)=0.00000000001;      
  
  
  
  double t_des;      /////////desired time during the current step
  t_des = (walktime)*dt_sample - (_tx(_bjx1-1)+_td(_bjx1-1));
  //cout << "t_des:"<< t_des<<endl;
  

  if (_bjx1 >= 2)
  {      
    
    if (_bjx1 % 2 == 0)           //odd:left support
    {
      _Lfootx_kmp(walktime) = _footxyz_real(0,_bjx1-1);
      _Lfooty_kmp(walktime) = _footxyz_real(1,_bjx1-1);
      _Lfootz_kmp(walktime) = _footxyz_real(2,_bjx1-1);
      
      _Lfootvx_kmp(walktime) = 0;
      _Lfootvy_kmp(walktime) = 0;
      _Lfootvz_kmp(walktime) = 0;    
      
      if (t_des<=0)  // j_index and _bjx1 coincident with matlab: double support
      {

        _Rfootx_kmp(walktime) = _footxyz_real(0,_bjx1-2);
        _Rfooty_kmp(walktime) = _footxyz_real(1,_bjx1-2);
        _Rfootz_kmp(walktime) = _footxyz_real(2,_bjx1-2);
        
        _Rfootvx_kmp(walktime) = 0;
        _Rfootvy_kmp(walktime) = 0;
        _Rfootvz_kmp(walktime) = 0;  
	
      }
      else
      {
        
        //initial state and final state and the middle state
        double t_des_k;
        t_des_k = (0.65)/((_ts(_bjx1-1)-_td(_bjx1-1)))*t_des;
        
        /////////////first sampling time of the current walking cycle: initialize the KMP_data
        if (t_des<=dt_sample)
        {
            kmp_leg_R.kmp_initialize(_data_kmp, _inDim_kmp, _outDim_kmp, _pvFlag_kmp,_lamda_kmp, _kh_kmp);
        //// %%% initial state and final state and the middle state
        /////%%%%% be careful the x position start from zero, the y
        //// %%%%% position start from -0.0726
            ////// add point************ current status****************////////
            via_point1(0) = (0.65)/((_ts(_bjx1-1)-_td(_bjx1-1)))*(t_des-dt_sample);
            via_point1(1) = _Rfootx_kmp(walktime-1)-_footxyz_real(0,_bjx1-2);
            via_point1(2) = _Rfooty_kmp(walktime-1)-(_footxyz_real(1,_bjx1-2)-(-0.0726));
            via_point1(3) = _Rfootz_kmp(walktime-1)-_footxyz_real(2,_bjx1-2);
            via_point1(4) = _Rfootvx_kmp(walktime-1);
            via_point1(5) = _Rfootvy_kmp(walktime-1);
            via_point1(6) = _Rfootvz_kmp(walktime-1);
            kmp_leg_R.kmp_insertPoint(via_point1);  // insert point into kmp
            
            ////// add point************ middle point***********////////
    // 	    via_point2(1) = _footxyz_real(0,_bjx1-1)-_footxyz_real(0,_bjx1-2);
            via_point2(1) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/2;
            via_point2(2) = (_footxyz_real(1,_bjx1-2)+_footxyz_real(1,_bjx1))/2-(_footxyz_real(1,_bjx1-2)-(-0.0726));
            via_point2(3) = (_footxyz_real(2,_bjx1-1)+Footz_ref)-_footxyz_real(2,_bjx1-2);
            via_point2(4) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/0.65*1.15;
            if (_bjx1==2)
            {
            via_point2(4) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/0.65*0.8;
            }	    
            via_point2(5) = (_footxyz_real(1,_bjx1)-_footxyz_real(1,_bjx1-2))/0.65;
    // 	    via_point2(6) = 0;
            if (_footxyz_real(2,_bjx1)-_footxyz_real(2,_bjx1-2)<=0)  ////downstairs
            {
            via_point2(6) = 0;
            }
            else
            {
            via_point2(6) = 0;
            }	    	    
            kmp_leg_R.kmp_insertPoint(via_point2);  // insert point into kmp

            ////// add point************ final status************////////	  
            via_point3(1) = _footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2)+0;
            via_point3(2) = _footxyz_real(1,_bjx1)-(_footxyz_real(1,_bjx1-2)-(-0.0726));
            via_point3(3) = _footxyz_real(2,_bjx1)-_footxyz_real(2,_bjx1-2);
            via_point3(4) = 0;
            via_point3(5) = 0;
    // 	   via_point3(6) = 0;
            if (via_point3(3)<0)  ////downstairs
            {
            via_point3(6) = 0.025;
            }
            else
            {
            via_point3(6) = 0.025;
            }
            
            
            kmp_leg_R.kmp_insertPoint(via_point3);  // insert point into kmp	
            
            kmp_leg_R.kmp_estimateMatrix();
        }

	  
	  
	_query_kmp(0) = t_des_k;
	kmp_leg_R.kmp_prediction(_query_kmp,_mean_kmp);   ////////time& predictive values
/*	    if (t_des_k<0.01){
	cout<<"_query_kmp:"<<endl<<trans(_query_kmp)<<endl;
	cout<<"kmp:"<<endl<<trans(_mean_kmp)<<endl;
	cout<<"error:"<<trans(_mean_kmp)-trans(via_point1(span(1,6)))<<endl<<endl;
	  
	  
	}*/	  
      


      
      _Rfootx_kmp(walktime) = _mean_kmp(0)+_footxyz_real(0,_bjx1-2);
      _Rfooty_kmp(walktime) = _mean_kmp(1)+(_footxyz_real(1,_bjx1-2)-(-0.0726));
      _Rfootz_kmp(walktime) = _mean_kmp(2)+_footxyz_real(2,_bjx1-2);
      
      _Rfootvx_kmp(walktime) = _mean_kmp(3);
      _Rfootvy_kmp(walktime) = _mean_kmp(4);
      _Rfootvz_kmp(walktime) = _mean_kmp(5); 	  
	  
	
	
      }   
    }
    else                       //right support
    {
      _Rfootx_kmp(walktime) = _footxyz_real(0,_bjx1-1);
      _Rfooty_kmp(walktime) = _footxyz_real(1,_bjx1-1);
      _Rfootz_kmp(walktime) = _footxyz_real(2,_bjx1-1);
      
      _Rfootvx_kmp(walktime) = 0;
      _Rfootvy_kmp(walktime) = 0;
      _Rfootvz_kmp(walktime) = 0;    
      
      if (t_des<=0)  // j_index and _bjx1 coincident with matlab: double support
      {

        _Lfootx_kmp(walktime) = _footxyz_real(0,_bjx1-2);
        _Lfooty_kmp(walktime) = _footxyz_real(1,_bjx1-2);
        _Lfootz_kmp(walktime) = _footxyz_real(2,_bjx1-2);
        
        _Lfootvx_kmp(walktime) = 0;
        _Lfootvy_kmp(walktime) = 0;
        _Lfootvz_kmp(walktime) = 0;  
	
      }
      else
      {
	
        //initial state and final state and the middle state
        double t_des_k;
        t_des_k = (0.65)/((_ts(_bjx1-1)-_td(_bjx1-1)))*t_des;
        
        if (t_des<=dt_sample)
        {	
            kmp_leg_L.kmp_initialize(_data_kmp, _inDim_kmp, _outDim_kmp, _pvFlag_kmp,_lamda_kmp, _kh_kmp);
                //// %%% initial state and final state and the middle state
                /////%%%%% be careful the x position start from zero, the y
                //// %%%%% position start from -0.0726
            ////// add point************ current status****************////////
            via_point1(0) = (0.65)/((_ts(_bjx1-1)-_td(_bjx1-1)))*(t_des-dt_sample);
            via_point1(1) = _Lfootx_kmp(walktime-1)-_footxyz_real(0,_bjx1-2);
            via_point1(2) = _Lfooty_kmp(walktime-1)-(_footxyz_real(1,_bjx1-2)-(-0.0726));
            via_point1(3) = _Lfootz_kmp(walktime-1)-_footxyz_real(2,_bjx1-2);
            via_point1(4) = _Lfootvx_kmp(walktime-1);
            via_point1(5) = _Lfootvy_kmp(walktime-1);
            via_point1(6) = _Lfootvz_kmp(walktime-1);
            kmp_leg_L.kmp_insertPoint(via_point1);  // insert point into kmp
            
            ////// add point************ middle point***********////////
        // 	  via_point2(1) = _footxyz_real(0,_bjx1-1)-_footxyz_real(0,_bjx1-2);
            via_point2(1) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/2;
            via_point2(2) = (_footxyz_real(1,_bjx1-2)+_footxyz_real(1,_bjx1))/2-(_footxyz_real(1,_bjx1-2)-(-0.0726));
            via_point2(3) = (_footxyz_real(2,_bjx1-1)+Footz_ref)-_footxyz_real(2,_bjx1-2);
            via_point2(4) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/0.65*1.45;
            via_point2(5) = (_footxyz_real(1,_bjx1)-_footxyz_real(1,_bjx1-2))/0.65;
        // 	  via_point2(6) = 0;
            if (_footxyz_real(2,_bjx1)-_footxyz_real(2,_bjx1-2)<=0)  ////downstairs
            {
                via_point2(6) = 0;	      
            }
            else
            {
                via_point2(6) = 0;
            }	  	  	  
            kmp_leg_L.kmp_insertPoint(via_point2);  // insert point into kmp

                ////// add point************ final status************////////	  
            via_point3(1) = _footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2)+0.000;
            via_point3(2) = _footxyz_real(1,_bjx1)-(_footxyz_real(1,_bjx1-2)-(-0.0726));
            via_point3(3) = _footxyz_real(2,_bjx1)-_footxyz_real(2,_bjx1-2);
            via_point3(4) = 0;
            via_point3(5) = 0;
        // 	   via_point3(6) = 0;
            if (via_point3(3)<0)  ////downstairs
            {
                via_point3(6) = 0.025;
            }
            else
            {
                via_point3(6) = 0.025;
            }	     	 
            kmp_leg_L.kmp_insertPoint(via_point3);  // insert point into kmp
            
	    //cout<<"right leg_point_insert finishing"<<endl;
            
            kmp_leg_L.kmp_estimateMatrix();
	    //cout<<"right leg_point_estimantion finishing"<<endl;
        
        
        }
	
        _query_kmp(0) = t_des_k;
        kmp_leg_L.kmp_prediction(_query_kmp,_mean_kmp);   ////////time& predictive values
        
    // 	  if (t_des_k<0.01){
    // 	  cout<<"_query_kmp:"<<endl<<trans(_query_kmp)<<endl;
    // 	  cout<<"kmp:"<<endl<<trans(_mean_kmp)<<endl;
    // 	  cout<<"error:"<<endl<<trans(_mean_kmp)-trans(via_point1(span(1,6)))<<endl;	
    // 	  }	
        
        _Lfootx_kmp(walktime) = _mean_kmp(0)+_footxyz_real(0,_bjx1-2);
        _Lfooty_kmp(walktime) = _mean_kmp(1)+(_footxyz_real(1,_bjx1-2)-(-0.0726));
        _Lfootz_kmp(walktime) = _mean_kmp(2)+_footxyz_real(2,_bjx1-2);
        
        _Lfootvx_kmp(walktime) = _mean_kmp(3);
        _Lfootvy_kmp(walktime) = _mean_kmp(4);
        _Lfootvz_kmp(walktime) = _mean_kmp(5); 	  

	
	
      }   

    }
      
    
  }
  
  
  /////Rfoot,xyz, Lfoot,XYZ
  com_inte(0) = _Rfootx_kmp(walktime);
  com_inte(1) = _Rfooty_kmp(walktime);
  com_inte(2) = _Rfootz_kmp(walktime);

  com_inte(3) = _Lfootx_kmp(walktime);
  com_inte(4) = _Lfooty_kmp(walktime);
  com_inte(5) = _Lfootz_kmp(walktime);

  _Lfootxyzx(0) = _Lfootx_kmp(walktime);
  _Lfootxyzx(1) = _Lfooty_kmp(walktime);
  _Lfootxyzx(2) = _Lfootz_kmp(walktime);

  _Rfootxyzx(0) = _Rfootx_kmp(walktime);
  _Rfootxyzx(1) = _Rfooty_kmp(walktime);
  _Rfootxyzx(2) = _Rfootz_kmp(walktime);


  return com_inte;

}

////////////////////////////////////////////////////// KMP : pos generation
Eigen::Matrix<double,6,1> NLPClass::XGetSolution_Foot_position_KMP_faster(int walktime, double dt_sample,int j_index)
{
  
  ///////walktime=====>ij;   int j_index====>i;  dt_sample========>dtx;   
   Eigen::Matrix<double,6,1> com_inte;	
 
  double  Footz_ref = _lift_height;    //0.05m 

  //// three via_points: time, mean, sigma 
  vec via_point1 =  zeros<vec>(13);
  vec via_point2 =  zeros<vec>(13);
  vec via_point3 =  zeros<vec>(13);
  
  via_point1(4) =0.00000000001; via_point1(8)=0.00000000001; via_point1(12)=0.00000000001;	
  
  via_point2(0) =0.65/2;
  via_point2(4) =0.00000000001; via_point2(8)=0.00000000001; via_point2(12)=0.00000000001;	
  
  via_point3(0) =0.65;
  via_point3(4) =0.00000000001; via_point3(8)=0.00000000001; via_point3(12)=0.00000000001;	    
  
  
  
  double t_des;      /////////desired time during the current step
  t_des = (walktime)*dt_sample - (_tx(_bjx1-1)+_td(_bjx1-1));
  //cout << "t_des:"<< t_des<<endl;
  

  if (_bjx1 >= 2)
  {      
    
    if (_bjx1 % 2 == 0)           //odd:left support
    {
      _Lfootx_kmp(walktime) = _footxyz_real(0,_bjx1-1);
      _Lfooty_kmp(walktime) = _footxyz_real(1,_bjx1-1);
      _Lfootz_kmp(walktime) = _footxyz_real(2,_bjx1-1);  
      
      if (t_des<=0)  // j_index and _bjx1 coincident with matlab: double support
      {
        _Rfootx_kmp(walktime) = _footxyz_real(0,_bjx1-2);
        _Rfooty_kmp(walktime) = _footxyz_real(1,_bjx1-2);
        _Rfootz_kmp(walktime) = _footxyz_real(2,_bjx1-2);
      }
      else
      {
	if (t_des>=(_ts(_bjx1-1) - 2*dt_sample))  // j_index and _bjx1 coincident with matlab: double support
	{
	  _Rfootx_kmp(walktime) = _footxyz_real(0,_bjx1);
	  _Rfooty_kmp(walktime) = _footxyz_real(1,_bjx1);
	  _Rfootz_kmp(walktime) = _footxyz_real(2,_bjx1);	  
	}
	else
	{
	  //initial state and final state and the middle state
	  double t_des_k;
	  t_des_k = (0.65)/((_ts(_bjx1-1)-_td(_bjx1-1)))*t_des;
	  
	  /////////////first sampling time of the current walking cycle: initialize the KMP_data
	  if (t_des<=dt_sample)
	  {
	      kmp_leg_R.kmp_initialize(_data_kmp, _inDim_kmp, _outDim_kmp, _pvFlag_kmp,_lamda_kmp, _kh_kmp);
	  }
	  //// %%% initial state and final state and the middle state
	  /////%%%%% be careful the x position start from zero, the y
	  //// %%%%% position start from -0.0726
	      ////// add point************ current status****************////////
	      via_point1(0) = (0.65)/((_ts(_bjx1-1)-_td(_bjx1-1)))*(t_des-dt_sample);
	      via_point1(1) = _Rfootx_kmp(walktime-1)-_footxyz_real(0,_bjx1-2);
	      via_point1(2) = _Rfooty_kmp(walktime-1)-(_footxyz_real(1,_bjx1-2)-(-0.0726));
	      via_point1(3) = _Rfootz_kmp(walktime-1)-_footxyz_real(2,_bjx1-2);
	      kmp_leg_R.kmp_insertPoint(via_point1);  // insert point into kmp
	      
	      ////// add point************ middle point***********////////
      // 	    via_point2(1) = _footxyz_real(0,_bjx1-1)-_footxyz_real(0,_bjx1-2);
	      via_point2(1) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/2;
	      via_point2(2) = (_footxyz_real(1,_bjx1-2)+_footxyz_real(1,_bjx1))/2-(_footxyz_real(1,_bjx1-2)-(-0.0726));
	      via_point2(3) = (_footxyz_real(2,_bjx1-1)+Footz_ref)-_footxyz_real(2,_bjx1-2);	    	    
	      kmp_leg_R.kmp_insertPoint(via_point2);  // insert point into kmp

	      ////// add point************ final status************////////	  
	      via_point3(1) = _footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2)+0;
	      via_point3(2) = _footxyz_real(1,_bjx1)-(_footxyz_real(1,_bjx1-2)-(-0.0726));
	      via_point3(3) = _footxyz_real(2,_bjx1)-_footxyz_real(2,_bjx1-2);     
	      kmp_leg_R.kmp_insertPoint(via_point3);  // insert point into kmp	
	      
	      kmp_leg_R.kmp_estimateMatrix();
	  
	  _query_kmp(0) = t_des_k;
	  kmp_leg_R.kmp_prediction(_query_kmp,_mean_kmp);   ////////time& predictive values

	 _Rfootx_kmp(walktime) = _mean_kmp(0)+_footxyz_real(0,_bjx1-2);
	 _Rfooty_kmp(walktime) = _mean_kmp(1)+(_footxyz_real(1,_bjx1-2)-(-0.0726));
	 _Rfootz_kmp(walktime) = _mean_kmp(2)+_footxyz_real(2,_bjx1-2);	  
	}

      }   
    }
    else                       //right support
    {
      _Rfootx_kmp(walktime) = _footxyz_real(0,_bjx1-1);
      _Rfooty_kmp(walktime) = _footxyz_real(1,_bjx1-1);
      _Rfootz_kmp(walktime) = _footxyz_real(2,_bjx1-1);    
      
      if (t_des<=0)  // j_index and _bjx1 coincident with matlab: double support
      {
        _Lfootx_kmp(walktime) = _footxyz_real(0,_bjx1-2);
        _Lfooty_kmp(walktime) = _footxyz_real(1,_bjx1-2);
        _Lfootz_kmp(walktime) = _footxyz_real(2,_bjx1-2);	
      }
      else
      {	
	if (t_des>=(_ts(_bjx1-1) - 2*dt_sample))  // j_index and _bjx1 coincident with matlab: double support
	{
	  _Lfootx_kmp(walktime) = _footxyz_real(0,_bjx1);
	  _Lfooty_kmp(walktime) = _footxyz_real(1,_bjx1);
	  _Lfootz_kmp(walktime) = _footxyz_real(2,_bjx1);	  
	}	
	else
	{
	  //initial state and final state and the middle state
	  double t_des_k;
	  t_des_k = (0.65)/((_ts(_bjx1-1)-_td(_bjx1-1)))*t_des;
	  
	  if (t_des<=dt_sample)
	  {	
	      kmp_leg_L.kmp_initialize(_data_kmp, _inDim_kmp, _outDim_kmp, _pvFlag_kmp,_lamda_kmp, _kh_kmp);
	  }
		  //// %%% initial state and final state and the middle state
		  /////%%%%% be careful the x position start from zero, the y
		  //// %%%%% position start from -0.0726
	      ////// add point************ current status****************////////
	      via_point1(0) = (0.65)/((_ts(_bjx1-1)-_td(_bjx1-1)))*(t_des-dt_sample);
	      via_point1(1) = _Lfootx_kmp(walktime-1)-_footxyz_real(0,_bjx1-2);
	      via_point1(2) = _Lfooty_kmp(walktime-1)-(_footxyz_real(1,_bjx1-2)-(-0.0726));
	      via_point1(3) = _Lfootz_kmp(walktime-1)-_footxyz_real(2,_bjx1-2);
	      kmp_leg_L.kmp_insertPoint(via_point1);  // insert point into kmp
	      
	      ////// add point************ middle point***********////////
	  // 	  via_point2(1) = _footxyz_real(0,_bjx1-1)-_footxyz_real(0,_bjx1-2);
	      via_point2(1) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/2;
	      via_point2(2) = (_footxyz_real(1,_bjx1-2)+_footxyz_real(1,_bjx1))/2-(_footxyz_real(1,_bjx1-2)-(-0.0726));
	      via_point2(3) = (_footxyz_real(2,_bjx1-1)+Footz_ref)-_footxyz_real(2,_bjx1-2);  	  	  
	      kmp_leg_L.kmp_insertPoint(via_point2);  // insert point into kmp

		  ////// add point************ final status************////////	  
	      via_point3(1) = _footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2)+0.000;
	      via_point3(2) = _footxyz_real(1,_bjx1)-(_footxyz_real(1,_bjx1-2)-(-0.0726));
	      via_point3(3) = _footxyz_real(2,_bjx1)-_footxyz_real(2,_bjx1-2);	     	 
	      kmp_leg_L.kmp_insertPoint(via_point3);  // insert point into kmp        
	      //cout<<"right leg_point_insert finishing"<<endl;        
	      kmp_leg_L.kmp_estimateMatrix();
	      //cout<<"right leg_point_estimantion finishing"<<endl;
	  
	  
	  _query_kmp(0) = t_des_k;
	  kmp_leg_L.kmp_prediction(_query_kmp,_mean_kmp);   ////////time& predictive values
	  
	  _Lfootx_kmp(walktime) = _mean_kmp(0)+_footxyz_real(0,_bjx1-2);
	  _Lfooty_kmp(walktime) = _mean_kmp(1)+(_footxyz_real(1,_bjx1-2)-(-0.0726));
	  _Lfootz_kmp(walktime) = _mean_kmp(2)+_footxyz_real(2,_bjx1-2);		  
	}

      }   

    }
      
    
  }
  
  
  /////Rfoot,xyz, Lfoot,XYZ
  com_inte(0) = _Rfootx_kmp(walktime);
  com_inte(1) = _Rfooty_kmp(walktime);
  com_inte(2) = _Rfootz_kmp(walktime);

  com_inte(3) = _Lfootx_kmp(walktime);
  com_inte(4) = _Lfooty_kmp(walktime);
  com_inte(5) = _Lfootz_kmp(walktime);

  _Lfootxyzx(0) = _Lfootx_kmp(walktime);
  _Lfootxyzx(1) = _Lfooty_kmp(walktime);
  _Lfootxyzx(2) = _Lfootz_kmp(walktime);

  _Rfootxyzx(0) = _Rfootx_kmp(walktime);
  _Rfootxyzx(1) = _Rfooty_kmp(walktime);
  _Rfootxyzx(2) = _Rfootz_kmp(walktime);


  return com_inte;

}

Eigen::Matrix<double, 9, 1> NLPClass::X_CoM_position_squat(int walktime, double dt_sample)
{
  Eigen::Matrix<double, 9, 1> com_inte;
  com_inte.setZero();

  double t_des  = walktime * dt_sample;

  Eigen::Vector3d t_plan;
  t_plan(0) = 0.00001;
  t_plan(1) = _height_squat_time/2+0.0001;
  t_plan(2) = _height_squat_time+0.0001;

  if (t_des<=_height_squat_time)
  {   
    Eigen::Matrix<double,7,7> AAA_inv = solve_AAA_inv_x(t_plan);
	    
    Eigen::Matrix<double, 1, 7> t_a_plan;
    t_a_plan.setZero();
    t_a_plan(0) = pow(t_des, 6);   t_a_plan(1) = pow(t_des, 5);   t_a_plan(2) = pow(t_des, 4);  t_a_plan(3) = pow(t_des, 3);
    t_a_plan(4) = pow(t_des, 2);   t_a_plan(5) = pow(t_des, 1);   t_a_plan(6) = 1;

    Eigen::Matrix<double, 1, 7> t_a_planv;
    t_a_planv.setZero();
    t_a_planv(0) = 6*pow(t_des, 5);   t_a_planv(1) = 5*pow(t_des, 4);   t_a_planv(2) = 4*pow(t_des, 3);  t_a_planv(3) = 3*pow(t_des, 2);
    t_a_planv(4) = 2*pow(t_des, 1);   t_a_planv(5) = 1;                 t_a_planv(6) = 0;

    Eigen::Matrix<double, 1, 7> t_a_plana;
    t_a_plana.setZero();
    t_a_plana(0) = 30*pow(t_des, 4);   t_a_plana(1) = 20*pow(t_des, 3);   t_a_plana(2) = 12*pow(t_des, 2);  t_a_plana(3) = 6*pow(t_des, 1);
    t_a_plana(4) = 2;                  t_a_plana(5) = 0;                  t_a_plana(6) = 0; 
    
    ////////////////////////////////////////////////////////////////////////////
    Eigen::Matrix<double, 7, 1> com_squat_plan;
    com_squat_plan.setZero();	
    com_squat_plan(0) = 0;     com_squat_plan(1) = 0; 
    com_squat_plan(2) = RobotPara_Z_C; 
    com_squat_plan(3) = RobotPara_Z_C-_height_offset/2;
    com_squat_plan(4) = RobotPara_Z_C-_height_offset;  
    com_squat_plan(5) = 0;                   
    com_squat_plan(6) = 0;


    Eigen::Matrix<double, 7, 1> com_co;
    com_co.setZero();
    com_co = AAA_inv * com_squat_plan;

    com_inte(2) = t_a_plan * com_co;
    com_inte(5) = t_a_planv * com_co;
    com_inte(8) = t_a_plana * com_co;    
    
  }
  else
  {
    com_inte(2) = RobotPara_Z_C-_height_offset;   
  }
  
  return com_inte; 
}



int NLPClass::Get_maximal_number_reference()
{
  int nsum_max;
  nsum_max = (_nsum -_n_loop_omit-1);  
  return nsum_max;
}

int NLPClass::Get_maximal_number(double dtx)
{
  int nsum_max;
  nsum_max = (_nsum -_n_loop_omit-1)*floor(_dt/dtx);
  
  return nsum_max;
}


////====================================================================================================================
/////////////////////////// using the lower-level control-loop  sampling time as the reference: every 5ms;  at the same time: just using the next one position + next one velocity

Vector3d NLPClass::XGetSolution_CoM_position(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{
  //reference com position
        _CoM_position_optimal.row(0) = _comx;
	_CoM_position_optimal.row(1) = _comy;
	_CoM_position_optimal.row(2) = _comz;
	
	Vector3d com_inte;	
	
	if (walktime>=2)
	{
	  int t_int; 
// 	  t_int = floor(walktime / (_dt / dt_sample) );
	  t_int = floor(walktime* dt_sample/ _dt);	  
//          cout <<"T_int_inter:"<<t_int<<endl;
	  ///// chage to be relative time
	  double t_cur;
	  t_cur = walktime * dt_sample ;
	  

	  Eigen::Matrix<double, 4, 1> t_plan;
	  t_plan.setZero();
	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);
          


	  Eigen::Matrix4d AAA_inv;
	  
	  double abx1, abx2, abx3, abx4;
	  abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
	  abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
	  abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
	  abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
	  

	  AAA_inv(0,0) = 1/ abx1;
	  AAA_inv(0,1) =  -1/ abx2;
	  AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
	  AAA_inv(0,3) = 1/ abx4;
	  
	  AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
	  AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
	  AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
	  AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
	  
	  AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
	  AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
	  AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
	  AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
	  
	  AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
	  AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
	  AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
	  AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;
	  	  
	  	  
	
	  
	  
	  Eigen::Matrix<double, 1, 4> t_a_plan;
	  t_a_plan.setZero();
	  t_a_plan(0) = pow(t_cur-( t_cur - 2*dt_sample), 3);   
	  t_a_plan(1) = pow(t_cur-( t_cur - 2*dt_sample), 2);   
	  t_a_plan(2) = pow(t_cur-( t_cur - 2*dt_sample), 1);  
	  t_a_plan(3) = pow(t_cur-( t_cur - 2*dt_sample), 0); 


	  
	  // COM&&foot trajectory interpolation
	  
	  Eigen::Vector3d  x10;
	  Eigen::Vector3d  x11;
	  Eigen::Vector3d  x12;
// 	  Eigen::Vector3d  x13;


/*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
	  x10 = body_in1; 
	  x11 = body_in2;  
	  x12 = _CoM_position_optimal.col(t_int);
// 	  x13 = _CoM_position_optimal.col(t_int+1);
	  
	  
	  Eigen::Matrix<double, 4, 1>  temp;
	  temp.setZero();
	  temp(0) = x10(0); temp(1) = x11(0); temp(2) = x12(0); temp(3) = _comvx(t_int);	  
	  com_inte(0) = t_a_plan * (AAA_inv)*temp;
	  temp(0) = x10(1); temp(1) = x11(1); temp(2) = x12(1); temp(3) = _comvy(t_int);	  
	  com_inte(1) = t_a_plan * (AAA_inv)*temp;
	  temp(0) = x10(2); temp(1) = x11(2); temp(2) = x12(2); temp(3) = _comvz(t_int);	  
	  com_inte(2) = t_a_plan *(AAA_inv)*temp;

  
	  
	  
	}
	else
	{
	  com_inte(0) = body_in3(0);	  
	  com_inte(1) = body_in3(1);	  	  
	  com_inte(2) = body_in3(2);	
	  
	}

 	return com_inte;
	cout<<"com_height"<< com_inte(2)<<endl;
	
}

Vector3d NLPClass::XGetSolution_body_inclination(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{
  //reference com position
        _torso_angle_optimal.row(0) = _thetax;
	_torso_angle_optimal.row(1) = _thetay;
	_torso_angle_optimal.row(2) = _thetaz;
	
	
	Vector3d com_inte;	
	
	if (walktime>=2)
	{
	  int t_int; 
	  t_int = floor(walktime* dt_sample/ _dt  );

	  
	  double t_cur;
	  t_cur = walktime * dt_sample;
	  

	  
	  Eigen::Matrix<double, 4, 1> t_plan;
	  t_plan.setZero();
	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);

// 	  Eigen::MatrixXd AAA;	
// 
// 	  AAA.setZero(4,4);	
// 	  AAA(0,0) = pow(t_plan(0), 3); AAA(0,1) = pow(t_plan(0), 2); AAA(0,2) = pow(t_plan(0), 1); AAA(0,3) = pow(t_plan(0), 0); 
// 	  AAA(1,0) = pow(t_plan(1), 3); AAA(1,1) = pow(t_plan(1), 2); AAA(1,2) = pow(t_plan(1), 1); AAA(1,3) = pow(t_plan(0), 0); 
// 	  AAA(2,0) = pow(t_plan(2), 3); AAA(2,1) = pow(t_plan(2), 2); AAA(2,2) = pow(t_plan(2), 1); AAA(2,3) = pow(t_plan(0), 0); 
// 	  AAA(3,0) = 3*pow(t_plan(2), 2); AAA(3,1) = 2*pow(t_plan(2), 1); AAA(3,2) = pow(t_plan(2), 0); AAA(3,3) = 0;  


	  Eigen::Matrix4d AAA_inv;
	  
	  double abx1, abx2, abx3, abx4;
	  abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
	  abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
	  abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
	  abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
	  

	  AAA_inv(0,0) = 1/ abx1;
	  AAA_inv(0,1) =  -1/ abx2;
	  AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
	  AAA_inv(0,3) = 1/ abx4;
	  
	  AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
	  AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
	  AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
	  AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
	  
	  AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
	  AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
	  AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
	  AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
	  
	  AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
	  AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
	  AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
	  AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;
	  	  
	  
	  
	  
	
	  
	  
	  Eigen::Matrix<double, 1, 4> t_a_plan;
	  t_a_plan.setZero();
	  t_a_plan(0) = pow(t_cur-( t_cur - 2*dt_sample), 3);   t_a_plan(1) = pow(t_cur-( t_cur - 2*dt_sample), 2);   t_a_plan(2) = pow(t_cur-( t_cur - 2*dt_sample), 1);  t_a_plan(3) = pow(t_cur-( t_cur - 2*dt_sample), 0); 


	  
	  // COM&&foot trajectory interpolation
	  
	  Eigen::Vector3d  x10;
	  Eigen::Vector3d  x11;
	  Eigen::Vector3d  x12;
// 	  Eigen::Vector3d  x13;


/*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
	  x10 = body_in1; 
	  x11 = body_in2;  
	  x12 = _torso_angle_optimal.col(t_int);
// 	  x13 = _torso_angle_optimal.col(t_int+1);
	  
	  
	  Eigen::Matrix<double, 4, 1>  temp;
	  temp.setZero();
	  temp(0) = x10(0); temp(1) = x11(0); temp(2) = x12(0); temp(3) = _thetavx(t_int);	  
	  com_inte(0) = t_a_plan * (AAA_inv)*temp;
	  temp(0) = x10(1); temp(1) = x11(1); temp(2) = x12(1); temp(3) = _thetavy(t_int);	  
	  com_inte(1) = t_a_plan * (AAA_inv)*temp;
	  temp(0) = x10(2); temp(1) = x11(2); temp(2) = x12(2); temp(3) = _thetavz(t_int);	  
	  com_inte(2) = t_a_plan *(AAA_inv)*temp;

	  
	  
	}
	else
	{
	  com_inte(0) = body_in3(0);	  
	  com_inte(1) = body_in3(1);	  	  
	  com_inte(2) = body_in3(2);	
	  
	}

 	return com_inte;
	
  
  
  
}


Vector3d NLPClass::XGetSolution_Foot_positionR(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{
	
        _R_foot_optition_optimal.row(0) = _Rfootx;
	_R_foot_optition_optimal.row(1) = _Rfooty;
	_R_foot_optition_optimal.row(2) = _Rfootz;
	
	Vector3d com_inte;	
	
	if (walktime>=2)
	{
	  int t_int; 
	  t_int = floor(walktime* dt_sample/ _dt  );

	  
	  double t_cur;
	  t_cur = walktime * dt_sample;
	  

	  
	  Eigen::Matrix<double, 4, 1> t_plan;
	  t_plan.setZero();
	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);

// 	  cout << "t_plan2:"<<endl<<t_plan<<endl;
// 	  Eigen::MatrixXd AAA;
// 
// 	  
// 	  AAA.setZero(4,4);	
// 	  AAA(0,0) = pow(t_plan(0), 3); AAA(0,1) = pow(t_plan(0), 2); AAA(0,2) = pow(t_plan(0), 1); AAA(0,3) = pow(t_plan(0), 0); 
// 	  AAA(1,0) = pow(t_plan(1), 3); AAA(1,1) = pow(t_plan(1), 2); AAA(1,2) = pow(t_plan(1), 1); AAA(1,3) = pow(t_plan(0), 0); 
// 	  AAA(2,0) = pow(t_plan(2), 3); AAA(2,1) = pow(t_plan(2), 2); AAA(2,2) = pow(t_plan(2), 1); AAA(2,3) = pow(t_plan(0), 0); 
// 	  AAA(3,0) = 3*pow(t_plan(2), 2); AAA(3,1) = 2*pow(t_plan(2), 1); AAA(3,2) = pow(t_plan(2), 0); AAA(3,3) = 0;  


	  
	  Eigen::Matrix4d AAA_inv;
	  
	  double abx1, abx2, abx3, abx4;
	  abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
	  abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
	  abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
	  abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
	  

	  AAA_inv(0,0) = 1/ abx1;
	  AAA_inv(0,1) =  -1/ abx2;
	  AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
	  AAA_inv(0,3) = 1/ abx4;
	  
	  AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
	  AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
	  AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
	  AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
	  
	  AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
	  AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
	  AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
	  AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
	  
	  AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
	  AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
	  AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
	  AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;
	  	  

	
	  
	  
	  Eigen::Matrix<double, 1, 4> t_a_plan;
	  t_a_plan.setZero();
	  t_a_plan(0) = pow(t_cur-( t_cur - 2*dt_sample), 3);  
	  t_a_plan(1) = pow(t_cur-( t_cur - 2*dt_sample), 2);  
	  t_a_plan(2) = pow(t_cur-( t_cur - 2*dt_sample), 1);  
	  t_a_plan(3) = pow(t_cur-( t_cur - 2*dt_sample), 0); 


	  
	  // COM&&foot trajectory interpolation
	  
	  Eigen::Vector3d  x10;
	  Eigen::Vector3d  x11;
	  Eigen::Vector3d  x12;
// 	  Eigen::Vector3d  x13;


/*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
	  x10 = body_in1; 
	  x11 = body_in2;  
	  x12 =  _R_foot_optition_optimal.col(t_int);
//	  x13 =  _R_foot_optition_optimal.col(t_int+1);
	  
	  
	  Eigen::Matrix<double, 4, 1>  temp;
	  temp.setZero();
	  temp(0) = x10(0); temp(1) = x11(0); temp(2) = x12(0); temp(3) = _Rfootvx(t_int);	  
	  com_inte(0) = t_a_plan * (AAA_inv)*temp;
	  temp(0) = x10(1); temp(1) = x11(1); temp(2) = x12(1); temp(3) = _Rfootvy(t_int);

// 	  cout << "Rfooty:"<<temp<<endl;
	  com_inte(1) = t_a_plan * (AAA_inv)*temp;
	  temp(0) = x10(2); temp(1) = x11(2); temp(2) = x12(2); temp(3) = _Rfootvz(t_int);	  
	  com_inte(2) = t_a_plan *(AAA_inv)*temp;
	  
////=====================================================////////////////////////////////////////////////
// 	  //////spline for Rfoot_
	  
	  
	  
	  
	  
	  
	  
	  
	  
	}
	else
	{
	  com_inte(0) = body_in3(0);	  
	  com_inte(1) = body_in3(1);	  	  
	  com_inte(2) = body_in3(2);

// 	  com_inte = Rfoot_IN.col(walktime);	  
	}
	
// 	cout << "Rfooty_generated:"<<com_inte(1)<<endl;

 	return com_inte;
	
	
	
	
}

Vector3d NLPClass::XGetSolution_Foot_positionL(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{
        _L_foot_optition_optimal.row(0) = _Lfootx;
	_L_foot_optition_optimal.row(1) = _Lfooty;
	_L_foot_optition_optimal.row(2) = _Lfootz;
	
	Vector3d com_inte;	
	
	if (walktime>=2)
	{
	  int t_int; 
	  t_int = floor(walktime* dt_sample/ _dt  );

	  
	  double t_cur;
	  t_cur = walktime * dt_sample;
	  

	  
	  Eigen::Matrix<double, 4, 1> t_plan;
	  t_plan.setZero();
	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);

//           cout << "t_plan3:"<<endl<<t_plan<<endl;
/*	  Eigen::MatrixXd  AAAaaa; /// should be Marix4d
	  AAAaaa.setZero(4,4);
	  
	  AAAaaa(0,0) = pow(t_plan(0), 3); AAAaaa(0,1) = pow(t_plan(0), 2); AAAaaa(0,2) = pow(t_plan(0), 1); AAAaaa(0,3) = pow(t_plan(0), 0); 
	  AAAaaa(1,0) = pow(t_plan(1), 3); AAAaaa(1,1) = pow(t_plan(1), 2); AAAaaa(1,2) = pow(t_plan(1), 1); AAAaaa(1,3) = pow(t_plan(1), 0); 
	  AAAaaa(2,0) = pow(t_plan(2), 3); AAAaaa(2,1) = pow(t_plan(2), 2); AAAaaa(2,2) = pow(t_plan(2), 1); AAAaaa(2,3) = pow(t_plan(2), 0); 
         // AAAaaa(3,0) = pow(t_plan(3), 3); AAAaaa(3,1) = pow(t_plan(3), 2); AAAaaa(3,2) = pow(t_plan(3), 1); AAAaaa(3,3) = pow(t_plan(3), 0); /// using the next to positioin would cause over-fitting	  
	  AAAaaa(3,0) = 3*pow(t_plan(2), 2); AAAaaa(3,1) = 2*pow(t_plan(2), 1); AAAaaa(3,2) = pow(t_plan(2), 0); AAAaaa(3,3) = 0.0;  	  
 */	  
//       MatrixXd.inverse( != Matrix4d (under Xd =4. so write the inverse of Matrix4d explicitly): for the time being)
	  
	  
	  Eigen::Matrix4d AAA_inv;
	  
	  double abx1, abx2, abx3, abx4;
	  abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
	  abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
	  abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
	  abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
	  

	  AAA_inv(0,0) = 1/ abx1;
	  AAA_inv(0,1) =  -1/ abx2;
	  AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
	  AAA_inv(0,3) = 1/ abx4;
	  
	  AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
	  AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
	  AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
	  AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
	  
	  AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
	  AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
	  AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
	  AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
	  
	  AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
	  AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
	  AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
	  AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;
	  	  

	  
	
// // 	  Eigen::Matrix4d  AAAaaa1_inv=AAA_inv;
	  
// 	  cout<< t_plan<<endl;
// 	  cout<< AAAaaa<<endl;
// 	  cout<< AAA_inv - AAAaaa1_inv<<endl;
// 	  cout<< AAA_inv - AAAaaa1.inverse()<<endl;
	  
	  Eigen::RowVector4d t_a_plan;
	  t_a_plan.setZero();
	  t_a_plan(0) = pow(t_cur-( t_cur - 2*dt_sample), 3);  
	  t_a_plan(1) = pow(t_cur-( t_cur - 2*dt_sample), 2);   
	  t_a_plan(2) = pow(t_cur-( t_cur - 2*dt_sample), 1); 
	  t_a_plan(3) = pow(t_cur-( t_cur - 2*dt_sample), 0); 


	  
	  // COM&&foot trajectory interpolation
	  
	  Eigen::Vector3d  x10;
	  Eigen::Vector3d  x11;
	  Eigen::Vector3d  x12;
// 	  Eigen::Vector3d  x13;


/*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
	  x10 = body_in1; 
	  x11 = body_in2;  
	  x12 =  _L_foot_optition_optimal.col(t_int);
// 	  x13 =  _L_foot_optition_optimal.col(t_int+1); /// using next two position would caused over-fitting
	  
	  
	  Eigen::Vector4d  temp;
	  temp.setZero();
	  temp(0) = x10(0); temp(1) = x11(0); temp(2) = x12(0);
	  temp(3) = _Lfootvx(t_int);
// 	  	  temp(3) = x13(0);
	  Eigen::Vector4d tmp111 = AAA_inv*temp;
	  com_inte(0) = t_a_plan * tmp111;
	  temp(0) = x10(1); temp(1) = x11(1); temp(2) = x12(1);
	  temp(3) = _Lfootvy(t_int);	 
/*          temp(3) = x13(1);	*/  
	  tmp111 = AAA_inv*temp;  
	  com_inte(1) = t_a_plan * tmp111;
	  temp(0) = x10(2); temp(1) = x11(2); temp(2) = x12(2); 
	  temp(3) = _Lfootvz(t_int);	
// 	  temp(3) = x13(2);
	  tmp111 = AAA_inv*temp;	  
	  com_inte(2) = t_a_plan * tmp111;

////================================================not use spline.h=====////////////////////////////////////////////////


	  
	}
	else
	{
	  com_inte(0) = body_in3(0);	  
	  com_inte(1) = body_in3(1);	  	  
	  com_inte(2) = body_in3(2);
	}

 	return com_inte;
	
	cout << "Lfooty_generated:"<<com_inte(1)<<endl;
}


////solve the inverse matrix of 4*4 coefficient matrices
void NLPClass::solve_AAA_inv(Eigen::Matrix<double, 4, 1> t_plan)
{
  
// 	  Eigen::MatrixXd AAA1;	
// 
// 	  AAA1.setZero(4,4);	
// 	  AAA1(0,0) = pow(t_plan(0), 3); AAA1(0,1) = pow(t_plan(0), 2); AAA1(0,2) = pow(t_plan(0), 1); AAA1(0,3) = pow(t_plan(0), 0); 
// 	  AAA1(1,0) = pow(t_plan(1), 3); AAA1(1,1) = pow(t_plan(1), 2); AAA1(1,2) = pow(t_plan(1), 1); AAA1(1,3) = pow(t_plan(0), 0); 
// 	  AAA1(2,0) = pow(t_plan(2), 3); AAA1(2,1) = pow(t_plan(2), 2); AAA1(2,2) = pow(t_plan(2), 1); AAA1(2,3) = pow(t_plan(0), 0); 
// 	  AAA1(3,0) = 3*pow(t_plan(2), 2); AAA1(3,1) = 2*pow(t_plan(2), 1); AAA1(3,2) = pow(t_plan(2), 0); AAA1(3,3) = 0;  

  
  
  double abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
  double abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
  double abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
  double abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
  

  _AAA_inv(0,0) = 1/ abx1;
  _AAA_inv(0,1) =  -1/ abx2;
  _AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
  _AAA_inv(0,3) = 1/ abx4;
  
  _AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
  _AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
  _AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
  _AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
  
  _AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
  _AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
  _AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
  _AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
  
  _AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
  _AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
  _AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
  _AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;   
}

////solve the inverse matrix of 7*7 coefficient matrices
Eigen::Matrix<double, 7, 7> NLPClass::solve_AAA_inv_x(Eigen::Vector3d t_plan)
{
  Eigen::Matrix<double,7,7> AAA;
  AAA.setZero();
  Eigen::Matrix<double, 1, 7> aaaa;
  aaaa.setZero();

  aaaa(0) = 6*pow(t_plan(0), 5);   aaaa(1) =  5*pow(t_plan(0), 4);  aaaa(2) =  4*pow(t_plan(0), 3);   aaaa(3) =  3*pow(t_plan(0), 2);
  aaaa(4) = 2*pow(t_plan(0), 1);   aaaa(5) = 1;                     aaaa(6) =  0;
  AAA.row(0) = aaaa;

  aaaa(0) = 30*pow(t_plan(0), 4);  aaaa(1) =  20*pow(t_plan(0), 3);  aaaa(2) =  12*pow(t_plan(0), 2);   aaaa(3) =  6*pow(t_plan(0), 1);
  aaaa(4) = 2;                     aaaa(5) = 0;                      aaaa(6) =  0;
  AAA.row(1) = aaaa;

  aaaa(0) = pow(t_plan(0), 6);     aaaa(1) =  pow(t_plan(0), 5);     aaaa(2) =  pow(t_plan(0), 4);   aaaa(3) =  pow(t_plan(0), 3);
  aaaa(4) = pow(t_plan(0), 2);     aaaa(5) = pow(t_plan(0), 1);      aaaa(6) =  1;	  
  AAA.row(2) = aaaa;

  aaaa(0) = pow(t_plan(1), 6);     aaaa(1) =  pow(t_plan(1), 5);     aaaa(2) =  pow(t_plan(1), 4);   aaaa(3) =  pow(t_plan(1), 3);
  aaaa(4) = pow(t_plan(1), 2);     aaaa(5) = pow(t_plan(1), 1);      aaaa(6) =  1;
  AAA.row(3) = aaaa;

  aaaa(0) = pow(t_plan(2), 6);     aaaa(1) =  pow(t_plan(2), 5);     aaaa(2) =  pow(t_plan(2), 4);   aaaa(3) =  pow(t_plan(2), 3);
  aaaa(4) = pow(t_plan(2), 2);     aaaa(5) = pow(t_plan(2), 1);      aaaa(6) =  1;
  AAA.row(4) = aaaa;	  

  aaaa(0) = 6*pow(t_plan(2), 5);   aaaa(1) =  5*pow(t_plan(2), 4);  aaaa(2) =  4*pow(t_plan(2), 3);   aaaa(3) =  3*pow(t_plan(2), 2);
  aaaa(4) = 2*pow(t_plan(2), 1);   aaaa(5) = 1;                     aaaa(6) =  0;
  AAA.row(5) = aaaa;

  aaaa(0) = 30*pow(t_plan(2), 4);  aaaa(1) =  20*pow(t_plan(2), 3);  aaaa(2) =  12*pow(t_plan(2), 2);   aaaa(3) =  6*pow(t_plan(2), 1);
  aaaa(4) = 2;                     aaaa(5) = 0;                      aaaa(6) =  0;
  AAA.row(6) = aaaa;  
  
  Eigen::Matrix<double,7,7> AAA_inv = AAA.inverse(); 
  
  return AAA_inv;
  
}

Eigen::Matrix<double, 4, 4> NLPClass::solve_AAA_inv2(Eigen::Vector3d t_plan)
{
  Eigen::Matrix<double,4,4> AAA1; 
  AAA1 << pow(t_plan(0), 3), pow(t_plan(0), 2), pow(t_plan(0), 1), 1, 
	        pow(t_plan(1), 3), pow(t_plan(1), 2), pow(t_plan(1), 1), 1, 
	        pow(t_plan(2), 3), pow(t_plan(2), 2), pow(t_plan(2), 1), 1,
	        3*pow(t_plan(2), 2), 2*pow(t_plan(2), 1), pow(t_plan(2), 0), 0;

  Eigen::Matrix<double,4,4> AAA1_inv = AAA1.inverse();  

  return AAA1_inv;
}


//////////////////////////////////////////=============================ZMP optimal distribution for lower level adimittance control=================================
////reference_force_torque_distribution========================================

void NLPClass::Zmp_distributor(int walktime, double dt_sample)
{
// //// judge if stop  
//   if(_stopwalking)  
//   {  
//     for (int i_t = _bjx1+1; i_t < _footstepsnumber; i_t++) {	  
//       _lift_height_ref(i_t) = 0;  
//     }	  
// 
//   }  
  
  int j_index = floor(walktime / (_dt / dt_sample));
  
  zmp_interpolation(j_index,walktime,dt_sample);  

// reference_force_torque_distribution 
  if (_bjx1 >= 2)
  {
      if (_bjx1 % 2 == 0)           //odd:left support
      {  
	/// right swing
	if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1))
	{
	  int nTx_n = round(_tx(_bjx1-1)/_dt);
	  int nTx_n_dsp = round((_tx(_bjx1-1)+_td(_bjx1-1))/_dt);
	  Vector2d ZMP_init(0,0);
	  ZMP_init(0) = _zmpx_real(0,nTx_n-2);
	  ZMP_init(1) = _zmpy_real(0,nTx_n-2);
	  
	  Vector2d ZMP_end(0,0); 
	  ZMP_end(0) = _zmpx_real(0,nTx_n_dsp-1);
	  ZMP_end(1) = _zmpy_real(0,nTx_n_dsp-1);
	  
// 	  if (abs(ZMP_end(0)-ZMP_init(0))<=0.001) 
// 	  {
// 	      _Co_L(1,1) = abs((_ZMPxy_realx(1)-ZMP_init(1))/(ZMP_end(1)-ZMP_init(1)));
// 	      if (_Co_L(1,1) >1)
// 	      {
// 		_Co_L(1,1)=1;
// 	      }               
// 	      _Co_L(0,0) = _Co_L(1,1);                
// 	      _Co_L(2,2) = sqrt((pow(_Co_L(0,0),2)+pow(_Co_L(0,0),2))/2);  
// 	  }
// 	  else
// 	  {
	    _Co_L(0,0) = abs(((ZMP_end(1)-ZMP_init(1))*(_ZMPxy_realx(1)-ZMP_init(1))+(ZMP_end(0)-ZMP_init(0))*(_ZMPxy_realx(0)-ZMP_init(0)))/(pow(ZMP_end(1)-ZMP_init(1),2)+pow(ZMP_end(0)-ZMP_init(0),2)));
	    _Co_L(1,1) = _Co_L(0,0);
	      
	    if (_Co_L(0,0) >1)
	    {
	      _Co_L(0,0)=1;
	    }
	    if (_Co_L(1,1) >1)
	    {
	      _Co_L(1,1)=1;
	    }
	    _Co_L(2,2) = sqrt((pow(_Co_L(0,0),2)+pow(_Co_L(0,0),2))/2); 
/*	  }  */ 
	  
	  Matrix3d II;
	  II.setIdentity(3,3);
	  _Co_R = II-_Co_L;     

	  Force_torque_calculate(_comxyzx,_comaxyzx,_thetaaxyx,_Lfootxyzx,_Rfootxyzx);	  
	  
	}
	else
	{
	 //Swing leg with left support
	  _Co_L.setIdentity(3,3);
	  _Co_R.setZero();
	  Force_torque_calculate(_comxyzx,_comaxyzx,_thetaaxyx,_Lfootxyzx,_Rfootxyzx);  	  
	}	
      }      
      else                       //right support
      {	/// left swing
	if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab: double suppot
	{
	  int nTx_n = round(_tx(_bjx1-1)/_dt);
	  int nTx_n_dsp = round((_tx(_bjx1-1)+_td(_bjx1-1))/_dt);
	  Vector2d ZMP_init(0,0);
	  ZMP_init(0) = _zmpx_real(0,nTx_n-2);
	  ZMP_init(1) = _zmpy_real(0,nTx_n-2);
	  
	  Vector2d ZMP_end(0,0); 
	  ZMP_end(0) = _zmpx_real(0,nTx_n_dsp-1);
	  ZMP_end(1) = _zmpy_real(0,nTx_n_dsp-1);
	  
// 	  if (abs(ZMP_end(0)-ZMP_init(0))<=0.001) 
// 	  {
// 	      _Co_R(1,1) = abs((_ZMPxy_realx(1)-ZMP_init(1))/(ZMP_end(1)-ZMP_init(1)));
// 	      if (_Co_R(1,1) >1)
// 	      {
// 		_Co_R(1,1)=1;
// 	      }               
// 	      _Co_R(0,0) = _Co_R(1,1);                
// 	      _Co_R(2,2) = sqrt((pow(_Co_R(0,0),2)+pow(_Co_R(0,0),2))/2);  
// 	  }
// 	  else
// 	  {
	    _Co_R(0,0) = abs(((ZMP_end(1)-ZMP_init(1))*(_ZMPxy_realx(1)-ZMP_init(1))+(ZMP_end(0)-ZMP_init(0))*(_ZMPxy_realx(0)-ZMP_init(0)))/(pow(ZMP_end(1)-ZMP_init(1),2)+pow(ZMP_end(0)-ZMP_init(0),2)));
	    _Co_R(1,1) = _Co_R(0,0);
	      
	    if (_Co_R(0,0) >1)
	    {
	      _Co_R(0,0)=1;
	    }
	    if (_Co_R(1,1) >1)
	    {
	      _Co_R(1,1)=1;
	    }
	    _Co_R(2,2) = sqrt((pow(_Co_R(0,0),2)+pow(_Co_R(0,0),2))/2); 
/*	  }*/   
	  
	  Matrix3d II;
	  II.setIdentity(3,3);
	  _Co_L = II-_Co_R;     

	  Force_torque_calculate(_comxyzx,_comaxyzx,_thetaaxyx,_Lfootxyzx,_Rfootxyzx);	  	 	  
	}
	else
	{
	 //Swing leg with right support
	  _Co_R.setIdentity(3,3);
	  _Co_L.setZero();
	  Force_torque_calculate(_comxyzx,_comaxyzx,_thetaaxyx,_Lfootxyzx,_Rfootxyzx);  	  
	}
      }
  }
  else
  {
    if (_bjx1==0)   ///stand still:
    {
//       _Co_R = 0.5*Eigen::Vector3d::Zero().setIdentity(3,3);
//       _Co_L.setZero();
      _Co_R(0,0) = _Co_R(1,1) = _Co_R(2,2) = _Co_L(0,0) = _Co_L(1,1) = _Co_L(2,2) = 0.5;
      Force_torque_calculate(_comxyzx,_comaxyzx,_thetaaxyx,_Lfootxyzx,_Rfootxyzx);       
    }
    else   ///right support: double support in the whole walking pattern: assuming that the initial and final ZMP is located at the foot locations 
    {/// left swing
	Vector2d ZMP_init;
	ZMP_init(0) = _footxyz_real(0,_bjx1-1);
	ZMP_init(1) = _footxyz_real(1,_bjx1-1);
	
	Vector2d ZMP_end; 
	ZMP_end(0) = _footxyz_real(0,_bjx1);
	ZMP_end(1) = _footxyz_real(1,_bjx1);
	
// 	if (abs(ZMP_end(0)-ZMP_init(0))<=0.001) 
// 	{
// 	    _Co_R(1,1) = abs((_ZMPxy_realx(1)-ZMP_init(1))/(ZMP_end(1)-ZMP_init(1)));
// 	    if (_Co_R(1,1) >1)
// 	    {
// 	      _Co_R(1,1)=1;
// 	    }               
// 	    _Co_R(0,0) = _Co_R(1,1);                
// 	    _Co_R(2,2) = sqrt((pow(_Co_R(0,0),2)+pow(_Co_R(0,0),2))/2);  
// 	}
// 	else
// 	{
	  _Co_R(0,0) = abs(((ZMP_end(1)-ZMP_init(1))*(_ZMPxy_realx(1)-ZMP_init(1))+(ZMP_end(0)-ZMP_init(0))*(_ZMPxy_realx(0)-ZMP_init(0)))/(pow(ZMP_end(1)-ZMP_init(1),2)+pow(ZMP_end(0)-ZMP_init(0),2)));
	  _Co_R(1,1) = _Co_R(0,0);
	    
	  if (_Co_R(0,0) >1)
	  {
	    _Co_R(0,0)=1;
	  }
	  if (_Co_R(1,1) >1)
	  {
	    _Co_R(1,1)=1;
	  }
	  _Co_R(2,2) = sqrt((pow(_Co_R(0,0),2)+pow(_Co_R(0,0),2))/2); 
/*	}*/   
	
	Matrix3d II;
	II.setIdentity(3,3);
	_Co_L = II-_Co_R;     

	Force_torque_calculate(_comxyzx,_comaxyzx,_thetaaxyx,_Lfootxyzx,_Rfootxyzx);	  	 	       
    }
  }   
}


void NLPClass::zmp_interpolation(int t_int,int walktime, double dt_sample)
{
  //// calculate by the nonlinear model:
//   if (t_int>=1)
//   {
//     _ZMPxy_realx(0) = _comxyzx(0) - (_comxyzx(2) - _Zsc(t_int-1))/(_comaxyzx(2)+_ggg(0))*_comaxyzx(0) - _j_ini * _thetaaxyx(1)/(_mass * (_ggg(0) + _comaxyzx(2)));
//     _ZMPxy_realx(1) = _comxyzx(1) - (_comxyzx(2) - _Zsc(t_int-1))/(_comaxyzx(2)+_ggg(0))*_comaxyzx(1) + _j_ini * _thetaaxyx(0)/(_mass * (_ggg(0) + _comaxyzx(2)));     
//   }
//   else
//   {
//     _ZMPxy_realx(0) = _comxyzx(0) - (_comxyzx(2))/(_comaxyzx(2)+_ggg(0))*_comaxyzx(0) - _j_ini * _thetaaxyx(1)/(_mass * (_ggg(0) + _comaxyzx(2)));
//     _ZMPxy_realx(1) = _comxyzx(1) - (_comxyzx(2))/(_comaxyzx(2)+_ggg(0))*_comaxyzx(1) + _j_ini * _thetaaxyx(0)/(_mass * (_ggg(0) + _comaxyzx(2)));     
//   }
  
  //// linear interpolation of ZMP reference:  
  double t_des = walktime * dt_sample-t_int*_dt;
  if (t_des<=0){
    t_des =0.0001;
  }	  
//   cout <<"//////////////////////////////////////////"<<endl;
//   cout <<"t_des:"<<t_des<<endl;
//   cout <<"//////////////////////////////////////////"<<endl;
  if (t_int>=1)
  {
    _ZMPxy_realx(0) = (_zmpx_real(0,t_int)-_zmpx_real(0,t_int-1))/_dt*t_des+_zmpx_real(0,t_int-1);
    _ZMPxy_realx(1) = (_zmpy_real(0,t_int)-_zmpy_real(0,t_int-1))/_dt*t_des+_zmpy_real(0,t_int-1);

  }
  else
  {
    _ZMPxy_realx(0) = _zmpx_real(0,t_int)/_dt*t_des+0;
    _ZMPxy_realx(1) = _zmpy_real(0,t_int)/_dt*t_des+0;	    
  }  
  
  
}


void NLPClass::Force_torque_calculate(Vector3d comxyzx1,Vector3d comaxyzx1,Vector3d thetaaxyx1,Vector3d Lfootxyz1,Vector3d Rfootxyz1)
{
  Vector3d gra;
  gra << 0,0, -_ggg;
  
  Vector3d F_total = _mass * (comaxyzx1 - gra);
  
  Vector3d the3a;
  the3a << thetaaxyx1(0),thetaaxyx1(1),0;
  Vector3d L_total = _j_ini * the3a;
  
  _F_R = _Co_R * F_total;
  _F_L = _Co_L * F_total;
  
  Vector3d R_det_foot_com = Rfootxyz1 -  comxyzx1;
  
  Vector3d L_det_foot_com = Lfootxyz1 -  comxyzx1;
  
  Vector3d M_total = L_total - _F_R.cross(R_det_foot_com) - _F_L.cross(L_det_foot_com);
  
  _M_R = _Co_R*M_total;  
  _M_L = _Co_L*M_total;
  
}




void NLPClass::File_wl_steptiming()
{
 
}


void NLPClass::File_wl_kmp()
{
        Eigen::MatrixXd foot_traj_kmp;
	foot_traj_kmp.setZero(12,_Rfootx_kmp.cols());
	foot_traj_kmp.block< Dynamic, Dynamic>(0, 0,1,_Rfootx_kmp.cols()) = _Rfootx_kmp;
	foot_traj_kmp.block< Dynamic, Dynamic>(1, 0,1,_Rfootx_kmp.cols()) = _Rfooty_kmp;	
	foot_traj_kmp.block< Dynamic, Dynamic>(2, 0,1,_Rfootx_kmp.cols()) = _Rfootz_kmp;	
	foot_traj_kmp.block< Dynamic, Dynamic>(3, 0,1,_Rfootx_kmp.cols()) = _Rfootvx_kmp;	
	foot_traj_kmp.block< Dynamic, Dynamic>(4, 0,1,_Rfootx_kmp.cols()) = _Rfootvy_kmp;
	foot_traj_kmp.block< Dynamic, Dynamic>(5, 0,1,_Rfootx_kmp.cols()) = _Rfootvz_kmp;	
	foot_traj_kmp.block< Dynamic, Dynamic>(6, 0,1,_Rfootx_kmp.cols()) = _Lfootx_kmp;	
	foot_traj_kmp.block< Dynamic, Dynamic>(7, 0,1,_Rfootx_kmp.cols()) = _Lfooty_kmp;
	foot_traj_kmp.block< Dynamic, Dynamic>(8, 0,1,_Rfootx_kmp.cols()) = _Lfootz_kmp;
	foot_traj_kmp.block< Dynamic, Dynamic>(9, 0,1,_Rfootx_kmp.cols()) = _Lfootvx_kmp;	
	foot_traj_kmp.block< Dynamic, Dynamic>(10, 0,1,_Rfootx_kmp.cols()) = _Lfootvy_kmp;	
	foot_traj_kmp.block< Dynamic, Dynamic>(11, 0,1,_Rfootx_kmp.cols()) = _Lfootvz_kmp;	
	
  
	std::string fileName1 = "KMP_optimal_leg_trajectory.txt" ;
	std::ofstream outfile1( fileName1.c_str() ) ; // file name and the operation type.        
	
        for(int i=0; i<foot_traj_kmp.rows(); i++){
           for(int j=0; j<foot_traj_kmp.cols(); j++){
                 outfile1 << (double) foot_traj_kmp(i,j) << " " ; 
           }
           outfile1 << std::endl;       // a   newline
        }
        outfile1.close();	
	
  
}


///DATA SAVING:modified=========================================================
void NLPClass::File_wl()
{
        
// // 	CoMMM_ZMP_foot.setZero();
// 	CoMMM_ZMP_foot.block<1,_nsum>(0, 0) = _comx;
// 	CoMMM_ZMP_foot.block<1,_nsum>(1, 0) = _comy;	
// 	CoMMM_ZMP_foot.block<1,_nsum>(2, 0) = _comz;	
// 	CoMMM_ZMP_foot.block<1,_nsum>(3, 0) = _zmpx_real;	
// 	CoMMM_ZMP_foot.block<1,_nsum>(4, 0) = _zmpy_real;
// 	CoMMM_ZMP_foot.block<1,_nsum>(5, 0) = _thetax;	
// 	CoMMM_ZMP_foot.block<1,_nsum>(6, 0) = _thetay;	
// 	CoMMM_ZMP_foot.block<1,_nsum>(7, 0) = _torquex_real;
// 	CoMMM_ZMP_foot.block<1,_nsum>(8, 0) = _torquey_real;
// 	CoMMM_ZMP_foot.block<1,_nsum>(9, 0) = _footx_real_next1.transpose();	
// 	CoMMM_ZMP_foot.block<1,_nsum>(10, 0) = _footy_real_next1.transpose();	
// 	CoMMM_ZMP_foot.block<1,_nsum>(11, 0) = _footz_real_next.transpose();
// 	
// 	CoMMM_ZMP_foot.block<1,_nsum>(12, 0) = _Lfootx;	
// 	CoMMM_ZMP_foot.block<1,_nsum>(13, 0) = _Lfooty;	
// 	CoMMM_ZMP_foot.block<1,_nsum>(14, 0) = _Lfootz;
// 	CoMMM_ZMP_foot.block<1,_nsum>(15, 0) = _Rfootx;	
// 	CoMMM_ZMP_foot.block<1,_nsum>(16, 0) = _Rfooty;	
// 	CoMMM_ZMP_foot.block<1,_nsum>(17, 0) = _Rfootz;
// 
// 	CoMMM_ZMP_foot.block<1,_nsum>(18, 0) = _comvx;
// 	CoMMM_ZMP_foot.block<1,_nsum>(19, 0) = _comax;
// 	
// 	CoMMM_ZMP_foot.block<1,_nsum>(20, 0) = _comvy;	
// 	CoMMM_ZMP_foot.block<1,_nsum>(21, 0) = _comay;
// 	
// 	CoMMM_ZMP_foot.block<1,_nsum>(22, 0) = _comvz;	
// 	CoMMM_ZMP_foot.block<1,_nsum>(23, 0) = _comaz;	
// 
// 	CoMMM_ZMP_foot.block<1,_nsum>(24, 0) = _thetavx;
// 	CoMMM_ZMP_foot.block<1,_nsum>(25, 0) = _thetaax;
// 	
// 	CoMMM_ZMP_foot.block<1,_nsum>(26, 0) = _thetavy;	
// 	CoMMM_ZMP_foot.block<1,_nsum>(27, 0) = _thetaay;
// 	
// 	CoMMM_ZMP_foot.block<1,_nsum>(28, 0) = _comx_ref_lip.transpose();
// 	CoMMM_ZMP_foot.block<1,_nsum>(29, 0) = _comy_ref_lip.transpose();
// 	CoMMM_ZMP_foot.block<1,_nsum>(30, 0) = _comvx_ref_lip.transpose();
// 	CoMMM_ZMP_foot.block<1,_nsum>(31, 0) = _comvy_ref_lip.transpose();

	
	
	
	
  
	std::string fileName = "NMPC2018_3robut3_runtime.txt" ;
	std::ofstream outfile( fileName.c_str() ) ; // file name and the operation type. 
       
        for(int i=0; i<_tcpu.rows(); i++){
           for(int j=0; j<_tcpu.cols(); j++){
                 outfile << (double) _tcpu(i,j) << " " ; 
           }
           outfile << std::endl;       // a   newline
        }
        outfile.close();
	
        for(int i=0; i<_tcpu.rows(); i++){
           for(int j=0; j<_tcpu.cols(); j++){
                 outfile << (double) _tcpu(i,j) << " " ; 
           }
           outfile << std::endl;       // a   newline
        }
        outfile.close();	


/*	std::string fileName1 = "NMPC_opt.txt" ;
	std::ofstream outfile1( fileName1.c_str() ) ; // file name and the operation type.        
	
        for(int i=0; i<CoMMM_ZMP_foot.rows(); i++){
           for(int j=0; j<CoMMM_ZMP_foot.cols(); j++){
                 outfile1 << (double) CoMMM_ZMP_foot(i,j) << " " ; 
           }
           outfile1 << std::endl;       // a   newline
        }
        outfile1.close();*/	
	
	
	
}


////// sin and cos curve;
Eigen::Matrix<double, 18, 1> NLPClass::XGetSolution_Foot_rotation(const int walktime, const double dt_sample,int j_index)
{
  ///////walktime=====>ij;   int j_index====>i;  dt_sample========>dtx;   
  Eigen::Matrix<double, 18, 1> footlr_r_inte;	
  footlr_r_inte.setZero();

  int dt_dtx = (int) round(_dt / dt_sample)+1;
  double t_desxx = (walktime+dt_dtx)*dt_sample - (_tx(_bjx1-1,0) +  2*_td(_bjx1-1,0)/4); /// phase transition: SSP starts at the beginning of each cycle:  //// add _td(_bjx1 -1) meaning the 
  //cout<< "t_desxx"<<t_desxx + 2*_td(_bjx1-1,0)/4<<endl;

  if ((_bjx1 >= 2)&&(j_index <= _t_end_footstep))
  {       
    if (_bjx1 % 2 == 0)           //odd:left support
    {  
      //// right foot roll:
      ///////  step forward and backward 1//////  
      _Rfoot_r(0,0) =  -0.065*(1 - cos(2*M_PI / (_ts(_bjx1-1,0) ) * (t_desxx + 2*_td(_bjx1-1,0)/4) ));
      _Rfoot_rv(0,0) =  -0.065*(2*M_PI / (_ts(_bjx1-1,0) ) * sin(2*M_PI / (_ts(_bjx1-1,0) ) * (t_desxx + 2*_td(_bjx1-1,0)/4) ));
      _Rfoot_ra(0,0) =  -0.065*(2*M_PI / (_ts(_bjx1-1,0) ) * 2*M_PI / (_ts(_bjx1-1,0) ) * cos(2*M_PI / (_ts(_bjx1-1,0) ) * (t_desxx + 2*_td(_bjx1-1,0)/4) ));

      // _Rfoot_r(0,0) =  -0.065*(1-fabs((_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max)))*(1 - cos(2*M_PI / (_ts(_bjx1-1) ) * (t_desxx + 2*_td(_bjx1-1)/4) ));
      

      ///// right foot pitch: ///later half cycle: minus angle;
      if ((t_desxx + 2*_td(_bjx1-1,0)/4) >= (_ts(_bjx1-1,0)/2))
      {
        // if ((_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))>0)
        // {
          _Rfoot_r(1,0) =  0.075*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (cos(4*M_PI / (_ts(_bjx1-1)) * (t_desxx + 2*_td(_bjx1-1)/4) ) - 1);  
          _Rfoot_rv(1,0) =  -abs(0.075*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (4*M_PI / (_ts(_bjx1-1,0)) * (-sin(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4)))));
          _Rfoot_ra(1,0) =  0.075*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (4*M_PI / (_ts(_bjx1-1,0)) * 4*M_PI / (_ts(_bjx1-1,0)) * (-cos(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4))));        
        // }
      }
      else
      {
      /*	if ((_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))>0)
        {
          _Rfoot_r(1,0) =  -0.025*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (cos(4*M_PI / (_ts(_bjx1-1)) * (t_desxx + 2*_td(_bjx1-1)/4) ) - 1);          
          _Rfoot_rv(1,0) = -0.025*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (4*M_PI / (_ts(_bjx1-1,0)) * (-sin(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4))));
                _Rfoot_ra(1,0) = -0.025*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (4*M_PI / (_ts(_bjx1-1,0)) * 4*M_PI / (_ts(_bjx1-1,0)) * (-cos(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4))));               	  
        }*/

        _Rfoot_r(1,0) =  0;          
        _Rfoot_rv(1,0) = 0;
        _Rfoot_ra(1,0) = 0;               	  


      }   
    }
    else                       //right support
    {      
      //// left foot roll:
      /////// step forward and backward 1//////
      _Lfoot_r(0,0) =  0.075*(1 - cos(2*M_PI / (_ts(_bjx1-1,0) ) * (t_desxx + 2*_td(_bjx1-1,0)/4) ));
      _Lfoot_rv(0,0) =  -0.075*(2*M_PI / (_ts(_bjx1-1,0) ) * sin(2*M_PI / (_ts(_bjx1-1,0) ) * (t_desxx + 2*_td(_bjx1-1,0)/4) ));
      _Lfoot_ra(0,0) =  -0.075*(2*M_PI / (_ts(_bjx1-1,0) ) * 2*M_PI / (_ts(_bjx1-1,0) ) * cos(2*M_PI / (_ts(_bjx1-1,0) ) * (t_desxx + 2*_td(_bjx1-1,0)/4) ));      
      // _Lfoot_r(0,0) =  0.075*(1-fabs((_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max)))*(1 - cos(2*M_PI / (_ts(_bjx1-1) ) * (t_desxx + 2*_td(_bjx1-1)/4) ));

      ////// left foot pitch:
      if ((t_desxx + 2*_td(_bjx1-1,0)/4) >= (_ts(_bjx1-1,0)/2)) ///later half cycle: minus angle;
      {
        // if ((_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))>0)
        // {
          _Lfoot_r(1,0) =  0.075*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (cos(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4)) - 1); 
          _Lfoot_rv(1,0) =  -abs(0.075*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (4*M_PI / (_ts(_bjx1-1,0)) * (-sin(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4)))));
          _Lfoot_ra(1,0) =  0.075*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (4*M_PI / (_ts(_bjx1-1,0)) * 4*M_PI / (_ts(_bjx1-1,0)) * (-cos(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4))));                    
        // } 
      }
      else
      {
      /*        if ((_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))>0)
              {
                _Lfoot_r(1,0) =  -0.025*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (cos(4*M_PI / (_ts(_bjx1-1)) * (t_desxx + 2*_td(_bjx1-1)/4) ) - 1);          
                _Lfoot_rv(1,0) =  -0.025*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (4*M_PI / (_ts(_bjx1-1,0)) * (-sin(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4))));
                _Lfoot_ra(1,0) =  -0.025*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (4*M_PI / (_ts(_bjx1-1,0)) * 4*M_PI / (_ts(_bjx1-1,0)) * (-cos(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4))));                      
        } */  

        _Lfoot_r(1,0) =  0;          
        _Lfoot_rv(1,0) = 0;
        _Lfoot_ra(1,0) = 0;   
      }   
    } 
  }
  
  
  /////Rfoot,xyz, Lfoot,XYZ
  footlr_r_inte(0,0) = _Rfoot_r(0,0);
  footlr_r_inte(1,0) = _Rfoot_r(1,0);
  footlr_r_inte(2,0) = _Rfoot_r(2,0);

  footlr_r_inte(3,0) = _Lfoot_r(0,0);
  footlr_r_inte(4,0) = _Lfoot_r(1,0);
  footlr_r_inte(5,0) = _Lfoot_r(2,0);

  footlr_r_inte(6,0) = _Rfoot_rv(0,0);
  footlr_r_inte(7,0) = _Rfoot_rv(1,0);
  footlr_r_inte(8,0) = _Rfoot_rv(2,0);

  footlr_r_inte(9,0) = _Lfoot_rv(0,0);
  footlr_r_inte(10,0) = _Lfoot_rv(1,0);
  footlr_r_inte(11,0) = _Lfoot_rv(2,0);

  footlr_r_inte(12,0) = _Rfoot_ra(0,0);
  footlr_r_inte(13,0) = _Rfoot_ra(1,0);
  footlr_r_inte(14,0) = _Rfoot_ra(2,0);

  footlr_r_inte(15,0) = _Lfoot_ra(0,0);
  footlr_r_inte(16,0) = _Lfoot_ra(1,0);
  footlr_r_inte(17,0) = _Lfoot_ra(2,0);  

  return footlr_r_inte;

}





