/*****************************************************************************
MPCClass.cpp

Description:    source file of MPCClass

@Version:   1.0
@Author:    Jiatao ding (jtdingx@gmail.com)
@Release:   Thu 02 Aug 2018 12:33:23 PM CEST
@Update:    Thu 02 Aug 2018 12:33:19 PM CEST
*****************************************************************************/
#include "NLP/MPCClass.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>
#include <vector>
#include "yaml.h"

using namespace Eigen;
using namespace std;


MPCClass::MPCClass()                    ///declaration function
	: QPBaseClass()
	, _robot_name("")
	, _robot_mass(0.0)
	, _lift_height(0.0)
	, _method_flag(0)
  ,_gait_mode(101)	
{
  
}

void MPCClass::config_set()
{   
    /////load default parameter from the yaml.file
    ///////////////////  yaml code . ///////// 
    // YAML::Node config = YAML::LoadFile("/home/pi/go1_catkin/src/unitree_ros_to_real/unitree_legged_real/config/config.yaml");
    YAML::Node config = YAML::LoadFile("/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/go1_rt_control/config/config.yaml");

    _dt =  config["dt_slow_mpc"].as<double>();
    _tstep =  config["t_period"].as<double>();

	_aax = config["_aax"].as<double>();         
  _aay = config["_aay"].as<double>();
	_aaxv = config["_aaxv"].as<double>();           
  _aayv= config["_aayv"].as<double>();
	_bbx = config["_bbx"].as<double>();       
  _bby = config["_bby"].as<double>();
	_rr1 = config["_rr1"].as<double>();     
  _rr2 = config["_rr2"].as<double>(); 
	_aax1 = config["_aax1"].as<double>();            
  _aay1 = config["_aay1"].as<double>();
	_bbx1 = config["_bbx1"].as<double>();           
  _bby1 = config["_bby1"].as<double>();
	_rr11 = config["_rr11"].as<double>();          
  _rr21 = config["_rr21"].as<double>(); 


  _lamda_comx = config["_lamda_comx"].as<double>(); 
  _lamda_comvx = config["_lamda_comvx"].as<double>(); 
  _lamda_comy = config["_lamda_comy"].as<double>(); 
  _lamda_comvy = config["_lamda_comvy"].as<double>();    

}




void MPCClass::FootStepInputs( double stepwidth, double steplength, double stepheight)
{	
  _steplength.setZero(_footstepsnumber,1);
  _stepwidth.setZero(_footstepsnumber,1);
  _stepheight.setZero(_footstepsnumber,1);
  _lift_height_ref.setZero(_footstepsnumber,1);

	_steplength.setConstant(steplength);
	_steplength(0) = 0;

	for(int i=15; i<_footstepsnumber;i++)
	{
       _steplength(i) *= (-1);
	}

  _steplength(_footstepsnumber-4) = 0;  
  _steplength(_footstepsnumber-3) = 0;  
  _steplength(_footstepsnumber-2) = 0;
  _steplength(_footstepsnumber-1) = 0;

	
	
	_stepwidth.setConstant(stepwidth);
	_stepwidth(0) = _stepwidth(0)/2;	

  _stepwidth(_footstepsnumber-4) = 0;  
  _stepwidth(_footstepsnumber-3) = 0;  
  _stepwidth(_footstepsnumber-2) = 0;
  _stepwidth(_footstepsnumber-1) = 0;

	for(int i=15; i<_footstepsnumber;i++)
	{
       _stepwidth(i) *= (-1);
	}

	
	_stepheight.setConstant(stepheight);
	
	_lift_height_ref.setConstant(_lift_height);
	
}


void MPCClass::Initialize()
{
  
  config_set();  
  // ==step parameters initialize==: given by the inputs
  ////NP initialization for step timing optimization
  //// reference steplength and stepwidth for timing optimization  
 	// ==step loctions setup==
  _Lxx_ref = _steplength;
  _Lyy_ref = _stepwidth;	   


  if(_gait_mode ==102) ///troting gait
  {

  }
  else
  {
    // local coordinate
  	_Lyy_ref(0) = 0;    
    for (int j =0; j<_footstepsnumber;j++)
    {
      _Lyy_ref(j) = (int)pow(-1,j)*_stepwidth(j);
    }
  }

  
// 	reference footstep locations setup
  _footx_ref.setZero(_footstepsnumber,1);
	_footy_ref.setZero(_footstepsnumber,1);
	_footz_ref.setZero(_footstepsnumber,1);		
  for (int i = 1; i < _footstepsnumber; i++) {
 	  _footx_ref(i) = _footx_ref(i-1) + _steplength(i-1);
    if(_gait_mode ==102) ///troting gait
    {
      _footy_ref(i) = _footy_ref(i-1) + _stepwidth(i-1);
    }
    else
    {     
      _footy_ref(i) = _footy_ref(i-1) + (int)pow(-1,i-1)*_stepwidth(i-1);  
    } 
	  _footz_ref(i) = _footz_ref(i-1) + _stepheight(i-1);
	}
	_footx_offline = _footx_ref;
	_footy_offline = _footy_ref;
	_footz_offline = _footz_ref;

	
	
	cout<<"_footy_ref:"<<_footy_ref.transpose()<<endl;
	cout<<"_Lyy_ref:"<<_Lyy_ref.transpose()<<endl;
	
	// == step cycle setup
  _ts.setZero(_footstepsnumber,1);
	_ts.setConstant(_tstep);
	// _ts(5) = 0.4;
	// _ts(6) = 0.4;	

	_td = 0.2*_ts;
	_tx.setZero(_footstepsnumber,1);
  	for (int i = 1; i < _footstepsnumber; i++) {
 	  _tx(i) = _tx(i-1) + _ts(i-1);
	  _tx(i) = round(_tx(i)/_dt)*_dt -0.000001;	  
	}	

	//whole sampling time sequnece    
	// _t.setLinSpaced(round(_tx(_footstepsnumber-1)/_dt),_dt,_tx(_footstepsnumber-1));
  
  cout<<"yyy"<<endl;    
	nsum_x = round(_tx(_footstepsnumber-1)/_dt);
	if (nsum_x >=_nsum)
	{
		nsum_x = _nsum;
	}


	//parameters
  //_hcom = gait::Z_c;	
	_g = gait::_g;	
	_ggg.setConstant(9.8);
	_Wn = sqrt(_g/_hcom);
	
	_Wndt = _Wn*_dt;	
	
  // COM state
	_comx.setZero(); _comvx.setZero(); _comax.setZero();
	_comy.setZero(); _comvy.setZero(); _comay.setZero();
	_comz.setConstant(_hcom); _comvz.setZero(); _comaz.setZero();
  ///actual steplengh,stepwidth and walking period
	_Lxx_ref_real.setZero(); 
	_Lyy_ref_real.setZero(); 
	_Ts_ref_real.setZero();
	 

	
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// %% parameters for first MPC-step timing adjustment and next one step location
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%	
	_px.setZero(); _py.setZero(); _pz.setZero();
	_zmpvx.setZero(); _zmpvy.setZero(); 
	_COMx_is.setZero(1,_footstepsnumber); _COMx_es.setZero(1,_footstepsnumber); _COMvx_is.setZero(1,_footstepsnumber); 
	_COMy_is.setZero(1,_footstepsnumber); _COMy_es.setZero(1,_footstepsnumber); _COMvy_is.setZero(1,_footstepsnumber);
	_comx_feed.setZero(); _comvx_feed.setZero(); _comax_feed.setZero();
	_comy_feed.setZero(); _comvy_feed.setZero(); _comay_feed.setZero();
	
	_Vari_ini.setZero(); //Lxx,Lyy,Tr1,Tr2,Lxx1,Lyy1,Tr11,Tr21;
	_vari_ini.setZero();

	//step timing constraints:
	_t_min = 0.2; _t_max = 1;
	
	// swing foot velocity constraints	
	_footx_vmax = 1.5;
	_footx_vmin = -1.875;
	_footy_vmax = 3;
	_footy_vmin = -3;		
	
	// CoM acceleration velocity:
	_comax_max = 4;
	_comax_min = -4;
	_comay_max = 4.5;
	_comay_min = -4.5;	

	
	if(_robot_name == "go1"){
	  _rad = 0.1; 	  
        //  foot location constraints 
	_footx_max=0.1;
	_footx_min=-0.05;
// 	_footy_max=2*gait::RobotParaClass_HALF_HIP_WIDTH + 0.2; 
// 	_footy_min=gait::RobotParaClass_HALF_HIP_WIDTH - 0.03;
	}
	else if (_robot_name == "bigman")
	{
	   _rad = 0.2; 	
			//  foot location constraints 
		_footx_max=0.5;
		_footx_min=-0.1;
	// 	_footy_max=2*gait::RobotParaClass_HALF_HIP_WIDTH + 0.2; 
	// 	_footy_min=gait::RobotParaClass_HALF_HIP_WIDTH - 0.03;

	}
	else if (_robot_name == "cogimon")
        {	  
			_rad = 0.2; 	  	  	  
				//  foot location constraints 
			_footx_max=0.4;
			_footx_min=-0.1;
		// 	_footy_max=2*gait::RobotParaClass_HALF_HIP_WIDTH + 0.2; 
		// 	_footy_min=gait::RobotParaClass_HALF_HIP_WIDTH - 0.03; 
	  
        } 
	

  if(_gait_mode ==102) ///troting gait
  {
    _footy_max= 0.1; 
    _footy_min= -0.05;
  }
  else
  {    
    _footy_max=2*gait::RobotParaClass_HALF_HIP_WIDTH + 0.2; 
    _footy_min=gait::RobotParaClass_HALF_HIP_WIDTH - 0.03; 	
  }

	cout<<"_footy_max"<<_footy_max<<endl;
	cout<<"_footy_min"<<_footy_min<<endl;
	
	_mass = _robot_mass; 
	_j_ini = _mass* pow(_rad,2);	
	
	///external force
	_FX =160;  _FY =120;
	_t_last = 0.5;
	_det_xa = _FX/_mass;  _det_ya = _FY/_mass; 
	_det_xv = _det_xa*_t_last; _det_yv = _det_ya*_t_last;
	_det_xp = pow(_det_xv,2)/(2*_det_xa); _det_yp = pow(_det_yv,2)/(2*_det_ya);
	
	
	
	
	_tcpu.setZero();
	
	_n_loop_omit = 2*round(_tstep/_dt);
	
	xyz0 = -1; //flag for find function 
	xyz1 = 0;  
	xyz2 = 1;
		
	_period_i = 0;  /// period number falls into which cycle 
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
	_SS5.setZero(); _SS5(4) =1;
	_SS6.setZero(); _SS6(5) =1;
	_SS7.setZero(); _SS7(6) =1;
	_SS8.setZero(); _SS8(7) =1;
	
	
	
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
	
	
	///////////////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	///%%%%%%%%%%%%% foot trajectory geneartion
	//////////////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  _t_f.setZero(_nh,1);	
	_bjx1 = 0;
	_bjxx = 0;
	_footxyz_real.setZero(3,_footstepsnumber);
	
	
	_Lfootx.setZero(); _Lfooty.setConstant(_stepwidth(0));_Lfootz.setZero(); _Lfootvx.setZero(); _Lfootvy.setZero();_Lfootvz.setZero(); 
	_Lfootax.setZero(); _Lfootay.setZero();_Lfootaz.setZero();
	_Rfootx.setZero(); _Rfooty.setConstant(-_stepwidth(0));_Rfootz.setZero(); _Rfootvx.setZero(); _Rfootvy.setZero();_Rfootvz.setZero(); 
	_Rfootax.setZero(); _Rfootay.setZero();_Rfootaz.setZero();	
	_ry_left_right = 0;	

	
  //footz refer: height of step
	_Zsc.setZero();		
   

   right_support = 0; 
   _j_count = 0;

  _t_end_footstep = round((_tx(_footstepsnumber-1)- 2*_tstep)/_dt); 
  
  
  _Lfoot_r.setZero();
  _Rfoot_r.setZero();   
  _Lfoot_rv.setZero();
  _Rfoot_rv.setZero();     
  _Lfoot_ra.setZero();
  _Rfoot_ra.setZero(); 

	solve_true = false;

  
}

void MPCClass::Re_Initialize(double step_length_ref, double step_width_ref)
{
  cout<<"reinitialization:"<<endl;


	// _stepwidth.setConstant(stepwidth);
	// _stepwidth(0) = _stepwidth(0)/2;	
  // _stepwidth(_footstepsnumber-4) = 0;  
  // _stepwidth(_footstepsnumber-3) = 0;  
  // _stepwidth(_footstepsnumber-2) = 0;
  // _stepwidth(_footstepsnumber-1) = 0;
	// for(int i=15; i<_footstepsnumber;i++)
	// {
  //      _stepwidth(i) *= (-1);
	// }

	_steplength.setConstant(step_length_ref);
	_steplength(0) = 0;
	for(int i=15; i<_footstepsnumber;i++)
	{
       _steplength(i) *= (-1);
	}
  _steplength(15) = 0;
  _steplength(_footstepsnumber-4) = 0;    
  _steplength(_footstepsnumber-3) = 0;  
  _steplength(_footstepsnumber-2) = 0;
  _steplength(_footstepsnumber-1) = 0;

	_stepwidth.setConstant(step_width_ref);
  if(_gait_mode !=102)
  {
	 _stepwidth(0) = _stepwidth(0)/2;
  }
  else
  {
    _stepwidth(0) = 0;
  }
	for(int i=15; i<_footstepsnumber;i++)
	{
    _stepwidth(i) *= (-1);
	}
  _stepwidth(15) = 0;
  _stepwidth(_footstepsnumber-4) = 0;    
  _stepwidth(_footstepsnumber-3) = 0; 
  _stepwidth(_footstepsnumber-2) = 0;
  _stepwidth(_footstepsnumber-1) = 0;



  ////NP initialization for step timing optimization
  //// reference steplength and stepwidth for timing optimization  
 	// ==step loctions setup==
  _Lxx_ref = _steplength;
  _Lyy_ref = _stepwidth;	   


  if(_gait_mode ==102) ///troting gait
  {

  }
  else
  {
    // local coordinate
  	_Lyy_ref(0) = 0;    
    for (int j =0; j<_footstepsnumber;j++)
    {
      _Lyy_ref(j) = (int)pow(-1,j)*_stepwidth(j);
    }
  }

  
// 	reference footstep locations setup
  _footx_ref.setZero();
	_footy_ref.setZero();
	_footz_ref.setZero();		
  for (int i = 1; i < _footstepsnumber; i++) {
 	  _footx_ref(i) = _footx_ref(i-1) + _steplength(i-1);
    if(_gait_mode ==102) ///troting gait
    {
      _footy_ref(i) = _footy_ref(i-1) + _stepwidth(i-1);
    }
    else
    {     
      _footy_ref(i) = _footy_ref(i-1) + (int)pow(-1,i-1)*_stepwidth(i-1);  
    } 
	  _footz_ref(i) = _footz_ref(i-1) + _stepheight(i-1);
	}
	_footx_offline = _footx_ref;
	_footy_offline = _footy_ref;
	_footz_offline = _footz_ref;

	
	
	cout<<"_footy_ref_reinitialization:"<<_footy_ref.transpose()<<endl;
	cout<<"_Lxx_ref_reinitialization:"<<_Lxx_ref.transpose()<<endl;  
	cout<<"_Lyy_ref_reinitialization:"<<_Lyy_ref.transpose()<<endl;
	
	// == step cycle setup
	_ts.setConstant(_tstep);
	// _ts(5) = 0.4;
	// _ts(6) = 0.4;	

	_td = 0.2*_ts;
	_tx.setZero();
  	for (int i = 1; i < _footstepsnumber; i++) {
 	  _tx(i) = _tx(i-1) + _ts(i-1);
	  _tx(i) = round(_tx(i)/_dt)*_dt -0.000001;	  
	}	

}




Eigen::Matrix<double, 38, 1> MPCClass::step_timing_opti_loop(int i,Eigen::Matrix<double,18,1> estimated_state, 
                                                             Eigen::Vector3d _Rfoot_location_feedback, Eigen::Vector3d _Lfoot_location_feedback,
                                                             double lamda, bool _stopwalking,int _t_walkdtime,int _t_walkdtime_old,
                                                             double step_length, double step_width)
{
  Eigen::Matrix<double, 38, 1> com_traj;
  com_traj.setZero();
  //// step cycle number when (i+1)*dt fall into: attention that _tstep+1 falls ionto the next cycle      
  Indexfind((i+1)*_dt,xyz0);	   /// next one sampling time
  _period_i = _j_period+1;      ///coincident with Matlab
  _j_period = 0;  
  //ZMP & ZMPv
  _px(0,i) = _footx_ref(_period_i-1,0); _zmpvx(0,i) = 0; 
  _py(0,i) = _footy_ref(_period_i-1,0); _zmpvy(0,i) = 0; 
  
//   cout<<"_py:"<<_py(0,i)<<endl;  
  
  ///remaining step timing for the next stepping
  _ki = round(_tx(_period_i-1,0)/_dt);
  _k_yu = i-_ki;                
  _Tk = _ts(_period_i-1) - _k_yu*_dt; 
  
  //// reference remaining time and step length and step width
//    position tracking 
//   _Lxx_refx = _footx_offline(_period_i)-_footx_ref(_period_i-1);        
//   _Lyy_refy = _footy_offline(_period_i)-_footy_ref(_period_i-1); //%% tracking the step location
//   _Lxx_refx1 = _footx_offline(_period_i+1)-_footx_offline(_period_i);        
//   _Lyy_refy1 = _footy_offline(_period_i+1)-_footy_offline(_period_i); // tracking the step location

  //    velocity tracking 
   _Lxx_refx = _Lxx_ref(_period_i-1);        
   _Lyy_refy = _Lyy_ref(_period_i-1); //%% tracking relative location
   _Lxx_refx1 = _Lxx_ref(_period_i);        
   _Lyy_refy1 = _Lyy_ref(_period_i); // tracking relative location
  
  _tr1_ref = cosh(_Wn*_Tk);        _tr2_ref = sinh(_Wn*_Tk);
  _tr1_ref1 = cosh(_Wn*_ts(_period_i));  _tr2_ref1 = sinh(_Wn*_ts(_period_i));  
  
  // warm start
  if (i==1)
  {
    _vari_ini << _Lxx_refx,
                 _Lyy_refy,
                _tr1_ref,
                _tr2_ref,
                _Lxx_refx1,
                 _Lyy_refy1,
                _tr1_ref1,
                _tr2_ref1;
  }
  else
  {
    _vari_ini = _Vari_ini;
  }
  
  
// step timing upper&lower boundaries  modification
  if ((_t_min -_k_yu*_dt)>=0.0001)
  {
    _tr1_min = cosh(_Wn*(_t_min-_k_yu*_dt));
    _tr2_min = sinh(_Wn*(_t_min-_k_yu*_dt));    
  }
  else
  {
    _tr1_min = cosh(_Wn*(0.0001));
    _tr2_min = sinh(_Wn*(0.0001));      
  }
 
  _tr1_max = cosh(_Wn*(_t_max-_k_yu*_dt));
  _tr2_max = sinh(_Wn*(_t_max-_k_yu*_dt));
  
  _tr11_min = cosh(_Wn*_t_min);
  _tr21_min = sinh(_Wn*_t_min);     
  _tr11_max = cosh(_Wn*_t_max);
  _tr21_max = sinh(_Wn*_t_max);    

  if (i==1)
  {
        _COMx_is(_period_i-1) = _comx_feed(i-1)-_footx_ref(_period_i-1);  
        _COMx_es.col(_period_i-1) = _SS1*_vari_ini*0.5;  
        _COMvx_is(_period_i-1)= (_COMx_es(_period_i-1)-_COMx_is(_period_i-1)*_SS3*_vari_ini)/(1/_Wn *_SS4*_vari_ini);
        _COMy_is(_period_i-1) = _comy_feed(i-1)-_footy_ref(_period_i-1);  
        _COMy_es.col(_period_i-1) = _SS2*_vari_ini*0.5;  
        _COMvy_is(_period_i-1)= (_COMy_es(_period_i-1)-_COMy_is(_period_i-1)*_SS3*_vari_ini)/(1/_Wn *_SS4*_vari_ini);  
        _comvx_endref= _Wn*_COMx_is(_period_i-1)*_SS4*_vari_ini + _COMvx_is(_period_i-1)*_SS3*_vari_ini;
        _comvy_endref= _Wn*_COMy_is(_period_i-1)*_SS4*_vari_ini + _COMvy_is(_period_i-1)*_SS3*_vari_ini;     
  }

 
// SEQUENCE QUADARTIC PROGRAMMING-step timing &step location optimization
  for (int xxxx=1; xxxx<=2; xxxx++)
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
    ////number of inequality constraints: 8+8+8+4+4+4
    solve_stepping_timing();
    
    if(solve_true)
    {
      _vari_ini += _X;
    } 
    else
    {
      break;
    }  
    
    
  }
  
  
// // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// // %% results postprocession
// // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
  if(!solve_true)
  {
    _vari_ini << _Lxx_refx,
                  _Lyy_refy,
                  _tr1_ref,
                  _tr2_ref,
                  _Lxx_refx1,
                  _Lyy_refy1,
                  _tr1_ref1,
                  _tr2_ref1;
  }

  _Vari_ini = _vari_ini;  

  
// update the optimal parameter in the real-time: at the result, the effect of optimization reduced gradually
  _Lxx_ref(_period_i-1) = _SS1*_vari_ini;
  _Lyy_ref(_period_i-1) = _SS2*_vari_ini;
  _ts(_period_i-1) = _k_yu*_dt+ log((_SS3+_SS4)*_vari_ini)/_Wn;   //check log function
  
  _Lxx_ref(_period_i) = _SS5*_vari_ini;
  _Lyy_ref(_period_i) = _SS6*_vari_ini;
  _ts(_period_i) = log((_SS7+_SS8)*_vari_ini)/_Wn;    
  
  
  _Lxx_ref_real(i) = _Lxx_ref(_period_i-1);
  _Lyy_ref_real(i) = _Lyy_ref(_period_i-1);
  _Ts_ref_real(i) = _ts(_period_i-1);  
  
  	// cout <<"_Lxx_ref_real:"<<_Lxx_ref_real(i) <<endl;
	// cout <<"_Lyy_ref_real:"<<_Lyy_ref_real(i) <<endl; 
	// cout <<"_Ts_ref_real:"<<_Ts_ref_real(i) <<endl;     
  

  _COMx_is(_period_i-1) = _comx_feed(i-1)-_footx_ref(_period_i-1);  
  _COMx_es.col(_period_i-1) = _SS1*_vari_ini*0.5;  
  _COMvx_is(_period_i-1)= (_COMx_es(_period_i-1)-_COMx_is(_period_i-1)*_SS3*_vari_ini)/(1/_Wn *_SS4*_vari_ini);
  _COMy_is(_period_i-1) = _comy_feed(i-1)-_footy_ref(_period_i-1);  
  _COMy_es.col(_period_i-1) = _SS2*_vari_ini*0.5;  
  _COMvy_is(_period_i-1)= (_COMy_es(_period_i-1)-_COMy_is(_period_i-1)*_SS3*_vari_ini)/(1/_Wn *_SS4*_vari_ini);    

/*  cout <<"_Lxx_ref_real:"<<endl<<_Lxx_ref_real<<endl;
  cout <<"_Lxx_ref_real:"<<endl<<_Lxx_ref_real<<endl; 
  cout <<"_Ts_ref_real:"<<endl<<_Ts_ref_real<<endl;  */   
  
  
// update walking period and step location 
  for (int jxx = _period_i+1; jxx<=_footstepsnumber; jxx++)
  {
    _tx(jxx-1) = _tx(jxx-2)+_ts(jxx-2);
  }
  
//   the real-time calcuated next one step location: similar with the the footx_real calulated in the NMPC ( _footx_real.row(_bjxx) = _V_ini.row(5*_nh);):
  _footx_ref(_period_i)=_footx_ref(_period_i-1)+_SS1*_vari_ini;
  _footy_ref(_period_i)=_footy_ref(_period_i-1)+_SS2*_vari_ini;    

  _comvx_endref= _Wn*_COMx_is(_period_i-1)*_SS4*_vari_ini + _COMvx_is(_period_i-1)*_SS3*_vari_ini;
  _comvy_endref= _Wn*_COMy_is(_period_i-1)*_SS4*_vari_ini + _COMvy_is(_period_i-1)*_SS3*_vari_ini;     
      


//// update CoM state  
   
//   _comx(i)= _COMx_is(_period_i-1)*cosh(_Wndt) + _COMvx_is(_period_i-1)*1/_Wn*sinh(_Wndt)+_px(i);
//   _comy(i)= _COMy_is(_period_i-1)*cosh(_Wndt) + _COMvy_is(_period_i-1)*1/_Wn*sinh(_Wndt)+_py(i);           
//   _comvx(i)= _Wn*_COMx_is(_period_i-1)*sinh(_Wndt) + _COMvx_is(_period_i-1)*cosh(_Wndt);
//   _comvy(i)= _Wn*_COMy_is(_period_i-1)*sinh(_Wndt) + _COMvy_is(_period_i-1)*cosh(_Wndt);     
//   _comax(i)= pow(_Wn,2)*_COMx_is(_period_i-1)*cosh(_Wndt) + _COMvx_is(_period_i-1)*_Wn*sinh(_Wndt);
//   _comay(i)= pow(_Wn,2)*_COMy_is(_period_i-1)*cosh(_Wndt) + _COMvy_is(_period_i-1)*_Wn*sinh(_Wndt);   

    /////  generate the trajectory during the double support phase'
    _nTdx = round(_td(1)/_dt)+1;
    
    //////CoM height variation
    CoM_height_solve(i, _stopwalking,_nTdx);
   
    for (int jxx=1; jxx <=_nTdx; jxx++)
    {
        double _wndtx = _Wn*_dt*jxx;

        _comx(i+jxx-1)= _COMx_is(_period_i-1)*cosh(_wndtx) + _COMvx_is(_period_i-1)*1/_Wn*sinh(_wndtx)+_px(i);
        _comy(i+jxx-1)= _COMy_is(_period_i-1)*cosh(_wndtx) + _COMvy_is(_period_i-1)*1/_Wn*sinh(_wndtx)+_py(i);           
        _comvx(i+jxx-1)= _Wn*_COMx_is(_period_i-1)*sinh(_wndtx) + _COMvx_is(_period_i-1)*cosh(_wndtx);
        _comvy(i+jxx-1)= _Wn*_COMy_is(_period_i-1)*sinh(_wndtx) + _COMvy_is(_period_i-1)*cosh(_wndtx);     
        _comax(i+jxx-1)= pow(_Wn,2)*_COMx_is(_period_i-1)*cosh(_wndtx) + _COMvx_is(_period_i-1)*_Wn*sinh(_wndtx);
        _comay(i+jxx-1)= pow(_Wn,2)*_COMy_is(_period_i-1)*cosh(_wndtx) + _COMvy_is(_period_i-1)*_Wn*sinh(_wndtx);   
            

        _zmpx_real(0,i+jxx-1) = _comx(0,i+jxx-1) - (_comz(0,i+jxx-1) - _Zsc(i+jxx-1))/(_comaz(0,i+jxx-1)+_ggg(0))*_comax(0,i+jxx-1);
        _zmpy_real(0,i+jxx-1) = _comy(0,i+jxx-1) - (_comz(0,i+jxx-1) - _Zsc(i+jxx-1))/(_comaz(0,i+jxx-1)+_ggg(0))*_comay(0,i+jxx-1);
        _dcmx_real(0,i+jxx-1) = _comx(0,i+jxx-1) + _comvx(0,i+jxx-1) * sqrt((_comz(0,i+jxx-1) - _Zsc(i+jxx-1))/(_comaz(0,i+jxx-1)+_ggg(0)));
        _dcmy_real(0,i+jxx-1) = _comy(0,i+jxx-1) + _comvy(0,i+jxx-1) * sqrt((_comz(0,i+jxx-1) - _Zsc(i+jxx-1))/(_comaz(0,i+jxx-1)+_ggg(0)));      
    }


//  external disturbances!!! using feedback data:
 
  /// /// relative state to the actual foot lcoation: very good
  if (_period_i % 2 == 0)  // odd : left support
  {
    estimated_state(0,0) =  estimated_state(0,0) - _Lfoot_location_feedback(0); //relative comx
    estimated_state(3,0) =  estimated_state(3,0) - _Lfoot_location_feedback(1);	// relative comy 	    
  }
  else
  {
    estimated_state(0,0) =  estimated_state(0,0) - _Rfoot_location_feedback(0);
    estimated_state(3,0) =  estimated_state(3,0) - _Rfoot_location_feedback(1);	  
  }  



  _comx_feed(i) = (_lamda_comx*(_comx(i)-_px(i))+(1-_lamda_comx)*estimated_state(0,0))+_px(i);    
  _comvx_feed(i) = _lamda_comvx*_comvx(i) + (1-_lamda_comvx)*estimated_state(1,0);
  _comax_feed(i) = _lamda_comx*_comax(i) + (1-_lamda_comx)*estimated_state(2,0);
  _comy_feed(i) = (_lamda_comy*(_comy(i)-_py(i))+(1-_lamda_comy)*estimated_state(3,0))+_py(i);    
  _comvy_feed(i) = _lamda_comvy*_comvy(i) + (1-_lamda_comvy)*estimated_state(4,0);  
  _comay_feed(i) = _lamda_comy*_comay(i) + (1-_lamda_comy)*estimated_state(5,0);  

  
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


  /// Re_initialize the step parameters when reaching the maximal counter
  if(_t_walkdtime>_t_walkdtime_old)
  {
    Re_Initialize(step_length,step_width);
  }  

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
  com_traj(34) = _period_i-1; 
  com_traj(35) = _ts(_period_i-1);  
  
  com_traj(36) = _Lxx_ref_real(i);
  com_traj(37) = _Lyy_ref_real(i);


  return com_traj;

}






void MPCClass::Indexfind(double goalvari, int xyz)
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

}


void MPCClass::step_timing_object_function(int i)
{
    _AxO(0) = _comx_feed(i-1)-_footx_ref(_period_i-1); _BxO(0) = _comvx_feed(i-1)/_Wn; _Cx(0,0) =-0.5*_Lxx_refx; 
    _Axv = _Wn*_AxO;  _Bxv = _Wn*_BxO; _Cxv=_comvx_endref; 
    _AyO(0) = _comy_feed(i-1)-_footy_ref(_period_i-1); _ByO(0) = _comvy_feed(i-1)/_Wn; _Cy(0,0) =-0.5*_Lyy_refy; 
    _Ayv = _Wn*_AyO;  _Byv = _Wn*_ByO; _Cyv=_comvy_endref;    
    
    _SQ_goal0(0,0)=0.5*_bbx;
    _SQ_goal0(1,1)=0.5*_bby;
    _SQ_goal0(2,2)=0.5*(_rr1+_aax*_AxO(0,0)*_AxO(0,0)+_aay*_AyO(0,0)*_AyO(0,0)+_aaxv*_Axv(0,0)*_Axv(0,0)+_aayv*_Ayv(0,0)*_Ayv(0,0));
    _SQ_goal0(2,3)=0.5*(     _aax*_AxO(0,0)*_BxO(0,0)+_aay*_AyO(0,0)*_ByO(0,0)+_aaxv*_Axv(0,0)*_Bxv(0,0)+_aayv*_Ayv(0,0)*_Byv(0,0));
    _SQ_goal0(3,2)=0.5*(     _aax*_BxO(0,0)*_AxO(0,0)+_aay*_ByO(0,0)*_AyO(0,0)+_aaxv*_Bxv(0,0)*_Axv(0,0)+_aayv*_Byv(0,0)*_Ayv(0,0));
    _SQ_goal0(3,3)=0.5*(_rr2+_aax*_BxO(0,0)*_BxO(0,0)+_aay*_ByO(0,0)*_ByO(0,0)+_aaxv*_Bxv(0,0)*_Bxv(0,0)+_aayv*_Byv(0,0)*_Byv(0,0));    
    _SQ_goal0(4,4)=0.5*_bbx1; 
    _SQ_goal0(5,5)=0.5*_bby1; 
    _SQ_goal0(6,6)=0.5*_rr11; 
    _SQ_goal0(7,7)=0.5*_rr21; 
    
    _SQ_goal = (_SQ_goal0+_SQ_goal0.transpose())/2.0;  
    _Sq_goal << -_bbx*_Lxx_refx,
                -_bby*_Lyy_refy,
                -_rr1*_tr1_ref+_aax*_AxO(0,0)*_Cx(0,0)+_aay*_AyO(0,0)*_Cy(0,0)+_aaxv*_Axv(0,0)*_Cxv(0,0)+_aayv*_Ayv(0,0)*_Cyv(0,0),
                -_rr2*_tr2_ref+_aax*_BxO(0,0)*_Cx(0,0)+_aay*_ByO(0,0)*_Cy(0,0)+_aaxv*_Bxv(0,0)*_Cxv(0,0)+_aayv*_Byv(0,0)*_Cyv(0,0),
                -_bbx1*_Lxx_refx1,
                -_bby1*_Lyy_refy1,
                -_rr11*_tr1_ref1,
                -_rr21*_tr2_ref1;   
	      
    _SQ_goal1 = 2 * _SQ_goal;
    _Sq_goal1 = 2 * _SQ_goal * _vari_ini + _Sq_goal;
    
/*  cout <<"_SQ_goal1:"<<_SQ_goal1<<endl;
  cout <<"_Sq_goal1:"<<_Sq_goal1<<endl;   */   
     
    
    
    _Ax= _SS3.transpose()*_AxO*_SS7+_SS4.transpose()*_BxO*_SS7-_SS1.transpose()*_SS7+_SS4.transpose()*_AxO*_SS8+_SS3.transpose()*_BxO*_SS8;   
    _Bx(0,0)= -0.5*_Lxx_refx1; 
    _Ay= _SS3.transpose()*_AyO*_SS7+_SS4.transpose()*_ByO*_SS7-_SS2.transpose()*_SS7+_SS4.transpose()*_AyO*_SS8+_SS3.transpose()*_ByO*_SS8;   
    _By(0,0)= -0.5*_Lyy_refy1;         ///check!!!!!!!!

    _ixi = _vari_ini.transpose()*_Ax*_vari_ini;
    _iyi = _vari_ini.transpose()*_Ay*_vari_ini;
    _SQ_goal20= _aax1/2.0*2*( 2*(_ixi(0,0)*_Ax.transpose() + 2*_Ax*_vari_ini*(_Ax*_vari_ini).transpose()) + 2*(2*_Bx(0,0)*_Ax.transpose()))+_aay1/2.0*2*( 2*(_iyi(0,0)*_Ay.transpose() + 2*_Ay*_vari_ini*(_Ay*_vari_ini).transpose()) + 2*(2*_By(0,0)*_Ay.transpose()));

    _SQ_goal2 = (_SQ_goal20.transpose()+_SQ_goal20)/2.0;    
    _Sq_goal2= _aax1/2.0*2*(2*_Ax*_vari_ini)*(_vari_ini.transpose()*_Ax*_vari_ini+_Bx)  +  _aay1/2.0*2*(2*_Ay*_vari_ini)*(_vari_ini.transpose()*_Ay*_vari_ini+_By);    
    
    _SQ_goal3 = _SQ_goal1+_SQ_goal2;
    _Sq_goal3 = _Sq_goal1+_Sq_goal2;   
  

}

void MPCClass::step_timing_constraints(int i)
{
  
// // %% constraints
// // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
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

    _trx3_up = _SS7;
    _det_trx3_up = -(_SS7*_vari_ini);
    _det_trx3_up(0,0) += _tr11_max;	

    _trx3_lp = -_SS7;
    _det_trx3_lp = -(-_SS7*_vari_ini);        
    _det_trx3_lp(0,0) -= _tr11_min;    
    
    _trx4_up = _SS8;
    _det_trx4_up = -(_SS8*_vari_ini);
    _det_trx4_up(0,0) += _tr21_max;
    
    _trx4_lp = -_SS8;
    _det_trx4_lp = -(-_SS8*_vari_ini);      
    _det_trx4_lp(0,0) -= _tr21_min;  
    
    _trx.row(0) = _trx1_up; _trx.row(1) = _trx1_lp; _trx.row(2) = _trx2_up; _trx.row(3) = _trx2_lp;
    _trx.row(4) = _trx3_up; _trx.row(5) = _trx3_lp; _trx.row(6) = _trx4_up; _trx.row(7) = _trx4_lp;	
    
    _det_trx.row(0) = _det_trx1_up; _det_trx.row(1) = _det_trx1_lp; _det_trx.row(2) = _det_trx2_up; _det_trx.row(3) = _det_trx2_lp;
    _det_trx.row(4) = _det_trx3_up; _det_trx.row(5) = _det_trx3_lp; _det_trx.row(6) = _det_trx4_up; _det_trx.row(7) = _det_trx4_lp;		
    
    
//     cout <<"_trx:"<<endl<<_trx<<endl;
//     cout <<"_det_trx:"<<endl<<_det_trx<<endl;  
  
  
// tr1 & tr2: equation constraints:   
    _trx12 = (2*(_SS3.transpose()*_SS3-_SS4.transpose()*_SS4)*_vari_ini).transpose();
    _det_trx12 = -(_vari_ini.transpose()*(_SS3.transpose()*_SS3-_SS4.transpose()*_SS4)*_vari_ini);
    _det_trx12(0,0) +=1; 

    _trx121 = (2*(_SS7.transpose()*_SS7-_SS8.transpose()*_SS8)*_vari_ini).transpose();
    _det_trx121 = -(_vari_ini.transpose()*(_SS7.transpose()*_SS7-_SS8.transpose()*_SS8)*_vari_ini);
    _det_trx121(0,0) +=1; 

    _trxx.row(0) = _trx12;   _trxx.row(1) = _trx121;
    _det_trxx.row(0) = _det_trx12; _det_trxx.row(1) = _det_trx121;    

    
/*    cout <<"_trxx:"<<endl<<_trxx<<endl;
    cout <<"_det_trxx:"<<endl<<_det_trxx<<endl;  */    

//  foot location constraints     
    if(_gait_mode==102)
    {

    }
    else
    {
      if (_period_i % 2 == 0)
      {
        
        if (i>=(round(2*_ts(1)/_dt))+1) ///update the footy_limit
        {
            
            _footy_min=-(2*gait::RobotParaClass_HALF_HIP_WIDTH + 0.2); 
      // 	    _footy_max=-(gait::RobotParaClass_HALF_HIP_WIDTH - 0.03); 
            _footy_max = -(gait::RobotPara_FOOT_WIDTH+0.01);
        }
        else
        {
            _footy_min=-(2*gait::RobotParaClass_HALF_HIP_WIDTH + 0.2); 
            _footy_max=-(gait::RobotParaClass_HALF_HIP_WIDTH - 0.03); 	    
        }  
      }
      else
      { 
        if (i>=(round(2*_ts(1)/_dt))+1)
        {
            _footy_max=2*gait::RobotParaClass_HALF_HIP_WIDTH + 0.2; 
          // 	_footy_min=gait::RobotParaClass_HALF_HIP_WIDTH - 0.03; 
            _footy_min =gait::RobotPara_FOOT_WIDTH+0.01;
        }
        else
        {
            _footy_max=2*gait::RobotParaClass_HALF_HIP_WIDTH + 0.2; 
            _footy_min=  gait::RobotParaClass_HALF_HIP_WIDTH - 0.03; 	 
        }     
        
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

    _h_lx_up1 = _SS5;
    _det_h_lx_up1 = -(_SS5*_vari_ini);
    _det_h_lx_up1(0,0) += _footx_max;    

    _h_lx_lp1 = -_SS5;
    _det_h_lx_lp1 = -(-_SS5*_vari_ini);
    _det_h_lx_lp1(0,0) -= _footx_min;     

    _h_ly_up1 = _SS6;
    _det_h_ly_up1 = -(_SS6*_vari_ini);
    _det_h_ly_up1(0,0) -= _footy_min;
    
    _h_ly_lp1 = -_SS6;
    _det_h_ly_lp1 = -(-_SS6*_vari_ini); 
    _det_h_ly_lp1(0,0) += _footy_max;     

    _h_lx_upx.row(0)= _h_lx_up;    _h_lx_upx.row(1)= _h_lx_lp;   _h_lx_upx.row(2)= _h_ly_up;     _h_lx_upx.row(3)= _h_ly_lp;
    _h_lx_upx.row(4)= _h_lx_up1;   _h_lx_upx.row(5)= _h_lx_lp1;  _h_lx_upx.row(6)= _h_ly_up1;    _h_lx_upx.row(7)= _h_ly_lp1;
    _det_h_lx_upx.row(0)=_det_h_lx_up;  _det_h_lx_upx.row(1)=_det_h_lx_lp;  _det_h_lx_upx.row(2)=_det_h_ly_up;  _det_h_lx_upx.row(3)=_det_h_ly_lp;
    _det_h_lx_upx.row(4)=_det_h_lx_up1; _det_h_lx_upx.row(5)=_det_h_lx_lp1; _det_h_lx_upx.row(6)=_det_h_ly_up1; _det_h_lx_upx.row(7)=_det_h_ly_lp1;    

 

    
    
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
      _det_h_lvx_up(0,0) = -(_SS1*_vari_ini-_Lxx_ref(_period_i-1)- _footx_vmax*_dt);

      _h_lvx_lp = -_SS1;
      _det_h_lvx_lp(0,0) = _SS1*_vari_ini-_Lxx_ref(_period_i-1)-_footx_vmin*_dt; 

      _h_lvy_up = _SS2;
      _det_h_lvy_up(0,0) = -(_SS2*_vari_ini-_Lyy_ref(_period_i-1)- _footy_vmax*_dt);

      _h_lvy_lp = -_SS2;
      _det_h_lvy_lp(0,0) = _SS2*_vari_ini-_Lyy_ref(_period_i-1)-_footy_vmin*_dt;  

      
      _h_lvx_up1.setZero(); 
      _h_lvx_lp1.setZero(); 
      _h_lvy_up1.setZero(); 
      _h_lvy_lp1.setZero();	
      
      _det_h_lvx_up1(0,0) = 0.001;

      _det_h_lvx_lp1(0,0) = 0.001; 

      _det_h_lvy_up1(0,0) = 0.001;

      _det_h_lvy_lp1(0,0) = 0.001;  

    }                                   
    _h_lvx_upx.row(0)= _h_lvx_up;    _h_lvx_upx.row(1)= _h_lvx_lp;  _h_lvx_upx.row(2)= _h_lvy_up;   _h_lvx_upx.row(3)= _h_lvy_lp;
    _h_lvx_upx.row(4)= _h_lvx_up1;   _h_lvx_upx.row(5)= _h_lvx_lp1; _h_lvx_upx.row(6)= _h_lvy_up1;  _h_lvx_upx.row(7)= _h_lvy_lp1;
    _det_h_lvx_upx.row(0)=_det_h_lvx_up; _det_h_lvx_upx.row(1)=_det_h_lvx_lp; _det_h_lvx_upx.row(2)=_det_h_lvy_up; _det_h_lvx_upx.row(3)=_det_h_lvy_lp;
    _det_h_lvx_upx.row(4)=_det_h_lvx_up1;_det_h_lvx_upx.row(5)=_det_h_lvx_lp1;_det_h_lvx_upx.row(6)=_det_h_lvy_up1;_det_h_lvx_upx.row(7)=_det_h_lvy_lp1;    

/*    cout <<"_h_lvx_upx:"<<endl<<_h_lvx_upx<<endl;
    cout <<"_det_h_lvx_upx:"<<endl<<_det_h_lvx_upx<<endl;   */    
    


////////////////////////// CoM position relative to the current support center
// CoM accelearation boundary    
    
    _AA= _Wn*sinh(_Wn*_dt); _CCx = _comx_feed(0,i-1)-_footx_ref(_period_i-1,0); _BBx = pow(_Wn,2)*_CCx*cosh(_Wn*_dt); 
		            _CCy = _comy_feed(0,i-1)-_footy_ref(_period_i-1,0); _BBy = pow(_Wn,2)*_CCy*cosh(_Wn*_dt);

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
    
//     cout <<"_CoM_lax_upx:"<<endl<<_CoM_lax_upx<<endl;
//     cout <<"_det_CoM_lax_upx:"<<endl<<_det_CoM_lax_upx<<endl;       
    
    
    
//   CoM velocity_inremental boundary  
    _VAA= cosh(_Wn*_dt); _VCCx = _comx_feed(0,i-1)-_footx_ref(_period_i-1,0); _VBBx = _Wn*_VCCx*sinh(_Wn*_dt); 
		         _VCCy = _comy_feed(0,i-1)-_footy_ref(_period_i-1,0); _VBBy = _Wn*_VCCy*sinh(_Wn*_dt);

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
    
/*    cout <<"_CoM_lvx_upx:"<<endl<<_CoM_lvx_upx<<endl;
    cout <<"_det_CoM_lvx_upx:"<<endl<<_det_CoM_lvx_upx<<endl; */          
    
    
    
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
    
//     cout <<"_CoM_lvx_upx1:"<<endl<<_CoM_lvx_upx1<<endl;
//     cout <<"_det_CoM_lvx_upx1:"<<endl<<_det_CoM_lvx_upx1<<endl; 
      
  
  
  
}




void MPCClass::solve_stepping_timing()
{
  int nVars = 8;
  int nEqCon = 2;
  int nIneqCon = 24 + 12;
  resizeQP(nVars, nEqCon, nIneqCon);	    

  _G = _SQ_goal3;
  _g0 = _Sq_goal3;
  _X = _vari_ini;

// min 0.5 * x G x + g0 x
// _s.t.
// 		CE^T x + ce0 = 0   ///// equality constraints
// 		CI^T x + ci0 >= 0  //// inequality constraints
  _CI.block<8,8>(0,0) = _trx.transpose() * (-1);
  _CI.block<8,8>(0,8) = _h_lx_upx.transpose() * (-1);
  _CI.block<8,8>(0,16) = _h_lvx_upx.transpose() * (-1);
  _CI.block<8,4>(0,24) = _CoM_lax_upx.transpose() * (-1);
  _CI.block<8,4>(0,28) = _CoM_lvx_upx.transpose() * (-1);
  _CI.block<8,4>(0,32) = _CoM_lvx_upx1.transpose() * (-1);
  
  _ci0.block<8,1>(0, 0) = _det_trx;
  _ci0.block<8,1>(8, 0) = _det_h_lx_upx;
  _ci0.block<8,1>(16, 0) = _det_h_lvx_upx;
  _ci0.block<4,1>(24, 0) = _det_CoM_lax_upx;
  _ci0.block<4,1>(28, 0) = _det_CoM_lvx_upx;
  _ci0.block<4,1>(32, 0) = _det_CoM_lvx_upx1;

  
  _CE = _trxx.transpose()*(-1);
  _ce0 = _det_trxx;
  
  
  
  Solve();  

}

void MPCClass::Solve()
{
// min 0.5 * x G x + g0 x
// _s.t.
// 		CE^T x + ce0 = 0
// 		CI^T x + ci0 >= 0
		solve_true = solveQP();

}


//// each time
Eigen::Matrix<double, 18, 1> MPCClass::Foot_trajectory_solve(int j_index, bool _stopwalking)
{
	Eigen::Matrix<double, 18, 1> rffoot_traj;
  rffoot_traj.setZero();

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
        t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.001;
        t_plan(2) = _ts(_bjx1-1) + 0.001;
        
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
          if(_gait_mode==102)
          {
            _ry_left_right = _Lfooty(j_index);
          }
          else
          {
            if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1)+_dt)
            {
              _ry_left_right = (_footxyz_real(1,_bjxx) + _footxyz_real(1,_bjxx-2))/2;
            }
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
          t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.001;
          t_plan(2) = _ts(_bjx1-1) + 0.001;
		
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
          if(_gait_mode==102)
          {
            _ry_left_right = _Rfooty(j_index);
          }
          else
          {
            if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1)+_dt)
            {
              _ry_left_right = (_footxyz_real(1,_bjxx) + _footxyz_real(1,_bjxx-2))/2;
            }
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
          Lfootz_plan(0) = _Lfootvz(j_index-1);     Lfootz_plan(1) = _Lfootaz(j_index-1); Lfootz_plan(2) = _Lfootz(j_index-1); Lfootz_plan(3) = _Rfootz(j_index)+_lift_height_ref(_bjx1-1);
          Lfootz_plan(4) = _footxyz_real(2,_bjxx);  Lfootz_plan(5) = 0;                   Lfootz_plan(6) = 0;		  
          
          
          Eigen::Matrix<double, 7, 1> Lfootz_co;
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
    _Rfooty(j_index) = -_stepwidth(0);
    _Lfooty(j_index) = _stepwidth(0);
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



void MPCClass::CoM_height_solve(int j_index, bool _stopwalking, int ntdx)
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
      
      
      Eigen::Matrix<double, 7, 1> Lfootz_co;
      Lfootz_co.setZero();
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


int MPCClass::Get_maximal_number_reference()
{
  int nsum_max;
  nsum_max = (nsum_x -_n_loop_omit-1);  
  return nsum_max;
}

int MPCClass::Get_maximal_number(double dtx)
{
  int nsum_max;
  nsum_max = (nsum_x -_n_loop_omit-1)*round(_dt/dtx);
  
  return nsum_max;
}



////====================================================================================================================
/////////////////////////// using the lower-level control-loop  sampling time as the reference: every 5ms;  at the same time: just using the next one position + next one velocity

Vector3d MPCClass::XGetSolution_CoM_position(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{
//   //reference com position
//     _CoM_position_optimal.row(0) = _comx;
// 	_CoM_position_optimal.row(1) = _comy;
// 	_CoM_position_optimal.row(2) = _comz;



	
	Vector3d com_inte;	
	com_inte.setZero();

// 	if (walktime>=2)
// 	{
// 	  int t_int; 
// // 	  t_int = floor(walktime / (_dt / dt_sample) );
// 	  t_int = floor(walktime* dt_sample/ _dt);	  
//     //   cout <<"T_int_inter:"<<t_int<<endl;
// 	  ///// chage to be relative time
// 	  double t_cur;
// 	  t_cur = walktime * dt_sample ;
	  

// 	  Eigen::Matrix<double, 4, 1> t_plan;
// 	  t_plan.setZero();
// 	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
// 	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
// 	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
// 	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);
          
// // 	  cout << "t_plan1:"<<endl<<t_plan<<endl;
// // 	  Eigen::MatrixXd AAA1;	
// // 
// // 	  AAA1.setZero(4,4);	
// // 	  AAA1(0,0) = pow(t_plan(0), 3); AAA1(0,1) = pow(t_plan(0), 2); AAA1(0,2) = pow(t_plan(0), 1); AAA1(0,3) = pow(t_plan(0), 0); 
// // 	  AAA1(1,0) = pow(t_plan(1), 3); AAA1(1,1) = pow(t_plan(1), 2); AAA1(1,2) = pow(t_plan(1), 1); AAA1(1,3) = pow(t_plan(0), 0); 
// // 	  AAA1(2,0) = pow(t_plan(2), 3); AAA1(2,1) = pow(t_plan(2), 2); AAA1(2,2) = pow(t_plan(2), 1); AAA1(2,3) = pow(t_plan(0), 0); 
// // 	  AAA1(3,0) = 3*pow(t_plan(2), 2); AAA1(3,1) = 2*pow(t_plan(2), 1); AAA1(3,2) = pow(t_plan(2), 0); AAA1(3,3) = 0;  


// 	  Eigen::Matrix4d AAA_inv;
	  
// 	  double abx1, abx2, abx3, abx4;
// 	  abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
// 	  abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
// 	  abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
// 	  abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
	  

// 	  AAA_inv(0,0) = 1/ abx1;
// 	  AAA_inv(0,1) =  -1/ abx2;
// 	  AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
// 	  AAA_inv(0,3) = 1/ abx4;
	  
// 	  AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
// 	  AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
// 	  AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
// 	  AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
	  
// 	  AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
// 	  AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
// 	  AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
// 	  AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
	  
// 	  AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
// 	  AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
// 	  AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
// 	  AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;
	  	  
	  	  
// /*	  cout << "AAA_inv:"<<endl<<AAA_inv<<endl;*/
	  

	
	  
	  
// 	  Eigen::Matrix<double, 1, 4> t_a_plan;
// 	  t_a_plan.setZero();
// 	  t_a_plan(0) = pow(t_cur-( t_cur - 2*dt_sample), 3);   
// 	  t_a_plan(1) = pow(t_cur-( t_cur - 2*dt_sample), 2);   
// 	  t_a_plan(2) = pow(t_cur-( t_cur - 2*dt_sample), 1);  
// 	  t_a_plan(3) = pow(t_cur-( t_cur - 2*dt_sample), 0); 


	  
// 	  // COM&&foot trajectory interpolation
	  
// 	  Eigen::Vector3d  x10;
// 	  Eigen::Vector3d  x11;
// 	  Eigen::Vector3d  x12;
// // 	  Eigen::Vector3d  x13;


// /*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
// 	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
// 	  x10 = body_in1; 
// 	  x11 = body_in2;  
// 	  x12 = _CoM_position_optimal.col(t_int);
// // 	  x13 = _CoM_position_optimal.col(t_int+1);
	  
	  
// 	  Eigen::Matrix<double, 4, 1>  temp;
// 	  temp.setZero();
// 	  temp(0) = x10(0); temp(1) = x11(0); temp(2) = x12(0); temp(3) = _comvx(t_int);	  
// 	  com_inte(0) = t_a_plan * (AAA_inv)*temp;
// 	  temp(0) = x10(1); temp(1) = x11(1); temp(2) = x12(1); temp(3) = _comvy(t_int);	  
// 	  com_inte(1) = t_a_plan * (AAA_inv)*temp;
// 	  temp(0) = x10(2); temp(1) = x11(2); temp(2) = x12(2); temp(3) = _comvz(t_int);	  
// 	  com_inte(2) = t_a_plan *(AAA_inv)*temp;

  
	  
	  
// 	}
// 	else
// 	{
// 	  com_inte(0) = body_in3(0);	  
// 	  com_inte(1) = body_in3(1);	  	  
// 	  com_inte(2) = body_in3(2);	
	  
// 	}

 	return com_inte;
	
}

Vector3d MPCClass::XGetSolution_body_inclination(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{
//   //reference com position
//         _torso_angle_optimal.row(0) = _thetax;
// 	_torso_angle_optimal.row(1) = _thetay;
// 	_torso_angle_optimal.row(2) = _thetaz;
	
	
	Vector3d com_inte;	
	com_inte.setZero();	
// 	if (walktime>=2)
// 	{
// 	  int t_int; 
// 	  t_int = floor(walktime* dt_sample/ _dt  );

	  
// 	  double t_cur;
// 	  t_cur = walktime * dt_sample;
	  

	  
// 	  Eigen::Matrix<double, 4, 1> t_plan;
// 	  t_plan.setZero();
// 	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
// 	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
// 	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
// 	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);

// // 	  Eigen::MatrixXd AAA;	
// // 
// // 	  AAA.setZero(4,4);	
// // 	  AAA(0,0) = pow(t_plan(0), 3); AAA(0,1) = pow(t_plan(0), 2); AAA(0,2) = pow(t_plan(0), 1); AAA(0,3) = pow(t_plan(0), 0); 
// // 	  AAA(1,0) = pow(t_plan(1), 3); AAA(1,1) = pow(t_plan(1), 2); AAA(1,2) = pow(t_plan(1), 1); AAA(1,3) = pow(t_plan(0), 0); 
// // 	  AAA(2,0) = pow(t_plan(2), 3); AAA(2,1) = pow(t_plan(2), 2); AAA(2,2) = pow(t_plan(2), 1); AAA(2,3) = pow(t_plan(0), 0); 
// // 	  AAA(3,0) = 3*pow(t_plan(2), 2); AAA(3,1) = 2*pow(t_plan(2), 1); AAA(3,2) = pow(t_plan(2), 0); AAA(3,3) = 0;  


// 	  Eigen::Matrix4d AAA_inv;
	  
// 	  double abx1, abx2, abx3, abx4;
// 	  abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
// 	  abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
// 	  abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
// 	  abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
	  

// 	  AAA_inv(0,0) = 1/ abx1;
// 	  AAA_inv(0,1) =  -1/ abx2;
// 	  AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
// 	  AAA_inv(0,3) = 1/ abx4;
	  
// 	  AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
// 	  AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
// 	  AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
// 	  AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
	  
// 	  AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
// 	  AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
// 	  AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
// 	  AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
	  
// 	  AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
// 	  AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
// 	  AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
// 	  AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;
	  	  
	  
	  
	  
	
	  
	  
// 	  Eigen::Matrix<double, 1, 4> t_a_plan;
// 	  t_a_plan.setZero();
// 	  t_a_plan(0) = pow(t_cur-( t_cur - 2*dt_sample), 3);   t_a_plan(1) = pow(t_cur-( t_cur - 2*dt_sample), 2);   t_a_plan(2) = pow(t_cur-( t_cur - 2*dt_sample), 1);  t_a_plan(3) = pow(t_cur-( t_cur - 2*dt_sample), 0); 


	  
// 	  // COM&&foot trajectory interpolation
	  
// 	  Eigen::Vector3d  x10;
// 	  Eigen::Vector3d  x11;
// 	  Eigen::Vector3d  x12;
// // 	  Eigen::Vector3d  x13;


// /*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
// 	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
// 	  x10 = body_in1; 
// 	  x11 = body_in2;  
// 	  x12 = _torso_angle_optimal.col(t_int);
// // 	  x13 = _torso_angle_optimal.col(t_int+1);
	  
	  
// 	  Eigen::Matrix<double, 4, 1>  temp;
// 	  temp.setZero();
// 	  temp(0) = x10(0); temp(1) = x11(0); temp(2) = x12(0); temp(3) = _thetavx(t_int);	  
// 	  com_inte(0) = t_a_plan * (AAA_inv)*temp;
// 	  temp(0) = x10(1); temp(1) = x11(1); temp(2) = x12(1); temp(3) = _thetavy(t_int);	  
// 	  com_inte(1) = t_a_plan * (AAA_inv)*temp;
// 	  temp(0) = x10(2); temp(1) = x11(2); temp(2) = x12(2); temp(3) = _thetavz(t_int);	  
// 	  com_inte(2) = t_a_plan *(AAA_inv)*temp;

	  
	  
// 	}
// 	else
// 	{
// 	  com_inte(0) = body_in3(0);	  
// 	  com_inte(1) = body_in3(1);	  	  
// 	  com_inte(2) = body_in3(2);	
	  
// 	}

 	return com_inte;
	
  
  
  
}


Vector3d MPCClass::XGetSolution_Foot_positionR(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{
	
    //     _R_foot_optition_optimal.row(0) = _Rfootx;
	// _R_foot_optition_optimal.row(1) = _Rfooty;
	// _R_foot_optition_optimal.row(2) = _Rfootz;
	
	Vector3d com_inte;	
	com_inte.setZero();
// 	if (walktime>=2)
// 	{
// 	  int t_int; 
// 	  t_int = floor(walktime* dt_sample/ _dt  );

	  
// 	  double t_cur;
// 	  t_cur = walktime * dt_sample;
	  

	  
// 	  Eigen::Matrix<double, 4, 1> t_plan;
// 	  t_plan.setZero();
// 	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
// 	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
// 	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
// 	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);

// // 	  cout << "t_plan2:"<<endl<<t_plan<<endl;
// // 	  Eigen::MatrixXd AAA;
// // 
// // 	  
// // 	  AAA.setZero(4,4);	
// // 	  AAA(0,0) = pow(t_plan(0), 3); AAA(0,1) = pow(t_plan(0), 2); AAA(0,2) = pow(t_plan(0), 1); AAA(0,3) = pow(t_plan(0), 0); 
// // 	  AAA(1,0) = pow(t_plan(1), 3); AAA(1,1) = pow(t_plan(1), 2); AAA(1,2) = pow(t_plan(1), 1); AAA(1,3) = pow(t_plan(0), 0); 
// // 	  AAA(2,0) = pow(t_plan(2), 3); AAA(2,1) = pow(t_plan(2), 2); AAA(2,2) = pow(t_plan(2), 1); AAA(2,3) = pow(t_plan(0), 0); 
// // 	  AAA(3,0) = 3*pow(t_plan(2), 2); AAA(3,1) = 2*pow(t_plan(2), 1); AAA(3,2) = pow(t_plan(2), 0); AAA(3,3) = 0;  


	  
// 	  Eigen::Matrix4d AAA_inv;
	  
// 	  double abx1, abx2, abx3, abx4;
// 	  abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
// 	  abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
// 	  abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
// 	  abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
	  

// 	  AAA_inv(0,0) = 1/ abx1;
// 	  AAA_inv(0,1) =  -1/ abx2;
// 	  AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
// 	  AAA_inv(0,3) = 1/ abx4;
	  
// 	  AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
// 	  AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
// 	  AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
// 	  AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
	  
// 	  AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
// 	  AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
// 	  AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
// 	  AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
	  
// 	  AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
// 	  AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
// 	  AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
// 	  AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;
	  	  
	  
// /*	  cout << "AAA_inv:"<<endl<<AAA_inv<<endl;	*/  	  
	  

	
	  
	  
// 	  Eigen::Matrix<double, 1, 4> t_a_plan;
// 	  t_a_plan.setZero();
// 	  t_a_plan(0) = pow(t_cur-( t_cur - 2*dt_sample), 3);  
// 	  t_a_plan(1) = pow(t_cur-( t_cur - 2*dt_sample), 2);  
// 	  t_a_plan(2) = pow(t_cur-( t_cur - 2*dt_sample), 1);  
// 	  t_a_plan(3) = pow(t_cur-( t_cur - 2*dt_sample), 0); 


	  
// 	  // COM&&foot trajectory interpolation
	  
// 	  Eigen::Vector3d  x10;
// 	  Eigen::Vector3d  x11;
// 	  Eigen::Vector3d  x12;
// // 	  Eigen::Vector3d  x13;


// /*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
// 	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
// 	  x10 = body_in1; 
// 	  x11 = body_in2;  
// 	  x12 =  _R_foot_optition_optimal.col(t_int);
// //	  x13 =  _R_foot_optition_optimal.col(t_int+1);
	  
	  
// 	  Eigen::Matrix<double, 4, 1>  temp;
// 	  temp.setZero();
// 	  temp(0) = x10(0); temp(1) = x11(0); temp(2) = x12(0); temp(3) = _Rfootvx(t_int);	  
// 	  com_inte(0) = t_a_plan * (AAA_inv)*temp;
// 	  temp(0) = x10(1); temp(1) = x11(1); temp(2) = x12(1); temp(3) = _Rfootvy(t_int);

// // 	  cout << "Rfooty:"<<temp<<endl;
// 	  com_inte(1) = t_a_plan * (AAA_inv)*temp;
// 	  temp(0) = x10(2); temp(1) = x11(2); temp(2) = x12(2); temp(3) = _Rfootvz(t_int);	  
// 	  com_inte(2) = t_a_plan *(AAA_inv)*temp;
	  
// ////=====================================================////////////////////////////////////////////////
// // 	  //////spline for Rfoot_
	  
	  
	  
	  
	  
	  
	  
	  
	  
// 	}
// 	else
// 	{
// 	  com_inte(0) = body_in3(0);	  
// 	  com_inte(1) = body_in3(1);	  	  
// 	  com_inte(2) = body_in3(2);

// // 	  com_inte = Rfoot_IN.col(walktime);	  
// 	}
	
// // 	cout << "Rfooty_generated:"<<com_inte(1)<<endl;

 	return com_inte;
	
	
	
	
}

Vector3d MPCClass::XGetSolution_Foot_positionL(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{
    //     _L_foot_optition_optimal.row(0) = _Lfootx;
	// _L_foot_optition_optimal.row(1) = _Lfooty;
	// _L_foot_optition_optimal.row(2) = _Lfootz;
	
	Vector3d com_inte;	
	com_inte.setZero();
	
// 	if (walktime>=2)
// 	{
// 	  int t_int; 
// 	  t_int = floor(walktime* dt_sample/ _dt  );

	  
// 	  double t_cur;
// 	  t_cur = walktime * dt_sample;
	  

	  
// 	  Eigen::Matrix<double, 4, 1> t_plan;
// 	  t_plan.setZero();
// 	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
// 	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
// 	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
// 	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);

// //           cout << "t_plan3:"<<endl<<t_plan<<endl;
// /*	  Eigen::MatrixXd  AAAaaa; /// should be Marix4d
// 	  AAAaaa.setZero(4,4);
	  
// 	  AAAaaa(0,0) = pow(t_plan(0), 3); AAAaaa(0,1) = pow(t_plan(0), 2); AAAaaa(0,2) = pow(t_plan(0), 1); AAAaaa(0,3) = pow(t_plan(0), 0); 
// 	  AAAaaa(1,0) = pow(t_plan(1), 3); AAAaaa(1,1) = pow(t_plan(1), 2); AAAaaa(1,2) = pow(t_plan(1), 1); AAAaaa(1,3) = pow(t_plan(1), 0); 
// 	  AAAaaa(2,0) = pow(t_plan(2), 3); AAAaaa(2,1) = pow(t_plan(2), 2); AAAaaa(2,2) = pow(t_plan(2), 1); AAAaaa(2,3) = pow(t_plan(2), 0); 
//          // AAAaaa(3,0) = pow(t_plan(3), 3); AAAaaa(3,1) = pow(t_plan(3), 2); AAAaaa(3,2) = pow(t_plan(3), 1); AAAaaa(3,3) = pow(t_plan(3), 0); /// using the next to positioin would cause over-fitting	  
// 	  AAAaaa(3,0) = 3*pow(t_plan(2), 2); AAAaaa(3,1) = 2*pow(t_plan(2), 1); AAAaaa(3,2) = pow(t_plan(2), 0); AAAaaa(3,3) = 0.0;  	  
//  */	  
// //       MatrixXd.inverse( != Matrix4d (under Xd =4. so write the inverse of Matrix4d explicitly): for the time being)
	  
	  
// 	  Eigen::Matrix4d AAA_inv;
	  
// 	  double abx1, abx2, abx3, abx4;
// 	  abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
// 	  abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
// 	  abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
// 	  abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
	  

// 	  AAA_inv(0,0) = 1/ abx1;
// 	  AAA_inv(0,1) =  -1/ abx2;
// 	  AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
// 	  AAA_inv(0,3) = 1/ abx4;
	  
// 	  AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
// 	  AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
// 	  AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
// 	  AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
	  
// 	  AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
// 	  AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
// 	  AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
// 	  AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
	  
// 	  AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
// 	  AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
// 	  AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
// 	  AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;
	  	  
// /*	  cout << "AAA_inv:"<<endl<<AAA_inv<<endl;	*/  
	  
	  
	  
	
// // // 	  Eigen::Matrix4d  AAAaaa1_inv=AAA_inv;
	  
// // 	  cout<< t_plan<<endl;
// // 	  cout<< AAAaaa<<endl;
// // 	  cout<< AAA_inv - AAAaaa1_inv<<endl;
// // 	  cout<< AAA_inv - AAAaaa1.inverse()<<endl;
	  
// 	  Eigen::RowVector4d t_a_plan;
// 	  t_a_plan.setZero();
// 	  t_a_plan(0) = pow(t_cur-( t_cur - 2*dt_sample), 3);  
// 	  t_a_plan(1) = pow(t_cur-( t_cur - 2*dt_sample), 2);   
// 	  t_a_plan(2) = pow(t_cur-( t_cur - 2*dt_sample), 1); 
// 	  t_a_plan(3) = pow(t_cur-( t_cur - 2*dt_sample), 0); 


	  
// 	  // COM&&foot trajectory interpolation
	  
// 	  Eigen::Vector3d  x10;
// 	  Eigen::Vector3d  x11;
// 	  Eigen::Vector3d  x12;
// // 	  Eigen::Vector3d  x13;


// /*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
// 	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
// 	  x10 = body_in1; 
// 	  x11 = body_in2;  
// 	  x12 =  _L_foot_optition_optimal.col(t_int);
// // 	  x13 =  _L_foot_optition_optimal.col(t_int+1); /// using next two position would caused over-fitting
	  
	  
// 	  Eigen::Vector4d  temp;
// 	  temp.setZero();
// 	  temp(0) = x10(0); temp(1) = x11(0); temp(2) = x12(0);
// 	  temp(3) = _Lfootvx(t_int);
// // 	  	  temp(3) = x13(0);
// 	  Eigen::Vector4d tmp111 = AAA_inv*temp;
// 	  com_inte(0) = t_a_plan * tmp111;
// 	  temp(0) = x10(1); temp(1) = x11(1); temp(2) = x12(1);
// 	  temp(3) = _Lfootvy(t_int);	 
// /*          temp(3) = x13(1);	*/  
// 	  tmp111 = AAA_inv*temp;  
// 	  com_inte(1) = t_a_plan * tmp111;
// 	  temp(0) = x10(2); temp(1) = x11(2); temp(2) = x12(2); 
// 	  temp(3) = _Lfootvz(t_int);	
// // 	  temp(3) = x13(2);
// 	  tmp111 = AAA_inv*temp;	  
// 	  com_inte(2) = t_a_plan * tmp111;

// ////================================================not use spline.h=====////////////////////////////////////////////////


	  
// 	}
// 	else
// 	{
// 	  com_inte(0) = body_in3(0);	  
// 	  com_inte(1) = body_in3(1);	  	  
// 	  com_inte(2) = body_in3(2);
// 	}

 	return com_inte;
	
	cout << "Lfooty_generated:"<<com_inte(1)<<endl;
}




void MPCClass::File_wl_steptiming()
{
}

////// sin and cos curve;
Eigen::Matrix<double, 18, 1> MPCClass::XGetSolution_Foot_rotation(const int walktime, const double dt_sample,int j_index)
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


