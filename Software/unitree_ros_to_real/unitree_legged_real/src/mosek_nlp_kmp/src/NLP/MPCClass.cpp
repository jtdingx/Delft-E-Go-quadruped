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
  YAML::Node config = YAML::LoadFile("/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/config/config.yaml");

  _dt =  config["dt_slow_mpc"].as<double>();
  _tstep =  config["t_period"].as<double>();
  // _footstepsnumber = config["footnumber"].as<double>();  
  _footstepsnumber = 10; /// fixed this 

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

  Homing_p_retarget.setZero();

  Homing_p_retarget(0,0)  = config["body_p_Homing_Retarget0"].as<double>();
  Homing_p_retarget(1,0)  = config["body_p_Homing_Retarget1"].as<double>();

  _lamda_comx = config["_lamda_comx"].as<double>(); 
  _lamda_comvx = config["_lamda_comvx"].as<double>(); 
  _lamda_comy = config["_lamda_comy"].as<double>(); 
  _lamda_comvy = config["_lamda_comvy"].as<double>();    

}




void MPCClass::FootStepInputs( double stepwidth, double steplength, double stepheight, double stepyaw)
{	
  _steplength.setZero(_footstepsnumber,1);
  _stepwidth.setZero(_footstepsnumber,1);
  _stepheight.setZero(_footstepsnumber,1);
  _lift_height_ref.setZero(_footstepsnumber,1);
 

  ///// rotation step parameters
  hip_width = 2* gait::RobotParaClass_HALF_HIP_WIDTH; 

	hip_width_ref.setZero(_footstepsnumber,1);
  width_ref.setZero(_footstepsnumber,1);
  for (int j =0; j<_footstepsnumber;j++) /// to the bipedal robots
  {
    hip_width_ref(j) = (int)pow(-1,j)*hip_width;
  }
  hip_width_ref(0) = 0;  
  width_ref = hip_width_ref;
  
	_steplength_yaw.setZero(_footstepsnumber,1);
  _stepwidth_yaw.setZero(_footstepsnumber,1); 


    // ////=========================step parameters with yaw motion=======================////
    // ///// recomputing the step parameters to match the rotation///////
    // stepyaw = 1*M_PI/6;
    // Eigen::Matrix2d r_rotation;
    // r_rotation(0,0) = cos(stepyaw);
    // r_rotation(0,1) = -sin(stepyaw);
    // r_rotation(1,0) = sin(stepyaw);
    // r_rotation(1,1) = cos(stepyaw);
    

    // steplength = 0.05;
    // Vector2d step_para;
    // for(int i = 5; i<_footstepsnumber-4; i++)
    // {
    //   step_para << steplength,
    //                stepwidth+width_ref(i);

    //   Vector2d step_para_rotation = r_rotation * step_para;
    //   _steplength_yaw(i) =  step_para_rotation(0);
    //   _stepwidth_yaw(i) =  step_para_rotation(1) - width_ref(i);    
    // }
    // cout<<"step length with yaw motion:"<<_steplength_yaw.block<10,1>(2,0).transpose()<<endl;
    // cout<<"step width with yaw motion:"<<_stepwidth_yaw.block<10,1>(2,0).transpose()<<endl;
    // cout<<"Rotation matrix with yaw motion:"<<r_rotation<<endl;
    // cout<<"step_para with yaw motion:"<<step_para<<endl;

    // //=========================== test rotation ========================================/////



  ///
	_steplength.setConstant(steplength);
	_steplength(0) = 0;
  if(steplength>=0.4)
  {
    _steplength(1) *= 0.5;
  }
  _steplength(_footstepsnumber-4) = 0;  
  _steplength(_footstepsnumber-3) = 0;  
  _steplength(_footstepsnumber-2) = 0;
  _steplength(_footstepsnumber-1) = 0;


	_stepwidth.setConstant(stepwidth);
	_stepwidth(0) = 0;	
  _stepwidth(_footstepsnumber-4) = 0;  
  _stepwidth(_footstepsnumber-3) = 0;  
  _stepwidth(_footstepsnumber-2) = 0;
  _stepwidth(_footstepsnumber-1) = 0;
  
	_stepheight.setConstant(stepheight);
	_lift_height_ref.setConstant(_lift_height);
  ////
  _step_yaw.setZero(_footstepsnumber,1);
	_step_yaw(0) = 0;	
  _step_yaw(_footstepsnumber-4) = 0;  
  _step_yaw(_footstepsnumber-3) = 0;  
  _step_yaw(_footstepsnumber-2) = 0;
  _step_yaw(_footstepsnumber-1) = 0;  
  _yaw_ref.setZero(_footstepsnumber,1);	 
     
  // /////// test yaw motion ===========
  _lift_height_ref_flag = _lift_height_ref.block<4,1>(0,0);


}


void MPCClass::Initialize()
{
  
  config_set();  
// 	reference footstep locations setup
  _footx_ref.setZero(10,1);
	_footy_ref.setZero(10,1);
	_footz_ref.setZero(10,1);		

  for (int i = 1; i < 10; i++) {
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

  // footx_pre = 0;
  // footy_pre = 0;
  // footz_pre = 0;

  _steptuneflag.setZero(_footstepsnumber,1);	
  _stepyawflag.setZero(_footstepsnumber,1);
  _length_key_pre = 0;
  _width_key_pre = 0;
  _yaw_key_pre = 0;
  quadrupedal_width = 2*gait::RobotParaClass_HALF_HIP_WIDTH;

  FR_fixed<< gait::RobotParaClass_HALF_HIP_LENGTH,
             -gait::RobotParaClass_HALF_HIP_WIDTH,
             0;

  FL_fixed<< gait::RobotParaClass_HALF_HIP_LENGTH,
             gait::RobotParaClass_HALF_HIP_WIDTH,
             0;
  RR_fixed<< -gait::RobotParaClass_HALF_HIP_LENGTH,
             -gait::RobotParaClass_HALF_HIP_WIDTH,
             0;
  RL_fixed<< -gait::RobotParaClass_HALF_HIP_LENGTH,
             gait::RobotParaClass_HALF_HIP_WIDTH,
             0;
  
	cout<<"FR_fixed:"<<FR_fixed.transpose()<<endl;
	cout<<"FL_fixed:"<<FL_fixed.transpose()<<endl;
	cout<<"RR_fixed:"<<RR_fixed.transpose()<<endl;
	cout<<"RL_fixed:"<<RL_fixed.transpose()<<endl;

	// == step cycle setup
  _ts.setZero(10,1);
	_ts.setConstant(_tstep);

	_td = 0.2*_tstep;
	_tx.setZero(10,1);
  	for (int i = 1; i < 10; i++) {
 	  _tx(i) = _tx(i-1) + _ts(i-1);
	  _tx(i) = round(_tx(i)/_dt)*_dt -0.000001;	  
	}	

	//parameters	
	_g = gait::_g;	
	_ggg.setConstant(9.8);
	_Wn = sqrt(_g/_hcom);
	
	_Wndt = _Wn*_dt;	
	
  // COM state
	_comx.setZero(); _comvx.setZero(); _comax.setZero();
	_comy.setZero(); _comvy.setZero(); _comay.setZero();
	_comz.setConstant(_hcom); _comvz.setZero(); _comaz.setZero();

	
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// %% parameters for first MPC-step timing adjustment and next one step location
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%	
	_px.setZero(); _py.setZero(); _pz.setZero();
	_zmpvx.setZero(); _zmpvy.setZero(); 
	_COMx_is.setZero(1,10); _COMx_es.setZero(1,10); _COMvx_is.setZero(1,10); 
	_COMy_is.setZero(1,10); _COMy_es.setZero(1,10); _COMvy_is.setZero(1,10);
	_comx_feed = 0; _comvx_feed = 0; //_comax_feed.setZero();
	_comy_feed = 0; _comvy_feed = 0; //_comay_feed.setZero();

	
	_Vari_ini.setZero(); //Lxx,Lyy,Tr1,Tr2,Lxx1,Lyy1,Tr11,Tr21;
	_vari_ini.setZero();

	//step timing constraints:
	_t_min = 0.2; _t_max = 0.5;
	
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


  _footx_max=0.07;
  _footx_min=-0.07;
  _footy_max= 0.07; 
  _footy_min= -0.07;
	
	_mass = _robot_mass; 
	_j_ini = _mass* pow(_rad,2);	
	
	///external force
	_FX =160;  _FY =120;
	_t_last = 0.5;
	_det_xa = _FX/_mass;  _det_ya = _FY/_mass; 
	_det_xv = _det_xa*_t_last; _det_yv = _det_ya*_t_last;
	_det_xp = pow(_det_xv,2)/(2*_det_xa); _det_yp = pow(_det_yv,2)/(2*_det_ya);
	

	
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
	
	
	_Lfootx=0; _Lfooty =(_stepwidth(0));_Lfootz=0; 
  _Lfootvx=0; _Lfootvy=0;_Lfootvz=0; 
	_Lfootax=0; _Lfootay=0;_Lfootaz=0;
	_Rfootx=0; _Rfooty =(-_stepwidth(0));_Rfootz=0; 
  _Rfootvx=0; _Rfootvy=0;_Rfootvz=0; 
	_Rfootax=0; _Rfootay=0;_Rfootaz=0;	
	_ry_left_right = 0;	

  _Rfootxyz_pre.setZero(); _Rfootvxyz_pre.setZero(); _Rfootaxyz_pre.setZero(); 
  _Lfootxyz_pre.setZero(); _Lfootvxyz_pre.setZero(); _Lfootaxyz_pre.setZero();

	
  //footz refer: height of step
	_Zsc.setZero();		
   

   right_support = 0; 
   _j_count = 0;

  _t_end_footstep = 1000000000;  
  
  _Lfoot_r.setZero();
  _Rfoot_r.setZero();   
  _Lfoot_rv.setZero();
  _Rfoot_rv.setZero();     
  _Lfoot_ra.setZero();
  _Rfoot_ra.setZero(); 

	solve_true = false;

	// com_mpc_ref.setZero();
  com_mpc_ref.setConstant(_hcom);
  comv_mpc_ref.setZero();
  rfoot_mpc_ref.setZero();
  lfoot_mpc_ref.setZero();
  yaw_mpc_ref.setZero();
  // yaw_velo_mpc_ref.setZero();
	support_prediction.setZero();
	support_position_mpc_ref.setZero();  

  ///// ====================================================
	///// period flag and time flag
	_steplength_flag = _steplength.block<4,1>(0,0); 
  _stepwidth_flag = _stepwidth.block<4,1>(0,0);
  _stepheight_flag = _stepheight.block<4,1>(0,0);
  _ts_flag.setConstant(_tstep);
  _tx_flag = _tx.block<4,1>(0,0);
	_bjx1_flag = 0;
  _bjxx_flag = 0;  
  _period_i_flag = 0;
  _period_i_flag_old = 0;

  _footx_ref_flag = _footx_ref.block<4,1>(0,0);
  _footy_ref_flag = _footy_ref.block<4,1>(0,0);
  _footz_ref_flag = _footz_ref.block<4,1>(0,0);
  _Lxx_ref_flag = _steplength.block<4,1>(0,0); 
  _Lyy_ref_flag = _stepwidth.block<4,1>(0,0); 

  _footx_offline_flag = _footx_ref_flag;
  _footy_offline_flag = _footy_ref_flag;
  _footz_offline_flag = _footz_ref_flag;

	index_flag = 0;
	period_flag << 0,
                 1,
                 2,
                 3;

  period_index_flag = 0;
  bjx1_index_flag = 0;
  bjxx_index_flag = 0;

	hip_width_ref_flag = hip_width_ref.block<4,1>(0,0);
  width_ref_flag = width_ref.block<4,1>(0,0);
	_steplength_yaw_flag = _steplength_yaw.block<4,1>(0,0);
  _stepwidth_yaw_flag = _stepwidth_yaw.block<4,1>(0,0);  
	_step_yaw_flag = _step_yaw.block<4,1>(0,0);
	_yaw_ref_flag = _yaw_ref.block<4,1>(0,0); 

  _footxyz_real_flag.setZero();

  _steptuneflag_flag.setZero();
  _stepyawflag_flag.setZero();

  _td_flag = 0.2 * _ts_flag;

  _footxyz_real_flag_fixed.setZero();

  cout<<"======================================="<<endl;
  cout<< "_steplength_flag:"<<_steplength_flag<<endl;
  cout<< "_stepwidth_flag:"<<_stepwidth_flag<<endl;
  cout<< "_tx_flag:"<<_tx_flag<<endl;
  cout<< "_ts_flag:"<<_ts_flag<<endl;
  cout<<"======================================="<<endl;

}

// void MPCClass::Re_Initialize(double step_length_ref, double step_width_ref)
// {
//   cout<<"reinitialization"<<endl;
//   _re_initilization = 1;
//   _steptuneflag.setZero();
// 	_steplength.setConstant(step_length_ref);
// 	_steplength(0) = 0;
// 	// for(int i=15; i<_footstepsnumber;i++)
// 	// {
//   //      _steplength(i) *= (-1);
// 	// }
//   //_steplength(15) = 0;
//   _steplength(_footstepsnumber-4) = 0;    
//   _steplength(_footstepsnumber-3) = 0;  
//   _steplength(_footstepsnumber-2) = 0;
//   _steplength(_footstepsnumber-1) = 0;
// 	_stepwidth.setConstant(step_width_ref);
//   if(_gait_mode !=102)
//   {
// 	 _stepwidth(0) = _stepwidth(0)/2;
//   }
//   else
//   {
//    _stepwidth(0) = 0;
//   }
// 	// for(int i=15; i<_footstepsnumber;i++)
// 	// {
//   //   _stepwidth(i) *= (-1);
// 	// }
//   //_stepwidth(15) = 0;
//   _stepwidth(_footstepsnumber-4) = 0;    
//   _stepwidth(_footstepsnumber-3) = 0; 
//   _stepwidth(_footstepsnumber-2) = 0;
//   _stepwidth(_footstepsnumber-1) = 0;
//   ////NP initialization for step timing optimization
//   //// reference steplength and stepwidth for timing optimization  
//  	// ==step loctions setup==
//   // _Lxx_ref = _steplength;
//   // _Lyy_ref = _stepwidth;	   
//   if(_gait_mode ==102) ///troting gait
//   {
//   }
//   else
//   {
//     // // local coordinate
//   	// _Lyy_ref(0) = 0;    
//     // for (int j =0; j<_footstepsnumber;j++)
//     // {
//     //   _Lyy_ref(j) = (int)pow(-1,j)*_stepwidth(j);
//     // }
//   }
// // 	reference footstep locations setup
//   _footx_ref.setZero();
// 	_footy_ref.setZero();
// 	_footz_ref.setZero();		
//   for (int i = 1; i < _footstepsnumber; i++) {
//  	  _footx_ref(i) = _footx_ref(i-1) + _steplength(i-1);
//     _yaw_ref(i-1) = _yaw_ref(_footstepsnumber-1); ////maintain the previous rotation angle
//     if(_gait_mode ==102) ///troting gait
//     {
//       _footy_ref(i) = _footy_ref(i-1) + _stepwidth(i-1);
//     }
//     else
//     {     
//       _footy_ref(i) = _footy_ref(i-1) + (int)pow(-1,i-1)*_stepwidth(i-1);  
//     } 
// 	  _footz_ref(i) = _footz_ref(i-1) + _stepheight(i-1);
// 	}
// 	// == step cycle setup
// 	_ts.setConstant(_tstep);
// 	_tx.setZero();
//   	for (int i = 1; i < _footstepsnumber; i++) {
//  	  _tx(i) = _tx(i-1) + _ts(i-1);
// 	  _tx(i) = round(_tx(i)/_dt)*_dt -0.000001;	  
// 	}	
//   // _t_end_footstep = round((_tx(_footstepsnumber-1)- 2*_tstep)/_dt);
//   _t_end_footstep = 1000000000;
//   _comx_feed = 0;
//   _comvx_feed = 0;
//   _comy_feed = 0;
//   _comvy_feed = 0; 
//   ///// ====================================================
// 	///// period flag and time flag
// 	_steplength_flag = _steplength.block<4,1>(0,0); 
//   _stepwidth_flag = _stepwidth.block<4,1>(0,0);
//   _ts_flag.setConstant(_tstep);
//   _tx_flag = _tx.block<4,1>(0,0);
// 	_bjx1_flag = 0;
//   _bjxx_flag = 0;  
//   _period_i_flag = 0;
//   _period_i_flag_old = 0;
//   _footx_ref_flag = _footx_ref.block<4,1>(0,0);
//   _footy_ref_flag = _footy_ref.block<4,1>(0,0);
//   _footz_ref_flag = _footz_ref.block<4,1>(0,0);
//   _Lxx_ref_flag = _steplength.block<4,1>(0,0); 
//   _Lyy_ref_flag = _stepwidth.block<4,1>(0,0); 
//   _footx_offline_flag = _footx_ref_flag;
//   _footy_offline_flag = _footy_ref_flag;
//   _footz_offline_flag = _footz_ref_flag;
// 	index_flag = 0;
// 	period_flag << 0,
//                  1,
//                  2,
//                  3;
//   period_index_flag = 0;
//   bjx1_index_flag = 0;
//   bjxx_index_flag = 0;
//   _steptuneflag_flag.setZero();
//   _stepyawflag_flag.setZero();  
//   _td_flag = 0.2 * _ts_flag;
//   _footxyz_real_flag_fixed.setZero();
//   cout<< "_steplength_flag:"<<_steplength_flag<<endl;
//   cout<< "_stepwidth_flag:"<<_stepwidth_flag<<endl;
//   cout<< "_tx_flag:"<<_tx_flag<<endl;
//   cout<< "_ts_flag:"<<_ts_flag<<endl;
// }


void MPCClass::command_foot_step(double  step_length_keyboard, double step_width_keyboard, double step_yaw_keyboard)
{
    ////=========================step parameters with yaw motion=======================////
    ///// recomputing the step parameters to match the rotation///////
    Eigen::Matrix2d r_rotation;
    r_rotation(0,0) = cos(step_yaw_keyboard);
    r_rotation(0,1) = -sin(step_yaw_keyboard);
    r_rotation(1,0) = sin(step_yaw_keyboard);
    r_rotation(1,1) = cos(step_yaw_keyboard);
    ////==========================update the parameters for vector_flag;
    Vector2d step_para_flag;
    for(int i = period_index_flag+1; i<4; i++)
    {
      step_para_flag << step_length_keyboard,
                        step_width_keyboard;

      Vector2d step_para_rotation_flag = r_rotation* step_para_flag;
      _steplength_yaw_flag(i) =  step_para_rotation_flag(0);
      _stepwidth_yaw_flag(i) =  step_para_rotation_flag(1);    
    }
    
    for(int i = period_index_flag+1; i<4; i++)
    {
      _steplength_flag(i) = _steplength_yaw_flag(i);
      _stepwidth_flag(i) = _stepwidth_yaw_flag(i);     
      _Lxx_ref_flag(i) = _steplength_flag(i);
      _Lyy_ref_flag(i) = _stepwidth_flag(i);   
    }	    

    // 	reference footstep locations setup	
    for (int i = period_index_flag+2; i < 4; i++) {
      _footx_ref_flag(i) = _footx_ref_flag(i-1) + _steplength_flag(i-1);
      if(_gait_mode ==102) ///troting gait
      {
        _footy_ref_flag(i) = _footy_ref_flag(i-1) + _stepwidth_flag(i-1);
      }
      else
      {     
        _footy_ref_flag(i) = _footy_ref_flag(i-1) + (int)pow(-1,i-1)*_stepwidth_flag(i-1);  
      } 
      _footz_ref_flag(i) = _footz_ref_flag(i-1) + _stepheight_flag(i-1);
    }
    for (int i = period_index_flag+2; i < 3; i++) {
      _yaw_ref_flag(i+1) = step_yaw_keyboard; //// after the robot stops, then rotates the body
    }
    _footx_offline_flag = _footx_ref_flag;
    _footy_offline_flag = _footy_ref_flag;
    _footz_offline_flag = _footz_ref_flag;    

    
    // if(_period_i+2<_footstepsnumber)
    // {
      cout<<"current period number:"<<period_index_flag<<endl;
      cout<<"desired step length:"<<step_length_keyboard<<endl;
      cout<<"desired step width:"<<step_width_keyboard<<endl;
      cout<<"Rotation matrix with yaw motion:"<<r_rotation<<endl;
    // }


}

Eigen::Matrix<double, 43, 1> MPCClass::step_timing_opti_loop(int i,Eigen::Matrix<double,18,1> estimated_state, 
                                                             Eigen::Vector3d _Rfoot_location_feedback, Eigen::Vector3d _Lfoot_location_feedback,
                                                             double lamda, bool _stopwalking,int _t_walkdtime,int _t_walkdtime_old,
                                                             double step_length, double step_width, double step_yaw)
{
  Eigen::Matrix<double, 43, 1> com_traj;
  com_traj.setZero();

  // /////// check the current step using the new flag vector //////
  period_index_flag = Indexfind_flag((i+1)*_dt,xyz0); 
  _period_i_flag = period_flag(period_index_flag)+1;
  if(i>1) /// not the first time moment
  {
    if(_period_i_flag>_period_i_flag_old) /// enter the new period, update the vector value 
    {
      _tx_flag.block<3,1>(0,0) = _tx_flag.block<3,1>(1,0);
      _tx_flag(3,0) += _ts_flag(3,0);
      period_flag.block<3,1>(0,0) = period_flag.block<3,1>(1,0);
      period_flag(3,0) += 1;      
      ///// also the reference state,should be updated ////
      _footxyz_real_flag_fixed(0) = _footx_ref_flag(0,0);
      _footxyz_real_flag_fixed(1) = _footy_ref_flag(0,0);
      _footxyz_real_flag_fixed(2) = _footz_ref_flag(0,0);

      _footx_ref_flag.block<3,1>(0,0) = _footx_ref_flag.block<3,1>(1,0);
      _footx_ref_flag(3,0) += _steplength_flag(3,0);
    
      _footy_ref_flag.block<3,1>(0,0) = _footy_ref_flag.block<3,1>(1,0);
      _footy_ref_flag(3,0) += _stepwidth_flag(3,0);

      _footz_ref_flag.block<3,1>(0,0) = _footz_ref_flag.block<3,1>(1,0);
      _footz_ref_flag(3,0) = _footz_ref_flag(3,0);


      _Lxx_ref_flag.block<3,1>(0,0) = _Lxx_ref_flag.block<3,1>(1,0);
      _Lxx_ref_flag(3,0) = _steplength_flag(3,0);
    
      _Lyy_ref_flag.block<3,1>(0,0) = _Lyy_ref_flag.block<3,1>(1,0);
      _Lyy_ref_flag(3,0) = _stepwidth_flag(3,0);      


      _steptuneflag_flag.block<3,1>(0,0) = _steptuneflag_flag.block<3,1>(1,0);
      _steptuneflag_flag(3,0) = 0;     
    
      _yaw_ref_flag.block<3,1>(0,0) = _yaw_ref_flag.block<3,1>(1,0);
      // _steptuneflag_flag(3,0) = 0; 

      _lift_height_ref_flag.block<3,1>(0,0) = _lift_height_ref_flag.block<3,1>(1,0);

      
      
      


      period_index_flag = Indexfind_flag((i+1)*_dt,xyz0); //// should be updated//////
      _period_i_flag = period_flag(period_index_flag)+1;    
    }
  }

  if(period_index_flag>0.5) //// in this formulation, the period_index_flag should always be zero
  {
     cout<<"period_index_flag"<<period_index_flag<<endl;
  }
  _period_i = _period_i_flag;
  _period_i_flag_old = _period_i_flag;

  //ZMP & ZMPv
  _px(0,0) = _footx_ref_flag(period_index_flag,0); 
  _zmpvx(0,0) = 0; 
  _py(0,0) = _footy_ref_flag(period_index_flag,0); 
  _zmpvy(0,0) = 0; 

  //// update the the following (_period -1,0) as (period_index_flag,0) 
  
  ///remaining step timing for the next stepping
  _ki = round(_tx_flag(period_index_flag,0)/_dt);
  _k_yu = i-_ki;                
  _Tk = _ts_flag(period_index_flag,0) - _k_yu*_dt; 
  
// velocity tracking 
  _Lxx_refx = _Lxx_ref_flag(period_index_flag,0);        
  _Lyy_refy = _Lyy_ref_flag(period_index_flag,0); //%% tracking relative location
  _Lxx_refx1 = _Lxx_ref_flag(period_index_flag+1,0);        
  _Lyy_refy1 = _Lyy_ref_flag(period_index_flag+1,0); // tracking relative location


  _tr1_ref = cosh(_Wn*_Tk);        _tr2_ref = sinh(_Wn*_Tk); 
  _tr1_ref1 = cosh(_Wn*_ts_flag(period_index_flag+1,0));  _tr2_ref1 = sinh(_Wn*_ts_flag(period_index_flag+1,0));
  
  // warm start
  if (i==1)
  {
    _vari_ini <<  _Lxx_refx,
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
    _COMx_is(0) = _comx_feed -_footx_ref_flag(period_index_flag,0);  
    _COMx_es.col(0) = _SS1*_vari_ini*0.5;  
    _COMvx_is(0)= (_COMx_es(0)-_COMx_is(0)*_SS3*_vari_ini)/(1/_Wn *_SS4*_vari_ini);
    _COMy_is(0) = _comy_feed-_footy_ref_flag(period_index_flag,0);  
    _COMy_es.col(0) = _SS2*_vari_ini*0.5;  
    _COMvy_is(0)= (_COMy_es(0)-_COMy_is(0)*_SS3*_vari_ini)/(1/_Wn *_SS4*_vari_ini);  
    _comvx_endref= _Wn*_COMx_is(0)*_SS4*_vari_ini + _COMvx_is(0)*_SS3*_vari_ini;
    _comvy_endref= _Wn*_COMy_is(0)*_SS4*_vari_ini + _COMvy_is(0)*_SS3*_vari_ini;     
  }

 
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
  if((!solve_true)||(_k_yu*_dt>0.8*_ts_flag((period_index_flag,0))))
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
  _Lxx_ref_flag(period_index_flag,0) = _SS1*_vari_ini;
  _Lyy_ref_flag(period_index_flag,0) = _SS2*_vari_ini;
  _ts_flag(period_index_flag,0) = _k_yu*_dt+ log((_SS3+_SS4)*_vari_ini)/_Wn;   //check log function _t_min
  
  _Lxx_ref_flag(period_index_flag+1,0) = _SS5*_vari_ini;
  _Lyy_ref_flag(period_index_flag+1,0) = _SS6*_vari_ini;
  _ts_flag(period_index_flag+1,0) = log((_SS7+_SS8)*_vari_ini)/_Wn; 

  //////// update step duration
  _Lxx_ref_flag(period_index_flag,0) = std:: min(std::max(_Lxx_ref_flag(period_index_flag,0),_footx_min), _footx_max);
  _Lxx_ref_flag(period_index_flag+1,0) = std:: min(std::max(_Lxx_ref_flag(period_index_flag+1,0),_footx_min), _footx_max);
  _Lyy_ref_flag(period_index_flag,0) = std:: min(std::max(_Lyy_ref_flag(period_index_flag,0),_footy_min), _footy_max);
  _Lyy_ref_flag(period_index_flag+1,0) = std:: min(std::max(_Lyy_ref_flag(period_index_flag+1,0),_footy_min), _footy_max);
  _ts_flag(period_index_flag,0) = std:: min(std::max(_ts_flag(period_index_flag,0),_t_min),_t_max);
  _ts_flag(period_index_flag+1,0) = std:: min(std::max(_ts_flag(period_index_flag+1,0),_t_min),_t_max);  

  _COMx_is(0) = _comx_feed-_footx_ref_flag(period_index_flag,0);  
  _COMx_es.col(0) = _SS1*_vari_ini*0.5;  
  _COMvx_is(0)= (_COMx_es(0)-_COMx_is(0)*_SS3*_vari_ini)/(1/_Wn *_SS4*_vari_ini);
  _COMy_is(0) = _comy_feed-_footy_ref_flag(period_index_flag,0);  
  _COMy_es.col(0) = _SS2*_vari_ini*0.5;  
  _COMvy_is(0)= (_COMy_es(0)-_COMy_is(0)*_SS3*_vari_ini)/(1/_Wn *_SS4*_vari_ini);    

// update walking period and step location  
  _t_end_footstep = 1000000000;

  for (int jxx = 2; jxx<=4; jxx++)
  {
    _tx_flag(jxx-1) = _tx_flag(jxx-2)+_ts_flag(jxx-2);
  }

  //// update robot_state_flag
  _footx_ref_flag(period_index_flag+1)=_footx_ref_flag(period_index_flag)+_SS1*_vari_ini;
  _footy_ref_flag(period_index_flag+1)=_footy_ref_flag(period_index_flag)+_SS2*_vari_ini;

  _comvx_endref= _Wn*_COMx_is(0)*_SS4*_vari_ini + _COMvx_is(0)*_SS3*_vari_ini;
  _comvy_endref= _Wn*_COMy_is(0)*_SS4*_vari_ini + _COMvy_is(0)*_SS3*_vari_ini;     
      
  /////  generate the trajectory during the double support phase'
  _nTdx = round(_td/_dt)+1;
  
  //////---------------------CoM height variation----------------------//////////
  CoM_height_solve(i, _stopwalking,_nTdx);
  
  for (int jxx=1; jxx <=_nTdx; jxx++)
  {
      double _wndtx = _Wn*_dt*jxx;

      _comx(jxx-1) = com_mpc_ref(0,jxx-1) = _COMx_is(0)*cosh(_wndtx) + _COMvx_is(0)*1/_Wn*sinh(_wndtx)+_px(0);
      _comy(jxx-1) = com_mpc_ref(1,jxx-1) = _COMy_is(0)*cosh(_wndtx) + _COMvy_is(0)*1/_Wn*sinh(_wndtx)+_py(0);           
      _comvx(jxx-1)= comv_mpc_ref(0,jxx-1)= _Wn*_COMx_is(0)*sinh(_wndtx) + _COMvx_is(0)*cosh(_wndtx);
      _comvy(jxx-1)= comv_mpc_ref(1,jxx-1)= _Wn*_COMy_is(0)*sinh(_wndtx) + _COMvy_is(0)*cosh(_wndtx);     
      _comax(jxx-1)= pow(_Wn,2)*_COMx_is(0)*cosh(_wndtx) + _COMvx_is(0)*_Wn*sinh(_wndtx);
      _comay(jxx-1)= pow(_Wn,2)*_COMy_is(0)*cosh(_wndtx) + _COMvy_is(0)*_Wn*sinh(_wndtx);   
          
      _zmpx_real(0,jxx-1) = _comx(0,jxx-1) - (_comz(0,jxx-1) - _Zsc(jxx-1))/(_comaz(0,jxx-1)+_ggg(0))*_comax(0,jxx-1);
      _zmpy_real(0,jxx-1) = _comy(0,jxx-1) - (_comz(0,jxx-1) - _Zsc(jxx-1))/(_comaz(0,jxx-1)+_ggg(0))*_comay(0,jxx-1);
      _dcmx_real(0,jxx-1) = _comx(0,jxx-1) + _comvx(0,jxx-1) * sqrt((_comz(0,jxx-1) - _Zsc(jxx-1))/(_comaz(0,jxx-1)+_ggg(0)));
      _dcmy_real(0,jxx-1) = _comy(0,jxx-1) + _comvy(0,jxx-1) * sqrt((_comz(0,jxx-1) - _Zsc(jxx-1))/(_comaz(0,jxx-1)+_ggg(0)));      
  }


//*************************************** feedback ************************************///
//  external disturbances!!! using feedback data:
  /// /// relative state to the actual foot lcoation: very good
  if (_period_i % 2 == 0)  // odd : left support
  {
    estimated_state(0,0) =  (estimated_state(0,0)-Homing_p_retarget(0,0)) - _Lfoot_location_feedback(0); //relative comx
    estimated_state(3,0) =  (estimated_state(3,0)-Homing_p_retarget(1,0)) - _Lfoot_location_feedback(1);	// relative comy 	    
  }
  else
  {
    estimated_state(0,0) =  (estimated_state(0,0)-Homing_p_retarget(0,0)) - _Rfoot_location_feedback(0);
    estimated_state(3,0) =  (estimated_state(3,0)-Homing_p_retarget(1,0)) - _Rfoot_location_feedback(1);	  
  }  



  _comx_feed = (_lamda_comx*(_comx(0)-_px(0))+(1-_lamda_comx)*estimated_state(0,0))+_px(0);    
  _comvx_feed = _lamda_comvx*_comvx(0) + (1-_lamda_comvx)*estimated_state(1,0);
  //_comax_feed(i) = _lamda_comx*_comax(0) + (1-_lamda_comx)*estimated_state(2,0);
  _comy_feed = (_lamda_comy*(_comy(0)-_py(0))+(1-_lamda_comy)*estimated_state(3,0))+_py(0);    
  _comvy_feed = _lamda_comvy*_comvy(0) + (1-_lamda_comvy)*estimated_state(4,0);  
  //_comay_feed(i) = _lamda_comy*_comay(0) + (1-_lamda_comy)*estimated_state(5,0);  

  
  _t_f.setLinSpaced(_nh,(i+1)*_dt, (i+_nh)*_dt);
  
  bjx1_index_flag = Indexfind_flag(_t_f(0),xyz1); 
  _bjx1_flag = period_flag(bjx1_index_flag)+1;
  _bjx1 = _bjx1_flag;

  
  _td_flag = 0.2 * _ts_flag;
     

  _footxyz_real_flag.row(0) = _footx_ref_flag.transpose();
  _footxyz_real_flag.row(1) = _footy_ref_flag.transpose();  
  _footxyz_real_flag.row(2) = _footz_ref_flag.transpose(); 


  if(_period_i_flag != _period_i)
  {
    cout<<"period index doesnot match:"<<i<<endl;
    cout<<"_tx_flag:"<<_tx_flag.transpose()<<endl;
    cout<<"_period_i_flag:"<<_period_i_flag << endl;

  }  

  
  /// Re_initialize the step parameters when reaching the maximal counter, the clock wont go beyond the limits
  if(_t_walkdtime>_t_walkdtime_old)
  {
    // cout<<"==================time index for reinitialization:"<<i<<endl;
    // Re_Initialize(step_length,step_width);
  } 
  else ////modulate the next step parameter by keyboard controlling;
  {
    if((abs(_length_key_pre - step_length)>0.001)||(abs(_width_key_pre - step_width)>0.001)||(abs(_yaw_key_pre - step_yaw)>0.001))
    {
      // if((_period_i<_footstepsnumber-4) &&(_k_yu*_dt<=0.8*_ts_flag(period_index_flag,0))&&(_steptuneflag_flag(period_index_flag+1,0)==0))
      if((_k_yu*_dt<=0.8*_ts_flag(period_index_flag,0))&&(_steptuneflag_flag(period_index_flag+1,0)==0))
      {
        // std::cout<<"enter the change loop"<<endl;
        command_foot_step(step_length,step_width, step_yaw);
        _steptuneflag_flag(period_index_flag+1,0) = 1;
        // if(_period_i+2<_footstepsnumber)
        // {
          if(abs(_yaw_key_pre - step_yaw)>0.001)
          {
            _steptuneflag_flag(period_index_flag+2,0) = 1;
            _steptuneflag_flag(period_index_flag+3,0) = 1;
            // _stepyawflag_flag(period_index_flag+2,0) = 1;
            // _stepyawflag_flag(period_index_flag+3,0) = 1;
          }
        // }
      }
    }
  } 

  _length_key_pre = step_length;
  _width_key_pre = step_width;
  _yaw_key_pre = step_yaw;


  com_traj(0) = _comx(0,0);
  com_traj(1) = _comy(0,0);
  com_traj(2) = _comz(0,0);
  com_traj(3) = _comvx(0,0);
  com_traj(4) = _comvy(0,0);
  com_traj(5) = _comvz(0,0);
  com_traj(6) = _comax(0,0);
  com_traj(7) = _comay(0,0);
  com_traj(8) = _comaz(0,0); 

  com_traj(9) = _zmpx_real(0,0);
  com_traj(10) = _zmpy_real(0,0);
  com_traj(11) = _dcmx_real(0,0);
  com_traj(12) = _dcmy_real(0,0); 
  com_traj(13) = _zmpx_real(0,1);
  com_traj(14) = _zmpy_real(0,1);
  com_traj(15) = _dcmx_real(0,1);
  com_traj(16) = _dcmy_real(0,1); 

  com_traj(17) = _zmpx_real(0,2);
  com_traj(18) = _zmpy_real(0,2);
  com_traj(19) = _dcmx_real(0,2);
  com_traj(20) = _dcmy_real(0,2);

  com_traj(21) = _comax(0,1);
  com_traj(22) = _comay(0,1);
  com_traj(23) = _comaz(0,1); 
  com_traj(24) = _comax(0,2);
  com_traj(25) = _comay(0,2);
  com_traj(26) = _comaz(0,2);   
  
  //////foot paramters
  com_traj(27) = _bjx1-1;  
  com_traj(28) = _footx_ref_flag(bjx1_index_flag); 
  com_traj(29) = _footx_ref_flag((bjx1_index_flag+1));  
  com_traj(30) = _footy_ref_flag(bjx1_index_flag); 
  com_traj(31) = _footy_ref_flag((bjx1_index_flag+1)); 
  com_traj(32) = _footz_ref_flag(bjx1_index_flag); 
  com_traj(33) = _footz_ref_flag((bjx1_index_flag+1));    
  com_traj(34) = _period_i-1; 
  com_traj(35) = _ts_flag(period_index_flag,0);  
  
  // if(_bjx1+1<_footstepsnumber-1)
  // {
  com_traj(36) = _footx_ref_flag(bjx1_index_flag+2);
  com_traj(37) = _footy_ref_flag(bjx1_index_flag+2);
  com_traj(41) = _yaw_ref_flag(bjx1_index_flag+2);
  // }
  // else
  // {
  //   com_traj(36) = _footx_ref_flag((bjx1_index_flag+1));
  //   com_traj(37) = _footy_ref_flag((bjx1_index_flag+1));
  //   com_traj(41) = _yaw_ref_flag((bjx1_index_flag+1));
  // }
  //// yaw motion
  com_traj(38) = _yaw_ref_flag(bjx1_index_flag);
  com_traj(39) = _yaw_ref_flag(bjx1_index_flag); 
  com_traj(40) = _yaw_ref_flag(bjx1_index_flag+1); 


  com_traj(42) = _tx_flag(period_index_flag,0);


 /////// reference com and com velocity in the prediction window ///

  for (int j = 2; j <= 10; j++)
  {
    period_index_flag = Indexfind_flag((i+j)*_dt,xyz0); //// should be //////
    _period_i_flag = period_flag(period_index_flag)+1;
    _period_i = _period_i_flag;      ///coincident with Matlab      

    comv_mpc_ref(0,j-1) = _Lxx_ref_flag(period_index_flag,0)/_ts_flag(period_index_flag,0);
    comv_mpc_ref(1,j-1) = _Lyy_ref_flag(period_index_flag,0)/_ts_flag(period_index_flag,0);
    comv_mpc_ref(2,j-1) =  (_footz_ref_flag(period_index_flag+1,0)- _footz_ref_flag(period_index_flag,0))/_ts_flag(period_index_flag,0);
    com_mpc_ref.col(j-1) = com_mpc_ref.col(j-2)+comv_mpc_ref.col(j-1)*_dt;
  }


  period_index_flag = Indexfind_flag((i+1)*_dt,xyz0); //// should be //////
  _period_i_flag = period_flag(period_index_flag)+1;
  _period_i = _period_i_flag;      ///coincident with Matlab 




  return com_traj;

}

int MPCClass::Indexfind_flag(double goalvari, int xyz)
{
  int _j_period_flag;
  _j_period_flag = 0;
  
  if (xyz<-0.5)
  {
      while (goalvari > (_tx_flag(_j_period_flag))+0.0001)
      {
      	_j_period_flag++;
      }    
        _j_period_flag = _j_period_flag-1;	    
  }
  else
  {
  
    if (xyz<0.05)
    {
      while (goalvari >= _tx_flag(_j_period_flag))
      {
	      _j_period_flag++;
      }    
        _j_period_flag = _j_period_flag-1;	  
    }
    else
    {
      while ( fabs(goalvari - _t_f(_j_period_flag)) >0.0001 )
      {
        _j_period_flag++;
      }	    
    }	  

      
  }
  if((_j_period_flag >3)||(_j_period_flag <0))
  {
    cout<<"goalvari:"<<goalvari<<endl;
    cout<<"_tx_flag:"<<_tx_flag.transpose()<<endl;
    cout<<"xyz:"<<xyz<<endl;
    if(_j_period_flag >3)
    {
      _j_period_flag =3;
    }
    else
    {
      _j_period_flag = 0;
    }

  }


  return _j_period_flag;

}



void MPCClass::step_timing_object_function(int i)
{
    _AxO(0) = _comx_feed-_footx_ref_flag(period_index_flag,0); _BxO(0) = _comvx_feed/_Wn; _Cx(0,0) =-0.5*_Lxx_refx; 
    _Axv = _Wn*_AxO;  _Bxv = _Wn*_BxO; _Cxv=_comvx_endref; 
    _AyO(0) = _comy_feed-_footy_ref_flag(period_index_flag,0); _ByO(0) = _comvy_feed/_Wn; _Cy(0,0) =-0.5*_Lyy_refy; 
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
        
        if (i>=(round(2*_tstep/_dt))+1) ///update the footy_limit
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
        if (i>=(round(2*_tstep/_dt))+1)
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
      _det_h_lvx_up(0,0) = -(_SS1*_vari_ini-_Lxx_ref_flag(period_index_flag,0)- _footx_vmax*_dt);

      _h_lvx_lp = -_SS1;
      _det_h_lvx_lp(0,0) = _SS1*_vari_ini-_Lxx_ref_flag(period_index_flag,0)-_footx_vmin*_dt; 

      _h_lvy_up = _SS2;
      _det_h_lvy_up(0,0) = -(_SS2*_vari_ini-_Lyy_ref_flag(period_index_flag,0)- _footy_vmax*_dt);

      _h_lvy_lp = -_SS2;
      _det_h_lvy_lp(0,0) = _SS2*_vari_ini-_Lyy_ref_flag(period_index_flag,0)-_footy_vmin*_dt;  

      
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


////////////////////////// CoM position relative to the current support center
// CoM accelearation boundary    
    
    _AA= _Wn*sinh(_Wn*_dt); _CCx = _comx_feed-_footx_ref_flag(period_index_flag,0); _BBx = pow(_Wn,2)*_CCx*cosh(_Wn*_dt); 
		            _CCy = _comy_feed-_footy_ref_flag(period_index_flag,0); _BBy = pow(_Wn,2)*_CCy*cosh(_Wn*_dt);

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
    _VAA= cosh(_Wn*_dt); _VCCx = _comx_feed-_footx_ref_flag(period_index_flag,0); _VBBx = _Wn*_VCCx*sinh(_Wn*_dt); 
		                     _VCCy = _comy_feed-_footy_ref_flag(period_index_flag,0); _VBBy = _Wn*_VCCy*sinh(_Wn*_dt);

    _VAA1x = _VAA*_Wn; _VAA2x = -2* _VAA*_VCCx*_Wn; _VAA3x = 2*_VBBx - 2*_comvx_feed; 
    _VAA1y = _VAA*_Wn; _VAA2y = -2* _VAA*_VCCy*_Wn; _VAA3y = 2*_VBBy - 2*_comvy_feed;

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
    _VAA1x1 = _Wn; _VAA2x1 = -2*_VCCx*_Wn; _VAA3x1 = - 2*_comvx_feed; 
    _VAA1y1 = _Wn; _VAA2y1 = -2*_VCCy*_Wn; _VAA3y1 = - 2*_comvy_feed;

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


//// each time: foot trajectory

Eigen::Matrix<double, 18, 1> MPCClass::Foot_trajectory_solve(int j_index, bool _stopwalking)
{
	Eigen::Matrix<double, 18, 1> rffoot_traj;
  rffoot_traj.setZero();

  //// judge if stop  
  if(_stopwalking)  
  {
    for (int i_t = bjx1_index_flag+2; i_t < 4; i_t++) {	  
      _lift_height_ref_flag(i_t) = 0;   ///////// ==========?????????????????? ///// change this ///////////
    }	     	  

  }    
  
  for (int j_f = 0; j_f < 10; j_f++)
  {
    bjx1_index_flag = Indexfind_flag(_t_f(j_f),xyz1); 
    _bjx1_flag = period_flag(bjx1_index_flag)+1;
    _bjx1 = _bjx1_flag;



     //   foot trajectory generation:
    if (_bjx1 >= 2)
    {
      if (_bjx1 % 2 == 0)           //odd:left support
      {
        right_support = 0; 
        //       cout << "left support"<<endl;
        _Lfootx = _footxyz_real_flag(0,bjx1_index_flag); 
        _Lfooty = _footxyz_real_flag(1,bjx1_index_flag);
        _Lfootz = _footxyz_real_flag(2,bjx1_index_flag); 	
        _Lfootvx = 0;
        _Lfootax = 0;
        _Lfootvy = 0;
        _Lfootay = 0;
        _Lfootvz = 0;
        _Lfootaz = 0;       
        
        double t_des = (j_index +1+j_f - round(_tx_flag(bjx1_index_flag,0)/_dt))*_dt;    

        if (t_des < _td_flag(bjx1_index_flag,0))  // j_index and _bjx1 coincident with matlab: double support
        {
          right_support = 2;
          // 	cout << "dsp"<<endl;
            _Rfootx = _Rfootxyz_pre(0,0) + _Rfootvxyz_pre(0,0)*_dt; ////
            _Rfooty = _Rfootxyz_pre(1,0) + _Rfootvxyz_pre(1,0)*_dt;
            _Rfootz = _Rfootxyz_pre(2,0) + _Rfootvxyz_pre(2,0)*_dt; 	            
            _Rfootvx = 0;
            _Rfootax = 0;
            _Rfootvy = 0;
            _Rfootay = 0;
            _Rfootvz = 0;
            _Rfootaz = 0;          
        
        }
        else
        {
    
          // 	cout << "ssp"<<endl;
          //initial state and final state and the middle state
          Eigen::Vector3d t_plan;
          t_plan(0) = t_des - _dt;
          t_plan(1) = (_td_flag(bjx1_index_flag,0) + _ts_flag(bjx1_index_flag,0))/2 + 0.001;
          t_plan(2) = _ts_flag(bjx1_index_flag,0) + 0.001;
          
          if (abs(t_des - _ts_flag(bjx1_index_flag,0)) <= (_dt + 0.0005))
          {
            _Rfootx = _footxyz_real_flag(0,bjx1_index_flag+1); 
            _Rfooty = _footxyz_real_flag(1,bjx1_index_flag+1);
            _Rfootz = _footxyz_real_flag(2,bjx1_index_flag+1); 	
            _Rfootvx = 0;
            _Rfootax = 0;
            _Rfootvy = 0;
            _Rfootay = 0;
            _Rfootvz = 0;
            _Rfootaz = 0;          
            right_support = 2;	 
            //cout<<"enter the double again"<<endl; 
          
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
          
            
            ////////////////////////////////////////////////////////////////////////////
            Eigen::Matrix<double, 7, 1> Rfootx_plan;
            Rfootx_plan.setZero();	
            Rfootx_plan(0) = _Rfootvxyz_pre(0,0);     Rfootx_plan(1) = _Rfootaxyz_pre(0,0); Rfootx_plan(2) = _Rfootxyz_pre(0,0); //Rfootx_plan(3) = _Lfootx;
            // Rfootx_plan(3) = (_footxyz_real(0,_bjx1-2)+_footxyz_real_flag(0,bjx1_index_flag+1))/2;
            Rfootx_plan(3) = (_footxyz_real_flag(0,bjx1_index_flag));
            Rfootx_plan(4) = _footxyz_real_flag(0,bjx1_index_flag+1);  Rfootx_plan(5) = 0;                   Rfootx_plan(6) = 0;         
            
            Eigen::Matrix<double, 7, 1> Rfootx_co;
            Rfootx_co.setZero();
            Rfootx_co = AAA_inv * Rfootx_plan;
            
            _Rfootx = t_a_plan * Rfootx_co;
            _Rfootvx = t_a_planv * Rfootx_co;
            _Rfootax = t_a_plana * Rfootx_co;
            
            /////////////////////////////////////////////////////////////////////////////
            if(_gait_mode==102)
            {
              _ry_left_right = _Lfooty;
            }

            
            Eigen::Matrix<double, 7, 1> Rfooty_plan;
            Rfooty_plan.setZero();	
            Rfooty_plan(0) = _Rfootvxyz_pre(1,0);     Rfooty_plan(1) = _Rfootaxyz_pre(1,0); Rfooty_plan(2) = _Rfootxyz_pre(1,0); //Rfooty_plan(3) = _ry_left_right;
            // Rfooty_plan(3) = (_footxyz_real(1,_bjx1-2)+_footxyz_real_flag(1,bjx1_index_flag+1))/2;
            Rfooty_plan(3) = (_footxyz_real_flag(1,bjx1_index_flag));
            Rfooty_plan(4) = _footxyz_real_flag(1,bjx1_index_flag+1);  Rfooty_plan(5) = 0;                   Rfooty_plan(6) = 0;	    
            
            Eigen::Matrix<double, 7, 1> Rfooty_co;
            Rfooty_co.setZero();
            Rfooty_co = AAA_inv * Rfooty_plan;
            
            _Rfooty = t_a_plan * Rfooty_co;
            _Rfootvy = t_a_planv * Rfooty_co;
            _Rfootay = t_a_plana * Rfooty_co;	
            
            
            //////////////////////////////////////////////////////////
            Eigen::Matrix<double, 7, 1> Rfootz_plan;
            Rfootz_plan.setZero();	
            Rfootz_plan(0) = _Rfootvxyz_pre(2,0);     Rfootz_plan(1) = _Rfootaxyz_pre(2,0); Rfootz_plan(2) = _Rfootxyz_pre(2,0); //Rfootz_plan(3) = _Lfootz+_lift_height_ref(_bjx1-1);
            // Rfootz_plan(3) = max(_footxyz_real(2,_bjx1-2),_footxyz_real_flag(2,bjx1_index_flag+1)) + _lift_height_ref(_bjx1-1);
            Rfootz_plan(3) = max(_footxyz_real_flag(2,bjx1_index_flag),_footxyz_real_flag(2,bjx1_index_flag+1)) + _lift_height_ref_flag(bjx1_index_flag,0);
            Rfootz_plan(4) = _footxyz_real_flag(2,bjx1_index_flag+1);  Rfootz_plan(5) = 0;                   Rfootz_plan(6) = 0;	
            
            Eigen::Matrix<double, 7, 1> Rfootz_co;
            Rfootz_co.setZero();
            Rfootz_co = AAA_inv * Rfootz_plan;
            
            _Rfootz = t_a_plan * Rfootz_co;
            _Rfootvz = t_a_planv * Rfootz_co;
            _Rfootaz = t_a_plana * Rfootz_co;	
          
          }
        }   
      }
      else                       //right support
      {
        right_support = 1; 

        _Rfootx = _footxyz_real_flag(0,bjx1_index_flag); 
        _Rfooty = _footxyz_real_flag(1,bjx1_index_flag);
        _Rfootz = _footxyz_real_flag(2,bjx1_index_flag); 
        _Rfootvx = 0;
        _Rfootax = 0;
        _Rfootvy = 0;
        _Rfootay = 0;
        _Rfootvz = 0;
        _Rfootaz = 0;          

        double t_des = (j_index +1+j_f - round(_tx_flag(bjx1_index_flag,0)/_dt))*_dt;   
        
        if (t_des < _td_flag(bjx1_index_flag,0))  // j_index and _bjx1 coincident with matlab: double suppot
        {
          right_support = 2; 
            _Lfootx = _Lfootxyz_pre(0,0) + _Lfootvxyz_pre(0,0)*_dt; ////
            _Lfooty = _Lfootxyz_pre(1,0) + _Lfootvxyz_pre(1,0)*_dt;
            _Lfootz = _Lfootxyz_pre(2,0) + _Lfootvxyz_pre(2,0)*_dt; 	      
            _Lfootvx = 0;
            _Lfootax = 0;
            _Lfootvy = 0;
            _Lfootay = 0;
            _Lfootvz = 0;
            _Lfootaz = 0;           	
        }
        else
        {
          // 	cout << "ssp"<<endl;
          //initial state and final state and the middle state
          Eigen::Vector3d t_plan;
          t_plan(0) = t_des - _dt;
          t_plan(1) = (_td_flag(bjx1_index_flag,0) + _ts_flag(bjx1_index_flag,0))/2 + 0.001;
          t_plan(2) = _ts_flag(bjx1_index_flag,0) + 0.001;
      
          if (abs(t_des - _ts_flag(bjx1_index_flag,0)) <= (_dt + 0.0005))
          {
          
            _Lfootx = _footxyz_real_flag(0,bjx1_index_flag+1); 
            _Lfooty = _footxyz_real_flag(1,bjx1_index_flag+1);
            _Lfootz = _footxyz_real_flag(2,bjx1_index_flag+1); 
            _Lfootvx = 0;
            _Lfootax = 0;
            _Lfootvy = 0;
            _Lfootay = 0;
            _Lfootvz = 0;
            _Lfootaz = 0;           

            right_support = 2;
          
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
            Lfootx_plan(0) = _Lfootvxyz_pre(0,0);     Lfootx_plan(1) = _Lfootaxyz_pre(0,0); Lfootx_plan(2) = _Lfootxyz_pre(0,0); //Lfootx_plan(3) = _Rfootx;
            // Lfootx_plan(3) = (_footxyz_real(0,_bjx1-2)+_footxyz_real_flag(0,bjx1_index_flag+1))/2;
            Lfootx_plan(3) = (_footxyz_real_flag(0,bjx1_index_flag));
            Lfootx_plan(4) = _footxyz_real_flag(0,bjx1_index_flag+1);  Lfootx_plan(5) = 0;                   Lfootx_plan(6) = 0;	  
            
            
            Eigen::Matrix<double, 7, 1> Lfootx_co;
            Lfootx_co.setZero();
            Lfootx_co = AAA_inv * Lfootx_plan;
            
            _Lfootx = t_a_plan * Lfootx_co;
            _Lfootvx = t_a_planv * Lfootx_co;
            _Lfootax = t_a_plana * Lfootx_co;
            
            /////////////////////////////////////////////////////////////////////////////
            if(_gait_mode==102)
            {
              _ry_left_right = _Rfooty;
            }


            Eigen::Matrix<double, 7, 1> Lfooty_plan;
            Lfooty_plan.setZero();	
            Lfooty_plan(0) = _Lfootvxyz_pre(1,0);     Lfooty_plan(1) = _Lfootaxyz_pre(1,0); Lfooty_plan(2) = _Lfootxyz_pre(1,0); //Lfooty_plan(3) = _ry_left_right;
            // Lfooty_plan(3) = (_footxyz_real(1,_bjx1-2)+_footxyz_real_flag(1,bjx1_index_flag+1))/2;
            Lfooty_plan(3) = (_footxyz_real_flag(1,bjx1_index_flag));
            Lfooty_plan(4) = _footxyz_real_flag(1,bjx1_index_flag+1);  Lfooty_plan(5) = 0;                   Lfooty_plan(6) = 0;		  
            
            
            Eigen::Matrix<double, 7, 1> Lfooty_co;
            Lfooty_co.setZero();
            Lfooty_co = AAA_inv * Lfooty_plan;
            
            _Lfooty = t_a_plan * Lfooty_co;
            _Lfootvy = t_a_planv * Lfooty_co;
            _Lfootay = t_a_plana * Lfooty_co;	
            
            
            //////////////////////////////////////////////////////////
            Eigen::Matrix<double, 7, 1> Lfootz_plan;
            Lfootz_plan.setZero();			
            Lfootz_plan(0) = _Lfootvxyz_pre(2,0);     Lfootz_plan(1) = _Lfootaxyz_pre(2,0); Lfootz_plan(2) = _Lfootxyz_pre(2,0); //Lfootz_plan(3) = _Rfootz+_lift_height_ref(_bjx1-1);
            // Lfootz_plan(3) = max(_footxyz_real(2,_bjx1-2),_footxyz_real_flag(2,bjx1_index_flag+1)) + _lift_height_ref(_bjx1-1);
            Lfootz_plan(3) = max(_footxyz_real_flag(2,bjx1_index_flag),_footxyz_real_flag(2,bjx1_index_flag+1)) + _lift_height_ref_flag(bjx1_index_flag,0);
            Lfootz_plan(4) = _footxyz_real_flag(2,bjx1_index_flag+1);  Lfootz_plan(5) = 0;                   Lfootz_plan(6) = 0;		  
            
            
            Eigen::Matrix<double, 7, 1> Lfootz_co;
            Lfootz_co.setZero();
            Lfootz_co = AAA_inv * Lfootz_plan;
            
            _Lfootz = t_a_plan * Lfootz_co;
            _Lfootvz = t_a_planv * Lfootz_co;
            _Lfootaz = t_a_plana * Lfootz_co;
            
            
          }
        }

      }
        
    }
    else
    {
      ///// initialization////// 
      right_support = 2; 
      _Rfooty = 0;
      _Lfooty = 0;
      _Rfootx = 0;
      _Lfootx = 0;    
      _Rfootz = 0;
      _Lfootz = 0;       
      _Lfootvx = 0;
      _Lfootax = 0;
      _Lfootvy = 0;
      _Lfootay = 0;
      _Lfootvz = 0;
      _Lfootaz = 0; 
      _Rfootvx = 0;
      _Rfootax = 0;
      _Rfootvy = 0;
      _Rfootay = 0;
      _Rfootvz = 0;
      _Rfootaz = 0;     
      //initial state and final state and the middle state
      double t_des = (j_index +1+j_f - round(_tx_flag(bjx1_index_flag,0)/_dt))*_dt;
      Eigen::Vector3d t_plan;
      t_plan(0) = t_des - _dt;
      t_plan(1) = (_td_flag(bjx1_index_flag,0) + _ts_flag(bjx1_index_flag,0))/2 + 0.001;
      t_plan(2) = _ts_flag(bjx1_index_flag,0) + 0.001;

      if (abs(t_des - _ts_flag(bjx1_index_flag,0)) <= (_dt + 0.0005))
      {
      
        _Lfootx = _footxyz_real_flag(0,bjx1_index_flag+1); 
        _Lfooty = _footxyz_real_flag(1,bjx1_index_flag+1);
        _Lfootz = _footxyz_real_flag(2,bjx1_index_flag+1); 
        right_support = 2;
      
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
        Eigen::Matrix<double, 7, 1> Lfootz_plan;
        Lfootz_plan.setZero();			
        Lfootz_plan(0) = _Lfootvxyz_pre(2,0);     Lfootz_plan(1) = _Lfootaxyz_pre(2,0); Lfootz_plan(2) = _Lfootxyz_pre(2,0); Lfootz_plan(3) = _Rfootz+_lift_height_ref_flag(bjx1_index_flag,0);
        Lfootz_plan(4) = _footxyz_real_flag(2,bjx1_index_flag+1);  Lfootz_plan(5) = 0;                   Lfootz_plan(6) = 0;		  
        
        
        Eigen::Matrix<double, 7, 1> Lfootz_co;
        Lfootz_co.setZero();
        Lfootz_co = AAA_inv * Lfootz_plan;
        
        _Lfootz = t_a_plan * Lfootz_co;
        _Lfootvz = t_a_planv * Lfootz_co;
        _Lfootaz = t_a_plana * Lfootz_co;
      
      }



    }
      

    support_prediction(0,j_f) = right_support;
    rfoot_mpc_ref(0,j_f) = _Rfootx;
    rfoot_mpc_ref(1,j_f) = _Rfooty;
    rfoot_mpc_ref(2,j_f) = _Rfootz;
    lfoot_mpc_ref(0,j_f) = _Lfootx;
    lfoot_mpc_ref(1,j_f) = _Lfooty;
    lfoot_mpc_ref(2,j_f) = _Lfootz;


    if(j_f==0)
    {
      _Rfootxyz_pre(0,0) = rffoot_traj(0,0) = _Rfootx;
      _Rfootxyz_pre(1,0) = rffoot_traj(1,0) = _Rfooty;
      _Rfootxyz_pre(2,0) = rffoot_traj(2,0) = _Rfootz;

      _Lfootxyz_pre(0,0) = rffoot_traj(3,0) = _Lfootx;
      _Lfootxyz_pre(1,0) = rffoot_traj(4,0) = _Lfooty;
      _Lfootxyz_pre(2,0) = rffoot_traj(5,0) = _Lfootz;

      _Rfootvxyz_pre(0,0) = rffoot_traj(6,0) = _Rfootvx;
      _Rfootvxyz_pre(1,0) = rffoot_traj(7,0) = _Rfootvy;
      _Rfootvxyz_pre(2,0) = rffoot_traj(8,0) = _Rfootvz;

      _Lfootvxyz_pre(0,0) = rffoot_traj(9,0) = _Lfootvx;
      _Lfootvxyz_pre(1,0) = rffoot_traj(10,0) = _Lfootvy;
      _Lfootvxyz_pre(2,0) = rffoot_traj(11,0) = _Lfootvz;


      _Rfootaxyz_pre(0,0) = rffoot_traj(12,0) = _Rfootax;
      _Rfootaxyz_pre(1,0) = rffoot_traj(13,0) = _Rfootay;
      _Rfootaxyz_pre(2,0) = rffoot_traj(14,0) = _Rfootaz;

      _Lfootaxyz_pre(0,0) = rffoot_traj(15,0) = _Lfootax;
      _Lfootaxyz_pre(1,0) = rffoot_traj(16,0) = _Lfootay;
      _Lfootaxyz_pre(2,0) = rffoot_traj(17,0) = _Lfootaz; 
    }
    else
    {
      if(j_f ==9)
      {
        /// update the pre_value for the next loop
        _Rfootxyz_pre(0,0) = rffoot_traj(0,0);
        _Rfootxyz_pre(1,0) = rffoot_traj(1,0);
        _Rfootxyz_pre(2,0) = rffoot_traj(2,0);

        _Lfootxyz_pre(0,0) = rffoot_traj(3,0);
        _Lfootxyz_pre(1,0) = rffoot_traj(4,0);
        _Lfootxyz_pre(2,0) = rffoot_traj(5,0);

        _Rfootvxyz_pre(0,0) = rffoot_traj(6,0);
        _Rfootvxyz_pre(1,0) = rffoot_traj(7,0);
        _Rfootvxyz_pre(2,0) = rffoot_traj(8,0);

        _Lfootvxyz_pre(0,0) = rffoot_traj(9,0);
        _Lfootvxyz_pre(1,0) = rffoot_traj(10,0);
        _Lfootvxyz_pre(2,0) = rffoot_traj(11,0);


        _Rfootaxyz_pre(0,0) = rffoot_traj(12,0);
        _Rfootaxyz_pre(1,0) = rffoot_traj(13,0);
        _Rfootaxyz_pre(2,0) = rffoot_traj(14,0);

        _Lfootaxyz_pre(0,0) = rffoot_traj(15,0);
        _Lfootaxyz_pre(1,0) = rffoot_traj(16,0);
        _Lfootaxyz_pre(2,0) = rffoot_traj(17,0); 


      }
      else
      {
        _Rfootxyz_pre(0,0) = _Rfootx;
        _Rfootxyz_pre(1,0) = _Rfooty;
        _Rfootxyz_pre(2,0) = _Rfootz;

        _Lfootxyz_pre(0,0) = _Lfootx;
        _Lfootxyz_pre(1,0) = _Lfooty;
        _Lfootxyz_pre(2,0) = _Lfootz;

        _Rfootvxyz_pre(0,0) = _Rfootvx;
        _Rfootvxyz_pre(1,0) = _Rfootvy;
        _Rfootvxyz_pre(2,0) = _Rfootvz;

        _Lfootvxyz_pre(0,0) = _Lfootvx;
        _Lfootvxyz_pre(1,0) = _Lfootvy;
        _Lfootvxyz_pre(2,0) = _Lfootvz;


        _Rfootaxyz_pre(0,0) = _Rfootax;
        _Rfootaxyz_pre(1,0) = _Rfootay;
        _Rfootaxyz_pre(2,0) = _Rfootaz;

        _Lfootaxyz_pre(0,0) = _Lfootax;
        _Lfootaxyz_pre(1,0) = _Lfootay;
        _Lfootaxyz_pre(2,0) = _Lfootaz; 
      }
    }


  }
 
    bjx1_index_flag = Indexfind_flag(_t_f(0),xyz1); 
    _bjx1_flag = period_flag(bjx1_index_flag)+1;
    _bjx1 = _bjx1_flag;
 

  return rffoot_traj;


  
}


/// @brief height trajectory generation
/// @param j_indexx 
/// @param _stopwalking 
/// @param ntdx 
void MPCClass::CoM_height_solve(int j_indexx, bool _stopwalking, int ntdx)
{
  double comz_inter;
  double comvz_inter;
  double comaz_inter;     
  for (int j_index = j_indexx; j_index<j_indexx+ntdx; j_index++ )
  {
    if (j_index <= _t_end_footstep)
    {
      bjx1_index_flag = Indexfind_flag((j_index+1)*_dt,xyz1); 
      _bjx1_flag = period_flag(bjx1_index_flag)+1;
      _bjx1 = _bjx1_flag;         
    }

    //   yaw angle generation:
    if (j_index <= _t_end_footstep)
    {
      // //   rotation angle generation:
      if (_bjx1 >= 2)
      {      
          if ((j_index +1 - round(_tx_flag(bjx1_index_flag)/_dt))*_dt < _td_flag(bjx1_index_flag))  // j_index and _bjx1 coincident with matlab: double suppot
          {
            comz_inter = _footxyz_real_flag(2,bjx1_index_flag) + _hcom;
            comvz_inter = 0;
            comaz_inter = 0;
          }
          else
          {
            //initial state and final state and the middle state
            double t_des = (j_index +1 - round(_tx_flag(bjx1_index_flag)/_dt))*_dt;
            Eigen::Vector3d t_plan;
            t_plan(0) = _td_flag(bjx1_index_flag)-_dt;
            t_plan(1) = (_td_flag(bjx1_index_flag) + _ts_flag(bjx1_index_flag))/2 + 0.001;
            t_plan(2) = _ts_flag(bjx1_index_flag) + 0.001;
      
            if (abs(t_des - _ts_flag(bjx1_index_flag)) <= (_dt + 0.0005))
            {
              comz_inter = _footxyz_real_flag(2,bjx1_index_flag+1) + _hcom;
              comvz_inter = 0;
              comaz_inter = 0;
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
              Eigen::Matrix<double, 7, 1> comz_plan;
              comz_plan.setZero();			
              comz_plan(0) = 0;                 comz_plan(1) = 0;                 
              comz_plan(2) = _footxyz_real_flag(2,bjx1_index_flag)+_hcom; 
              comz_plan(3) = (_footxyz_real_flag(2,bjx1_index_flag)+_footxyz_real_flag(2,bjx1_index_flag+1))/2+_hcom;      
              comz_plan(4) = _footxyz_real_flag(2,bjx1_index_flag+1)+_hcom;  
              comz_plan(5) = 0;                 comz_plan(6) = 0;	  
              
              
              Eigen::Matrix<double, 7, 1> rotation_co;
              rotation_co.setZero();
              rotation_co = AAA_inv * comz_plan;
              
              comz_inter = t_a_plan * rotation_co;
              comvz_inter = t_a_planv * rotation_co;
              comaz_inter = t_a_plana * rotation_co;
              // cout<<"rotation_pac in the swing phase: \t"<<rotation_pac(0)<<endl;
              // cout<<"rotation_plan in the swing phase: \t"<<rotation_plan<<endl;
            }
          } 
      }
      else
      {
        comz_inter = _footxyz_real_flag(2,0)+_hcom;
        comvz_inter = 0;
        comaz_inter = 0;
      }
    }
    else
    {
        comz_inter = _footxyz_real_flag(2,3)+_hcom;
        comvz_inter = 0;
        comaz_inter = 0;
    } 

    _comz(j_index-j_indexx) = comz_inter;
    _comvz(j_index-j_indexx) = comvz_inter;
    _comaz(j_index-j_indexx) = comaz_inter;    
  }
}


/// @brief rotation angle generation
/// @param j_index 
/// @return 
Eigen::Vector3d MPCClass::rotation_angle_solve(int j_index)     
{  
  Eigen::Vector3d rotation_pac;
  rotation_pac.setZero();

  Eigen::Vector3d rotation_pac_cmd;
  rotation_pac_cmd.setZero();

  for (int j_f = 0; j_f < 10; j_f++)
  {

    bjx1_index_flag = Indexfind_flag(_t_f(j_f),xyz1); 
    _bjx1_flag = period_flag(bjx1_index_flag)+1;
    _bjx1 = _bjx1_flag;  
  
    // //   rotation angle generation:
    if (_bjx1 >= 2)
    {      
        if ((j_index +1+j_f - round(_tx_flag(bjx1_index_flag,0)/_dt))*_dt < _td_flag(bjx1_index_flag,0))  // j_index and _bjx1 coincident with matlab: double suppot
        {
          rotation_pac(0) = _yaw_ref_flag(bjx1_index_flag,0);
          rotation_pac(1) = 0;
          rotation_pac(2) = 0;
        }
        else
        {
          //initial state and final state and the middle state
          double t_des = (j_index +1+j_f - round(_tx_flag(bjx1_index_flag,0)/_dt))*_dt;
          Eigen::Vector3d t_plan;
          t_plan(0) = _td_flag(bjx1_index_flag,0)-_dt;
          t_plan(1) = (_td_flag(bjx1_index_flag,0) + _ts_flag(bjx1_index_flag,0))/2 + 0.001;
          t_plan(2) = _ts_flag(bjx1_index_flag,0) + 0.001;
    
          if (abs(t_des - _ts_flag(bjx1_index_flag,0)) <= (_dt + 0.0005))
          {
            rotation_pac(0) = _yaw_ref_flag(bjx1_index_flag+1,0);
            rotation_pac(1) = 0;
            rotation_pac(2) = 0; 
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
            Eigen::Matrix<double, 7, 1> rotation_plan;
            rotation_plan.setZero();	
            rotation_plan(0) = 0;     
            rotation_plan(1) = 0; 
            rotation_plan(2) = _yaw_ref_flag(bjx1_index_flag,0); 
            rotation_plan(3) = (_yaw_ref_flag(bjx1_index_flag,0) + _yaw_ref_flag(bjx1_index_flag+1,0))/2;
            rotation_plan(4) = _yaw_ref_flag(bjx1_index_flag+1,0);  
            rotation_plan(5) = 0;                   
            rotation_plan(6) = 0;	  
            
            
            Eigen::Matrix<double, 7, 1> rotation_co;
            rotation_co.setZero();
            rotation_co = AAA_inv * rotation_plan;
            
            rotation_pac(0) = t_a_plan * rotation_co;
            rotation_pac(1) = t_a_planv * rotation_co;
            rotation_pac(2) = t_a_plana * rotation_co;
            // cout<<"rotation_pac in the swing phase: \t"<<rotation_pac(0)<<endl;
            // cout<<"rotation_plan in the swing phase: \t"<<rotation_plan<<endl;
            
          }
        }
        
    }
    else
    {
      
      rotation_pac(0) = _yaw_ref_flag(0,0);
      rotation_pac(1) = 0;
      rotation_pac(2) = 0;
    } 

    // yaw_mpc_ref.col(j_f) = rotation_pac;

    yaw_mpc_ref.block<3,1>(3*j_f,0) = rotation_pac;

    if(j_f==0)
    {
      rotation_pac_cmd = rotation_pac;      
    } 
    
  } 


  bjx1_index_flag = Indexfind_flag(_t_f(0),xyz1); 
  _bjx1_flag = period_flag(bjx1_index_flag)+1;
  _bjx1 = _bjx1_flag;  

  return rotation_pac_cmd;
  
}

/// @brief rotation the support leg around the body center
/// @param j_index 
/// @return 
Eigen::Matrix<double, 12,1> MPCClass::rotation_support_position_solve(int j_index, Eigen::Matrix<double,12,1> suppor_position_estimation)     
{ 
  ///// return four support positions,  FR, FL, RR, RL////
  Eigen::Matrix<double, 12,1> rotation_pac;
  rotation_pac.setZero();

  Eigen::Matrix<double, 12,1> rotation_pac_cmd;
  rotation_pac_cmd.setZero();

  //// rotation
  Eigen::Matrix3d r_rotation;
  r_rotation.setZero();


  for (int j_f = 0; j_f < 10; j_f++)
  {
    bjx1_index_flag = Indexfind_flag(_t_f(j_f),xyz1); 
    _bjx1_flag = period_flag(bjx1_index_flag)+1;
    _bjx1 = _bjx1_flag;      

    r_rotation(0,0) = cos(_yaw_ref_flag(bjx1_index_flag+1,0));
    r_rotation(0,1) = -sin(_yaw_ref_flag(bjx1_index_flag+1,0));
    r_rotation(1,0) = sin(_yaw_ref_flag(bjx1_index_flag+1,0));
    r_rotation(1,1) = cos(_yaw_ref_flag(bjx1_index_flag+1,0));
    r_rotation(2,2) = 1;

    if(_bjx1<=2) /// right support//// estimated supported 
    { 
      if(_re_initilization==1)
      {
        rotation_pac.block<3,1>(0,0) = FR_des;
        rotation_pac.block<3,1>(3,0) = FL_des;
        rotation_pac.block<3,1>(6,0) = RR_des;
        rotation_pac.block<3,1>(9,0) = RL_des;
      } 
      else // first initialization, using the estimated results;
      {
        FR_fixed = FR_current = suppor_position_estimation.block<3,1>(0,0);
        FL_fixed = FL_current = suppor_position_estimation.block<3,1>(3,0);
        RR_fixed = RR_current = suppor_position_estimation.block<3,1>(6,0);
        RL_fixed = RL_current = suppor_position_estimation.block<3,1>(9,0);
        // cout<<"estimated:"<<suppor_position_estimation <<endl;

        rotation_pac.block<3,1>(0,0) = FR_current;
        rotation_pac.block<3,1>(3,0) = FL_current;
        rotation_pac.block<3,1>(6,0) = RR_current;
        rotation_pac.block<3,1>(9,0) = RL_current;
      } 

    }
    else
    {
      if(_bjx1 % 2 ==0) //////odd:left support, keep left foot fixed ===> that is FR & RL
      {
        ///// FR & RL
        rotation_pac.block<3,1>(0,0) = FR_current;
        rotation_pac.block<3,1>(9,0) = RL_current;
        ///// update right support position===> FL & RR
        double t_des = (j_index +1+j_f - round(_tx_flag(bjx1_index_flag,0)/_dt))*_dt;
        if (t_des < _td_flag(bjx1_index_flag,0))  // j_index and _bjx1 coincident with matlab: double suppot
        {
          rotation_pac.block<3,1>(3,0) = FL_current;
          rotation_pac.block<3,1>(6,0) = RR_current;
        }
        else
        {
          //initial state and final state and the middle state
          Eigen::Vector3d t_plan;
          t_plan(0) = _td_flag(bjx1_index_flag,0)-_dt;
          t_plan(1) = (_td_flag(bjx1_index_flag,0) + _ts_flag(bjx1_index_flag,0))/2 + 0.001;
          t_plan(2) = _ts_flag(bjx1_index_flag,0) + 0.001;
          
          //// support after rotation
          FL_des = r_rotation * FL_fixed; 
          RR_des = r_rotation * RR_fixed;


          if (abs(t_des - _ts_flag(bjx1_index_flag,0)) <= (_dt + 0.0005))
          {
            rotation_pac.block<3,1>(3,0) = FL_des;
            rotation_pac.block<3,1>(6,0) = RR_des;
            FL_current = FL_des;
            RR_current = RR_des;
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
            // Eigen::Matrix<double, 1, 7> t_a_planv;
            // t_a_planv.setZero();
            // t_a_planv(0) = 6*pow(t_des, 5);   t_a_planv(1) = 5*pow(t_des, 4);   t_a_planv(2) = 4*pow(t_des, 3);  t_a_planv(3) = 3*pow(t_des, 2);
            // t_a_planv(4) = 2*pow(t_des, 1);   t_a_planv(5) = 1;                 t_a_planv(6) = 0;        
            // Eigen::Matrix<double, 1, 7> t_a_plana;
            // t_a_plana.setZero();
            // t_a_plana(0) = 30*pow(t_des, 4);   t_a_plana(1) = 20*pow(t_des, 3);   t_a_plana(2) = 12*pow(t_des, 2);  t_a_plana(3) = 6*pow(t_des, 1);
            // t_a_plana(4) = 2;                  t_a_plana(5) = 0;                  t_a_plana(6) = 0;         
            ////////////////////////////////////////////////////////////////////////////
            Eigen::Matrix<double, 7, 3> rotation_plan;
            Eigen::Matrix<double, 7, 3> rotation_co;
            ///// FL
            rotation_plan.setZero();
            rotation_plan.block<1,3>(2,0) = FL_current.transpose(); 
            rotation_plan.block<1,3>(3,0) = (FL_current.transpose() + FL_des.transpose())/2;
            rotation_plan.block<1,3>(4,0) = FL_des.transpose();    
            rotation_co.setZero();
            rotation_co = AAA_inv * rotation_plan;
            rotation_pac.block<3,1>(3,0) = (t_a_plan * rotation_co).transpose();

            ///// RR //////
            rotation_plan.setZero();
            rotation_plan.block<1,3>(2,0) = RR_current.transpose(); 
            rotation_plan.block<1,3>(3,0) = (RR_current.transpose() + RR_des.transpose())/2;
            rotation_plan.block<1,3>(4,0) = RR_des.transpose();    
            rotation_co.setZero();
            rotation_co = AAA_inv * rotation_plan;
            rotation_pac.block<3,1>(6,0) = (t_a_plan * rotation_co).transpose();

          }
        } 
      }
      else //////odd:right support, right left foot fixed ===> that is FL & RR
      {
        ///// FL & RR
        rotation_pac.block<3,1>(3,0) = FL_current;
        rotation_pac.block<3,1>(6,0) = RR_current;
        ///// update right support position===> FR & RL
        double t_des = (j_index +1 - round(_tx_flag(bjx1_index_flag,0)/_dt))*_dt;
        if (t_des < _td_flag(bjx1_index_flag,0))  // j_index and _bjx1 coincident with matlab: double suppot
        {
          rotation_pac.block<3,1>(0,0) = FR_current;
          rotation_pac.block<3,1>(9,0) = RL_current;
        }
        else
        {
          //initial state and final state and the middle state
          Eigen::Vector3d t_plan;
          t_plan(0) = _td_flag(bjx1_index_flag,0)-_dt;
          t_plan(1) = (_td_flag(bjx1_index_flag,0) + _ts_flag(bjx1_index_flag,0))/2 + 0.001;
          t_plan(2) = _ts_flag(bjx1_index_flag,0) + 0.001;
          
          //// support after rotation
          FR_des = r_rotation * FR_fixed; 
          RL_des = r_rotation * RL_fixed;


          if (abs(t_des - _ts_flag(bjx1_index_flag,0)) <= (_dt + 0.0005))
          {
            rotation_pac.block<3,1>(0,0) = FR_des;
            rotation_pac.block<3,1>(9,0) = RL_des;
            FR_current = FR_des;
            RL_current = RL_des;
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
            // Eigen::Matrix<double, 1, 7> t_a_planv;
            // t_a_planv.setZero();
            // t_a_planv(0) = 6*pow(t_des, 5);   t_a_planv(1) = 5*pow(t_des, 4);   t_a_planv(2) = 4*pow(t_des, 3);  t_a_planv(3) = 3*pow(t_des, 2);
            // t_a_planv(4) = 2*pow(t_des, 1);   t_a_planv(5) = 1;                 t_a_planv(6) = 0;        
            // Eigen::Matrix<double, 1, 7> t_a_plana;
            // t_a_plana.setZero();
            // t_a_plana(0) = 30*pow(t_des, 4);   t_a_plana(1) = 20*pow(t_des, 3);   t_a_plana(2) = 12*pow(t_des, 2);  t_a_plana(3) = 6*pow(t_des, 1);
            // t_a_plana(4) = 2;                  t_a_plana(5) = 0;                  t_a_plana(6) = 0;         
            ////////////////////////////////////////////////////////////////////////////
            Eigen::Matrix<double, 7, 3> rotation_plan;
            Eigen::Matrix<double, 7, 3> rotation_co;
            ///// FR
            rotation_plan.setZero();
            rotation_plan.block<1,3>(2,0) = FR_current.transpose(); 
            rotation_plan.block<1,3>(3,0) = (FR_current.transpose() + FR_des.transpose())/2;
            rotation_plan.block<1,3>(4,0) = FR_des.transpose();    
            rotation_co.setZero();
            rotation_co = AAA_inv * rotation_plan;
            rotation_pac.block<3,1>(0,0) = (t_a_plan * rotation_co).transpose();

            ///// RL //////
            rotation_plan.setZero();
            rotation_plan.block<1,3>(2,0) = RL_current.transpose(); 
            rotation_plan.block<1,3>(3,0) = (RL_current.transpose() + RL_des.transpose())/2;
            rotation_plan.block<1,3>(4,0) = RL_des.transpose();    
            rotation_co.setZero();
            rotation_co = AAA_inv * rotation_plan;
            rotation_pac.block<3,1>(9,0) = (t_a_plan * rotation_co).transpose();

          }
        } 
      }  
    }
    

    support_position_mpc_ref.block<12,1>(j_f*12,0) = rotation_pac;



    if(j_f ==0 )
    {
      rotation_pac_cmd = rotation_pac;
    }
  
  
  
  }



  bjx1_index_flag = Indexfind_flag(_t_f(0),xyz1); 
  _bjx1_flag = period_flag(bjx1_index_flag)+1;
  _bjx1 = _bjx1_flag;  

  return rotation_pac_cmd;
}




// int MPCClass::Get_maximal_number_reference()
// {
//   int nsum_max;
//   nsum_max = (nsum_x -_n_loop_omit);  
//   return nsum_max;
// }

// int MPCClass::Get_maximal_number(double dtx)
// {
//   int nsum_max;
//   nsum_max = (nsum_x -_n_loop_omit)*round(_dt/dtx);
  
//   return nsum_max;
// }



////====================================================================================================================
// /////////////////////////// using the lower-level control-loop  sampling time as the reference: every 5ms;  at the same time: just using the next one position + next one velocity

// Vector3d MPCClass::XGetSolution_CoM_position(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
// {
// //   //reference com position
// //     _CoM_position_optimal.row(0) = _comx;
// // 	_CoM_position_optimal.row(1) = _comy;
// // 	_CoM_position_optimal.row(2) = _comz;



	
// 	Vector3d com_inte;	
// 	com_inte.setZero();

// // 	if (walktime>=2)
// // 	{
// // 	  int t_int; 
// // // 	  t_int = floor(walktime / (_dt / dt_sample) );
// // 	  t_int = floor(walktime* dt_sample/ _dt);	  
// //     //   cout <<"T_int_inter:"<<t_int<<endl;
// // 	  ///// chage to be relative time
// // 	  double t_cur;
// // 	  t_cur = walktime * dt_sample ;
	  

// // 	  Eigen::Matrix<double, 4, 1> t_plan;
// // 	  t_plan.setZero();
// // 	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
// // 	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
// // 	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
// // 	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);
          
// // // 	  cout << "t_plan1:"<<endl<<t_plan<<endl;
// // // 	  Eigen::MatrixXd AAA1;	
// // // 
// // // 	  AAA1.setZero(4,4);	
// // // 	  AAA1(0,0) = pow(t_plan(0), 3); AAA1(0,1) = pow(t_plan(0), 2); AAA1(0,2) = pow(t_plan(0), 1); AAA1(0,3) = pow(t_plan(0), 0); 
// // // 	  AAA1(1,0) = pow(t_plan(1), 3); AAA1(1,1) = pow(t_plan(1), 2); AAA1(1,2) = pow(t_plan(1), 1); AAA1(1,3) = pow(t_plan(0), 0); 
// // // 	  AAA1(2,0) = pow(t_plan(2), 3); AAA1(2,1) = pow(t_plan(2), 2); AAA1(2,2) = pow(t_plan(2), 1); AAA1(2,3) = pow(t_plan(0), 0); 
// // // 	  AAA1(3,0) = 3*pow(t_plan(2), 2); AAA1(3,1) = 2*pow(t_plan(2), 1); AAA1(3,2) = pow(t_plan(2), 0); AAA1(3,3) = 0;  


// // 	  Eigen::Matrix4d AAA_inv;
	  
// // 	  double abx1, abx2, abx3, abx4;
// // 	  abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
// // 	  abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
// // 	  abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
// // 	  abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
	  

// // 	  AAA_inv(0,0) = 1/ abx1;
// // 	  AAA_inv(0,1) =  -1/ abx2;
// // 	  AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
// // 	  AAA_inv(0,3) = 1/ abx4;
	  
// // 	  AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
// // 	  AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
// // 	  AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
// // 	  AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
	  
// // 	  AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
// // 	  AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
// // 	  AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
// // 	  AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
	  
// // 	  AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
// // 	  AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
// // 	  AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
// // 	  AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;
	  	  
	  	  
// // /*	  cout << "AAA_inv:"<<endl<<AAA_inv<<endl;*/
	  

	
	  
	  
// // 	  Eigen::Matrix<double, 1, 4> t_a_plan;
// // 	  t_a_plan.setZero();
// // 	  t_a_plan(0) = pow(t_cur-( t_cur - 2*dt_sample), 3);   
// // 	  t_a_plan(1) = pow(t_cur-( t_cur - 2*dt_sample), 2);   
// // 	  t_a_plan(2) = pow(t_cur-( t_cur - 2*dt_sample), 1);  
// // 	  t_a_plan(3) = pow(t_cur-( t_cur - 2*dt_sample), 0); 


	  
// // 	  // COM&&foot trajectory interpolation
	  
// // 	  Eigen::Vector3d  x10;
// // 	  Eigen::Vector3d  x11;
// // 	  Eigen::Vector3d  x12;
// // // 	  Eigen::Vector3d  x13;


// // /*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
// // 	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
// // 	  x10 = body_in1; 
// // 	  x11 = body_in2;  
// // 	  x12 = _CoM_position_optimal.col(t_int);
// // // 	  x13 = _CoM_position_optimal.col(t_int+1);
	  
	  
// // 	  Eigen::Matrix<double, 4, 1>  temp;
// // 	  temp.setZero();
// // 	  temp(0) = x10(0); temp(1) = x11(0); temp(2) = x12(0); temp(3) = _comvx(t_int);	  
// // 	  com_inte(0) = t_a_plan * (AAA_inv)*temp;
// // 	  temp(0) = x10(1); temp(1) = x11(1); temp(2) = x12(1); temp(3) = _comvy(t_int);	  
// // 	  com_inte(1) = t_a_plan * (AAA_inv)*temp;
// // 	  temp(0) = x10(2); temp(1) = x11(2); temp(2) = x12(2); temp(3) = _comvz(t_int);	  
// // 	  com_inte(2) = t_a_plan *(AAA_inv)*temp;

  
	  
	  
// // 	}
// // 	else
// // 	{
// // 	  com_inte(0) = body_in3(0);	  
// // 	  com_inte(1) = body_in3(1);	  	  
// // 	  com_inte(2) = body_in3(2);	
	  
// // 	}

//  	return com_inte;
	
// }

// Vector3d MPCClass::XGetSolution_body_inclination(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
// {
// //   //reference com position
// //         _torso_angle_optimal.row(0) = _thetax;
// // 	_torso_angle_optimal.row(1) = _thetay;
// // 	_torso_angle_optimal.row(2) = _thetaz;
	
	
// 	Vector3d com_inte;	
// 	com_inte.setZero();	
// // 	if (walktime>=2)
// // 	{
// // 	  int t_int; 
// // 	  t_int = floor(walktime* dt_sample/ _dt  );

	  
// // 	  double t_cur;
// // 	  t_cur = walktime * dt_sample;
	  

	  
// // 	  Eigen::Matrix<double, 4, 1> t_plan;
// // 	  t_plan.setZero();
// // 	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
// // 	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
// // 	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
// // 	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);

// // // 	  Eigen::MatrixXd AAA;	
// // // 
// // // 	  AAA.setZero(4,4);	
// // // 	  AAA(0,0) = pow(t_plan(0), 3); AAA(0,1) = pow(t_plan(0), 2); AAA(0,2) = pow(t_plan(0), 1); AAA(0,3) = pow(t_plan(0), 0); 
// // // 	  AAA(1,0) = pow(t_plan(1), 3); AAA(1,1) = pow(t_plan(1), 2); AAA(1,2) = pow(t_plan(1), 1); AAA(1,3) = pow(t_plan(0), 0); 
// // // 	  AAA(2,0) = pow(t_plan(2), 3); AAA(2,1) = pow(t_plan(2), 2); AAA(2,2) = pow(t_plan(2), 1); AAA(2,3) = pow(t_plan(0), 0); 
// // // 	  AAA(3,0) = 3*pow(t_plan(2), 2); AAA(3,1) = 2*pow(t_plan(2), 1); AAA(3,2) = pow(t_plan(2), 0); AAA(3,3) = 0;  


// // 	  Eigen::Matrix4d AAA_inv;
	  
// // 	  double abx1, abx2, abx3, abx4;
// // 	  abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
// // 	  abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
// // 	  abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
// // 	  abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
	  

// // 	  AAA_inv(0,0) = 1/ abx1;
// // 	  AAA_inv(0,1) =  -1/ abx2;
// // 	  AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
// // 	  AAA_inv(0,3) = 1/ abx4;
	  
// // 	  AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
// // 	  AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
// // 	  AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
// // 	  AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
	  
// // 	  AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
// // 	  AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
// // 	  AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
// // 	  AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
	  
// // 	  AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
// // 	  AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
// // 	  AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
// // 	  AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;
	  	  
	  
	  
	  
	
	  
	  
// // 	  Eigen::Matrix<double, 1, 4> t_a_plan;
// // 	  t_a_plan.setZero();
// // 	  t_a_plan(0) = pow(t_cur-( t_cur - 2*dt_sample), 3);   t_a_plan(1) = pow(t_cur-( t_cur - 2*dt_sample), 2);   t_a_plan(2) = pow(t_cur-( t_cur - 2*dt_sample), 1);  t_a_plan(3) = pow(t_cur-( t_cur - 2*dt_sample), 0); 


	  
// // 	  // COM&&foot trajectory interpolation
	  
// // 	  Eigen::Vector3d  x10;
// // 	  Eigen::Vector3d  x11;
// // 	  Eigen::Vector3d  x12;
// // // 	  Eigen::Vector3d  x13;


// // /*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
// // 	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
// // 	  x10 = body_in1; 
// // 	  x11 = body_in2;  
// // 	  x12 = _torso_angle_optimal.col(t_int);
// // // 	  x13 = _torso_angle_optimal.col(t_int+1);
	  
	  
// // 	  Eigen::Matrix<double, 4, 1>  temp;
// // 	  temp.setZero();
// // 	  temp(0) = x10(0); temp(1) = x11(0); temp(2) = x12(0); temp(3) = _thetavx(t_int);	  
// // 	  com_inte(0) = t_a_plan * (AAA_inv)*temp;
// // 	  temp(0) = x10(1); temp(1) = x11(1); temp(2) = x12(1); temp(3) = _thetavy(t_int);	  
// // 	  com_inte(1) = t_a_plan * (AAA_inv)*temp;
// // 	  temp(0) = x10(2); temp(1) = x11(2); temp(2) = x12(2); temp(3) = _thetavz(t_int);	  
// // 	  com_inte(2) = t_a_plan *(AAA_inv)*temp;

	  
	  
// // 	}
// // 	else
// // 	{
// // 	  com_inte(0) = body_in3(0);	  
// // 	  com_inte(1) = body_in3(1);	  	  
// // 	  com_inte(2) = body_in3(2);	
	  
// // 	}

//  	return com_inte;
	
  
  
  
// }


// Vector3d MPCClass::XGetSolution_Foot_positionR(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
// {
	
//     //     _R_foot_optition_optimal.row(0) = _Rfootx;
// 	// _R_foot_optition_optimal.row(1) = _Rfooty;
// 	// _R_foot_optition_optimal.row(2) = _Rfootz;
	
// 	Vector3d com_inte;	
// 	com_inte.setZero();
// // 	if (walktime>=2)
// // 	{
// // 	  int t_int; 
// // 	  t_int = floor(walktime* dt_sample/ _dt  );

	  
// // 	  double t_cur;
// // 	  t_cur = walktime * dt_sample;
	  

	  
// // 	  Eigen::Matrix<double, 4, 1> t_plan;
// // 	  t_plan.setZero();
// // 	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
// // 	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
// // 	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
// // 	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);

// // // 	  cout << "t_plan2:"<<endl<<t_plan<<endl;
// // // 	  Eigen::MatrixXd AAA;
// // // 
// // // 	  
// // // 	  AAA.setZero(4,4);	
// // // 	  AAA(0,0) = pow(t_plan(0), 3); AAA(0,1) = pow(t_plan(0), 2); AAA(0,2) = pow(t_plan(0), 1); AAA(0,3) = pow(t_plan(0), 0); 
// // // 	  AAA(1,0) = pow(t_plan(1), 3); AAA(1,1) = pow(t_plan(1), 2); AAA(1,2) = pow(t_plan(1), 1); AAA(1,3) = pow(t_plan(0), 0); 
// // // 	  AAA(2,0) = pow(t_plan(2), 3); AAA(2,1) = pow(t_plan(2), 2); AAA(2,2) = pow(t_plan(2), 1); AAA(2,3) = pow(t_plan(0), 0); 
// // // 	  AAA(3,0) = 3*pow(t_plan(2), 2); AAA(3,1) = 2*pow(t_plan(2), 1); AAA(3,2) = pow(t_plan(2), 0); AAA(3,3) = 0;  


	  
// // 	  Eigen::Matrix4d AAA_inv;
	  
// // 	  double abx1, abx2, abx3, abx4;
// // 	  abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
// // 	  abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
// // 	  abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
// // 	  abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
	  

// // 	  AAA_inv(0,0) = 1/ abx1;
// // 	  AAA_inv(0,1) =  -1/ abx2;
// // 	  AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
// // 	  AAA_inv(0,3) = 1/ abx4;
	  
// // 	  AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
// // 	  AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
// // 	  AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
// // 	  AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
	  
// // 	  AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
// // 	  AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
// // 	  AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
// // 	  AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
	  
// // 	  AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
// // 	  AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
// // 	  AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
// // 	  AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;
	  	  
	  
// // /*	  cout << "AAA_inv:"<<endl<<AAA_inv<<endl;	*/  	  
	  

	
	  
	  
// // 	  Eigen::Matrix<double, 1, 4> t_a_plan;
// // 	  t_a_plan.setZero();
// // 	  t_a_plan(0) = pow(t_cur-( t_cur - 2*dt_sample), 3);  
// // 	  t_a_plan(1) = pow(t_cur-( t_cur - 2*dt_sample), 2);  
// // 	  t_a_plan(2) = pow(t_cur-( t_cur - 2*dt_sample), 1);  
// // 	  t_a_plan(3) = pow(t_cur-( t_cur - 2*dt_sample), 0); 


	  
// // 	  // COM&&foot trajectory interpolation
	  
// // 	  Eigen::Vector3d  x10;
// // 	  Eigen::Vector3d  x11;
// // 	  Eigen::Vector3d  x12;
// // // 	  Eigen::Vector3d  x13;


// // /*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
// // 	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
// // 	  x10 = body_in1; 
// // 	  x11 = body_in2;  
// // 	  x12 =  _R_foot_optition_optimal.col(t_int);
// // //	  x13 =  _R_foot_optition_optimal.col(t_int+1);
	  
	  
// // 	  Eigen::Matrix<double, 4, 1>  temp;
// // 	  temp.setZero();
// // 	  temp(0) = x10(0); temp(1) = x11(0); temp(2) = x12(0); temp(3) = _Rfootvx(t_int);	  
// // 	  com_inte(0) = t_a_plan * (AAA_inv)*temp;
// // 	  temp(0) = x10(1); temp(1) = x11(1); temp(2) = x12(1); temp(3) = _Rfootvy(t_int);

// // // 	  cout << "Rfooty:"<<temp<<endl;
// // 	  com_inte(1) = t_a_plan * (AAA_inv)*temp;
// // 	  temp(0) = x10(2); temp(1) = x11(2); temp(2) = x12(2); temp(3) = _Rfootvz(t_int);	  
// // 	  com_inte(2) = t_a_plan *(AAA_inv)*temp;
	  
// // ////=====================================================////////////////////////////////////////////////
// // // 	  //////spline for Rfoot_
	  
	  
	  
	  
	  
	  
	  
	  
	  
// // 	}
// // 	else
// // 	{
// // 	  com_inte(0) = body_in3(0);	  
// // 	  com_inte(1) = body_in3(1);	  	  
// // 	  com_inte(2) = body_in3(2);

// // // 	  com_inte = Rfoot_IN.col(walktime);	  
// // 	}
	
// // // 	cout << "Rfooty_generated:"<<com_inte(1)<<endl;

//  	return com_inte;
	
	
	
	
// }

// Vector3d MPCClass::XGetSolution_Foot_positionL(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
// {
//     //     _L_foot_optition_optimal.row(0) = _Lfootx;
// 	// _L_foot_optition_optimal.row(1) = _Lfooty;
// 	// _L_foot_optition_optimal.row(2) = _Lfootz;
	
// 	Vector3d com_inte;	
// 	com_inte.setZero();
	
// // 	if (walktime>=2)
// // 	{
// // 	  int t_int; 
// // 	  t_int = floor(walktime* dt_sample/ _dt  );

	  
// // 	  double t_cur;
// // 	  t_cur = walktime * dt_sample;
	  

	  
// // 	  Eigen::Matrix<double, 4, 1> t_plan;
// // 	  t_plan.setZero();
// // 	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
// // 	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
// // 	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
// // 	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);

// // //           cout << "t_plan3:"<<endl<<t_plan<<endl;
// // /*	  Eigen::MatrixXd  AAAaaa; /// should be Marix4d
// // 	  AAAaaa.setZero(4,4);
	  
// // 	  AAAaaa(0,0) = pow(t_plan(0), 3); AAAaaa(0,1) = pow(t_plan(0), 2); AAAaaa(0,2) = pow(t_plan(0), 1); AAAaaa(0,3) = pow(t_plan(0), 0); 
// // 	  AAAaaa(1,0) = pow(t_plan(1), 3); AAAaaa(1,1) = pow(t_plan(1), 2); AAAaaa(1,2) = pow(t_plan(1), 1); AAAaaa(1,3) = pow(t_plan(1), 0); 
// // 	  AAAaaa(2,0) = pow(t_plan(2), 3); AAAaaa(2,1) = pow(t_plan(2), 2); AAAaaa(2,2) = pow(t_plan(2), 1); AAAaaa(2,3) = pow(t_plan(2), 0); 
// //          // AAAaaa(3,0) = pow(t_plan(3), 3); AAAaaa(3,1) = pow(t_plan(3), 2); AAAaaa(3,2) = pow(t_plan(3), 1); AAAaaa(3,3) = pow(t_plan(3), 0); /// using the next to positioin would cause over-fitting	  
// // 	  AAAaaa(3,0) = 3*pow(t_plan(2), 2); AAAaaa(3,1) = 2*pow(t_plan(2), 1); AAAaaa(3,2) = pow(t_plan(2), 0); AAAaaa(3,3) = 0.0;  	  
// //  */	  
// // //       MatrixXd.inverse( != Matrix4d (under Xd =4. so write the inverse of Matrix4d explicitly): for the time being)
	  
	  
// // 	  Eigen::Matrix4d AAA_inv;
	  
// // 	  double abx1, abx2, abx3, abx4;
// // 	  abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
// // 	  abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
// // 	  abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
// // 	  abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
	  

// // 	  AAA_inv(0,0) = 1/ abx1;
// // 	  AAA_inv(0,1) =  -1/ abx2;
// // 	  AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
// // 	  AAA_inv(0,3) = 1/ abx4;
	  
// // 	  AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
// // 	  AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
// // 	  AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
// // 	  AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
	  
// // 	  AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
// // 	  AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
// // 	  AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
// // 	  AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
	  
// // 	  AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
// // 	  AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
// // 	  AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
// // 	  AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;
	  	  
// // /*	  cout << "AAA_inv:"<<endl<<AAA_inv<<endl;	*/  
	  
	  
	  
	
// // // // 	  Eigen::Matrix4d  AAAaaa1_inv=AAA_inv;
	  
// // // 	  cout<< t_plan<<endl;
// // // 	  cout<< AAAaaa<<endl;
// // // 	  cout<< AAA_inv - AAAaaa1_inv<<endl;
// // // 	  cout<< AAA_inv - AAAaaa1.inverse()<<endl;
	  
// // 	  Eigen::RowVector4d t_a_plan;
// // 	  t_a_plan.setZero();
// // 	  t_a_plan(0) = pow(t_cur-( t_cur - 2*dt_sample), 3);  
// // 	  t_a_plan(1) = pow(t_cur-( t_cur - 2*dt_sample), 2);   
// // 	  t_a_plan(2) = pow(t_cur-( t_cur - 2*dt_sample), 1); 
// // 	  t_a_plan(3) = pow(t_cur-( t_cur - 2*dt_sample), 0); 


	  
// // 	  // COM&&foot trajectory interpolation
	  
// // 	  Eigen::Vector3d  x10;
// // 	  Eigen::Vector3d  x11;
// // 	  Eigen::Vector3d  x12;
// // // 	  Eigen::Vector3d  x13;


// // /*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
// // 	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
// // 	  x10 = body_in1; 
// // 	  x11 = body_in2;  
// // 	  x12 =  _L_foot_optition_optimal.col(t_int);
// // // 	  x13 =  _L_foot_optition_optimal.col(t_int+1); /// using next two position would caused over-fitting
	  
	  
// // 	  Eigen::Vector4d  temp;
// // 	  temp.setZero();
// // 	  temp(0) = x10(0); temp(1) = x11(0); temp(2) = x12(0);
// // 	  temp(3) = _Lfootvx(t_int);
// // // 	  	  temp(3) = x13(0);
// // 	  Eigen::Vector4d tmp111 = AAA_inv*temp;
// // 	  com_inte(0) = t_a_plan * tmp111;
// // 	  temp(0) = x10(1); temp(1) = x11(1); temp(2) = x12(1);
// // 	  temp(3) = _Lfootvy(t_int);	 
// // /*          temp(3) = x13(1);	*/  
// // 	  tmp111 = AAA_inv*temp;  
// // 	  com_inte(1) = t_a_plan * tmp111;
// // 	  temp(0) = x10(2); temp(1) = x11(2); temp(2) = x12(2); 
// // 	  temp(3) = _Lfootvz(t_int);	
// // // 	  temp(3) = x13(2);
// // 	  tmp111 = AAA_inv*temp;	  
// // 	  com_inte(2) = t_a_plan * tmp111;

// // ////================================================not use spline.h=====////////////////////////////////////////////////


	  
// // 	}
// // 	else
// // 	{
// // 	  com_inte(0) = body_in3(0);	  
// // 	  com_inte(1) = body_in3(1);	  	  
// // 	  com_inte(2) = body_in3(2);
// // 	}

//  	return com_inte;
	
// 	cout << "Lfooty_generated:"<<com_inte(1)<<endl;
// }




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
  double t_desxx = (walktime+dt_dtx)*dt_sample - (_tx_flag(bjx1_index_flag,0) +  2*_td_flag(bjx1_index_flag,0)/4); /// phase transition: SSP starts at the beginning of each cycle
  //cout<< "t_desxx"<<t_desxx + 2*_td_flag(bjx1_index_flag,0)/4<<endl;

  if ((_bjx1 >= 2)&&(j_index <= _t_end_footstep))
  {       
    if (_bjx1 % 2 == 0)           //odd:left support
    {  
      //// right foot roll:
      ///////  step forward and backward 1//////  
      _Rfoot_r(0,0) =  -0.065*(1 - cos(2*M_PI / (_ts_flag(bjx1_index_flag,0) ) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4) ));
      _Rfoot_rv(0,0) =  -0.065*(2*M_PI / (_ts_flag(bjx1_index_flag,0) ) * sin(2*M_PI / (_ts_flag(bjx1_index_flag,0) ) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4) ));
      _Rfoot_ra(0,0) =  -0.065*(2*M_PI / (_ts_flag(bjx1_index_flag,0) ) * 2*M_PI / (_ts_flag(bjx1_index_flag,0) ) * cos(2*M_PI / (_ts_flag(bjx1_index_flag,0) ) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4) ));

      // _Rfoot_r(0,0) =  -0.065*(1-fabs((_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))/(_footx_max)))*(1 - cos(2*M_PI / (_ts_flag(bjx1_index_flag,0) ) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4) ));
      

      ///// right foot pitch: ///later half cycle: minus angle;
      if ((t_desxx + 2*_td_flag(bjx1_index_flag,0)/4) >= (_ts_flag(bjx1_index_flag,0)/2))
      {
        // if ((_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))>0)
        // {
          _Rfoot_r(1,0) =  0.075*(_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))/(_footx_max) * (cos(4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4) ) - 1);  
          _Rfoot_rv(1,0) =  -abs(0.075*(_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))/(_footx_max) * (4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (-sin(4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4)))));
          _Rfoot_ra(1,0) =  0.075*(_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))/(_footx_max) * (4*M_PI / (_ts_flag(bjx1_index_flag,0)) * 4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (-cos(4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4))));        
        // }
      }
      else
      {
      /*	if ((_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))>0)
        {
          _Rfoot_r(1,0) =  -0.025*(_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))/(_footx_max) * (cos(4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4) ) - 1);          
          _Rfoot_rv(1,0) = -0.025*(_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))/(_footx_max) * (4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (-sin(4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4))));
                _Rfoot_ra(1,0) = -0.025*(_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))/(_footx_max) * (4*M_PI / (_ts_flag(bjx1_index_flag,0)) * 4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (-cos(4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4))));               	  
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
      _Lfoot_r(0,0) =  0.075*(1 - cos(2*M_PI / (_ts_flag(bjx1_index_flag,0) ) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4) ));
      _Lfoot_rv(0,0) =  -0.075*(2*M_PI / (_ts_flag(bjx1_index_flag,0) ) * sin(2*M_PI / (_ts_flag(bjx1_index_flag,0) ) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4) ));
      _Lfoot_ra(0,0) =  -0.075*(2*M_PI / (_ts_flag(bjx1_index_flag,0) ) * 2*M_PI / (_ts_flag(bjx1_index_flag,0) ) * cos(2*M_PI / (_ts_flag(bjx1_index_flag,0) ) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4) ));      
      // _Lfoot_r(0,0) =  0.075*(1-fabs((_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))/(_footx_max)))*(1 - cos(2*M_PI / (_ts_flag(bjx1_index_flag,0) ) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4) ));

      ////// left foot pitch:
      if ((t_desxx + 2*_td_flag(bjx1_index_flag,0)/4) >= (_ts_flag(bjx1_index_flag,0)/2)) ///later half cycle: minus angle;
      {
        // if ((_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))>0)
        // {
          _Lfoot_r(1,0) =  0.075*(_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))/(_footx_max) * (cos(4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4)) - 1); 
          _Lfoot_rv(1,0) =  -abs(0.075*(_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))/(_footx_max) * (4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (-sin(4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4)))));
          _Lfoot_ra(1,0) =  0.075*(_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))/(_footx_max) * (4*M_PI / (_ts_flag(bjx1_index_flag,0)) * 4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (-cos(4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4))));                    
        // } 
      }
      else
      {
      /*        if ((_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))>0)
              {
                _Lfoot_r(1,0) =  -0.025*(_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))/(_footx_max) * (cos(4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4) ) - 1);          
                _Lfoot_rv(1,0) =  -0.025*(_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))/(_footx_max) * (4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (-sin(4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4))));
                _Lfoot_ra(1,0) =  -0.025*(_footxyz_real_flag(0,bjx1_index_flag+1)-_footxyz_real_flag(0,bjx1_index_flag))/(_footx_max) * (4*M_PI / (_ts_flag(bjx1_index_flag,0)) * 4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (-cos(4*M_PI / (_ts_flag(bjx1_index_flag,0)) * (t_desxx + 2*_td_flag(bjx1_index_flag,0)/4))));                      
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


