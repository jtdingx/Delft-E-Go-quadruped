/*****************************************************************************
PRMPCClass.cpp

Description:    source file of PRMPCClass

@Version:   1.0
@Author:    Jiatao Ding
@Release:   Thu 02 Aug 2018 12:33:23 PM CEST
@Update:    Thu 02 Aug 2018 12:33:19 PM CEST
*****************************************************************************/
#include <cstdio>
#include <cstdlib>

#include <iostream>
// #include "/usr/local/include/eigen3/Eigen/Dense"
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>  
#include <vector>
#include "PRMPCClass.h"
#include "yaml.h"

using namespace Eigen;
using namespace std;


PRMPCClass::PRMPCClass()                    ///declaration function
	: QPBaseClass()
    , _RobotPara_Z_C(0.309458)
    , _RobotPara_G(9.8)
    , _RobotPara_FOOT_LENGTH(0.02)
    , _RobotPara_FOOT_WIDTH(0.02)
    , _RobotPara_HIP_TO_ANKLE_X_OFFSET(0)
    , _RobotParaClass_HALF_HIP_WIDTH(0.12675)    
    , _robot_name("")
    , _robot_mass(0.0)
    , _j_period(0)  
    , _pvFlag_kmp(1)
{  

}

void PRMPCClass::config_set()
{   
    /////load default parameter from the yaml.file
    ///////////////////  yaml code . ///////// 
    // YAML::Node config = YAML::LoadFile("/home/pi/go1_catkin/src/unitree_ros_to_real/unitree_legged_real/config/config.yaml");
    YAML::Node config = YAML::LoadFile("/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/go1_rt_control/config/config.yaml");

    _dt =  config["dt_slow_mpc"].as<double>();
    _RobotPara_Z_C =  config["body_p_Homing_Retarget2"].as<double>();
    _tstep =  config["t_period"].as<double>();
    steplengthx = config["step_length"].as<double>();
    stepwidth = config["step_width"].as<double>();
    stepheight = config["step_height"].as<double>();
    _lift_height = config["foot_clear_height"].as<double>();   

	_Rthetax = config["_Rthetax"].as<double>();         
  _Rthetay = config["_Rthetay"].as<double>();
	_alphathetax = config["_alphathetax"].as<double>();           
  _alphathetay= config["_alphathetay"].as<double>();
	_beltathetax = config["_beltathetax"].as<double>();       
  _beltathetay = config["_beltathetay"].as<double>();
	_gama_zmpx = config["_gama_zmpx"].as<double>();     
  _gama_zmpy = config["_gama_zmpy"].as<double>();     

}


/////////////////////// initialize all the variables============================================================
void PRMPCClass::Initialize()
{    
    
    config_set(); 
    // _lift_height -=0.01;
    FootStepInputs( stepwidth, steplengthx, stepheight,_lift_height); 
   
    
    _pvFlag_kmp = 0;                   //model position, pvFlag=1, model position and velocity
    //_pvFlag_kmp = 1;			    // output: pos (and vel)

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
    
	

    
    /////// swing leg trajectory generation using kmp_initialize
    _Lfootx_kmp.setZero();  _Lfooty_kmp.setConstant(_stepwidth(0)); _Lfootz_kmp.setZero(); 
    _Lfootvx_kmp.setZero(); _Lfootvy_kmp.setZero();                 _Lfootvz_kmp.setZero(); 

    _Rfootx_kmp.setZero();  _Rfooty_kmp.setConstant(-_stepwidth(0));_Rfootz_kmp.setZero(); 
    _Rfootvx_kmp.setZero(); _Rfootvy_kmp.setZero();                 _Rfootvz_kmp.setZero(); 

    
   
    _footx_ref.setZero();
    _footy_ref.setZero();
    _footz_ref.setZero();	
    for (int i = 1; i < _footstepsnumber; i++) {
      _footx_ref(i) = _footx_ref(i-1) + _steplength(i-1);
      _footy_ref(i) = _footy_ref(i-1) + (int)pow(-1,i-1)*_stepwidth(i-1);   
      _footz_ref(i) = _footz_ref(i-1) + _stepheight(i-1);
    }  
    
    //////
    _footxyz_real.setZero();
    _footxyz_real.row(0) = _footx_ref.transpose();
    _footxyz_real.row(1) = _footy_ref.transpose();	  
    _footxyz_real.row(2) = _footz_ref.transpose();
    

    _bjxx = 0;

    
    
    /// for foot trajectory generation

    _Lfootx.setZero();
    _Lfooty.setZero(); _Lfooty.setConstant(_stepwidth(0)); _Lfootz.setZero(); 
    _Lfootvx.setZero(); _Lfootvy.setZero();_Lfootvz.setZero(); 
    _Lfootax.setZero(); _Lfootay.setZero();_Lfootaz.setZero();
    _Rfootx.setZero(); 
    _Rfooty.setZero(); _Rfooty.setConstant(-_stepwidth(0));_Rfootz.setZero(); 
    _Rfootvx.setZero(); _Rfootvy.setZero();_Rfootvz.setZero(); 
    _Rfootax.setZero(); _Rfootay.setZero();_Rfootaz.setZero();	
    _ry_left_right = 0;      
    
    
    _Lfoot_r.setZero();
    _Rfoot_r.setZero();   
    _Lfoot_rv.setZero();
    _Rfoot_rv.setZero();     
    _Lfoot_ra.setZero();
    _Rfoot_ra.setZero();
    
    _footx_max= 0.07;
    _footx_min= -0.05;    
    _footy_max= 0.05;
    _footy_min=-0.05;  
  
  
    //////////////////////////////////////////////
    ////////////////////////////////////////////
    /////////////////////////////////////////////////////
    _robot_name = "go1";

    _RobotPara_G = 9.8; 
    _RobotPara_FOOT_LENGTH = 0.02; 
    _RobotPara_FOOT_WIDTH = 0.02;
    _RobotPara_HIP_TO_ANKLE_X_OFFSET = 0; 
    _RobotParaClass_HALF_HIP_WIDTH = 0.12675;

    _ggg(0,0) = gait::g;
//  sampling time & step cycle	
    _ts.setConstant(_tstep);
    _tdsp_ratio = 0.1;
    _td = _tdsp_ratio*_ts;                                // dsp time

    _nstepx = round(_tstep/_dt_mpc);                
    
    _tx.setZero();   // start time for each step cycle	
    for (int i = 1; i < _footstepsnumber; i++) {
      _tx(i) = _tx(i-1) + _ts(i-1);
      _tx(i) = round(_tx(i)/_dt)*_dt -0.00001;	  
    }
    _t_end_footstep = round((_tx(_footstepsnumber-1)- 3*_tstep)/_dt_mpc);
    
    _tx_total = _tx(_footstepsnumber - 1);
    
    cout << "_t_end_footstep:"<<_t_end_footstep<<endl;

    _nsum_mpc = int (floor( _tx(_footstepsnumber-1)/_dt_mpc));

    _thetax.setZero(); _thetavx.setZero(); _thetaax.setZero();
    _thetay.setZero(); _thetavy.setZero(); _thetaay.setZero();
    _thetaz.setZero(); _thetavz.setZero(); _thetaaz.setZero();	
    _torquex_real.setZero(); _torquey_real.setZero();  

    //// state variable for mpc
    _thetaxk.setZero(); 
    _thetayk.setZero();
    _thetax_vacc_k.setZero(); 
    _thetay_vacc_k.setZero(); 	
///////================================================================================================			
    _a << 1, _dt_mpc,    
	        0,   1;
    _b << pow(_dt_mpc,2)/2,
		     _dt_mpc;		
    
    _cp.setZero();
    _cp(0,0) = 1;
    _cv.setZero();
    _cv(0,1) = 1;	
	    
    //predictive model matrixs: just calculated once
    _pps.setZero(); _ppu.setZero();
    _pvs.setZero(); _pvu.setZero();
    _pthetax.setZero(); _pthetay.setZero(); 

    _pps = Matrix_ps(_a,_nh,_cp);
    _pvs = Matrix_ps(_a,_nh,_cv);
  
    _ppu = Matrix_pu(_a,_b,_nh,_cp);
    _pvu = Matrix_pu(_a,_b,_nh,_cv);
    
    _pvu_2 = _pvu.transpose()*_pvu;
    _ppu_2 = _ppu.transpose()*_ppu;

    xyz1 = 0;  //flag for Indexfind function: 
    xyz2 = 1;

// /////=========================================constraints initialize========================================
    // angle range
    _thetax_max = 10*M_PI/180;  
    _thetax_min = -10*M_PI/180;
    _thetay_max = 10*M_PI/180;  
    _thetay_min = -10*M_PI/180;
    
    _thetax_max_vec.setConstant(_thetax_max);  
    _thetax_min_vec.setConstant(_thetax_min);
    _thetay_max_vec.setConstant(_thetay_max);  
    _thetay_min_vec.setConstant(_thetay_min);


    _mass = gait::mass;

    // torque range
    _j_ini = gait::J_ini;
    _torquex_max = 20/_j_ini; 
    _torquex_min = -20/_j_ini;
    _torquey_max = 20/_j_ini;  
    _torquey_min = -20/_j_ini;	

    _torquex_max_vec.setConstant(_torquex_max); 
    _torquex_min_vec.setConstant(_torquex_min);
    _torquey_max_vec.setConstant(_torquey_max);  
    _torquey_min_vec.setConstant(_torquey_min);	

    /// ZMP range
    _zmpx_max=(_RobotPara_FOOT_LENGTH/2+_RobotPara_HIP_TO_ANKLE_X_OFFSET);  
    _zmpx_min=(-(_RobotPara_FOOT_LENGTH/2-_RobotPara_HIP_TO_ANKLE_X_OFFSET));
    _zmpy_max=(_RobotPara_FOOT_WIDTH/2); 
    _zmpy_min=(-_RobotPara_FOOT_WIDTH/2);

    _zmpx_max_vec.setConstant(_zmpx_max); 
    _zmpx_min_vec.setConstant(_zmpx_min);
    _zmpy_max_vec.setConstant(_zmpy_max);  
    _zmpy_min_vec.setConstant(_zmpy_min);	

    
// ///===========initiallize: preparation for MPC solution	

    _V_ini.setZero();                                /// initialize optimal variable
    
           
    _copx_center_ref.setZero();
    _copy_center_ref.setZero();	
    _thetax_center_ref.setZero(); 
    _thetay_center_ref.setZero();	
	      
    // if(_robot_name == "coman"){
    //   _Rthetax = 1;       _Rthetay = 1;          //acceleration
    //   _alphathetax =100;   _alphathetay = 100;          //velocity
    //   _beltathetax = 50000000;      _beltathetay = 500000000;         //position
    //   _gama_zmpx = 1000;      _gama_zmpy= 1000;   ////ZMP minimization
    // }
    // else if(_robot_name  == "go1"){ 
          
    //   //// for push recovery:lamdax, y >= 0.4;beyond the limitation: not used
    //       _Rthetax = 100;                        _Rthetay = 100;           //acceleration
    //   _alphathetax = 10;                     _alphathetay = 10;       //velocity
    //   _beltathetax = 5000000000;              _beltathetay = 5000000000; //position 
    //     _gama_zmpx = 5000;                      _gama_zmpy= 5000;     ////ZMP minimization: _gama_zmp should be smaller to reduce bodythetax    
    // }
    // else if (_robot_name == "cogimon"){
    //   _Rthetax = 1;      _Rthetay = 1;          //acceleration
    //   _alphathetax =1;   _alphathetay = 1;          //velocity
    //   _beltathetax = 10; _beltathetay = 10;         //position
    //   _gama_zmpx = 100000;      _gama_zmpy= 100000;   ////ZMP minimization
    // } 



   

// ///// next code just run once	
    A_unit.setIdentity(_nh,_nh);	
    
// /////////// initialize each variable
    _t_f.setZero();     ///predictive window time-period
    _bjx1 = 0;
    _bjx2 = 0;
    
    // optimization objective function 	
    _WthetaX.setZero();
    _WthetaY.setZero();
    _Q_goal.setZero();
    _q_goal.setZero();
    
    _det_px_ref.setZero();
    _det_py_ref.setZero();

    // constraints
    _Sjthetax.setZero();
    _Sjthetay.setZero();
    
    _Sjthetax.block<_nh, _nh>(0, 0) = A_unit;
    _Sjthetay.block<_nh, _nh>(0, _nh) = A_unit;

     // zmp boundary preparation
		_z_upx.setZero();
    _z_lowx.setZero();
    _z_upy.setZero();
    _z_lowy.setZero();
		_zz_upx.setZero();
    _zz_lowx.setZero();
    _zz_upy.setZero();
    _zz_lowy.setZero();


    // angle boundary preparation
    _q_upx.setZero();
    _qq_upx.setZero();
    _q_lowx.setZero();
    _qq_lowx.setZero();
    _q_upy.setZero();
    _qq_upy.setZero();
    _q_lowy.setZero();
    _qq_lowy.setZero();
  

    // torque bondary preparation
    _t_upx.setZero();
    _tt_upx.setZero();
    _t_lowx.setZero();
    _tt_lowx.setZero();
    _t_upy.setZero();
    _tt_upy.setZero();
    _t_lowy.setZero();
    _tt_lowy.setZero();

    ///QP initiallize
    int nVars = _Nt;
    int nEqCon = 0;
    // int nIneqCon = 8*_nh;  
    int nIneqCon = 12*_nh;
    resizeQP(nVars, nEqCon, nIneqCon);	

  
    
    //////polynomial intepolation for lower level interpolation
    _AAA_inv.setZero();
    solve_AAA_inv1();
    
    _AAA_inv_mod.setZero();
    solve_AAA_inv_mod1();
    
    qp_solution = true;

       
}


/////////////////////////////////////////////////////////////============================================================================================
/////////////////////// local coordinate CoM solution---modified---------------------------------
Eigen::Matrix<double, 14, 1> PRMPCClass::body_theta_mpc(int i, Eigen::Matrix<double,4,1> bodyangle_state, Eigen::Matrix<double,2,5> zmp_mpc_refx, 
                          Eigen::Matrix<double,2,5> bodyangle_mpc_refx, Eigen::Matrix<double,2,5> rfoot_mpc_refx, Eigen::Matrix<double,2,5> lfoot_mpc_refx,
                          Eigen::Matrix<double,3,5> comacc_mpc_refx, Eigen::Matrix<double, 9, 1> Nrtfoorpr_gen)
{
  
  Eigen::Matrix<double,2,_nh> zmp_mpc_ref = zmp_mpc_refx.block<2, _nh>(0, 0);
  Eigen::Matrix<double,2,_nh> bodyangle_mpc_ref = bodyangle_mpc_refx.block<2, _nh>(0, 0);
  Eigen::Matrix<double,2,_nh> rfoot_mpc_ref = rfoot_mpc_refx.block<2, _nh>(0, 0);
  Eigen::Matrix<double,2,_nh> lfoot_mpc_ref = lfoot_mpc_refx.block<2, _nh>(0, 0);
  Eigen::Matrix<double,3,_nh> comacc_mpc_ref = comacc_mpc_refx.block<3, _nh>(0, 0);
  
  
  Eigen::Matrix<double, 14, 1> com_traj;
  com_traj.setZero();


  if (i < round(_height_offset_time /_dt_mpc))
  {
     com_traj.setZero();   
  }
  else
  {
    i -= (int) round(_height_offset_time /_dt_mpc);
    
    if (i < (_nsum_mpc - _nh))
    {
    
      _t_f.setLinSpaced(_nh,(i+1)*_dt_mpc, (i+_nh)*_dt_mpc);
      _j_period = 0;

      Indexfind(_t_f(0),xyz1);                /// step cycle number when (i+1)*dt fall into : current sampling time
      _bjx1 = _j_period+1;
      _j_period = 0;
      
      Indexfind(_t_f(_nh-1),xyz1);           /// step cycle number when (i+_nh)*dt fall into : current sampling time
      _bjx2 = _j_period+1;
      _j_period = 0;   

      int t_yu  = ((i+1) % _nstepx);   /////
      


      _zmpx_max_vec.setConstant(_zmpx_max); 
      _zmpx_min_vec.setConstant(_zmpx_min);
      _zmpy_max_vec.setConstant(_zmpy_max);  
      _zmpy_min_vec.setConstant(_zmpy_min);	

      ///// CoP reference========
      if (_bjx1 <2)  ///left support
      {
        _copx_center_ref = (lfoot_mpc_ref.row(0)).transpose();
        _copy_center_ref = (lfoot_mpc_ref.row(1)).transpose();      
      }
      else
      {
        if (_bjx1 % 2 == 0) //left support
        {
          if (((t_yu + _nh -1) < _nstepx))  ///no support switch 
          {
            _copx_center_ref = (lfoot_mpc_ref.row(0)).transpose();
            _copy_center_ref = (lfoot_mpc_ref.row(1)).transpose();  

          }
          else
          {
            int t_yu_k = (t_yu + _nh ) - _nstepx;
            // cout << "t_yu_k1:"<< t_yu_k<<endl;
            // cout << "t_yu1:"<< t_yu<<endl;
            _copx_center_ref = (lfoot_mpc_ref.row(0)).transpose();
            _copy_center_ref = (lfoot_mpc_ref.row(1)).transpose();

            for (int jx=1; jx <= t_yu_k; jx ++)
            {
              _copx_center_ref(_nh - jx, 0) = rfoot_mpc_ref(0, _nh - jx);
              _copy_center_ref(_nh - jx, 0) = rfoot_mpc_ref(1, _nh - jx);

              _zmpx_max_vec(_nh - jx, 0) = max(rfoot_mpc_ref(0, _nh - jx), lfoot_mpc_ref(0, _nh - jx))  -  rfoot_mpc_ref(0, _nh - jx) + _zmpx_max; 
              _zmpx_min_vec(_nh - jx, 0) = min(rfoot_mpc_ref(0, _nh - jx), lfoot_mpc_ref(0, _nh - jx))  -  rfoot_mpc_ref(0, _nh - jx) + _zmpx_min;
              _zmpy_max_vec(_nh - jx, 0) = max(rfoot_mpc_ref(1, _nh - jx), lfoot_mpc_ref(1, _nh - jx))  -  rfoot_mpc_ref(1, _nh - jx) + _zmpy_max;  
              _zmpy_min_vec(_nh - jx, 0) = min(rfoot_mpc_ref(1, _nh - jx), lfoot_mpc_ref(1, _nh - jx))  -  rfoot_mpc_ref(1, _nh - jx) + _zmpy_min;

            }
            // cout << _copx_center_ref.transpose() <<endl;
            // cout << _copy_center_ref.transpose() <<endl;        


          }   
        }
        else           /// right support
        {
          if (((t_yu + _nh -1) < _nstepx))  ///no support switch 
          {
            _copx_center_ref = (rfoot_mpc_ref.row(0)).transpose();
            _copy_center_ref = (rfoot_mpc_ref.row(1)).transpose();  

          }
          else
          {
            int t_yu_k = (t_yu + _nh ) - _nstepx;
            // cout << "t_yu_k2:"<< t_yu_k<<endl;
            // cout << "t_yu2:"<< t_yu<<endl;                       
            _copx_center_ref = (rfoot_mpc_ref.row(0)).transpose();
            _copy_center_ref = (rfoot_mpc_ref.row(1)).transpose();

            for (int jx=1; jx <= t_yu_k; jx ++)
            {
              _copx_center_ref(_nh - jx, 0) = lfoot_mpc_ref(0, _nh - jx);
              _copy_center_ref(_nh - jx, 0) = lfoot_mpc_ref(1, _nh - jx);

              _zmpx_max_vec(_nh - jx, 0) = max(rfoot_mpc_ref(0, _nh - jx), lfoot_mpc_ref(0, _nh - jx))  -  lfoot_mpc_ref(0, _nh - jx) + _zmpx_max; 
              _zmpx_min_vec(_nh - jx, 0) = min(rfoot_mpc_ref(0, _nh - jx), lfoot_mpc_ref(0, _nh - jx))  -  lfoot_mpc_ref(0, _nh - jx) + _zmpx_min;
              _zmpy_max_vec(_nh - jx, 0) = max(rfoot_mpc_ref(1, _nh - jx), lfoot_mpc_ref(1, _nh - jx))  -  lfoot_mpc_ref(1, _nh - jx) + _zmpy_max;  
              _zmpy_min_vec(_nh - jx, 0) = min(rfoot_mpc_ref(1, _nh - jx), lfoot_mpc_ref(1, _nh - jx))  -  lfoot_mpc_ref(1, _nh - jx) + _zmpy_min;

            }
            // cout << _copx_center_ref.transpose() <<endl;
            // cout << _copy_center_ref.transpose() <<endl;
          }        
        }
        
      }

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
      ///	================ iterative calulcation: predictive control_tracking with time-varying height+angular momentum	   

      for (int jx=0; jx<_nh; jx ++)
      {
         _pthetax(jx,jx) = _j_ini/(_mass * (comacc_mpc_ref(2,jx) + _ggg(0)));  
        //_pthetax(jx,jx) = _j_ini/(_mass * (_ggg(0)));     
      }
      _pthetay  = -_pthetax; 

      _WthetaX = _Rthetax/2 * A_unit + _alphathetax/2 * _pvu_2 + _beltathetax/2 * _ppu_2 + _gama_zmpy/2 *(_pthetax * _pthetax.transpose());
      _WthetaY = _Rthetay/2 * A_unit + _alphathetay/2 * _pvu_2 + _beltathetay/2 * _ppu_2 + _gama_zmpx/2 *(_pthetay * _pthetay.transpose());

      _Q_goal.block<_nh, _nh>(0, 0) = 2 *_WthetaX;
      _Q_goal.block<_nh, _nh>(_nh, _nh) = 2 *_WthetaY;

      _det_px_ref =  (zmp_mpc_ref.row(0)).transpose() - _copx_center_ref;
      _det_py_ref =  (zmp_mpc_ref.row(1)).transpose() - _copy_center_ref;

      //bodyangle_mpc_ref.setZero();

       //QP MOdels	 
      _q_goal.block<_nh, 1>(0, 0)   = _alphathetax * _pvu.transpose() * _pvs * _thetaxk + _beltathetax * _ppu.transpose() * _pps * _thetaxk 
                                    - _beltathetax * _ppu.transpose() * (bodyangle_mpc_ref.row(0)).transpose() +  _gama_zmpy * _pthetax.transpose() * _det_py_ref;
      _q_goal.block<_nh, 1>(_nh, 0) = _alphathetay * _pvu.transpose() * _pvs * _thetayk + _beltathetay * _ppu.transpose() * _pps * _thetayk 
                                    - _beltathetay * _ppu.transpose() * (bodyangle_mpc_ref.row(1)).transpose() +  _gama_zmpx * _pthetay.transpose() * _det_px_ref;
      ///// the following code only run once in each loop 
      // zmp boundary preparation
      _z_upx = _pthetax* _Sjthetax;
      _zz_upx = _zmpy_max_vec - _det_py_ref;
      _z_lowx = - _z_upx;
      _zz_lowx = -_zmpy_min_vec + _det_py_ref;

      _z_upy = _pthetay* _Sjthetay;
      _zz_upy = _zmpx_max_vec - _det_px_ref;
      _z_lowy = - _z_upy;
      _zz_lowy = -_zmpx_min_vec + _det_px_ref;



      // angle boundary preparation
      _q_upx = _ppu* _Sjthetax;
      _qq_upx = _thetax_max_vec - _pps * _thetaxk;
      _q_lowx = - _q_upx;
      _qq_lowx = -_thetax_min_vec + _pps * _thetaxk;

      _q_upy = _ppu* _Sjthetay;
      _qq_upy = _thetay_max_vec - _pps * _thetayk;
      _q_lowy = - _q_upy;
      _qq_lowy = -_thetay_min_vec + _pps * _thetayk;

      // torque bondary preparation
      _t_upx = _j_ini * _Sjthetax;
      _tt_upx = _torquex_max_vec;
      _t_lowx = - _t_upx;
      _tt_lowx = - _torquex_min_vec;
      
      _t_upy = _j_ini * _Sjthetay;
      _tt_upy = _torquey_max_vec;
      _t_lowy = - _t_upy;
      _tt_lowy = - _torquey_min_vec;  
      ////////////////////////////////////////////////////QP solution ////////////    
      solve_body_rotation(); 
        
      /////////===========results postprocessed:===========================%%%%%	
      /////////===========results postprocessed:===========================%%%%%	  
      _thetaax.col(0) = _V_ini.row(0);
      _thetaay.col(0) = _V_ini.row(_nh); 
      
      if (!qp_solution)
      {
        // using the current reference state
        _thetaax(0,0) = (_thetaxk(0,0) - _a.row(0) * _thetaxk)/_b(0,0); 

        
        _thetaay(0,0) = (_thetayk(0,0) - _a.row(0) * _thetayk)/_b(0,0);     
        cout << "fast_qp_solution_failed, using the current state"<<endl;
          
      }
      else
      {
        // using the boundary states
        Eigen::Vector2d _thetaxk_new = _a * _thetaxk  + _b* _thetaax(0,0);
        if (_thetaxk_new(0) > _thetax_max)
        {
            _thetaax(0,0) = (_thetax_max - _a.row(0) * _thetaxk)/_b(0,0); 
            cout << "roll angle larger than upper bound"<<endl;
        }
        else
        {
            if (_thetaxk_new(0) < _thetax_min)
            {
                _thetaax(0,0) = (_thetax_min - _a.row(0) * _thetaxk)/_b(0,0); 
                cout << "roll angle samller than lower bound"<<endl;
            }
            
        }

        
        // cout<<  _thetaay.col(0)<<endl;
        Eigen::Vector2d _thetayk_new = _a * _thetayk  + _b* _thetaay(0,0);
        if (_thetayk_new(0) > _thetay_max)
        {
            _thetaay(0,0) = (_thetay_max - _a.row(0) * _thetayk)/_b(0,0); 
            cout << "pitch angle larger than upper bound"<<endl;
        }
        else
        {
            if (_thetayk_new(0) < _thetay_min)
            {
                _thetaay(0,0) = (_thetay_min - _a.row(0) * _thetayk)/_b(0,0); 
                cout << "pitch angle samller than lower bound"<<endl;
            }
            
        }          
          
      }
      
        
      
      _V_ini.row(0) = _thetaax.col(0);
      _V_ini.row(_nh) = _thetaay.col(0); 

      _thetaax.col(0) =  _V_ini.row(0);
      _thetaay.col(0) =  _V_ini.row(_nh+0);
    
      Vector2d _thetaxk_temp, _thetayk_temp;
    
      _thetaxk_temp = _a * _thetaxk  + _b* _thetaax(0,0);
      _thetayk_temp = _a * _thetayk  + _b* _thetaay(0,0);      
      
    ////////////////===============================================================================================	  
    _torquex_real.col(0) = _j_ini * _thetaax.col(0);
    _torquey_real.col(0) = _j_ini * _thetaay.col(0);    
    
      for (int jj = 0; jj<_nsum; jj ++)
      {
        _thetaax.col(jj) =  _V_ini.row(jj);
        _thetaay.col(jj) =  _V_ini.row(_nh+jj);
        
        _thetaxk = _a * _thetaxk  + _b* _thetaax(0,jj);
        _thetax.col(jj)= _thetaxk.row(0); 
        _thetavx.col(jj) = _thetaxk.row(1);
        
        _thetayk = _a * _thetayk  + _b* _thetaay(0,jj);
        _thetay.col(jj)= _thetayk.row(0); 
        _thetavy.col(jj) = _thetayk.row(1);		 


        
        _zmpx_real(0,jj) = zmp_mpc_ref(0,jj) - _j_ini * _thetaay(0,jj)/(_mass * (_ggg(0) + comacc_mpc_ref(2,jj)));
        _zmpy_real(0,jj) = zmp_mpc_ref(1,jj) + _j_ini * _thetaax(0,jj)/(_mass * (_ggg(0) + comacc_mpc_ref(2,jj))); 
        
                
      }

      /////reference relative state: for closed-loop    
        
    _thetaxk = _thetaxk_temp; 
    _thetayk = _thetayk_temp;
      ////============================================================================================================================	      
      //////////////////////////// state modified:====================================================================================
      /////////////================state feedback: determined by ratio parameter: lamda==============================////
        double lamdax, lamday,lamdaz,lamdavx,lamdavy,lamdavz;
 
        // limitation test for lamda: :
        lamdax = 0.1;  
        lamdavx = 0.001;    
        lamday = 0.1;
        lamdavy = 0.001;
        lamdaz = 0;
        lamdavz = 0;  
	
        // // For stepping+hip+height variation/// multi-directional push recovery
        // lamdax = 0.2;
        // lamdavx = 0.001;
        // lamday = 0.2;
        // lamdavy = 0.001;
        // lamdaz = 0;
        // lamdavz = 0;     

      _thetaxk(0,0) = lamdax*bodyangle_state(0,0)+(1-lamdax)*_thetaxk(0,0);             
      _thetaxk(1,0) = lamdavx*bodyangle_state(1,0)+(1-lamdavx)*_thetaxk(1,0); 	
      _thetayk(0,0) = (lamday*bodyangle_state(2,0)+(1-lamday)*_thetayk(0,0));
      _thetayk(1,0) = (lamdavy*bodyangle_state(3,0)+(1-lamdavy)*_thetayk(1,0));	        
    }
  }

  com_traj(0) = _thetax(0,0);
  com_traj(1) = _thetay(0,0);
  com_traj(2) = _torquex_real(0,0);
  com_traj(3) = _torquey_real(0,0);
  com_traj(4) = _zmpx_real(0,0);
  com_traj(5) = _zmpy_real(0,0);
  com_traj(6) = _thetax(0,1);
  com_traj(7) = _thetay(0,1);
  com_traj(8) = _zmpx_real(0,1);
  com_traj(9) = _zmpy_real(0,1);  
  com_traj(10) = _thetax(0,2);
  com_traj(11) = _thetay(0,2);
  com_traj(12) = _zmpx_real(0,2);
  com_traj(13) = _zmpy_real(0,2);    
  

  return com_traj;  

}
//////////////////////////// modified
void PRMPCClass::Indexfind(double goalvari, int xyz)
{
  _j_period = 0;
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

///// only walking once when initialize
Eigen::MatrixXd  PRMPCClass::Matrix_ps(Eigen::Matrix<double,2,2> a, int nh,Eigen::RowVector2d cxps)
{
//   Eigen::MatrixXd matrixps(nh,3);
  Eigen::MatrixXd matrixps;
  matrixps.setZero(nh,2);  
  
  
  
  Eigen::MatrixXd A;
//   A.setIdentity(a.rows(),a.cols());
  
  for (int i = 0; i < nh; i++) {
    A.setIdentity(a.rows(),a.cols());
    for (int j = 1; j < i+2; j++)
    {
      A = A*a;
    }  
    
     matrixps.middleRows(i, 1)= cxps * A;      
  }
    
  return matrixps;
}

Eigen::MatrixXd PRMPCClass::Matrix_pu(Eigen::Matrix<double,2,2> a, Eigen::Matrix<double,2,1> b, int nh, Eigen::RowVector2d cxpu)
{
  Eigen::MatrixXd matrixpu;
  matrixpu.setZero(nh,nh);
  
  Eigen::MatrixXd A;
  Eigen::MatrixXd Tempxx;
  
  
  for (int i = 1; i < nh+1; i++) {
    for (int j = 1; j < i+1; j++)
    { 
      A.setIdentity(a.rows(),a.cols());      
      if (j==i)
      {
	Tempxx = cxpu * A * b;
	matrixpu(i-1,j-1) = Tempxx(0,0);
      }
      else
      {	
	for (int k = 1; k < i-j+1; k++)
	{
	  A = A*a;
	}
	Tempxx = cxpu * A * b;
	matrixpu(i-1,j-1) = Tempxx(0,0);
      }          
    }       
  }
    
  return matrixpu;  
}

///// three model MPC solution :modified================================================================
void PRMPCClass::solve_body_rotation()
{
  _G = _Q_goal;
  _g0 = _q_goal;
  _X = _V_ini;

  _CI.block<_Nt,_nh>(0,0) = _q_upx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,_nh) = _q_lowx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,2*_nh) = _q_upy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,3*_nh) = _q_lowy.transpose() * (-1); 
  _CI.block<_Nt,_nh>(0,4*_nh) = _t_upx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,5*_nh) = _t_lowx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,6*_nh) = _t_upy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,7*_nh) = _t_lowy.transpose() * (-1);
//   _CI.block<_Nt,_nh>(0,8*_nh) = _z_upx.transpose() * (-1);  ///results non-smooth ZMPy
//   _CI.block<_Nt,_nh>(0,9*_nh) = _z_lowx.transpose() * (-1);
//   _CI.block<_Nt,_nh>(0,10*_nh) = _z_upy.transpose() * (-1);
//   _CI.block<_Nt,_nh>(0,11*_nh) = _z_lowy.transpose() * (-1);  

  _ci0.block(0, 0,_nh,1) = _qq_upx;
  _ci0.block(_nh, 0,_nh,1) = _qq_lowx;
  _ci0.block(2*_nh, 0,_nh,1) = _qq_upy;
  _ci0.block(3*_nh, 0,_nh,1) = _qq_lowy;
  _ci0.block(4*_nh, 0,_nh,1) = _tt_upx; 
  _ci0.block(5*_nh, 0,_nh,1) = _tt_lowx; 
  _ci0.block(6*_nh, 0,_nh,1) = _tt_upy;
  _ci0.block(7*_nh, 0,_nh,1) = _tt_lowy;  
/*  _ci0.block(8*_nh, 0,_nh,1) = _zz_upx; 
  _ci0.block(9*_nh, 0,_nh,1) = _zz_lowx; 
  _ci0.block(10*_nh, 0,_nh,1) = _zz_upy;
  _ci0.block(11*_nh, 0,_nh,1) = _zz_lowy; */ 



  Solve();  

}

void PRMPCClass::Solve()
{
// min 0.5 * x G x + g0 x
// _s.t.
// 		CE^T x + ce0 = 0
// 		CI^T x + ci0 >= 0
		qp_solution = solveQP();
		if (qp_solution)
		{
		  _V_ini = _X;
		}

}

////====================================================================================================================
/////////////////////////// trajectory intepolation:  just using the next one position + next one velocity
Eigen::Matrix<double, 9, 1>  PRMPCClass::XGetSolution_position(const int walktime, const double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_ref, Eigen::Vector3d bodyv_ref)
{
  //reference com position
	Eigen::Matrix<double, 9, 1> com_inte;
        com_inte.setZero();	
	
	if (walktime>=2)
	{
	  int t_int= floor(walktime / (_dt / dt_sample) );

	  ///// chage to be relative time
	  double t_cur = walktime * dt_sample ;
	  
	  Eigen::Matrix<double, 4, 1> t_plan;
	  t_plan.setZero();
	  t_plan(0) = 0;
	  t_plan(1) = dt_sample;
	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);
	  
	  Eigen::Matrix<double, 3, 3> A3_INVE = solve_AAA_inv(t_plan);
	  	  
	  Eigen::Matrix<double, 1, 4> t_a_plan;
	  t_a_plan.setZero();
	  t_a_plan(0) = pow(2*dt_sample, 3);   t_a_plan(1) = pow(2*dt_sample, 2);   
	  t_a_plan(2) = pow(2*dt_sample, 1);   t_a_plan(3) = pow(2*dt_sample, 0); 
	  
	  Eigen::Matrix<double, 1, 4> t_a_planv;
	  t_a_planv.setZero();
	  t_a_planv(0) = 3*pow(2*dt_sample, 2);   t_a_planv(1) = 2*pow(2*dt_sample, 1);   
	  t_a_planv(2) = 1;   t_a_planv(3) = 0; 	  
	  
	  Eigen::Matrix<double, 1, 4> t_a_plana;
	  t_a_plana.setZero();
	  t_a_plana(0) = 6*pow(2*dt_sample, 1);   t_a_plana(1) = 2;   
	  t_a_plana(2) = 0;   t_a_plana(3) = 0;	  
	  	  
	  
	  // COM&&foot trajectory interpolation	   	  
	  Eigen::Matrix<double, 4, 1>  temp;
	  temp.setZero();
	  temp(0) = body_in1(0); temp(1) = body_in2(0); temp(2) = body_ref(0); temp(3) = bodyv_ref(0);	  
	  com_inte(0) = t_a_plan * (_AAA_inv)*temp;
	  com_inte(3) = t_a_planv * (_AAA_inv)*temp;
	  com_inte(6) = t_a_plana * (_AAA_inv)*temp;
	  
	  temp.setZero();
	  temp(0) = body_in1(1); temp(1) = body_in2(1); temp(2) = body_ref(1); temp(3) = bodyv_ref(1);	  
	  com_inte(1) = t_a_plan * (_AAA_inv)*temp;
	  com_inte(4) = t_a_planv * (_AAA_inv)*temp;
	  com_inte(7) = t_a_plana * (_AAA_inv)*temp;	  
	  
          temp.setZero();
	  temp(0) = body_in1(2); temp(1) = body_in2(2); temp(2) = body_ref(2); temp(3) = bodyv_ref(2);	  
	  com_inte(2) = t_a_plan *(_AAA_inv)*temp;
	  com_inte(5) = t_a_planv * (_AAA_inv)*temp;
	  com_inte(8) = t_a_plana * (_AAA_inv)*temp;	 
	  
	  // /////be careful, the polynomial may cause overfitting
	  // double t_des = t_cur-t_int*_dt;
	  // if (t_des<=0){
	  //   t_des =0.00001;
	  // }
	    
	  // if (t_int>=1)
	  // {
	  //   _comaxyzx(0) = (_comax(t_int)-_comax(t_int-1))/_dt*t_des+_comax(t_int-1);
	  //   _comaxyzx(1) = (_comay(t_int)-_comay(t_int-1))/_dt*t_des+_comay(t_int-1);
	  //   _comaxyzx(2) = (_comaz(t_int)-_comaz(t_int-1))/_dt*t_des+_comaz(t_int-1);
	  // }
	  // else
	  // {
	  //   _comaxyzx(0) = (_comax(t_int)-0)/_dt*t_des+0;
	  //   _comaxyzx(1) = (_comay(t_int)-0)/_dt*t_des+0;
	  //   _comaxyzx(2) = (_comaz(t_int)-0)/_dt*t_des+0;	    
	  // } 
	}
	else
	{
	  com_inte(0) = body_ref(0);
	  com_inte(1) = body_ref(1);
	  com_inte(2) = body_ref(2);	
	}

 	return com_inte;
	
}

Eigen::Matrix<double, 9, 1>  PRMPCClass::XGetSolution_position_mod(const int walktime, const double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_ref, Eigen::Vector3d body_ref2)
{
  //reference com position
	Eigen::Matrix<double, 9, 1> com_inte;
        com_inte.setZero();	
	
	if (walktime>=2)
	{
	  int t_int= floor(walktime / (_dt / dt_sample) );

	  ///// chage to be relative time
	  double t_cur = walktime * dt_sample ;
	  
	  Eigen::Matrix<double, 4, 1> t_plan;
	  t_plan.setZero();
	  t_plan(0) = 0;
	  t_plan(1) = dt_sample;
	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);
	  
	  Eigen::Matrix<double, 3, 3> A3_INVE = solve_AAA_inv_mod(t_plan);
	  	  
	  Eigen::Matrix<double, 1, 4> t_a_plan;
	  t_a_plan.setZero();
	  t_a_plan(0) = pow(2*dt_sample, 3);   t_a_plan(1) = pow(2*dt_sample, 2);   
	  t_a_plan(2) = pow(2*dt_sample, 1);   t_a_plan(3) = pow(2*dt_sample, 0); 
	  
	  Eigen::Matrix<double, 1, 4> t_a_planv;
	  t_a_planv.setZero();
	  t_a_planv(0) = 3*pow(2*dt_sample, 2);   t_a_planv(1) = 2*pow(2*dt_sample, 1);   
	  t_a_planv(2) = 1;   t_a_planv(3) = 0; 	  
	  
	  Eigen::Matrix<double, 1, 4> t_a_plana;
	  t_a_plana.setZero();
	  t_a_plana(0) = 6*pow(2*dt_sample, 1);   t_a_plana(1) = 2;   
	  t_a_plana(2) = 0;   t_a_plana(3) = 0;	  
	  	  
	  
	  // COM&&foot trajectory interpolation	   	  
	  Eigen::Matrix<double, 4, 1>  temp;
	  temp.setZero();
	  temp(0) = body_in1(0); temp(1) = body_in2(0); temp(2) = body_ref(0); temp(3) = body_ref2(0);	  
	  com_inte(0) = t_a_plan * (_AAA_inv_mod)*temp;
	  com_inte(3) = t_a_planv * (_AAA_inv_mod)*temp;
	  com_inte(6) = t_a_plana * (_AAA_inv_mod)*temp;
	  
	  temp.setZero();
	  temp(0) = body_in1(1); temp(1) = body_in2(1); temp(2) = body_ref(1); temp(3) = body_ref2(1);	  
	  com_inte(1) = t_a_plan * (_AAA_inv_mod)*temp;
	  com_inte(4) = t_a_planv * (_AAA_inv_mod)*temp;
	  com_inte(7) = t_a_plana * (_AAA_inv_mod)*temp;	  
	  
          temp.setZero();
	  temp(0) = body_in1(2); temp(1) = body_in2(2); temp(2) = body_ref(2); temp(3) = body_ref2(2);	  
	  com_inte(2) = t_a_plan *(_AAA_inv_mod)*temp;
	  com_inte(5) = t_a_planv * (_AAA_inv_mod)*temp;
	  com_inte(8) = t_a_plana * (_AAA_inv_mod)*temp;	 
	  
	  // /////be careful, the polynomial may cause overfitting
	  // double t_des = t_cur-t_int*_dt;
	  // if (t_des<=0){
	  //   t_des =0.00001;
	  // }
	    
	  // if (t_int>=1)
	  // {
	  //   _comaxyzx(0) = (_comax(t_int)-_comax(t_int-1))/_dt*t_des+_comax(t_int-1);
	  //   _comaxyzx(1) = (_comay(t_int)-_comay(t_int-1))/_dt*t_des+_comay(t_int-1);
	  //   _comaxyzx(2) = (_comaz(t_int)-_comaz(t_int-1))/_dt*t_des+_comaz(t_int-1);
	  // }
	  // else
	  // {
	  //   _comaxyzx(0) = (_comax(t_int)-0)/_dt*t_des+0;
	  //   _comaxyzx(1) = (_comay(t_int)-0)/_dt*t_des+0;
	  //   _comaxyzx(2) = (_comaz(t_int)-0)/_dt*t_des+0;	    
	  // } 
	}
	else
	{
	  com_inte(0) = body_ref(0);
	  com_inte(1) = body_ref(1);
	  com_inte(2) = body_ref(2);	
	}

 	return com_inte;
	
}

Eigen::Matrix<double, 21, 1>  PRMPCClass::XGetSolution_position_mod1(const int walktime, const double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_ref, Eigen::Vector3d bodyv_ref)
{
  
  Eigen::Matrix<double, 21, 1> com_inte;
  com_inte.setZero();	
	
  //reference com position
  for (int jx =0;jx<5;jx++)
  {

    ///// chage to be relative time
    double t_cur = (walktime * dt_sample + jx *dt_sample) ;
    
// 	  Eigen::Matrix<double, 4, 1> t_plan;
// 	  t_plan.setZero();
// 	  t_plan(0) = - _dt;
// 	  t_plan(1) = 0;
// 	  t_plan(2) = _dt + 0.001;
// 	  t_plan(3) = 2*_dt;
// 	  
// 	  Eigen::Matrix<double, 3, 3> A3_INVE = solve_AAA_inv(t_plan);
	    
    Eigen::Matrix<double, 1, 4> t_a_plan;
    t_a_plan.setZero();
    t_a_plan(0) = pow(t_cur, 3);   t_a_plan(1) = pow(t_cur, 2);   
    t_a_plan(2) = pow(t_cur, 1);   t_a_plan(3) = pow(t_cur, 0); 
    
    Eigen::Matrix<double, 1, 4> t_a_planv;
    t_a_planv.setZero();
    t_a_planv(0) = 3*pow(t_cur, 2);   t_a_planv(1) = 2*pow(t_cur, 1);   
    t_a_planv(2) = 1;   t_a_planv(3) = 0; 	  
    
    Eigen::Matrix<double, 1, 4> t_a_plana;
    t_a_plana.setZero();
    t_a_plana(0) = 6*pow(t_cur, 1);   t_a_plana(1) = 2;   
    t_a_plana(2) = 0;   t_a_plana(3) = 0;	  
	    
    
    // COM&&foot trajectory interpolation	   	  
    Eigen::Matrix<double, 4, 1>  temp;
    temp.setZero();
    temp(0) = body_in1(0); temp(1) = body_in2(0); temp(2) = body_ref(0); temp(3) = bodyv_ref(0);

    if (jx==0)
    {
      com_inte(0) = t_a_plan * (_AAA_inv_mod)*temp;
      com_inte(3) = t_a_planv * (_AAA_inv_mod)*temp;
      com_inte(6) = t_a_plana * (_AAA_inv_mod)*temp;
    }
    else
    {
      com_inte(8+3*jx-2) = t_a_plan * (_AAA_inv_mod)*temp;     
    }	  
// 	  com_inte(0) = t_a_plan * (_AAA_inv)*temp;
// 	  com_inte(3) = t_a_planv * (_AAA_inv)*temp;
// 	  com_inte(6) = t_a_plana * (_AAA_inv)*temp;
    
    temp.setZero();
    temp(0) = body_in1(1); temp(1) = body_in2(1); temp(2) = body_ref(1); temp(3) = bodyv_ref(1);	  
/*	  com_inte(1) = t_a_plan * (_AAA_inv)*temp;
    com_inte(4) = t_a_planv * (_AAA_inv)*temp;
    com_inte(7) = t_a_plana * (_AAA_inv)*temp;*/	  
    if (jx==0)	 
    {
      com_inte(1) = t_a_plan * (_AAA_inv_mod)*temp;
      com_inte(4) = t_a_planv * (_AAA_inv_mod)*temp;
      com_inte(7) = t_a_plana * (_AAA_inv_mod)*temp;	
    }
    else
    {
      com_inte(8+3*jx-1) = t_a_plan * (_AAA_inv_mod)*temp;   
    }
    
    
    temp.setZero();
    temp(0) = body_in1(2); temp(1) = body_in2(2); temp(2) = body_ref(2); temp(3) = bodyv_ref(2);	  
/*	  com_inte(2) = t_a_plan *(_AAA_inv)*temp;
    com_inte(5) = t_a_planv * (_AAA_inv)*temp;
    com_inte(8) = t_a_plana * (_AAA_inv)*temp;	*/ 
    if (jx ==0)  
    {
      com_inte(2) = t_a_plan *(_AAA_inv_mod)*temp;
      com_inte(5) = t_a_planv * (_AAA_inv_mod)*temp;
      com_inte(8) = t_a_plana * (_AAA_inv_mod)*temp;
    }
    else
    {
      com_inte(8+3*jx) = t_a_plan * (_AAA_inv_mod)*temp;
    }
    
    // /////be careful, the polynomial may cause overfitting
    // double t_des = t_cur-t_int*_dt;
    // if (t_des<=0){
    //   t_des =0.00001;
    // }
      
    // if (t_int>=1)
    // {
    //   _comaxyzx(0) = (_comax(t_int)-_comax(t_int-1))/_dt*t_des+_comax(t_int-1);
    //   _comaxyzx(1) = (_comay(t_int)-_comay(t_int-1))/_dt*t_des+_comay(t_int-1);
    //   _comaxyzx(2) = (_comaz(t_int)-_comaz(t_int-1))/_dt*t_des+_comaz(t_int-1);
    // }
    // else
    // {
    //   _comaxyzx(0) = (_comax(t_int)-0)/_dt*t_des+0;
    //   _comaxyzx(1) = (_comay(t_int)-0)/_dt*t_des+0;
    //   _comaxyzx(2) = (_comaz(t_int)-0)/_dt*t_des+0;	    
    // } 
  }

  return com_inte;
	
}

Eigen::Matrix<double, 9, 1>  PRMPCClass::XGetSolution_position_mod2(const int walktime, const double dt_sample, Eigen::Vector3d body_in2, Eigen::Vector3d body_ref)
{
  //reference com position
	Eigen::Matrix<double, 9, 1> com_inte;
  com_inte.setZero();	
	
  ///// chage to be relative time
  double t_cur = (walktime * dt_sample) ; 

  com_inte(0) = (body_ref(0) - body_in2(0))/_dt*t_cur+ body_in2(0);
  com_inte(3) = (body_ref(0) - body_in2(0))/_dt;
  com_inte(6) = 0;

  com_inte(1) = (body_ref(1) - body_in2(1))/_dt*t_cur+ body_in2(1);
  com_inte(4) = (body_ref(1) - body_in2(1))/_dt;
  com_inte(7) = 0;

  com_inte(2) = (body_ref(2) - body_in2(2))/_dt*t_cur+ body_in2(2);
  com_inte(5) = (body_ref(2) - body_in2(2))/_dt;
  com_inte(8) = 0;  



  return com_inte;
	
}

Eigen::Matrix<double, 21, 1>  PRMPCClass::XGetSolution_position_mod3(const int walktime, const double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_ref, Eigen::Vector3d body_ref2)
{
  //reference com position
    Eigen::Matrix<double, 21, 1> com_inte;
    com_inte.setZero();	
	
//     Eigen::Matrix<double, 4, 1> t_plan;
//     t_plan.setZero();
//     t_plan(0) = - _dt;
//     t_plan(1) = 0;
//     t_plan(2) = _dt;
//     t_plan(3) = 2*_dt;
    
//    Eigen::Matrix<double, 3, 3> A3_INVE = solve_AAA_inv_mod(t_plan);
	
    
    int jx_max = _nh;
    
    if (walktime <= _t_end_footstep)
    {
      

    
      for (int jx =0;jx<jx_max;jx++)
      {
	double t_cur = (walktime * dt_sample + jx *dt_sample) ;

	Eigen::Matrix<double, 1, 4> t_a_plan;
	t_a_plan.setZero();
	t_a_plan(0) = pow(t_cur, 3);   t_a_plan(1) = pow(t_cur, 2);   
	t_a_plan(2) = pow(t_cur, 1);   t_a_plan(3) = pow(t_cur, 0); 
	
	Eigen::Matrix<double, 1, 4> t_a_planv;
	t_a_planv.setZero();
	t_a_planv(0) = 3*pow(t_cur, 2);   t_a_planv(1) = 2*pow(t_cur, 1);   
	t_a_planv(2) = 1;   t_a_planv(3) = 0; 	  
	
	Eigen::Matrix<double, 1, 4> t_a_plana;
	t_a_plana.setZero();
	t_a_plana(0) = 6*pow(t_cur, 1);   t_a_plana(1) = 2;   
	t_a_plana(2) = 0;   t_a_plana(3) = 0;	  
	    
	
	// COM&&foot trajectory interpolation	   	  
	Eigen::Matrix<double, 4, 1>  temp;
	temp.setZero();
	temp(0) = body_in1(0); temp(1) = body_in2(0); temp(2) = body_ref(0); temp(3) = body_ref2(0);	  
	if (jx==0)
	{
	  com_inte(0) = t_a_plan * (_AAA_inv_mod)*temp;
	  com_inte(3) = t_a_planv * (_AAA_inv_mod)*temp;
	  com_inte(6) = t_a_plana * (_AAA_inv_mod)*temp;
	}
	else
	{
	  com_inte(8+3*jx-2) = t_a_plan * (_AAA_inv_mod)*temp;     
	}

	
	temp.setZero();
	temp(0) = body_in1(1); temp(1) = body_in2(1); temp(2) = body_ref(1); temp(3) = body_ref2(1);
	if (jx==0)	 
	{
	  com_inte(1) = t_a_plan * (_AAA_inv_mod)*temp;
	  com_inte(4) = t_a_planv * (_AAA_inv_mod)*temp;
	  com_inte(7) = t_a_plana * (_AAA_inv_mod)*temp;	
	}
	else
	{
	  com_inte(8+3*jx-1) = t_a_plan * (_AAA_inv_mod)*temp;   
	} 
    
	
	temp.setZero();
	temp(0) = body_in1(2); temp(1) = body_in2(2); temp(2) = body_ref(2); temp(3) = body_ref2(2);	
	if (jx ==0)  
	{
	  com_inte(2) = t_a_plan *(_AAA_inv_mod)*temp;
	  com_inte(5) = t_a_planv * (_AAA_inv_mod)*temp;
	  com_inte(8) = t_a_plana * (_AAA_inv_mod)*temp;
	}
	else
	{
	  com_inte(8+3*jx) = t_a_plan * (_AAA_inv_mod)*temp;
	}

      }
    }

    return com_inte;
	
}

////solve the inverse matrix of 4*4 coefficient matrices
Eigen::Matrix<double, 3, 3> PRMPCClass::solve_AAA_inv(Eigen::Matrix<double, 4, 1> t_plan)
{
  Eigen::Matrix<double,4,4> AAA1; 
  AAA1 << pow(t_plan(0), 3), pow(t_plan(0), 2), pow(t_plan(0), 1), 1, 
	        pow(t_plan(1), 3), pow(t_plan(1), 2), pow(t_plan(1), 1), 1, 
	        pow(t_plan(2), 3), pow(t_plan(2), 2), pow(t_plan(2), 1), 1,
	        3*pow(t_plan(2), 2), 2*pow(t_plan(2), 1), pow(t_plan(2), 0), 0;

  _AAA_inv = AAA1.inverse();

  Eigen::Matrix<double,3,3> AAA3,_AAA33_inv; 
  AAA3 << pow(t_plan(0), 2), pow(t_plan(0), 1), 1, 
	        pow(t_plan(1), 2), pow(t_plan(1), 1), 1, 
	        pow(t_plan(2), 2), pow(t_plan(2), 1), 1;

  _AAA33_inv = AAA3.inverse();  

  return _AAA33_inv;



}

void PRMPCClass::solve_AAA_inv1()
{
  
  Eigen::Matrix<double, 4, 1> t_plan;
  t_plan.setZero();
  t_plan(0) = - _dt;
  t_plan(1) = 0;
  t_plan(2) = _dt;
  t_plan(3) = 2*_dt;
  
  Eigen::Matrix<double,4,4> AAA1; 
  AAA1 << pow(t_plan(0), 3), pow(t_plan(0), 2), pow(t_plan(0), 1), 1, 
	        pow(t_plan(1), 3), pow(t_plan(1), 2), pow(t_plan(1), 1), 1, 
	        pow(t_plan(2), 3), pow(t_plan(2), 2), pow(t_plan(2), 1), 1,
	        3*pow(t_plan(2), 2), 2*pow(t_plan(2), 1), pow(t_plan(2), 0), 0;

  _AAA_inv = AAA1.inverse();

//   Eigen::Matrix<double,3,3> AAA3,_AAA33_inv; 
//   AAA3 << pow(t_plan(0), 2), pow(t_plan(0), 1), 1, 
// 	        pow(t_plan(1), 2), pow(t_plan(1), 1), 1, 
// 	        pow(t_plan(2), 2), pow(t_plan(2), 1), 1;
// 
//   _AAA33_inv = AAA3.inverse();  
// 
//   return _AAA33_inv;



}

////solve the inverse matrix of 4*4 coefficient matrices
Eigen::Matrix<double, 3, 3> PRMPCClass::solve_AAA_inv_mod(Eigen::Matrix<double, 4, 1> t_plan)
{
  Eigen::Matrix<double,4,4> AAA1; 
  AAA1 << pow(t_plan(0), 3), pow(t_plan(0), 2), pow(t_plan(0), 1), 1, 
	        pow(t_plan(1), 3), pow(t_plan(1), 2), pow(t_plan(1), 1), 1, 
	        pow(t_plan(2), 3), pow(t_plan(2), 2), pow(t_plan(2), 1), 1,
	        pow(t_plan(3), 3), pow(t_plan(3), 2), pow(t_plan(3), 1), 1;

  _AAA_inv = AAA1.inverse();


  Eigen::Matrix<double,3,3> AAA3,_AAA33_inv; 
  AAA3 << pow(t_plan(0), 2), pow(t_plan(0), 1), 1, 
	        pow(t_plan(1), 2), pow(t_plan(1), 1), 1, 
	        pow(t_plan(2), 2), pow(t_plan(2), 1), 1;

  _AAA33_inv = AAA3.inverse();  

  return _AAA33_inv;



}

////solve the inverse matrix of 4*4 coefficient matrices: only run once 
void PRMPCClass::solve_AAA_inv_mod1()
{
  
  Eigen::Matrix<double, 4, 1> t_plan;
  t_plan.setZero();
  t_plan(0) = - _dt;
  t_plan(1) = 0;
  t_plan(2) = _dt;
  t_plan(3) = 2*_dt;
    
  Eigen::Matrix<double,4,4> AAA1; 
  AAA1 << pow(t_plan(0), 3), pow(t_plan(0), 2), pow(t_plan(0), 1), 1, 
	        pow(t_plan(1), 3), pow(t_plan(1), 2), pow(t_plan(1), 1), 1, 
	        pow(t_plan(2), 3), pow(t_plan(2), 2), pow(t_plan(2), 1), 1,
	        pow(t_plan(3), 3), pow(t_plan(3), 2), pow(t_plan(3), 1), 1;

  _AAA_inv_mod = AAA1.inverse();
}

////solve the inverse matrix of 7*7 coefficient matrices
Eigen::Matrix<double, 7, 7> PRMPCClass::solve_AAA_inv_x(Eigen::Vector3d t_plan)
{
  Eigen::Matrix<double,7,7> AAA;
  AAA.setZero();

  AAA << 6*pow(t_plan(0), 5), 5*pow(t_plan(0), 4), 4*pow(t_plan(0), 3), 3*pow(t_plan(0), 2), 2*pow(t_plan(0), 1), 1, 0,
        30*pow(t_plan(0), 4),20*pow(t_plan(0), 3),12*pow(t_plan(0), 2), 6*pow(t_plan(0), 1), 2,                   0,0,
           pow(t_plan(0), 6),   pow(t_plan(0), 5),   pow(t_plan(0), 4),   pow(t_plan(0), 3),   pow(t_plan(0), 2), pow(t_plan(0), 1), 1,	  
           pow(t_plan(1), 6),   pow(t_plan(1), 5),   pow(t_plan(1), 4),   pow(t_plan(1), 3),   pow(t_plan(1), 2), pow(t_plan(1), 1), 1,
           pow(t_plan(2), 6),   pow(t_plan(2), 5),   pow(t_plan(2), 4),   pow(t_plan(2), 3),   pow(t_plan(2), 2), pow(t_plan(2), 1),1,
         6*pow(t_plan(2), 5), 5*pow(t_plan(2), 4), 4*pow(t_plan(2), 3), 3*pow(t_plan(2), 2), 2*pow(t_plan(2), 1), 1, 0,
        30*pow(t_plan(2), 4),20*pow(t_plan(2), 3),12*pow(t_plan(2), 2), 6*pow(t_plan(2), 1), 2,                   0,  0; 

  Eigen::Matrix<double,7,7> AAA_inv = AAA.inverse(); 
  
  return AAA_inv;
  
}

////solve the inverse matrix of 6*6 coefficient matrices
Eigen::Matrix<double, 6, 6> PRMPCClass::solve_AAA_inv_x_mod(Eigen::Vector3d t_plan)
{
  Eigen::Matrix<double,6,6> AAA;
  AAA.setZero();
  // Eigen::Matrix<double, 1, 6> aaaa;
  // aaaa.setZero();
  AAA <<  5*pow(t_plan(0), 4), 4*pow(t_plan(0), 3), 3*pow(t_plan(0), 2), 2*pow(t_plan(0), 1), 1, 0,
            pow(t_plan(0), 5),   pow(t_plan(0), 4),   pow(t_plan(0), 3),   pow(t_plan(0), 2), pow(t_plan(0), 1),1,
            pow(t_plan(1), 5),   pow(t_plan(1), 4),   pow(t_plan(1), 3),   pow(t_plan(1), 2), pow(t_plan(1), 1),1,
            pow(t_plan(2), 5),   pow(t_plan(2), 4),   pow(t_plan(2), 3),   pow(t_plan(2), 2), pow(t_plan(2), 1),1,
          5*pow(t_plan(2), 4), 4*pow(t_plan(2), 3), 3*pow(t_plan(2), 2), 2*pow(t_plan(2), 1), 1, 0,
         20*pow(t_plan(2), 3),12*pow(t_plan(2), 2), 6*pow(t_plan(2), 1), 2,                   0, 0;

  Eigen::Matrix<double,6,6> AAA_inv = AAA.inverse(); 
  
  return AAA_inv;
  
}

//// foot trajectory solve--------polynomial: to reduce the overfitting using 4 data ================================================
Eigen::Matrix<double, 30, 1> PRMPCClass::Foot_trajectory_solve_mod1(int j_indexx,bool _stopwalking, Eigen::Matrix<double, 9, 1> Nrtfoorpr_gen)
{
    
  int bjxx_nrt = (int) Nrtfoorpr_gen(0,0);
  _footxyz_real(0,bjxx_nrt) = Nrtfoorpr_gen(1,0);
  _footxyz_real(0,bjxx_nrt+1) = Nrtfoorpr_gen(2,0);
  _footxyz_real(1,bjxx_nrt) = Nrtfoorpr_gen(3,0);
  _footxyz_real(1,bjxx_nrt+1) = Nrtfoorpr_gen(4,0);
  _footxyz_real(2,bjxx_nrt) = Nrtfoorpr_gen(5,0);
  _footxyz_real(2,bjxx_nrt) = Nrtfoorpr_gen(6,0);
  

  
   Eigen::Matrix<double, 30, 1> rffoot_traj;
   rffoot_traj.setZero();
  int jx_max = _nh;
  for (int j_index = j_indexx; j_index<j_indexx+jx_max; j_index++ )
  {
    
    
    if (j_index <= _t_end_footstep)
    {
      Indexfind(j_index*_dt_mpc,xyz1);                   //// step cycle number when (i)*dt fall into : current sampling time
      _bjxx = _j_period+1;  //coincidence with matlab 
      _j_period = 0;	  
      
      
      Indexfind((j_index+1)*_dt_mpc,xyz1);                /// step cycle number when (i+1)*dt fall into : current sampling time
      _bjx1 = _j_period+1;
      _j_period = 0;      
    }



    //// judge if stop  
    if ((_stopwalking) ||(j_index > _t_end_footstep)) 
    {
      
      for (int i_t = _bjx1+1; i_t < _footstepsnumber; i_t++) {	  
	_lift_height_ref(i_t) = 0;  
      }	  

    }
    for (int i_t = 24; i_t < _footstepsnumber; i_t++) {	  
       _lift_height_ref(i_t) = 0;  
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
        //cout <<"Fast footpr generation"<<endl;
        if (_bjx1 % 2 == 0)           //odd:left support
        {
        //     no change on the left support location
            _Lfootx(j_index - j_indexx+1) = _Lfootx(j_index - j_indexx);
            _Lfooty(j_index - j_indexx+1) = _Lfooty(j_index - j_indexx);
            _Lfootz(j_index - j_indexx+1) = _Lfootz(j_index - j_indexx);
            
            _Lfootx(j_index - j_indexx+1+1) = _Lfootx(j_index - j_indexx);
            _Lfooty(j_index - j_indexx+1+1) = _Lfooty(j_index - j_indexx);
            _Lfootz(j_index - j_indexx+1+1) = _Lfootz(j_index - j_indexx);            
            
            /// right swing
            if ((j_index +1 - round(_tx(_bjx1-1)/_dt_mpc))*_dt_mpc < _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab: double support
            {
                
                _Rfootx(j_index - j_indexx+1) = _Rfootx(j_index - j_indexx);
                _Rfooty(j_index - j_indexx+1) = _Rfooty(j_index - j_indexx);
                _Rfootz(j_index - j_indexx+1) = _Rfootz(j_index - j_indexx);
                
                _Rfootx(j_index - j_indexx+1+1) = _Rfootx(j_index - j_indexx);
                _Rfooty(j_index - j_indexx+1+1) = _Rfooty(j_index - j_indexx);
                _Rfootz(j_index - j_indexx+1+1) = _Rfootz(j_index - j_indexx);                
            }
            else
            {
                double t_des = (j_index +1 - round(_tx(_bjx1-1)/_dt_mpc) +1)*_dt_mpc;
                Eigen::Vector3d t_plan(0,0,0);
                t_plan(0) = t_des - _dt_mpc;
                t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.0001;
                t_plan(2) = _ts(_bjx1-1)+0.0001;	    
                
                if (abs(t_des - _ts(_bjx1-1)) <= ( + 0.0005))
                {

                    _Rfootx(j_index - j_indexx+1) = _footxyz_real(0,_bjxx); 
                    _Rfooty(j_index - j_indexx+1) = _footxyz_real(1,_bjxx);
                    _Rfootz(j_index - j_indexx+1) = _footxyz_real(2,_bjxx); 
                    
                    _Rfootx(j_index - j_indexx+1+1) = _footxyz_real(0,_bjxx); 
                    _Rfooty(j_index - j_indexx+1+1) = _footxyz_real(1,_bjxx);
                    _Rfootz(j_index - j_indexx+1+1) = _footxyz_real(2,_bjxx);                     
                }
                else
                {
                    
                    cout<<"left support: right leg generation:"<<endl;
                    cout<<"t_des:"<<t_des<<endl;
                    Eigen::Matrix<double,5,5> AAA_inv = solve_AAA_inv_x_coma(t_plan);
                        
                    Eigen::Matrix<double, 1, 5> t_a_plan;
                    t_a_plan << pow(t_des, 4), pow(t_des, 3), pow(t_des, 2), pow(t_des, 1), 1;
                    
                    Eigen::Matrix<double, 1, 5> t_a_planv;
                    t_a_planv << 4*pow(t_des, 3), 3*pow(t_des, 2), 2*pow(t_des, 1), 1, 0;

                    Eigen::Matrix<double, 1, 5> t_a_plana;
                    t_a_plana << 12*pow(t_des, 2), 6*pow(t_des, 1), 2, 0, 0;
                    
                    ////////////////////////////////////////////////////////////////////////////
                    Eigen::Matrix<double, 5, 1> Rfootx_plan;
                    Rfootx_plan << _Rfootx(j_index - j_indexx+1-1),_Rfootvx(j_index - j_indexx+1-1), (_footxyz_real(0,_bjxx-2)+ _footxyz_real(0,_bjxx))/2, _footxyz_real(0,_bjxx), 0;
                    
                    Eigen::Matrix<double, 5, 1> Rfootx_co;
                    Rfootx_co.setZero();
                    Rfootx_co = AAA_inv * Rfootx_plan;
                    
                    _Rfootx(j_index - j_indexx+1) = t_a_plan * Rfootx_co;
                    _Rfootvx(j_index - j_indexx+1) = t_a_planv * Rfootx_co;
                    _Rfootax(j_index - j_indexx+1) = t_a_plana * Rfootx_co;
                    
                    /////////////////////////////////////////////////////////////////////////////
                    if ((j_index +1 - round(_tx(_bjx1-1)/_dt_mpc))*_dt_mpc < _td(_bjx1-1)+_dt_mpc)
                    {
                    _ry_left_right = (_footxyz_real(1,_bjxx) + _footxyz_real(1,_bjxx-2))/2;
                    }
                    
                    Eigen::Matrix<double, 5, 1> Rfooty_plan;
                    Rfooty_plan << _Rfooty(j_index - j_indexx+1-1),_Rfootvy(j_index - j_indexx+1-1), _ry_left_right, _footxyz_real(1,_bjxx), 0;	    
                    
                    Eigen::Matrix<double, 5, 1> Rfooty_co;
                    Rfooty_co.setZero();
                    Rfooty_co = AAA_inv * Rfooty_plan;
                    
                    _Rfooty(j_index - j_indexx+1) = t_a_plan * Rfooty_co;
                    _Rfootvy(j_index - j_indexx+1) = t_a_planv * Rfooty_co;
                    _Rfootay(j_index - j_indexx+1) = t_a_plana * Rfooty_co;	
                    
                    
                    //////////////////////////////////////////////////////////
                    Eigen::Matrix<double, 5, 1> Rfootz_plan;	
                    Rfootz_plan << _Rfootz(j_index - j_indexx+1-1),_Rfootvz(j_index - j_indexx+1-1), max(_footxyz_real(2,_bjxx-2),_footxyz_real(2,_bjxx))+_lift_height_ref(_bjx1-1), _footxyz_real(2,_bjxx), 0;
                    
                    Eigen::Matrix<double, 5, 1> Rfootz_co;
                    Rfootz_co.setZero();
                    Rfootz_co = AAA_inv * Rfootz_plan;
                    
                    _Rfootz(j_index - j_indexx+1) = t_a_plan * Rfootz_co;
                    _Rfootvz(j_index - j_indexx+1) = t_a_planv * Rfootz_co;
                    _Rfootaz(j_index - j_indexx+1) = t_a_plana * Rfootz_co;	
                        
                    
                    _Rfootx(j_index - j_indexx+1+1) = _Rfootx(j_index - j_indexx+1)+_dt_mpc * _Rfootvx(j_index - j_indexx+1);
                    _Rfooty(j_index - j_indexx+1+1) = _Rfooty(j_index - j_indexx+1)+_dt_mpc * _Rfootvy(j_index - j_indexx+1);
                    _Rfootz(j_index - j_indexx+1+1) = _Rfootz(j_index - j_indexx+1)+_dt_mpc * _Rfootvz(j_index - j_indexx+1);	    
                }
                
                
            }
        }
        else                       //right support
        {
        //       no change on right support
            _Rfootx(j_index - j_indexx+1) = _Rfootx(j_index - j_indexx);
            _Rfooty(j_index - j_indexx+1) = _Rfooty(j_index - j_indexx);
            _Rfootz(j_index - j_indexx+1) = _Rfootz(j_index - j_indexx);
            
            _Rfootx(j_index - j_indexx+1+1) = _Rfootx(j_index - j_indexx);
            _Rfooty(j_index - j_indexx+1+1) = _Rfooty(j_index - j_indexx);
            _Rfootz(j_index - j_indexx+1+1) = _Rfootz(j_index - j_indexx);      
            
            /// left swing
            if ((j_index +1 - round(_tx(_bjx1-1)/_dt_mpc))*_dt_mpc < _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab: double suppot
            {
                _Lfootx(j_index - j_indexx+1) = _Lfootx(j_index - j_indexx);
                _Lfooty(j_index - j_indexx+1) = _Lfooty(j_index - j_indexx);
                _Lfootz(j_index - j_indexx+1) = _Lfootz(j_index - j_indexx);
    
                _Lfootx(j_index - j_indexx+1+1) = _Lfootx(j_index - j_indexx);
                _Lfooty(j_index - j_indexx+1+1) = _Lfooty(j_index - j_indexx);
                _Lfootz(j_index - j_indexx+1+1) = _Lfootz(j_index - j_indexx);
            
            }
            else
            {
                //initial state and final state and the middle state
                double t_des = (j_index +1 - round(_tx(_bjx1-1)/_dt_mpc) +1)*_dt_mpc;
                Eigen::Vector3d t_plan(0,0,0);
                t_plan(0) = t_des - _dt_mpc;
                t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.0001;
                t_plan(2) = _ts(_bjx1-1) + 0.0001;	
                
                if (abs(t_des - _ts(_bjx1-1)) <= ( + 0.0005))
                {
                    
                    _Lfootx(j_index - j_indexx+1) = _footxyz_real(0,_bjxx); 
                    _Lfooty(j_index - j_indexx+1) = _footxyz_real(1,_bjxx);
                    _Lfootz(j_index - j_indexx+1) = _footxyz_real(2,_bjxx); 
    
                    _Lfootx(j_index - j_indexx+1+1) = _footxyz_real(0,_bjxx); 
                    _Lfooty(j_index - j_indexx+1+1) = _footxyz_real(1,_bjxx);
                    _Lfootz(j_index - j_indexx+1+1) = _footxyz_real(2,_bjxx); 
                    
                }
                else
                {
                    cout<<"right support: left leg generation:"<<endl;
                    cout<<"t_des:"<<t_des<<endl;
                    Eigen::Matrix<double,5,5> AAA_inv = solve_AAA_inv_x_coma(t_plan);
                        
                    Eigen::Matrix<double, 1, 5> t_a_plan;
                    t_a_plan << pow(t_des, 4), pow(t_des, 3), pow(t_des, 2), pow(t_des, 1), 1;
                    
                    Eigen::Matrix<double, 1, 5> t_a_planv;
                    t_a_planv << 4*pow(t_des, 3), 3*pow(t_des, 2), 2*pow(t_des, 1), 1, 0;

                    Eigen::Matrix<double, 1, 5> t_a_plana;
                    t_a_plana << 12*pow(t_des, 2), 6*pow(t_des, 1), 2, 0, 0;
                    
                    
                    ////////////////////////////////////////////////////////////////////////////
                    Eigen::Matrix<double, 5, 1> Lfootx_plan;
                    Lfootx_plan << _Lfootx(j_index - j_indexx+1-1),_Lfootvx(j_index - j_indexx+1-1), (_footxyz_real(0,_bjxx-2)+ _footxyz_real(0,_bjxx))/2, _footxyz_real(0,_bjxx), 0;
                    
                    Eigen::Matrix<double, 5, 1> Lfootx_co;
                    Lfootx_co.setZero();
                    Lfootx_co = AAA_inv * Lfootx_plan;
                    
                    _Lfootx(j_index - j_indexx+1) = t_a_plan * Lfootx_co;
                    _Lfootvx(j_index - j_indexx+1) = t_a_planv * Lfootx_co;
                    _Lfootax(j_index - j_indexx+1) = t_a_plana * Lfootx_co;
                    
                    /////////////////////////////////////////////////////////////////////////////
                    if ((j_index +1 - round(_tx(_bjx1-1)/_dt_mpc))*_dt_mpc < _td(_bjx1-1)+_dt_mpc)
                    {
                    _ry_left_right = (_footxyz_real(1,_bjxx) + _footxyz_real(1,_bjxx-2))/2;
                    }
                    
                    Eigen::Matrix<double, 5, 1> Lfooty_plan;
                    Lfooty_plan << _Lfooty(j_index - j_indexx+1-1),_Lfootvy(j_index - j_indexx+1-1), _ry_left_right, _footxyz_real(1,_bjxx), 0;		  
                    
                    
                    Eigen::Matrix<double, 5, 1> Lfooty_co;
                    Lfooty_co.setZero();
                    Lfooty_co = AAA_inv * Lfooty_plan;
                    
                    _Lfooty(j_index - j_indexx+1) = t_a_plan * Lfooty_co;
                    _Lfootvy(j_index - j_indexx+1) = t_a_planv * Lfooty_co;
                    _Lfootay(j_index - j_indexx+1) = t_a_plana * Lfooty_co;	
                    
                    
                    //////////////////////////////////////////////////////////
                    Eigen::Matrix<double, 5, 1> Lfootz_plan;
                    Lfootz_plan << _Lfootz(j_index - j_indexx+1-1),_Lfootvz(j_index - j_indexx+1-1), max(_footxyz_real(2,_bjxx-2),_footxyz_real(2,_bjxx))+_lift_height_ref(_bjx1-1), _footxyz_real(2,_bjxx), 0;		  
                    
                    
                    Eigen::Matrix<double, 5, 1> Lfootz_co;
                    Lfootz_co.setZero();
                    Lfootz_co = AAA_inv * Lfootz_plan;
                    
                    _Lfootz(j_index - j_indexx+1) = t_a_plan * Lfootz_co;
                    _Lfootvz(j_index - j_indexx+1) = t_a_planv * Lfootz_co;
                    _Lfootaz(j_index - j_indexx+1) = t_a_plana * Lfootz_co;
                    
                
                    _Lfootx(j_index - j_indexx+1+1) = _Lfootx(j_index - j_indexx+1)+_dt_mpc * _Lfootvx(j_index - j_indexx+1);
                    _Lfooty(j_index - j_indexx+1+1) = _Lfooty(j_index - j_indexx+1)+_dt_mpc * _Lfootvy(j_index - j_indexx+1);
                    _Lfootz(j_index - j_indexx+1+1) = _Lfootz(j_index - j_indexx+1)+_dt_mpc * _Lfootvz(j_index - j_indexx+1);
    
                }
            }

        }

    }
    else
    {
        if ((j_index > _t_end_footstep))
        {
          _Rfootx(j_index - j_indexx+1) = _Rfootx(j_index - j_indexx+1-1);
          _Rfooty(j_index - j_indexx+1) = _Rfooty(j_index - j_indexx+1-1);
          _Rfootz(j_index - j_indexx+1) = _Rfootz(j_index - j_indexx+1-1);
          _Lfootx(j_index - j_indexx+1) = _Lfootx(j_index - j_indexx+1-1);
          _Lfooty(j_index - j_indexx+1) = _Lfooty(j_index - j_indexx+1-1);
          _Lfootz(j_index - j_indexx+1) = _Lfootz(j_index - j_indexx+1-1);
        }
        else
        {
          _Rfooty(j_index - j_indexx+1) = -_stepwidth(0);
          _Lfooty(j_index - j_indexx+1) = _stepwidth(0);
        }

    }      
      
      
}

  for (int jjj=0; jjj<5; jjj++ )
  {
    rffoot_traj(0+6*jjj,0) = _Rfootx(jjj+1);
    rffoot_traj(1+6*jjj,0) = _Rfooty(jjj+1);
    rffoot_traj(2+6*jjj,0) = _Rfootz(jjj+1);

    rffoot_traj(3+6*jjj,0) = _Lfootx(jjj+1);
    rffoot_traj(4+6*jjj,0) = _Lfooty(jjj+1);
    rffoot_traj(5+6*jjj,0) = _Lfootz(jjj+1);    
  }
  _Rfootx(0) = _Rfootx(1);
  _Rfooty(0) = _Rfooty(1);
  _Rfootz(0) = _Rfootz(1);
  _Lfootx(0) = _Lfootx(1);
  _Lfooty(0) = _Lfooty(1);
  _Lfootz(0) = _Lfootz(1);
  
  _Rfootvx(0) = _Rfootvx(1);
  _Rfootvy(0) = _Rfootvy(1);
  _Rfootvz(0) = _Rfootvz(1);
  _Lfootvx(0) = _Lfootvx(1);
  _Lfootvy(0) = _Lfootvy(1);
  _Lfootvz(0) = _Lfootvz(1);  
  
  _Rfootax(0) = _Rfootax(1);
  _Rfootay(0) = _Rfootay(1);
  _Rfootaz(0) = _Rfootaz(1);
  _Lfootax(0) = _Lfootax(1);
  _Lfootay(0) = _Lfootay(1);
  _Lfootaz(0) = _Lfootaz(1);

  return rffoot_traj;

}

Eigen::Matrix<double, 30, 1> PRMPCClass::Foot_trajectory_solve_mod2(int j_indexx,bool _stopwalking, Eigen::Matrix<double, 9, 1> Nrtfoorpr_gen)
{
  int bjxx_nrt = (int) Nrtfoorpr_gen(0,0);
  _footxyz_real(0,bjxx_nrt) = Nrtfoorpr_gen(1,0);
  _footxyz_real(0,bjxx_nrt+1) = Nrtfoorpr_gen(2,0);
  _footxyz_real(1,bjxx_nrt) = Nrtfoorpr_gen(3,0);
  _footxyz_real(1,bjxx_nrt+1) = Nrtfoorpr_gen(4,0);
  _footxyz_real(2,bjxx_nrt) = Nrtfoorpr_gen(5,0);
  _footxyz_real(2,bjxx_nrt+1) = Nrtfoorpr_gen(6,0);
  int bjx_period_nrt = (int) Nrtfoorpr_gen(7,0);
  if (Nrtfoorpr_gen(8,0) > 0)
  {
    _ts(bjx_period_nrt) = Nrtfoorpr_gen(8,0);
  }
  
  

  _td = _tdsp_ratio*_ts;                                // dsp time              
  
  _tx.setZero();   // start time for each step cycle	
  for (int i = 1; i < _footstepsnumber; i++) {
    _tx(i) = _tx(i-1) + _ts(i-1);
    _tx(i) = round(_tx(i)/_dt)*_dt -0.00001;	  
  }
  _t_end_footstep = round((_tx(_footstepsnumber-1))/_dt_mpc);  

  _tx_total = _tx(_footstepsnumber - 1);
  
  
   Eigen::Matrix<double, 30, 1> rffoot_traj;
   rffoot_traj.setZero();
   
   int jx_max = _nh;
  
  for (int j_index = j_indexx; j_index<j_indexx+jx_max; j_index++ )
  {
    if (j_index <= _t_end_footstep)
    {
      Indexfind(j_index*_dt_mpc,xyz1);                   //// step cycle number when (i)*dt fall into : current sampling time
      _bjxx = _j_period+1;  //coincidence with matlab 
      _j_period = 0;	  
      
      
      Indexfind((j_index+1)*_dt_mpc,xyz1);                /// step cycle number when (i+1)*dt fall into : current sampling time
      _bjx1 = _j_period+1;
      _j_period = 0;      
    }

    //// judge if stop  
    if ((_stopwalking) ||(j_index > _t_end_footstep)) 
    {
      
      for (int i_t = _bjx1+1; i_t < _footstepsnumber; i_t++) {	  
      	_lift_height_ref(i_t) = 0;  
      }	  

    }
    // for (int i_t = 24; i_t < _footstepsnumber; i_t++) {	  
    //    _lift_height_ref(i_t) = 0;  
    // }   

    _footxyz_real(1,0) = -_stepwidth(0);
    
/*    ///// case1: uneven terrain
    /////// step parameters: for climibing & resting on a stone//////
    //0.05-0.06m    
    if((_bjx1>=6)&&(_bjx1<=6))
    {
	_lift_height_ref(_bjx1-1) = 0.045;
    }
    else
    {
	if ((_bjx1>=18)&&(_bjx1<=18))
	{
	    _lift_height_ref(_bjx1-1) = 0.03; 
	}
// 	else
// 	{
// 	    _lift_height_ref(_bjx1-1) = 0.05; 
// 	}

    }*/ 
    
    ///// case2: uneven terrain
    ///////  push recovery for climibing stones//////
    //0.05-0.06m    
/*    if((_bjx1>=6)&&(_bjx1<=6))
    {
	_lift_height_ref(_bjx1-1) = 0.045;
    }
    else
    {
	if ((_bjx1>=7)&&(_bjx1<=7))
	{
	    _lift_height_ref(_bjx1-1) = 0.06; 
	}
	else
	{
	  if ((_bjx1>=8)&&(_bjx1<=10))
	  {
	      _lift_height_ref(_bjx1-1) = 0.045; 
	  }
	}

    } */   
    
    
    
    //   foot trajectory generation:
    if ((_bjx1 >= 2)&&(j_index <= _t_end_footstep))
    {
        //cout <<"Fast footpr generation"<<endl;
        if (_bjx1 % 2 == 0)           //odd:left support
        {
        //     no change on the left support location
            _Lfootx(j_index - j_indexx+1) = _Lfootx(j_index - j_indexx);
            _Lfooty(j_index - j_indexx+1) = _Lfooty(j_index - j_indexx);
            _Lfootz(j_index - j_indexx+1) = _Lfootz(j_index - j_indexx);
            
            _Lfootx(j_index - j_indexx+1+1) = _Lfootx(j_index - j_indexx);
            _Lfooty(j_index - j_indexx+1+1) = _Lfooty(j_index - j_indexx);
            _Lfootz(j_index - j_indexx+1+1) = _Lfootz(j_index - j_indexx); 

//                 _Lfootx(j_index - j_indexx+1) = _footxyz_real(0,_bjxx-1);
//                 _Lfooty(j_index - j_indexx+1) = _footxyz_real(1,_bjxx-1);
//                 _Lfootz(j_index - j_indexx+1) = _footxyz_real(2,_bjxx-1);
//     
//                 _Lfootx(j_index - j_indexx+1+1) = _footxyz_real(0,_bjxx-1);
//                 _Lfooty(j_index - j_indexx+1+1) = _footxyz_real(1,_bjxx-1);
//                 _Lfootz(j_index - j_indexx+1+1) = _footxyz_real(2,_bjxx-1);
            
            /// right swing
            if ((j_index +1 - round(_tx(_bjx1-1)/_dt_mpc))*_dt_mpc < _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab: double support
            {
                
                _Rfootx(j_index - j_indexx+1) = _Rfootx(j_index - j_indexx);
                _Rfooty(j_index - j_indexx+1) = _Rfooty(j_index - j_indexx);
                _Rfootz(j_index - j_indexx+1) = _Rfootz(j_index - j_indexx);
                
                _Rfootx(j_index - j_indexx+1+1) = _Rfootx(j_index - j_indexx);
                _Rfooty(j_index - j_indexx+1+1) = _Rfooty(j_index - j_indexx);
                _Rfootz(j_index - j_indexx+1+1) = _Rfootz(j_index - j_indexx); 
		
          // 		if (_bjxx>2)
          // 		{
          //     	          _Rfootx(j_index - j_indexx+1) = _footxyz_real(0,_bjxx-2);
          // 		  _Rfooty(j_index - j_indexx+1) = _footxyz_real(1,_bjxx-2);
          // 		  _Rfootz(j_index - j_indexx+1) = _footxyz_real(2,_bjxx-2);
          // 		  
          // 		  _Rfootx(j_index - j_indexx+1+1) = _footxyz_real(0,_bjxx-2);
          // 		  _Rfooty(j_index - j_indexx+1+1) = _footxyz_real(1,_bjxx-2);
          // 		  _Rfootz(j_index - j_indexx+1+1) = _footxyz_real(2,_bjxx-2); 		  
          // 		}
		
            }
            else
            {
                double t_des = (j_index +1 - round(_tx(_bjx1-1)/_dt_mpc) +1)*_dt_mpc;
                Eigen::Vector3d t_plan(0,0,0);
                t_plan(0) = t_des - _dt_mpc;
                t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.0001;
                t_plan(2) = _ts(_bjx1-1)- (2*_dt_mpc + 0.001);	    
                
                if (abs(t_des - _ts(_bjx1-1)) <= ( _dt_mpc))
                {

                    _Rfootx(j_index - j_indexx+1) = _footxyz_real(0,_bjxx); 
                    _Rfooty(j_index - j_indexx+1) = _footxyz_real(1,_bjxx);
                    _Rfootz(j_index - j_indexx+1) = _footxyz_real(2,_bjxx); 
                    
                    _Rfootx(j_index - j_indexx+1+1) = _footxyz_real(0,_bjxx); 
                    _Rfooty(j_index - j_indexx+1+1) = _footxyz_real(1,_bjxx);
                    _Rfootz(j_index - j_indexx+1+1) = _footxyz_real(2,_bjxx);                     
                }
                else
                {
                    
//                     cout<<"left support: right leg generation:"<<endl;
//                     cout<<"t_des:"<<t_des<<endl;
                    Eigen::Matrix<double,4,4> AAA_inv = solve_AAA_inv2(t_plan);
                        
                    Eigen::Matrix<double, 1, 4> t_a_plan;
                    t_a_plan << pow(t_des, 3), pow(t_des, 2), pow(t_des, 1), 1;
                    
                    Eigen::Matrix<double, 1, 4> t_a_planv;
                    t_a_planv << 3*pow(t_des, 2), 2*pow(t_des, 1), 1, 0;
    
                    Eigen::Matrix<double, 1, 4> t_a_plana;
                    t_a_plana << 6*pow(t_des, 1), 2, 0, 0;
                    
                    ////////////////////////////////////////////////////////////////////////////
                    Eigen::Matrix<double, 4, 1> Rfootx_plan;
                    Rfootx_plan << _Rfootx(j_index - j_indexx+1-1), (_footxyz_real(0,_bjxx-2)+ _footxyz_real(0,_bjxx))/2, _footxyz_real(0,_bjxx), 0;
                    
                    Eigen::Matrix<double, 4, 1> Rfootx_co;
                    Rfootx_co.setZero();
                    Rfootx_co = AAA_inv * Rfootx_plan;
                    
                    _Rfootx(j_index - j_indexx+1) = t_a_plan * Rfootx_co;
                    _Rfootvx(j_index - j_indexx+1) = t_a_planv * Rfootx_co;
                    _Rfootax(j_index - j_indexx+1) = t_a_plana * Rfootx_co;
                    
                    /////////////////////////////////////////////////////////////////////////////
                    if ((j_index +1 - round(_tx(_bjx1-1)/_dt_mpc))*_dt_mpc < _td(_bjx1-1)+_dt_mpc)
                    {
                    _ry_left_right = (_footxyz_real(1,_bjxx) + _footxyz_real(1,_bjxx-2))/2;
                    }
                    
                    Eigen::Matrix<double, 4, 1> Rfooty_plan;
                    Rfooty_plan << _Rfooty(j_index - j_indexx+1-1), _ry_left_right, _footxyz_real(1,_bjxx), 0;	    
                    
                    Eigen::Matrix<double, 4, 1> Rfooty_co;
                    Rfooty_co.setZero();
                    Rfooty_co = AAA_inv * Rfooty_plan;
                    
                    _Rfooty(j_index - j_indexx+1) = t_a_plan * Rfooty_co;
                    _Rfootvy(j_index - j_indexx+1) = t_a_planv * Rfooty_co;
                    _Rfootay(j_index - j_indexx+1) = t_a_plana * Rfooty_co;	
                    
                    
                    //////////////////////////////////////////////////////////
                    Eigen::Matrix<double, 4, 1> Rfootz_plan;	
                    Rfootz_plan << _Rfootz(j_index - j_indexx+1-1), max(_footxyz_real(2,_bjxx-2),_footxyz_real(2,_bjxx))+_lift_height_ref(_bjx1-1), _footxyz_real(2,_bjxx), 0;
                    
                    Eigen::Matrix<double, 4, 1> Rfootz_co;
                    Rfootz_co.setZero();
                    Rfootz_co = AAA_inv * Rfootz_plan;
                    
                    _Rfootz(j_index - j_indexx+1) = t_a_plan * Rfootz_co;
                    _Rfootvz(j_index - j_indexx+1) = t_a_planv * Rfootz_co;
                    _Rfootaz(j_index - j_indexx+1) = t_a_plana * Rfootz_co;	
                        
                    
                    _Rfootx(j_index - j_indexx+1+1) = _Rfootx(j_index - j_indexx+1)+_dt_mpc * _Rfootvx(j_index - j_indexx+1);
                    _Rfooty(j_index - j_indexx+1+1) = _Rfooty(j_index - j_indexx+1)+_dt_mpc * _Rfootvy(j_index - j_indexx+1);
                    _Rfootz(j_index - j_indexx+1+1) = _Rfootz(j_index - j_indexx+1)+_dt_mpc * _Rfootvz(j_index - j_indexx+1);	    
                }
                
                
            }
        }
        else                       //right support
        {
        //       no change on right support
            _Rfootx(j_index - j_indexx+1) = _Rfootx(j_index - j_indexx);
            _Rfooty(j_index - j_indexx+1) = _Rfooty(j_index - j_indexx);
            _Rfootz(j_index - j_indexx+1) = _Rfootz(j_index - j_indexx);
            
            _Rfootx(j_index - j_indexx+1+1) = _Rfootx(j_index - j_indexx);
            _Rfooty(j_index - j_indexx+1+1) = _Rfooty(j_index - j_indexx);
            _Rfootz(j_index - j_indexx+1+1) = _Rfootz(j_index - j_indexx); 

//             _Rfootx(j_index - j_indexx+1) = _footxyz_real(0,_bjxx-1);
//             _Rfooty(j_index - j_indexx+1) = _footxyz_real(1,_bjxx-1);
//             _Rfootz(j_index - j_indexx+1) = _footxyz_real(2,_bjxx-1);
//             
//             _Rfootx(j_index - j_indexx+1+1) = _footxyz_real(0,_bjxx-1);
//             _Rfooty(j_index - j_indexx+1+1) = _footxyz_real(1,_bjxx-1);
//             _Rfootz(j_index - j_indexx+1+1) = _footxyz_real(2,_bjxx-1); 
	    
            
            /// left swing
            if ((j_index +1 - round(_tx(_bjx1-1)/_dt_mpc))*_dt_mpc < _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab: double suppot
            {
                _Lfootx(j_index - j_indexx+1) = _Lfootx(j_index - j_indexx);
                _Lfooty(j_index - j_indexx+1) = _Lfooty(j_index - j_indexx);
                _Lfootz(j_index - j_indexx+1) = _Lfootz(j_index - j_indexx);
    
                _Lfootx(j_index - j_indexx+1+1) = _Lfootx(j_index - j_indexx);
                _Lfooty(j_index - j_indexx+1+1) = _Lfooty(j_index - j_indexx);
                _Lfootz(j_index - j_indexx+1+1) = _Lfootz(j_index - j_indexx);
	      
          // 		if (_bjxx >2)
          // 		{
          // 		  _Lfootx(j_index - j_indexx+1) = _footxyz_real(0,_bjxx-2);
          // 		  _Lfooty(j_index - j_indexx+1) = _footxyz_real(1,_bjxx-2);
          // 		  _Lfootz(j_index - j_indexx+1) = _footxyz_real(2,_bjxx-2);
          //       
          // 		  _Lfootx(j_index - j_indexx+1+1) = _footxyz_real(0,_bjxx-2);
          // 		  _Lfooty(j_index - j_indexx+1+1) = _footxyz_real(1,_bjxx-2);
          // 		  _Lfootz(j_index - j_indexx+1+1) = _footxyz_real(2,_bjxx-2);		  
          // 		}
	      
	      
            
            }
            else
            {
                //initial state and final state and the middle state
                double t_des = (j_index +1 - round(_tx(_bjx1-1)/_dt_mpc) +1)*_dt_mpc;
                Eigen::Vector3d t_plan(0,0,0);
                t_plan(0) = t_des - _dt_mpc;
                t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.0001;
                t_plan(2) = _ts(_bjx1-1) -(2*_dt_mpc + 0.001);	
                
                if (abs(t_des - _ts(_bjx1-1)) <= ( _dt_mpc))
                {
                    
                    _Lfootx(j_index - j_indexx+1) = _footxyz_real(0,_bjxx); 
                    _Lfooty(j_index - j_indexx+1) = _footxyz_real(1,_bjxx);
                    _Lfootz(j_index - j_indexx+1) = _footxyz_real(2,_bjxx); 
    
                    _Lfootx(j_index - j_indexx+1+1) = _footxyz_real(0,_bjxx); 
                    _Lfooty(j_index - j_indexx+1+1) = _footxyz_real(1,_bjxx);
                    _Lfootz(j_index - j_indexx+1+1) = _footxyz_real(2,_bjxx); 
                    
                }
                else
                {
//                     cout<<"right support: left leg generation:"<<endl;
//                     cout<<"t_des:"<<t_des<<endl;
                    Eigen::Matrix<double,4,4> AAA_inv = solve_AAA_inv2(t_plan);;
                        
                    Eigen::Matrix<double, 1, 4> t_a_plan;
                    t_a_plan << pow(t_des, 3), pow(t_des, 2), pow(t_des, 1), 1;
                    
    
                    Eigen::Matrix<double, 1, 4> t_a_planv;
                    t_a_planv << 3*pow(t_des, 2), 2*pow(t_des, 1), 1, 0;
                    
                    
                    Eigen::Matrix<double, 1, 4> t_a_plana;
                    t_a_plana << 6*pow(t_des, 1), 2, 0, 0;
                    
                    
                    ////////////////////////////////////////////////////////////////////////////
                    Eigen::Matrix<double, 4, 1> Lfootx_plan;
                    Lfootx_plan << _Lfootx(j_index - j_indexx+1-1), (_footxyz_real(0,_bjxx-2)+ _footxyz_real(0,_bjxx))/2, _footxyz_real(0,_bjxx), 0;
                    
                    Eigen::Matrix<double, 4, 1> Lfootx_co;
                    Lfootx_co.setZero();
                    Lfootx_co = AAA_inv * Lfootx_plan;
                    
                    _Lfootx(j_index - j_indexx+1) = t_a_plan * Lfootx_co;
                    _Lfootvx(j_index - j_indexx+1) = t_a_planv * Lfootx_co;
                    _Lfootax(j_index - j_indexx+1) = t_a_plana * Lfootx_co;
                    
                    /////////////////////////////////////////////////////////////////////////////
                    if ((j_index +1 - round(_tx(_bjx1-1)/_dt_mpc))*_dt_mpc < _td(_bjx1-1)+_dt_mpc)
                    {
                    _ry_left_right = (_footxyz_real(1,_bjxx) + _footxyz_real(1,_bjxx-2))/2;
                    }
                    
                    Eigen::Matrix<double, 4, 1> Lfooty_plan;
                    Lfooty_plan << _Lfooty(j_index - j_indexx+1-1), _ry_left_right, _footxyz_real(1,_bjxx), 0;		  
                    
                    
                    Eigen::Matrix<double, 4, 1> Lfooty_co;
                    Lfooty_co.setZero();
                    Lfooty_co = AAA_inv * Lfooty_plan;
                    
                    _Lfooty(j_index - j_indexx+1) = t_a_plan * Lfooty_co;
                    _Lfootvy(j_index - j_indexx+1) = t_a_planv * Lfooty_co;
                    _Lfootay(j_index - j_indexx+1) = t_a_plana * Lfooty_co;	
                    
                    
                    //////////////////////////////////////////////////////////
                    Eigen::Matrix<double, 4, 1> Lfootz_plan;
                    Lfootz_plan << _Lfootz(j_index - j_indexx+1-1), max(_footxyz_real(2,_bjxx-2),_footxyz_real(2,_bjxx))+_lift_height_ref(_bjx1-1), _footxyz_real(2,_bjxx), 0;		  
                    
                    
                    Eigen::Matrix<double, 4, 1> Lfootz_co;
                    Lfootz_co.setZero();
                    Lfootz_co = AAA_inv * Lfootz_plan;
                    
                    _Lfootz(j_index - j_indexx+1) = t_a_plan * Lfootz_co;
                    _Lfootvz(j_index - j_indexx+1) = t_a_planv * Lfootz_co;
                    _Lfootaz(j_index - j_indexx+1) = t_a_plana * Lfootz_co;
                    
                
                    _Lfootx(j_index - j_indexx+1+1) = _Lfootx(j_index - j_indexx+1)+_dt_mpc * _Lfootvx(j_index - j_indexx+1);
                    _Lfooty(j_index - j_indexx+1+1) = _Lfooty(j_index - j_indexx+1)+_dt_mpc * _Lfootvy(j_index - j_indexx+1);
                    _Lfootz(j_index - j_indexx+1+1) = _Lfootz(j_index - j_indexx+1)+_dt_mpc * _Lfootvz(j_index - j_indexx+1);
    
                }
            }

        }

    }
    else
    {
        if ((j_index > _t_end_footstep))
        {
          _Rfootx(j_index - j_indexx+1) = _Rfootx(j_index - j_indexx+1-1);
          _Rfooty(j_index - j_indexx+1) = _Rfooty(j_index - j_indexx+1-1);
          _Rfootz(j_index - j_indexx+1) = _Rfootz(j_index - j_indexx+1-1);
          _Lfootx(j_index - j_indexx+1) = _Lfootx(j_index - j_indexx+1-1);
          _Lfooty(j_index - j_indexx+1) = _Lfooty(j_index - j_indexx+1-1);
          _Lfootz(j_index - j_indexx+1) = _Lfootz(j_index - j_indexx+1-1);
        }
        else
        {
          _Rfooty(j_index - j_indexx+1) = -_stepwidth(0);
          _Lfooty(j_index - j_indexx+1) = _stepwidth(0);
        }

    }      
      
      
}


  for (int jjj=0; jjj<5; jjj++ )
  {
    rffoot_traj(0+6*jjj,0) = _Rfootx(jjj+1);
    rffoot_traj(1+6*jjj,0) = _Rfooty(jjj+1);
    rffoot_traj(2+6*jjj,0) = _Rfootz(jjj+1);

    rffoot_traj(3+6*jjj,0) = _Lfootx(jjj+1);
    rffoot_traj(4+6*jjj,0) = _Lfooty(jjj+1);
    rffoot_traj(5+6*jjj,0) = _Lfootz(jjj+1);    
  }


  _Rfootx(0) = _Rfootx(1);
  _Rfooty(0) = _Rfooty(1);
  _Rfootz(0) = _Rfootz(1);
  _Lfootx(0) = _Lfootx(1);
  _Lfooty(0) = _Lfooty(1);
  _Lfootz(0) = _Lfootz(1);
  
  _Rfootvx(0) = _Rfootvx(1);
  _Rfootvy(0) = _Rfootvy(1);
  _Rfootvz(0) = _Rfootvz(1);
  _Lfootvx(0) = _Lfootvx(1);
  _Lfootvy(0) = _Lfootvy(1);
  _Lfootvz(0) = _Lfootvz(1);  
  
  _Rfootax(0) = _Rfootax(1);
  _Rfootay(0) = _Rfootay(1);
  _Rfootaz(0) = _Rfootaz(1);
  _Lfootax(0) = _Lfootax(1);
  _Lfootay(0) = _Lfootay(1);
  _Lfootaz(0) = _Lfootaz(1);  
  

  return rffoot_traj;

}

////////////////step parameters input============================================================
void PRMPCClass::FootStepInputs(double stepwidth, double steplengthx, double stepheight, double _lift_height)
{	
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
    // _lift_height_ref(_footstepsnumber-1) = 0;
    // _lift_height_ref(_footstepsnumber-2) = 0;	
    // _lift_height_ref(_footstepsnumber-3) = _lift_height/2; 
    // _lift_height_ref(_footstepsnumber-4) = _lift_height; 	
}


Eigen::Matrix<double, 4, 4> PRMPCClass::solve_AAA_inv2(Eigen::Vector3d t_plan)
{
  Eigen::Matrix<double,4,4> AAA1; 
  AAA1 << pow(t_plan(0), 3), pow(t_plan(0), 2), pow(t_plan(0), 1), 1, 
	        pow(t_plan(1), 3), pow(t_plan(1), 2), pow(t_plan(1), 1), 1, 
	        pow(t_plan(2), 3), pow(t_plan(2), 2), pow(t_plan(2), 1), 1,
	        3*pow(t_plan(2), 2), 2*pow(t_plan(2), 1), pow(t_plan(2), 0), 0;

  Eigen::Matrix<double,4,4> AAA1_inv = AAA1.inverse();  

  return AAA1_inv;
}

////solve the inverse matrix of 5*5 coefficient matrices
Eigen::Matrix<double, 5, 5> PRMPCClass::solve_AAA_inv_x_coma(Eigen::Vector3d t_plan)
{
 	Eigen::Matrix<double,5,5> AAA1;	

  AAA1 <<   pow(t_plan(0), 4),  pow(t_plan(0), 3),  pow(t_plan(0), 2),pow(t_plan(0), 1), 1,
          4*pow(t_plan(0), 3),3*pow(t_plan(0), 2),2*pow(t_plan(0), 1),                1, 0,
            pow(t_plan(1), 4),  pow(t_plan(1), 3),  pow(t_plan(1), 2),pow(t_plan(1), 1), 1,             
            pow(t_plan(2), 4),  pow(t_plan(2), 3),  pow(t_plan(2), 2),pow(t_plan(2), 1), 1,
          4*pow(t_plan(2), 3),3*pow(t_plan(2), 2),2*pow(t_plan(2), 1),                1, 0;
  
  Eigen::Matrix<double,5,5> AAA1_inv = AAA1.inverse(); 

  return AAA1_inv; 
}

////// sin and cos curve;
Eigen::Matrix<double, 30, 1> PRMPCClass::XGetSolution_Foot_rotation(int walktimex, double dt_sample)
{
  ///////walktime=====>ij;  dt_sample========>dtx;   
  Eigen::Matrix<double, 30, 1> footlr_r_inte;	
  footlr_r_inte.setZero();
  
  int jx_max = _nh;
  
  for (int walktime = walktimex; walktime < walktimex+jx_max; walktime ++)
  {
    
    if (walktime <= _t_end_footstep)
    {
      Indexfind(walktime*_dt_mpc,xyz1);                   //// step cycle number when (i)*dt fall into : current sampling time
      _bjxx = _j_period+1;  //coincidence with matlab 
      _j_period = 0;	  
      
      
      Indexfind((walktime+1)*_dt_mpc,xyz1);                /// step cycle number when (i+1)*dt fall into : current sampling time
      _bjx1 = _j_period+1;
      _j_period = 0;      
    }
 
  
    double t_desxx = (walktime+1)*dt_sample - (_tx(_bjx1-1,0) +  2*_td(_bjx1-1,0)/4);
    double footx_step = 0;
    double footy_step = 0;   
    double sing_x = 0;
    double sing_y = 0;

    if ((_bjx1 >= 2)&&(walktime <= _t_end_footstep))
    {  
      footx_step = _footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1); 
      sing_x = sign_function(footx_step);    
      footy_step = _footxyz_real(1,_bjx1)-_footxyz_real(1,_bjx1-1); 
      sing_y = sign_function(footy_step);        
      if (_bjx1 % 2 == 0)           //odd:left support
      {  
        //// right foot roll:
        ///////step forward and backward 1//////  
        _Rfoot_r(0, walktime - walktimex) =  -0.00*(1 - cos(2*M_PI / (_ts(_bjx1-1,0) ) * (t_desxx + 2*_td(_bjx1-1,0)/4) )) * sing_x;
        //       _Rfoot_rv(0, walktime - walktimex) =  -0.065*(2*M_PI / (_ts(_bjx1-1,0) ) * sin(2*M_PI / (_ts(_bjx1-1,0) ) * (t_desxx + 2*_td(_bjx1-1,0)/4) ));
        //       _Rfoot_ra(0, walktime - walktimex) =  -0.065*(2*M_PI / (_ts(_bjx1-1,0) ) * 2*M_PI / (_ts(_bjx1-1,0) ) * cos(2*M_PI / (_ts(_bjx1-1,0) ) * (t_desxx + 2*_td(_bjx1-1,0)/4) ));

        // _Rfoot_r(0, walktime - walktimex) =  -0.065*(1-fabs((_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max)))*(1 - cos(2*M_PI / (_ts(_bjx1-1) ) * (t_desxx + 2*_td(_bjx1-1)/4) ));
        

        ///// right foot pitch: ///later half cycle: minus angle;
        if ((t_desxx + 2*_td(_bjx1-1,0)/4) >= (_ts(_bjx1-1,0)/2))
        {
          if ((_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))>0)
          {
            //cout<< "t_desxxy_rfoot"<<t_desxx + 2*_td(_bjx1-1,0)/4<<endl;
            _Rfoot_r(1, walktime - walktimex) =  -0.1*(_footxyz_real(1,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (cos(4*M_PI / (_ts(_bjx1-1)) * (t_desxx + 2*_td(_bjx1-1)/4) ) - 1) * sing_x;  
        //           _Rfoot_rv(1, walktime - walktimex) =  0.075*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (4*M_PI / (_ts(_bjx1-1,0)) * (-sin(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4))));
        //           _Rfoot_ra(1, walktime - walktimex) =  0.075*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (4*M_PI / (_ts(_bjx1-1,0)) * 4*M_PI / (_ts(_bjx1-1,0)) * (-cos(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4))));        
        //         
            
          }
        }
        else
        {
        /*	if ((_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))>0)
          {
            _Rfoot_r(1, walktime - walktimex) =  -0.025*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (cos(4*M_PI / (_ts(_bjx1-1)) * (t_desxx + 2*_td(_bjx1-1)/4) ) - 1);          
            _Rfoot_rv(1, walktime - walktimex) = -0.025*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (4*M_PI / (_ts(_bjx1-1,0)) * (-sin(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4))));
            _Rfoot_ra(1, walktime - walktimex) = -0.025*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (4*M_PI / (_ts(_bjx1-1,0)) * 4*M_PI / (_ts(_bjx1-1,0)) * (-cos(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4))));               	  
          }*/

          _Rfoot_r(1, walktime - walktimex) =  0;          
        /*	_Rfoot_rv(1, walktime - walktimex) = 0;
          _Rfoot_ra(1, walktime - walktimex) = 0;    */           	  


        }

        _Rfoot_r(2, walktime - walktimex) =  -0.15*(_footxyz_real(1,_bjx1)-_footxyz_real(1,_bjx1-1))/(_footy_max) * (cos(4*M_PI / (_ts(_bjx1-1)) * (t_desxx + 2*_td(_bjx1-1)/4) ) - 1) * sing_y;             
      }
      else                       //right support
      {      
        //// left foot roll:
        /////// step forward and backward 1//////
        _Lfoot_r(0, walktime - walktimex) =  0.00*(1 - cos(2*M_PI / (_ts(_bjx1-1,0) ) * (t_desxx + 2*_td(_bjx1-1,0)/4) )) * sing_x;
        /*      _Lfoot_rv(0, walktime - walktimex) =  -0.075*(2*M_PI / (_ts(_bjx1-1,0) ) * sin(2*M_PI / (_ts(_bjx1-1,0) ) * (t_desxx + 2*_td(_bjx1-1,0)/4) ));
        _Lfoot_ra(0, walktime - walktimex) =  -0.075*(2*M_PI / (_ts(_bjx1-1,0) ) * 2*M_PI / (_ts(_bjx1-1,0) ) * cos(2*M_PI / (_ts(_bjx1-1,0) ) * (t_desxx + 2*_td(_bjx1-1,0)/4) ));      
            */ 
        // _Lfoot_r(0, walktime - walktimex) =  0.075*(1-fabs((_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max)))*(1 - cos(2*M_PI / (_ts(_bjx1-1) ) * (t_desxx + 2*_td(_bjx1-1)/4) ));

        ////// left foot pitch:
        if ((t_desxx + 2*_td(_bjx1-1,0)/4) >= (_ts(_bjx1-1,0)/2)) ///later half cycle: minus angle;
        {
          if ((_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))>0)
          {
            //cout<< "t_desxxy_lfoot"<<t_desxx + 2*_td(_bjx1-1,0)/4<<endl;
            _Lfoot_r(1, walktime - walktimex) =  -0.1*(_footxyz_real(1,_bjx1)-_footxyz_real(1,_bjx1-1))/(_footx_max) * (cos(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4)) - 1) * sing_x; 
        /*          _Lfoot_rv(1, walktime - walktimex) =  0.075*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (4*M_PI / (_ts(_bjx1-1,0)) * (-sin(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4))));
            _Lfoot_ra(1, walktime - walktimex) =  0.075*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (4*M_PI / (_ts(_bjx1-1,0)) * 4*M_PI / (_ts(_bjx1-1,0)) * (-cos(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4))));                    
          */
            
          } 
        }
        else
        {
        /*        if ((_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))>0)
          {
            _Lfoot_r(1, walktime - walktimex) =  -0.025*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (cos(4*M_PI / (_ts(_bjx1-1)) * (t_desxx + 2*_td(_bjx1-1)/4) ) - 1);          
            _Lfoot_rv(1, walktime - walktimex) =  -0.025*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (4*M_PI / (_ts(_bjx1-1,0)) * (-sin(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4))));
            _Lfoot_ra(1, walktime - walktimex) =  -0.025*(_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-1))/(_footx_max) * (4*M_PI / (_ts(_bjx1-1,0)) * 4*M_PI / (_ts(_bjx1-1,0)) * (-cos(4*M_PI / (_ts(_bjx1-1,0)) * (t_desxx + 2*_td(_bjx1-1,0)/4))));                      
          } */  

          _Lfoot_r(1, walktime - walktimex) =  0;          
        /*	_Lfoot_rv(1, walktime - walktimex) = 0;
          _Lfoot_ra(1, walktime - walktimex) = 0; */  
        } 
        _Lfoot_r(2, walktime - walktimex) =  -0.15*(_footxyz_real(1,_bjx1)-_footxyz_real(1,_bjx1-1))/(_footy_max) * (cos(4*M_PI / (_ts(_bjx1-1)) * (t_desxx + 2*_td(_bjx1-1)/4) ) - 1) * sing_y;                       
      } 
    }
    
    
    /////Rfoot,xyz, Lfoot,XYZ
    footlr_r_inte(0+ 6*(walktime - walktimex),0) = _Rfoot_r(0, walktime - walktimex);
    footlr_r_inte(1+ 6*(walktime - walktimex),0) = _Rfoot_r(1, walktime - walktimex);
    footlr_r_inte(2+ 6*(walktime - walktimex),0) = _Rfoot_r(2, walktime - walktimex);

    footlr_r_inte(3+ 6*(walktime - walktimex),0) = _Lfoot_r(0, walktime - walktimex);
    footlr_r_inte(4+ 6*(walktime - walktimex),0) = _Lfoot_r(1, walktime - walktimex);
    footlr_r_inte(5+ 6*(walktime - walktimex),0) = _Lfoot_r(2, walktime - walktimex);    
    
    
  }


 

  return footlr_r_inte;

}






int PRMPCClass::sign_function(double x)
{
  int y;
  if(x>=0)
  {
    y = 1;
  }
  else
  {
     y = -1;
  }
  return y;

}

////////////////////////////////////////////////////// KMP : pos+vel generation
Eigen::Matrix<double,6,1> PRMPCClass::XGetSolution_Foot_position_KMP(int walktime, double dt_sample)
{
  
  ///////walktime=====>ij;   int j_index====>i;  dt_sample========>dtx;   
   Eigen::Matrix<double,6,1> com_inte;	
 
  double  Footz_ref = _lift_height_ref(_bjx1-1);    //0.05m 
 
  double t_kmp_demo = 0.7;   
  //// three via_points: time, mean, sigma 
  vec via_point1 =  zeros<vec>(43);
  vec via_point2 =  zeros<vec>(43);
  vec via_point3 =  zeros<vec>(43);
  
  via_point1(7) =0.00000000001; via_point1(14)=0.00000000001; via_point1(21)=0.00000000001;	
  via_point1(28)=0.00000000001; via_point1(35)=0.00000000001; via_point1(42)=0.00000000001;  
  
  via_point2(0) =t_kmp_demo/2;
  via_point2(7) =0.00000000001; via_point2(14)=0.00000000001; via_point2(21)=0.00000000001;	
  via_point2(28)=0.00000000001; via_point2(35)=0.00000000001; via_point2(42)=0.00000000001;
  
  via_point3(0) =t_kmp_demo;
  via_point3(7) =0.00000000001; via_point3(14)=0.00000000001; via_point3(21)=0.00000000001;	
  via_point3(28)=0.00000000001; via_point3(35)=0.00000000001; via_point3(42)=0.00000000001;      
  
  
  
  double t_des;      /////////desired time during the current step
  t_des = (walktime+1)*dt_sample - (_tx(_bjx1-1)+_td(_bjx1-1));
  //cout << "t_des:"<< t_des<<endl;
  

  if ((_bjx1 >= 2)&&(walktime <=_t_end_footstep))
  {      
    
    if (_bjx1 % 2 == 0)           //odd:left support
    {
      _Lfootx_kmp(0) = _footxyz_real(0,_bjx1-1);
      _Lfooty_kmp(0) = _footxyz_real(1,_bjx1-1);
      _Lfootz_kmp(0) = _footxyz_real(2,_bjx1-1);
      
      _Lfootvx_kmp(0) = 0;
      _Lfootvy_kmp(0) = 0;
      _Lfootvz_kmp(0) = 0;    
      
      if (t_des<=0)  // j_index and _bjx1 coincident with matlab: double support
      {

        _Rfootx_kmp(0) = _footxyz_real(0,_bjx1-2);
        _Rfooty_kmp(0) = _footxyz_real(1,_bjx1-2);
        _Rfootz_kmp(0) = _footxyz_real(2,_bjx1-2);
        
        _Rfootvx_kmp(0) = 0;
        _Rfootvy_kmp(0) = 0;
        _Rfootvz_kmp(0) = 0;  
	
      }
      else
      {
        
	if (t_des>=(_ts(_bjx1-1) - dt_sample))  // j_index and _bjx1 coincident with matlab: double support
	{
	  _Rfootx_kmp(0) = _footxyz_real(0,_bjx1);
	  _Rfooty_kmp(0) = _footxyz_real(1,_bjx1);
	  _Rfootz_kmp(0) = _footxyz_real(2,_bjx1);	  
	}
	else
	{
	  
        //initial state and final state and the middle state
        double t_des_k;
        t_des_k = (t_kmp_demo)/((_ts(_bjx1-1)-_td(_bjx1-1)))*t_des;
        
        /////////////first sampling time of the current walking cycle: initialize the KMP_data
        if (t_des<=dt_sample)
        {
            kmp_leg_R.kmp_initialize(_data_kmp, _inDim_kmp, _outDim_kmp, _pvFlag_kmp,_lamda_kmp, _kh_kmp);
	}
        //// %%% initial state and final state and the middle state
        /////%%%%% be careful the x position start from zero, the y
        //// %%%%% position start from -0.0726
            ////// add point************ current status****************////////
            via_point1(0) = (t_kmp_demo)/((_ts(_bjx1-1)-_td(_bjx1-1)))*(t_des-dt_sample);
            via_point1(1) = _Rfootx_kmp(0)-_footxyz_real(0,_bjx1-2);
            via_point1(2) = _Rfooty_kmp(0)-(_footxyz_real(1,_bjx1-2)-(-0.0726));
            via_point1(3) = _Rfootz_kmp(0)-_footxyz_real(2,_bjx1-2);
            via_point1(4) = _Rfootvx_kmp(0);
            via_point1(5) = _Rfootvy_kmp(0);
            via_point1(6) = _Rfootvz_kmp(0);
            kmp_leg_R.kmp_insertPoint(via_point1);  // insert point into kmp
            
            ////// add point************ middle point***********////////
    // 	    via_point2(1) = _footxyz_real(0,_bjx1-1)-_footxyz_real(0,_bjx1-2);
            via_point2(1) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/2;
            via_point2(2) = (_footxyz_real(1,_bjx1-2)+_footxyz_real(1,_bjx1))/2-(_footxyz_real(1,_bjx1-2)-(-0.0726));
            via_point2(3) = (_footxyz_real(2,_bjx1-1)+Footz_ref)-_footxyz_real(2,_bjx1-2);
            via_point2(4) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/t_kmp_demo*1.15;
            if (_bjx1==2)
            {
            via_point2(4) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/t_kmp_demo*0.8;
            }	    
            via_point2(5) = (_footxyz_real(1,_bjx1)-_footxyz_real(1,_bjx1-2))/t_kmp_demo;
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
            via_point3(3) = _footxyz_real(2,_bjx1)-_footxyz_real(2,_bjx1-2)+0.001;
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

	    _query_kmp(0) = t_des_k;
	    kmp_leg_R.kmp_prediction(_query_kmp,_mean_kmp);   ////////time& predictive values
    /*	    if (t_des_k<0.01){
	    cout<<"_query_kmp:"<<endl<<trans(_query_kmp)<<endl;
	    cout<<"kmp:"<<endl<<trans(_mean_kmp)<<endl;
	    cout<<"error:"<<trans(_mean_kmp)-trans(via_point1(span(1,6)))<<endl<<endl;
	      
	      
	    }*/	  
	  


	  
	  _Rfootx_kmp(0) = _mean_kmp(0)+_footxyz_real(0,_bjx1-2);
	  _Rfooty_kmp(0) = _mean_kmp(1)+(_footxyz_real(1,_bjx1-2)-(-0.0726));
	  _Rfootz_kmp(0) = _mean_kmp(2)+_footxyz_real(2,_bjx1-2);
	  
	  _Rfootvx_kmp(0) = _mean_kmp(3);
	  _Rfootvy_kmp(0) = _mean_kmp(4);
	  _Rfootvz_kmp(0) = _mean_kmp(5); 	 	  
	  
	}
	
 
	  
	
	
      }   
    }
    else                       //right support
    {
      _Rfootx_kmp(0) = _footxyz_real(0,_bjx1-1);
      _Rfooty_kmp(0) = _footxyz_real(1,_bjx1-1);
      _Rfootz_kmp(0) = _footxyz_real(2,_bjx1-1);
      
      _Rfootvx_kmp(0) = 0;
      _Rfootvy_kmp(0) = 0;
      _Rfootvz_kmp(0) = 0;    
      
      if (t_des<=0)  // j_index and _bjx1 coincident with matlab: double support
      {

        _Lfootx_kmp(0) = _footxyz_real(0,_bjx1-2);
        _Lfooty_kmp(0) = _footxyz_real(1,_bjx1-2);
        _Lfootz_kmp(0) = _footxyz_real(2,_bjx1-2);
        
        _Lfootvx_kmp(0) = 0;
        _Lfootvy_kmp(0) = 0;
        _Lfootvz_kmp(0) = 0;  
	
      }
      else
      {
	
	if (t_des>=(_ts(_bjx1-1) - dt_sample))  // j_index and _bjx1 coincident with matlab: double support
	{
	  _Lfootx_kmp(0) = _footxyz_real(0,_bjx1);
	  _Lfooty_kmp(0) = _footxyz_real(1,_bjx1);
	  _Lfootz_kmp(0) = _footxyz_real(2,_bjx1);	
	}
	else
	{
	    //initial state and final state and the middle state
	    double t_des_k;
	    t_des_k = (t_kmp_demo)/((_ts(_bjx1-1)-_td(_bjx1-1)))*t_des;
	    
	    if (t_des<=dt_sample)
	    {	
		kmp_leg_L.kmp_initialize(_data_kmp, _inDim_kmp, _outDim_kmp, _pvFlag_kmp,_lamda_kmp, _kh_kmp);
	    }
                //// %%% initial state and final state and the middle state
                /////%%%%% be careful the x position start from zero, the y
                //// %%%%% position start from -0.0726
            ////// add point************ current status****************////////
            via_point1(0) = (t_kmp_demo)/((_ts(_bjx1-1)-_td(_bjx1-1)))*(t_des-dt_sample);
            via_point1(1) = _Lfootx_kmp(0)-_footxyz_real(0,_bjx1-2);
            via_point1(2) = _Lfooty_kmp(0)-(_footxyz_real(1,_bjx1-2)-(-0.0726));
            via_point1(3) = _Lfootz_kmp(0)-_footxyz_real(2,_bjx1-2);
            via_point1(4) = _Lfootvx_kmp(0);
            via_point1(5) = _Lfootvy_kmp(0);
            via_point1(6) = _Lfootvz_kmp(0);
            kmp_leg_L.kmp_insertPoint(via_point1);  // insert point into kmp
            
            ////// add point************ middle point***********////////
        // 	  via_point2(1) = _footxyz_real(0,_bjx1-1)-_footxyz_real(0,_bjx1-2);
            via_point2(1) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/2;
            via_point2(2) = (_footxyz_real(1,_bjx1-2)+_footxyz_real(1,_bjx1))/2-(_footxyz_real(1,_bjx1-2)-(-0.0726));
            via_point2(3) = (_footxyz_real(2,_bjx1-1)+Footz_ref)-_footxyz_real(2,_bjx1-2);
            via_point2(4) = (_footxyz_real(0,_bjx1)-_footxyz_real(0,_bjx1-2))/t_kmp_demo*1.45;
            via_point2(5) = (_footxyz_real(1,_bjx1)-_footxyz_real(1,_bjx1-2))/t_kmp_demo;
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
            via_point3(3) = _footxyz_real(2,_bjx1)-_footxyz_real(2,_bjx1-2)+0.001;
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
        
        
        
	
	  _query_kmp(0) = t_des_k;
	  kmp_leg_L.kmp_prediction(_query_kmp,_mean_kmp);   ////////time& predictive values
	  
      // 	  if (t_des_k<0.01){
      // 	  cout<<"_query_kmp:"<<endl<<trans(_query_kmp)<<endl;
      // 	  cout<<"kmp:"<<endl<<trans(_mean_kmp)<<endl;
      // 	  cout<<"error:"<<endl<<trans(_mean_kmp)-trans(via_point1(span(1,6)))<<endl;	
      // 	  }	
	  
	  _Lfootx_kmp(0) = _mean_kmp(0)+_footxyz_real(0,_bjx1-2);
	  _Lfooty_kmp(0) = _mean_kmp(1)+(_footxyz_real(1,_bjx1-2)-(-0.0726));
	  _Lfootz_kmp(0) = _mean_kmp(2)+_footxyz_real(2,_bjx1-2);
	  
	  _Lfootvx_kmp(0) = _mean_kmp(3);
	  _Lfootvy_kmp(0) = _mean_kmp(4);
	  _Lfootvz_kmp(0) = _mean_kmp(5); 	  
	  
	}
	
	  

	
	
      }   

    }
      
    
  }
  
  
  /////Rfoot,xyz, Lfoot,XYZ
  com_inte(0) = _Rfootx_kmp(0);
  com_inte(1) = _Rfooty_kmp(0);
  com_inte(2) = _Rfootz_kmp(0);

  com_inte(3) = _Lfootx_kmp(0);
  com_inte(4) = _Lfooty_kmp(0);
  com_inte(5) = _Lfootz_kmp(0);

  return com_inte;

}

////////////////////////////////////////////////////// KMP : pos generation
Eigen::Matrix<double,6,1> PRMPCClass::XGetSolution_Foot_position_KMP_faster(int walktime, double dt_sample)
{
  
  ///////walktime=====>ij;   int j_index====>i;  dt_sample========>dtx;   
   Eigen::Matrix<double,6,1> com_inte;	
 
  double  Footz_ref = _lift_height_ref(_bjx1-1);    //0.05m 
  
  double t_kmp_demo = 0.65;  
  //// three via_points: time, mean, sigma 
  vec via_point1 =  zeros<vec>(13);
  vec via_point2 =  zeros<vec>(13);
  vec via_point3 =  zeros<vec>(13);
  
  via_point1(4) =0.00000000001; via_point1(8)=0.00000000001; via_point1(12)=0.00000000001;	
  
  via_point2(0) =t_kmp_demo/2;
  via_point2(4) =0.00000000001; via_point2(8)=0.00000000001; via_point2(12)=0.00000000001;	
  
  via_point3(0) =0.7;
  via_point3(4) =0.00000000001; via_point3(8)=0.00000000001; via_point3(12)=0.00000000001;	    
  
  
  
  double t_des;      /////////desired time during the current step
  t_des = (walktime+1)*dt_sample - (_tx(_bjx1-1)+_td(_bjx1-1));
  //cout << "t_des:"<< t_des<<endl;
  

  if ((_bjx1 >= 2)&&((walktime <=_t_end_footstep)))
  {      
    
    if (_bjx1 % 2 == 0)           //odd:left support
    {
      _Lfootx_kmp(0) = _footxyz_real(0,_bjx1-1);
      _Lfooty_kmp(0) = _footxyz_real(1,_bjx1-1);
      _Lfootz_kmp(0) = _footxyz_real(2,_bjx1-1);  
      
      if (t_des<=0)  // j_index and _bjx1 coincident with matlab: double support
      {
        _Rfootx_kmp(0) = _footxyz_real(0,_bjx1-2);
        _Rfooty_kmp(0) = _footxyz_real(1,_bjx1-2);
        _Rfootz_kmp(0) = _footxyz_real(2,_bjx1-2);
      }
      else
      {
	if (t_des>=(_ts(_bjx1-1) - 2*dt_sample))  // j_index and _bjx1 coincident with matlab: double support
	{
	  _Rfootx_kmp(0) = _footxyz_real(0,_bjx1);
	  _Rfooty_kmp(0) = _footxyz_real(1,_bjx1);
	  _Rfootz_kmp(0) = _footxyz_real(2,_bjx1);	  
	}
	else
	{
	  //initial state and final state and the middle state
	  double t_des_k;
	  t_des_k = (t_kmp_demo)/((_ts(_bjx1-1)-_td(_bjx1-1)))*t_des;
	  
	  /////////////first sampling time of the current walking cycle: initialize the KMP_data
	  if (t_des<=dt_sample)
	  {
	      kmp_leg_R.kmp_initialize(_data_kmp, _inDim_kmp, _outDim_kmp, _pvFlag_kmp,_lamda_kmp, _kh_kmp);
	  }
	  //// %%% initial state and final state and the middle state
	  /////%%%%% be careful the x position start from zero, the y
	  //// %%%%% position start from -0.0726
	      ////// add point************ current status****************////////
	      via_point1(0) = (t_kmp_demo)/((_ts(_bjx1-1)-_td(_bjx1-1)))*(t_des-dt_sample);
	      via_point1(1) = _Rfootx_kmp(0)-_footxyz_real(0,_bjx1-2);
	      via_point1(2) = _Rfooty_kmp(0)-(_footxyz_real(1,_bjx1-2)-(-0.0726));
	      via_point1(3) = _Rfootz_kmp(0)-_footxyz_real(2,_bjx1-2);
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

	 _Rfootx_kmp(0) = _mean_kmp(0)+_footxyz_real(0,_bjx1-2);
	 _Rfooty_kmp(0) = _mean_kmp(1)+(_footxyz_real(1,_bjx1-2)-(-0.0726));
	 _Rfootz_kmp(0) = _mean_kmp(2)+_footxyz_real(2,_bjx1-2);	  
	}

      }   
    }
    else                       //right support
    {
      _Rfootx_kmp(0) = _footxyz_real(0,_bjx1-1);
      _Rfooty_kmp(0) = _footxyz_real(1,_bjx1-1);
      _Rfootz_kmp(0) = _footxyz_real(2,_bjx1-1);    
      
      if (t_des<=0)  // j_index and _bjx1 coincident with matlab: double support
      {
        _Lfootx_kmp(0) = _footxyz_real(0,_bjx1-2);
        _Lfooty_kmp(0) = _footxyz_real(1,_bjx1-2);
        _Lfootz_kmp(0) = _footxyz_real(2,_bjx1-2);	
      }
      else
      {	
	if (t_des>=(_ts(_bjx1-1) - 2*dt_sample))  // j_index and _bjx1 coincident with matlab: double support
	{
	  _Lfootx_kmp(0) = _footxyz_real(0,_bjx1);
	  _Lfooty_kmp(0) = _footxyz_real(1,_bjx1);
	  _Lfootz_kmp(0) = _footxyz_real(2,_bjx1);	  
	}	
	else
	{
	  //initial state and final state and the middle state
	  double t_des_k;
	  t_des_k = (t_kmp_demo)/((_ts(_bjx1-1)-_td(_bjx1-1)))*t_des;
	  
	  if (t_des<=dt_sample)
	  {	
	      kmp_leg_L.kmp_initialize(_data_kmp, _inDim_kmp, _outDim_kmp, _pvFlag_kmp,_lamda_kmp, _kh_kmp);
	  }
		  //// %%% initial state and final state and the middle state
		  /////%%%%% be careful the x position start from zero, the y
		  //// %%%%% position start from -0.0726
	      ////// add point************ current status****************////////
	      via_point1(0) = (t_kmp_demo)/((_ts(_bjx1-1)-_td(_bjx1-1)))*(t_des-dt_sample);
	      via_point1(1) = _Lfootx_kmp(0)-_footxyz_real(0,_bjx1-2);
	      via_point1(2) = _Lfooty_kmp(0)-(_footxyz_real(1,_bjx1-2)-(-0.0726));
	      via_point1(3) = _Lfootz_kmp(0)-_footxyz_real(2,_bjx1-2);
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
	  
	  _Lfootx_kmp(0) = _mean_kmp(0)+_footxyz_real(0,_bjx1-2);
	  _Lfooty_kmp(0) = _mean_kmp(1)+(_footxyz_real(1,_bjx1-2)-(-0.0726));
	  _Lfootz_kmp(0) = _mean_kmp(2)+_footxyz_real(2,_bjx1-2);		  
	}

      }   

    }
      
    
  }
  
  
  /////Rfoot,xyz, Lfoot,XYZ
  com_inte(0) = _Rfootx_kmp(0);
  com_inte(1) = _Rfooty_kmp(0);
  com_inte(2) = _Rfootz_kmp(0);

  com_inte(3) = _Lfootx_kmp(0);
  com_inte(4) = _Lfooty_kmp(0);
  com_inte(5) = _Lfootz_kmp(0);

  return com_inte;

}
