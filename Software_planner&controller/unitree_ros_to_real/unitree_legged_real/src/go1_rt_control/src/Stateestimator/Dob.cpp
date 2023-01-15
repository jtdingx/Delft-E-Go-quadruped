/*****************************************************************************
DisturbanceobserverClass.cpp

Description:	cpp file of DisturbanceobserverClass


*****************************************************************************/
#include "Dob.h"


using namespace Eigen;
using namespace std;


namespace gait{
  DisturbanceobserverClass::DisturbanceobserverClass()
		:_g(9.8)
		,_dt(0.001)
  {
	/// estimated state
	_comxk_est.setZero();
	_comyk_est.setZero();
	_comzk_est.setZero();
	_comzk_est(0) = _z_c;
	_thetaxk_est.setZero();
	_thetayk_est.setZero();
	//estimated disturbance
	_dis_est.setZero();  
	
	
	//// hippose reference
	_hip_pos_mean.setZero();
	
      _L1.setZero();
      _L2.setZero();
      _L3.setZero();
      
      _u1 = 0;
      _u2 = 0; 
      _u3 = 0;
      _Delta.setZero();
	
      _L_com.setZero();
      _L_com_old.setZero();
      
      _z_c = Z_c;
      _rbot_mas = mass;
	
      x1x =1.0/3.0;	
      x2x =2.0/3.0;
      x3x =1.0;
      L1x.setZero();
      L2x.setZero();
      L3x.setZero();
      
      X1_s.setZero(); 
      X1_real.setZero();
      X2_s.setZero();	      
      fn_est = _rbot_mas * _g;
      
      F.setZero();
      det_L_com.setZero();
      h_lip = _z_c;
  }


  DisturbanceobserverClass::~DisturbanceobserverClass()
  {

  }

  Matrix<double,15,1> DisturbanceobserverClass::hosomo_dob(int i_count,Vector3d gcom_mea,Vector3d cop_mea,Vector3d imu_mea, 
							   Vector3d IMU_anglacc,Vector3d _L_com_mea, 
							   double FT_fl_filter, double FT_fr_filter, double J_ini_xx_est, double J_ini_yy_est,
							   Vector3d comv_mea, Vector3d thetav_mea)
  {
    
    Matrix<double,15,1> est_result;
    
    //// note ///////
    if (i_count <5)
    {
      _comxk_est(0) = gcom_mea(0);     _comxk_est(1) = comv_mea(0);
      _comyk_est(0) = gcom_mea(1);     _comyk_est(1) = comv_mea(1);
      _comzk_est(0) = gcom_mea(2);     _comzk_est(1) = comv_mea(2);  
      _thetaxk_est(0) = imu_mea(0);   _thetaxk_est(1) = thetav_mea(0);
      _thetayk_est(0) = imu_mea(1);   _thetayk_est(1) = thetav_mea(1);       
      
      _L_com_old = _L_com_mea;
    }
    else
    {
      // _Delta should be largers to guarantee the accurary,
      _Delta << 200,200,350,100,100;

      
      
      // u1, u2 should be much larger than u3 to guarantee the stability,
      _u1 = 1.5;
      _u2 = 1;
      _u3 = 0.05;    
      

      L1x = Matrix_expo(_Delta,x1x);
      _L1 = _u1* L1x;
      
      
      L2x = Matrix_expo(_Delta,x2x);  
      _L2 = _u2*sqrt(_u1)* L2x;  
      
      L3x = Matrix_expo(_Delta,x3x);  
      _L3 = _u3* L3x;     
      

      X1_s(0,0) = _comxk_est(0);     X2_s(0,0) = _comxk_est(1);
      X1_s(1,0) = _comyk_est(0);     X2_s(1,0) = _comyk_est(1);
      X1_s(2,0) = _comzk_est(0);     X2_s(2,0) = _comzk_est(1);  
      X1_s(3,0) = _thetaxk_est(0);   X2_s(3,0) = _thetaxk_est(1);
      X1_s(4,0) = _thetayk_est(0);   X2_s(4,0) = _thetayk_est(1);   
      
      X1_real(0,0) = gcom_mea(0);    // X2_s(0,0) = _comxk_est(1);
      X1_real(1,0) = gcom_mea(1);   // X2_s(1,0) = _comyk_est(1);
      X1_real(2,0) = gcom_mea(2);    // X2_s(2,0) = _comzk_est(1);  
      X1_real(3,0) = imu_mea(0);  // X2_s(3,0) = _thetaxk_est(1);
      X1_real(4,0) = imu_mea(1);  // X2_s(4,0) = _thetayk_est(1);    
      
      ////// reference acceleration of the state or the measured results
      fn_est = FT_fl_filter + FT_fr_filter;
      if (fn_est < (_rbot_mas * _g /2))
      {
	fn_est = _rbot_mas * _g /2;
      }
      
      
      ///// using the measured ones
    /*  F(0,0) = gddcom_mea(0);
      F(1,0) = gddcom_mea(1);
      F(2,0) = gddcom_mea(2);
      F(3,0) = IMU_anglacc(0);  
      F(4,0) = IMU_anglacc(1); */ 

    //   ///////////////////////////////////////
    //   //// using the measured parameter to caclute the F--> nonlinear Inverted Pendulum model plus flywheel;
      _L_com = _L_com_mea;
      
      det_L_com = _L_com - _L_com_old;  
      
      /// 
 
      h_lip = (gcom_mea(2) - cop_mea(2));
      if (h_lip < (_z_c-0.1))
      {
	h_lip = (_z_c-0.1);
      }
	
    //   F(0,0) = ((gcom_mea(0) - cop_mea(0))*fn_est - det_L_com(1))/(_rbot_mas*h_lip); 
    //   F(1,0) = ((gcom_mea(1) - cop_mea(1))*fn_est + det_L_com(0))/(_rbot_mas*h_lip); 
    //   F(2,0) = fn_est/_rbot_mas - _g; 
    //   F(3,0) = IMU_anglacc(0);  
    //   F(4,0) = IMU_anglacc(1);  

      F(0,0) = ((_comxk_est(0) - cop_mea(0))*fn_est - det_L_com(1))/(_rbot_mas*h_lip); 
      F(1,0) = ((_comyk_est(0) - cop_mea(1))*fn_est + det_L_com(0))/(_rbot_mas*h_lip); 
      F(2,0) = fn_est/_rbot_mas - _g; 
      F(3,0) = det_L_com(0)/J_ini_xx_est;  
      F(4,0) = det_L_com(1)/J_ini_yy_est;     
      
      
      Eigen::Matrix<double, 5,1> _det_x1_err = X1_real - X1_s;
      
      Eigen::Matrix<double, 5,5> _L1_diag = diag_function(_L1);
      Eigen::Matrix<double, 5,1> _dot_X1_s2 = sig_function(_det_x1_err,x2x);
      Eigen::Matrix<double, 5,1> _dot_X1_s = X2_s +  _L1_diag * _dot_X1_s2;
      
      Eigen::Matrix<double, 5,5> _L2_diag = diag_function(_L2);    
      Eigen::Matrix<double, 5,1> _dot_X2_s2 = sig_function(_det_x1_err,x1x);
      Eigen::Matrix<double, 5,1> _dot_X2_s = F + _dis_est + _L2_diag * _dot_X2_s2;
      
      Eigen::Matrix<double, 5,5> _L3_diag = diag_function(_L3);    
      Eigen::Matrix<double, 5,1> _dot_d_is2 = sgn_function(_det_x1_err);  
      Eigen::Matrix<double, 5,1> _dot_d_is = _L3_diag * _dot_d_is2;
      
      /// disturbance_est
      
      _dis_est += _dot_d_is * _dt;
      
      /// estimiated postion
      _comxk_est(0) +=  _dt * _dot_X1_s(0,0)  + 0.5 * pow(_dt, 2) * _dot_X2_s(0,0); 
      _comyk_est(0) +=  _dt * _dot_X1_s(1,0)  + 0.5 * pow(_dt, 2) * _dot_X2_s(1,0);
      _comzk_est(0) +=  _dt * _dot_X1_s(2,0)  + 0.5 * pow(_dt, 2) * _dot_X2_s(2,0);  
      _thetaxk_est(0) +=  _dt * _dot_X1_s(3,0)  + 0.5 * pow(_dt, 2) * _dot_X2_s(3,0);  
      _thetayk_est(0) +=  _dt * _dot_X1_s(4,0)  + 0.5 * pow(_dt, 2) * _dot_X2_s(4,0);    
    
      
      _comxk_est(1) +=  _dt * _dot_X2_s(0,0); 
      _comyk_est(1) +=  _dt * _dot_X2_s(1,0);
      _comzk_est(1) +=  _dt * _dot_X2_s(2,0);  
      _thetaxk_est(1) +=  _dt * _dot_X2_s(3,0);  
      _thetayk_est(1) +=  _dt * _dot_X2_s(4,0);   
      
      _comxk_est(2) =  _dot_X2_s(0,0); 
      _comyk_est(2) =  _dot_X2_s(1,0);
      _comzk_est(2) =  _dot_X2_s(2,0);  
      _thetaxk_est(2) =  _dot_X2_s(3,0);  
      _thetayk_est(2) =  _dot_X2_s(4,0); 
      
      
	_L_com_old = _L_com_mea;      
      
      
    }



      est_result(0) = _comxk_est(0) ;    
      est_result(1) = _comyk_est(0) ;    
      est_result(2) = (_comzk_est(0)+ cop_mea(2)); 
      est_result(3) = _comxk_est(1);     
      est_result(4) = _comyk_est(1);    
      est_result(5) = _comzk_est(1); 
      est_result(6) = _thetaxk_est(0);
      est_result(7) = _thetayk_est(0);
      est_result(8) = _thetaxk_est(1); 
      est_result(9) = _thetayk_est(1);
      est_result(10) = _dis_est(0) * _rbot_mas;    
      est_result(11) = _dis_est(1) * _rbot_mas;    
      est_result(12) = _dis_est(2) * _rbot_mas;  
      est_result(13) = _dis_est(3) * _rbot_mas;
      est_result(14) = _dis_est(4) * _rbot_mas; 
    
    
    return est_result;
    
  }



  Eigen::Matrix<double, 5,1> DisturbanceobserverClass::Matrix_expo(Eigen::Matrix<double, 5,1> ma, double lamda)
  {
    Eigen::Matrix<double, 5,1> matx;
    for (int j=0;j<5;j++)
    {
      matx(j,0) = pow(ma(j,0), lamda);
    }
    
    return matx;
  }


  Eigen::Matrix<double, 5,1> DisturbanceobserverClass::sig_function(Eigen::Matrix<double, 5,1> ma, double lamda)
  {
    Eigen::Matrix<double, 5,1> matx;
    int sym_par =1;
    
    for (int j=0;j<5;j++)
    {
      sym_par = ((ma(j,0)>0)-(ma(j,0)<0));
      
      matx(j,0) = pow(abs(ma(j,0)), lamda)*sym_par;
    }
    
    return matx;  

  }

  Eigen::Matrix<double, 5,1> DisturbanceobserverClass::sgn_function(Eigen::Matrix<double, 5,1> ma)
  {
    Eigen::Matrix<double, 5,1> matx;
    for (int j=0;j<5;j++)
    {
      matx(j,0) = ((ma(j,0)>0)-(ma(j,0)<0));
    }    

    return matx;
  }

  Eigen::Matrix<double, 5,5> DisturbanceobserverClass::diag_function(Eigen::Matrix<double, 5,1> ma)
  {
    Eigen::Matrix<double, 5,5> matx;
    matx.setZero();
    
    for (int j=0;j<5;j++)
    {
      matx(j,j) = ma(j,0);
    }    

    return matx;
  }


  /*
  #ifdef USE_XBOT_LOGGER
  void DisturbanceobserverClass::initLogger(XBot::MatLogger::Ptr xbot_logger, int buffer_size, int interleave)
  {
	  xbot_logger->createVectorVariable(_name + "_" + "dob_comx_state", 3, interleave, buffer_size);
	  xbot_logger->createVectorVariable(_name + "_" + "dob_comy_state", 3, interleave, buffer_size);
	  xbot_logger->createVectorVariable(_name + "_" + "dob_comz_state", 3, interleave, buffer_size);
	  xbot_logger->createVectorVariable(_name + "_" + "dob_thetax_state", 3, interleave, buffer_size);
	  xbot_logger->createVectorVariable(_name + "_" + "dob_thetay_state", 3, interleave, buffer_size);
  }

  void DisturbanceobserverClass::addToLog(XBot::MatLogger::Ptr xbot_logger)
  {
	  xbot_logger->add(_name + "_" + "dob_comx_state", Ft_force_diff_ref);
	  xbot_logger->add(_name + "_" + "dob_comy_state", Ft_force_diff_msr);
	  xbot_logger->add(_name + "_" + "dob_comz_state", deltaFtPos);
	  xbot_logger->add(_name + "_" + "dob_thetax_state", deltaFtAng_l);
	  xbot_logger->add(_name + "_" + "dob_thetay_state", deltaFtAng_r);

  }
  #endif*/
}



