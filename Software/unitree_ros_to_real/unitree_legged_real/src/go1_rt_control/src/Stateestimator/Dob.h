/*****************************************************************************
DOB.h

Description:	Header file of DisturbanceobserverClass


*****************************************************************************/
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>
#include "/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/src/go1_rt_control/src/Robotpara/robot_const_para_config.h"

// #ifdef USE_XBOT_LOGGER
// #include "XBotInterface/Logger.hpp"
// #endif
using namespace Eigen;
using namespace std;

namespace gait{
  class DisturbanceobserverClass
  {
  public:
	  DisturbanceobserverClass();
	  ~DisturbanceobserverClass();


	  
	  Eigen::Vector3d _comxk_est,_comyk_est, _comzk_est, _thetaxk_est, _thetayk_est, _hip_pos_mean;
	  Eigen::Matrix<double, 5,1> _dis_est;
	  double _rbot_mas, _g, _z_c, _dt;
	  
	  Matrix<double,15,1> hosomo_dob(int i_count,Vector3d gcom_mea,Vector3d cop_mea,Vector3d imu_mea, 
				         Vector3d IMU_anglacc,Vector3d _L_com_mea, 
				         double FT_fl_filter, double FT_fr_filter, double J_ini_xx_est, double J_ini_yy_est,
					 Vector3d comv_mea, Vector3d thetav_mea);	
	  
	  
  protected:
    


  private:
    
	Eigen::Matrix<double, 5,1> _L1, _L2,_L3;
	
	double _u1, _u2, _u3;
	Eigen::Matrix<double, 5,1> _Delta;
	
	Eigen::Matrix<double, 5,1> Matrix_expo(Eigen::Matrix<double, 5,1> ma, double lamda);
	
	Eigen::Matrix<double, 5,1> sig_function(Eigen::Matrix<double, 5,1> ma, double lamda);
	Eigen::Matrix<double, 5,1> sgn_function(Eigen::Matrix<double, 5,1> ma);
	Eigen::Matrix<double, 5,5> diag_function(Eigen::Matrix<double, 5,1> ma);
	

	
	Eigen::Vector3d _L_com, _L_com_old;
	
	double x1x,x2x,x3x;
	Eigen::Matrix<double, 5, 1> L1x,L2x,L3x;
	
	Eigen::Matrix<double, 5, 1> X1_s, X1_real;
	Eigen::Matrix<double, 5, 1> X2_s;
        double fn_est;	
	Eigen::Matrix<double, 5, 1> F;
	
	Eigen::Vector3d det_L_com;
	double h_lip;
	
  };
}

