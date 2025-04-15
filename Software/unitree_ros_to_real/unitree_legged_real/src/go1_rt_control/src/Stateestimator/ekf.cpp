/*****************************************************************************
EkfestimatorClass.cpp

Description:	cpp file of EkfestimatorClass
*****************************************************************************/
#include "ekf.h"

namespace gait{
    EkfestimatorClass::EkfestimatorClass()
    {

      _z_c = Z_c + 0.1;
      _dt = t_program_cyclic;       
      fz_total_mea = 0;
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
      
      
      X_t.setZero();
      hat_X_t.setZero();
      u_t.setZero();
      P_cov.setZero();
      hat_P_j.setZero();

      P_cov = diag_function(0.00001,0.0000001,0.001);    
    }


    EkfestimatorClass::~EkfestimatorClass()
    {

    }

    void EkfestimatorClass::ekf_est(const int count_mpc, const sensor_msgs::Imu body_imu, const double *com_sensor, const double *comv_sensor, const double *coma_sensor,
                                    const double *zmp_sensor, const geometry_msgs::Wrench left_ft_, const geometry_msgs::Wrench right_ft_)
    {
      if (count_mpc<2)
      {
        _comxk_est(0) = com_sensor[0];
        _comxk_est(1) = comv_sensor[0];
        _comyk_est(0) = com_sensor[1];
        _comyk_est(1) = comv_sensor[1];
        _comzk_est(0) = com_sensor[2];
        _comzk_est(1) = comv_sensor[2];
        
      }
      X_t(0,0) = _comxk_est(0);     
      X_t(3,0) = _comxk_est(1);
      X_t(1,0) = _comyk_est(0);     
      X_t(4,0) = _comyk_est(1);
      X_t(2,0) = _comzk_est(0);     
      X_t(5,0) = _comzk_est(1);  

    

      // Eigen::Vector3d hip_l = irobot.glft - irobot.IMU_abs * irobot.Lft;
      // Eigen::Vector3d hip_r = irobot.grft - irobot.IMU_abs * irobot.Rft;
      // double Fz_ratio_l = irobot.Fzl / (irobot.Fzl + irobot.Fzr);
      // double Fz_ratio_r = irobot.Fzr / (irobot.Fzl + irobot.Fzr);
      // _hip_pos_mean = Fz_ratio_l * hip_l + Fz_ratio_r * hip_r;
      

    //   Eigen::Vector3d cop_mea = irobot.gcop;
    //   Eigen::Vector3d gddcom_mea = irobot.gddcom;
    //   Eigen::Vector3d IMU_anglacc = irobot.IMU_AngularAcc;
    //   ////// reference acceleration of the state or the measured results
    //   double fn_est = irobot.FT_foot_left(2,0)+irobot.FT_foot_right(2,0);  
      
      
      fz_total_mea = left_ft_.force.z + right_ft_.force.z;
      u_t(0,0) = zmp_sensor[0];   
      u_t(1,0) = zmp_sensor[1];   
      u_t(2,0) = butterforcefilter.ForceFilter(fz_total_mea);  
      
      
      // EKF process:
    
      /// process model
      Eigen::Matrix3d I3, Z3, Ct, Dt;
      I3 << 1,0,0,
            0,1,0,
    	    0,0,1;
      
      Z3.setZero();
      Ct.setZero();
      
      Ct(0,0) = (u_t(2,0)+X_t(8,0))/(mass*X_t(2,0));
      Ct(0,2) = -(u_t(2,0)+X_t(8,0))*(X_t(0,0)-u_t(0,0))/(mass*pow(X_t(2,0),2));
      Ct(1,1) = (u_t(2,0)+X_t(8,0))/(mass*X_t(2,0));
      Ct(1,2) = -(u_t(2,0)+X_t(8,0))*(X_t(1,0)-u_t(1,0))/(mass*pow(X_t(2,0),2));
      
      //std::cout<< "Ct"<<Ct<<endl;
      
      Dt.setZero();
      Dt(0,0) = 1/mass;
      Dt(0,2) = (X_t(0,0)-u_t(0,0))/(mass*X_t(2,0));
      Dt(1,1) = 1/mass;
      Dt(1,2) = (X_t(1,0)-u_t(1,0))/(mass*X_t(2,0));
      Dt(2,2) = 1/mass;
      
      //std::cout<< "Dt"<<Dt<<endl;  
      
      
      Eigen::Matrix<double, 9,9> Gt, Gk;
      Gt.setZero();
      Gk.setIdentity(9,9);
      
      Gt.block<3, 3>(0, 0) = Z3;
      Gt.block<3, 3>(0, 3) = I3;
      Gt.block<3, 3>(0, 6) = Z3;
      Gt.block<3, 3>(3, 0) = Ct;
      Gt.block<3, 3>(3, 3) = Z3;
      Gt.block<3, 3>(3, 6) = Dt;
      Gt.block<3, 3>(6, 0) = Z3;
      Gt.block<3, 3>(6, 3) = Z3;
      Gt.block<3, 3>(6, 6) = Z3;  
      
      /*std::cout<< "Gt"<<Gt<<endl; */ 
      
      Gk += Gt*_dt;
      
      //std::cout<< "Gk"<<Gk<<endl;  
      
      Eigen::Matrix<double, 9,1> f_state;
      f_state.setZero();
      f_state(0,0) = X_t(3,0);
      f_state(1,0) = X_t(4,0);
      f_state(2,0) = X_t(5,0);
      f_state(3,0) = (u_t(2,0)+X_t(8,0))*(X_t(0,0) - u_t(0,0))/(mass*X_t(2,0)) + X_t(6,0)/mass;
      f_state(4,0) = (u_t(2,0)+X_t(8,0))*(X_t(1,0) - u_t(1,0))/(mass*X_t(2,0)) + X_t(7,0)/mass;
      f_state(5,0) = (u_t(2,0)+X_t(8,0))/mass - _g; 
      
      // state estimation
      hat_X_t = f_state*_dt + X_t;
      
      
      // covariance estimation
      Eigen::Matrix<double,9,9> Q;
      Q = diag_function(0.001,0.01,0.2); // the higher, the better 
      
      hat_P_j = Gk*P_cov*Gk.transpose() + Q;
      
      /////////////////////////correction Process///////////////////////////////////////
      Eigen::Matrix<double,6,9>  Hk;
      Hk.setZero();
      Hk.block<3, 3>(0, 0) = I3;
      Hk.block<3, 3>(0, 3) = Z3;
      Hk.block<3, 3>(0, 6) = Z3;
      Hk.block<3, 3>(3, 0) = Ct;
      Hk.block<3, 3>(3, 3) = Z3;
      Hk.block<3, 3>(3, 6) = Dt; 
      
      Eigen::Matrix<double,6,6>  R;
      R.setZero();
      R.block<3, 3>(0, 0) = 0.00000000001*I3; // the lower,the better
      R.block<3, 3>(3, 3) = 0.00000000005*I3;
      
      //std::cout<< "R"<<R<<endl;    
      
      ///kalmam gain
      Eigen::Matrix<double,9,6> Kj;
      Kj.setZero();
      
      Eigen::Matrix<double,6,6> k_inv;
      k_inv = (Hk*hat_P_j*(Hk.transpose())+R).inverse();
      
      Kj = hat_P_j * Hk.transpose() * k_inv;
      
      
      //measurment: cx, cy, cz, accx, accy, accz
      Eigen::Matrix<double, 6,1> Y, h_Y;
      Y(0,0) = com_sensor[0];
      Y(1,0) = com_sensor[1];
      Y(2,0) = com_sensor[2];
//       Y(3,0) = body_imu.linear_acceleration.x ;
//       Y(4,0) = body_imu.linear_acceleration.y;
//       Y(5,0) = body_imu.linear_acceleration.z;
      Y(3,0) = coma_sensor[0];
      Y(4,0) = coma_sensor[1];
      Y(5,0) = coma_sensor[2];      
      
      h_Y(0,0) = hat_X_t(0,0);
      h_Y(1,0) = hat_X_t(1,0);
      h_Y(2,0) = hat_X_t(2,0);
      h_Y(3,0) = (u_t(2,0) + hat_X_t(8,0))*(hat_X_t(0,0) - u_t(0,0))/(mass*hat_X_t(2,0)) + hat_X_t(6,0)/mass;
      h_Y(4,0) = (u_t(2,0) + hat_X_t(8,0))*(hat_X_t(1,0) - u_t(1,0))/(mass*hat_X_t(2,0)) + hat_X_t(7,0)/mass;
      h_Y(5,0) = (u_t(2,0) + hat_X_t(8,0))/mass - _g;  
      
      // state correction
      X_t = hat_X_t +  Kj *(Y - h_Y);
      
      ///covariance correction
      Eigen::Matrix<double,9,9> I9;
      I9.setIdentity();
      
      P_cov = (I9 - Kj*Hk) * hat_P_j;
      
      
      ////state_estimation
      _comxk_est(0) = X_t(0,0);   
      _comxk_est(1) = X_t(3,0);
      _comyk_est(0) = X_t(1,0);   
      _comyk_est(1) = X_t(4,0);
      _comzk_est(0) = X_t(2,0);   
      _comzk_est(1) = X_t(5,0);
      _dis_est(0) = X_t(6,0);
      _dis_est(1) = X_t(7,0);
      _dis_est(2) = X_t(8,0);  
    
    }


    Eigen::Matrix< double, 9, 9> EkfestimatorClass::diag_function(double m1, double m2, double m3)
    {
      Eigen::Matrix<double,9,9> p_cov_diag;
      p_cov_diag.setZero();
      
      for (int i=0;i<3; i++)
      {
        p_cov_diag(i,i) = m1;
      }
      
      
      for (int i=3;i<6; i++)
      {
        p_cov_diag(i,i) = m2;
      }
      
      for (int i=6;i<9; i++)
      {
        p_cov_diag(i,i) = m3;
      }  
      
      return p_cov_diag;
      
      
    }
}



