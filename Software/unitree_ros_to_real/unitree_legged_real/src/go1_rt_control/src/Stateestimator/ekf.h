/*****************************************************************************
DOB.h

Description:	Header file of EkfestimatorClass
*****************************************************************************/
#include "ros/ros.h"
#include "KinematicsApi/kinematics.h"
#include "Filter/butterworth_filter.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>
#include "/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/src/go1_rt_control/src/Robotpara/robot_const_para_config.h"

// #ifdef USE_XBOT_LOGGER
// #include "XBotInterface/Logger.hpp"
// #endif

namespace gait{
      class EkfestimatorClass
      {
      public:
            EkfestimatorClass();
            ~EkfestimatorClass();

            //////
            ButterworthFilter butterforcefilter;

            Eigen::Vector3d _comxk_est,_comyk_est, _comzk_est, _thetaxk_est, _thetayk_est, _dis_est,_hip_pos_mean;
            double _z_c, _dt;
            double fz_total_mea;
            
            void ekf_est(const int count_mpc, const sensor_msgs::Imu body_imu, const double *com_sensor, const double *comv_sensor, const double *coma_sensor,
                         const double *zmp_sensor, const geometry_msgs::Wrench left_ft_, const geometry_msgs::Wrench right_ft_);	


            
            
      protected:
      

      private:
      
      
            ///estimated state: cx, cy,cz, cvx,cvy,cvz, fx, fy,fz
            Eigen::Matrix<double, 9,1> X_t, hat_X_t;
            
            //// measure control input: px,py,fN
            Eigen::Matrix<double, 3,1> u_t;
            
            //// covariance matrix
            Eigen::Matrix<double, 9,9> P_cov, hat_P_j;
            
            
            Eigen::Matrix<double, 9,9> diag_function(double m1, double m2, double m3);
            
            
      };
}


