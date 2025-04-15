
#pragma once
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <math.h>

#include <Eigen/Dense>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/multibody/joint/joint-spherical.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/centroidal-derivatives.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"
#include <iostream>
#include "pinocchio/parsers/urdf.hpp"




using namespace std;
using namespace pinocchio;




class WBDClass
{
    public: 
        WBDClass();
        ~WBDClass();
        
        Model model;  
        // Data data(model);

        Eigen::Matrix<double,3,3>  Ig;
        Eigen::MatrixXd Coriolis_M;
        Eigen::VectorXd gravity_M;
        Eigen::VectorXd torque_drift;
        Eigen::VectorXd tau_nle;

        Eigen::Matrix<double,12,1> torque_bias; /////coriolis, centrifual and gravity;
        Eigen::Matrix<double,12,1> torque_gravity; //// generalized gravity;
        Eigen::Matrix<double,12,1> torque_coriolis; //// generalized coriolis;



        // void build_pino_model();

        void computeInertiaMatrix(Eigen::Matrix<double,3,1> base_vel,Eigen::Matrix<double,3,1> base_ang_vel,Eigen::Matrix<double,12,1> q0, Eigen::Matrix<double,12,1> dq0,Eigen::Matrix<double,3,1> base_pos_Ig,Eigen::Quaterniond base_quat_Ig);
        void compute_gravity(Eigen::Matrix<double,3,1> base_vel,Eigen::Matrix<double,3,1> base_ang_vel,Eigen::Matrix<double,12,1> q0, Eigen::Matrix<double,12,1> dq0,Eigen::Matrix<double,3,1> base_pos_Ig,Eigen::Quaterniond base_quat_Ig);

        
 
};




