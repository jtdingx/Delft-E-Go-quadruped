/*****************************************************************************
state_estimator.cpp

Description:    source file of MpcRTControlClass

@Version:   1.0
@Author:    Jiatao Ding
@Release:   Tue 27 Jun 2017 09:33:32 AM CEST
@Update:    Tue 27 Jun 2017 09:33:37 AM CEST
*****************************************************************************/
#include "state_estimator.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>

using namespace Eigen;
using namespace std;

namespace gait{
    StateestimatorClass::StateestimatorClass()
    {
        ////// ***********  end-effectors ********************////
        fz_double = mass * _g /2;
    	_z_c = Z_c + 0.1;
        _dt = t_program_cyclic;
        fz_limit = force_z_limt;
        rfz[0] = fz_limit;
        rfz[1] = fz_limit;
        rfz[2] = fz_limit;
        rfz[3] = fz_limit;
        rfz[4] = fz_limit;
        lfz[0] = fz_limit;
        lfz[1] = fz_limit;
        lfz[2] = fz_limit;
        lfz[3] = fz_limit;
        lfz[4] = fz_limit;

        com_sensor_old[0] = 0;
        com_sensor_old[1] = 0;
        com_sensor_old[2] = 0;

        comv_sensor[0] = 0;
        comv_sensor[1] = 0;
        comv_sensor[2] = 0; 

        comv_sensor_old[0] = 0;
        comv_sensor_old[1] = 0;
        comv_sensor_old[2] = 0;

        coma_sensor[0] = 0;
        coma_sensor[1] = 0;
        coma_sensor[2] = 0;
	
        com_butterworth[0] = 0;
        com_butterworth[1] = 0;
        com_butterworth[2] = 0;

        comv_butterworth[0] = 0;  
        comv_butterworth[1] = 0;
        comv_butterworth[2] = 0;

        coma_butterworth[0] = 0;  
        coma_butterworth[1] = 0;
        coma_butterworth[2] = 0;

        theta_sensor[0] = 0;
        theta_sensor[1] = 0;
        theta_sensor[2] = 0;	
	
        theta_sensor_old[0] = 0;
        theta_sensor_old[1] = 0;
        theta_sensor_old[2] = 0;	
	
        thetav_sensor[0] = 0;
        thetav_sensor[1] = 0;
        thetav_sensor[2] = 0;	
	
        thetav_sensor_old[0] = 0;
        thetav_sensor_old[1] = 0;
        thetav_sensor_old[2] = 0;

        thetaa_sensor[0] = 0;
        thetaa_sensor[1] = 0;
        thetaa_sensor[2] = 0;

        thetaa_butterworth[0] = 0;
        thetaa_butterworth[1] = 0;
        thetaa_butterworth[2] = 0;


        fsampling = 1/_dt;
        fcutoff = 5;
        fcutoff1 = 3;
        fcutoff2 = 3;
        
        butterworthLPFcomx.init(fsampling,fcutoff);
        butterworthLPFcomy.init(fsampling,fcutoff);
        butterworthLPFcomz.init(fsampling,fcutoff);
	
        butterworthLPFx.init(fsampling,fcutoff1);
        butterworthLPFy.init(fsampling,fcutoff1);
        butterworthLPFz.init(fsampling,fcutoff1);

        butterworthLPFcomax.init(fsampling,fcutoff2);
        butterworthLPFcomay.init(fsampling,fcutoff2);
        butterworthLPFcomaz.init(fsampling,fcutoff2);

        butterworthLPFthetaax.init(fsampling,fcutoff2);  
        butterworthLPFthetaay.init(fsampling,fcutoff2);    
        butterworthLPFthetaaz.init(fsampling,fcutoff2); 
	

    }

    StateestimatorClass::~StateestimatorClass()
    {

    }


    //////merely kinematic-based
    Eigen::Matrix<double,23,1> StateestimatorClass::state_estimator(const int count_mpc, int& support_flag, const Walker_Leg waist_pL, const Walker_Leg waist_pR,
                                                                   const geometry_msgs::Wrench left_ft_, const geometry_msgs::Wrench right_ft_, Eigen::Vector2d theta_default,const sensor_msgs::Imu body_imu, const geometry_msgs::Point zmp_ankle2waist_,
                                                                   double *lfoot_pose_sensor, double *rfoot_pose_sensor, double *support_pos_sensor, double *com_sensor, 
								                                   double *zmp_sensor, double *com_sensor_hip, double *dcm_sensor, double omega_sensor)
    {
        Eigen::Matrix<double,23,1> butter_worth;
        rfz[4] =  right_ft_.force.z; 
        lfz[4] =  left_ft_.force.z;
//         double Fz_ratio_l = lfz[4] / (rfz[4] + lfz[4]);
//         double Fz_ratio_r = rfz[4] / (rfz[4] + lfz[4]);

        if (((rfz[4] > fz_limit)&&(lfz[4] > fz_limit))||(count_mpc < 5))
        {
            ////left support

            rfoot_pose_sensor[0] = lfoot_pose_sensor[0] + (waist_pR.joint6.translation[0] - waist_pL.joint6.translation[0]);
            rfoot_pose_sensor[1] = lfoot_pose_sensor[1] + (waist_pR.joint6.translation[1] - waist_pL.joint6.translation[1]);
            rfoot_pose_sensor[2] = lfoot_pose_sensor[2] + (waist_pR.joint6.translation[2] - waist_pL.joint6.translation[2]);
            
            //// support position
            support_pos_sensor[0] = lfoot_pose_sensor[0];
            support_pos_sensor[1] = lfoot_pose_sensor[1];
            support_pos_sensor[2] = lfoot_pose_sensor[2];

            /// COM
            com_sensor[0] =  - waist_pL.joint6.translation[0] + lfoot_pose_sensor[0];
            com_sensor[1] =  - waist_pL.joint6.translation[1] + lfoot_pose_sensor[1];
            com_sensor[2] =  - waist_pL.joint6.translation[2] + lfoot_pose_sensor[2];

            support_flag = 0; 
        }
        else
        {
            if ((rfz[0] > fz_limit)&&(rfz[1] > fz_limit)&&(rfz[2] > fz_limit)&&(rfz[3] > fz_limit)&&(rfz[4] > fz_limit)&&(support_flag == 0)&&(lfz[4] <= fz_double)) ///change to be right support
            {
            lfoot_pose_sensor[0] = rfoot_pose_sensor[0] + (waist_pL.joint6.translation[0] - waist_pR.joint6.translation[0]);
            lfoot_pose_sensor[1] = rfoot_pose_sensor[1] + (waist_pL.joint6.translation[1] - waist_pR.joint6.translation[1]);
            lfoot_pose_sensor[2] = rfoot_pose_sensor[2] + (waist_pL.joint6.translation[2] - waist_pR.joint6.translation[2]); 
            //// support position
            support_pos_sensor[0] = rfoot_pose_sensor[0];
            support_pos_sensor[1] = rfoot_pose_sensor[1];
            support_pos_sensor[2] = rfoot_pose_sensor[2];

            /// COM
            com_sensor[0] =  - waist_pR.joint6.translation[0] + rfoot_pose_sensor[0];
            com_sensor[1] =  - waist_pR.joint6.translation[1] + rfoot_pose_sensor[1];
            com_sensor[2] =  - waist_pR.joint6.translation[2] + rfoot_pose_sensor[2];

            support_flag = 1;     
            //cout<<"left support!!!!!!!!!!!!1"<<endl;  

            }
            else
            {
                if (((lfz[0] <= fz_limit)||(lfz[1] <= fz_limit)||(lfz[2] <= fz_limit)||(lfz[3] <= fz_limit)||(lfz[4] <= fz_limit))&&((support_flag == 1)||((rfz[4] > fz_double)&&((rfz[3] > fz_double))))) ///right support
                {
                    lfoot_pose_sensor[0] = rfoot_pose_sensor[0] + (waist_pL.joint6.translation[0] - waist_pR.joint6.translation[0]);
                    lfoot_pose_sensor[1] = rfoot_pose_sensor[1] + (waist_pL.joint6.translation[1] - waist_pR.joint6.translation[1]);
                    lfoot_pose_sensor[2] = rfoot_pose_sensor[2] + (waist_pL.joint6.translation[2] - waist_pR.joint6.translation[2]); 
                    //// support position
                    support_pos_sensor[0] = rfoot_pose_sensor[0];
                    support_pos_sensor[1] = rfoot_pose_sensor[1];
                    support_pos_sensor[2] = rfoot_pose_sensor[2];

                    /// COM
                    com_sensor[0] =  - waist_pR.joint6.translation[0] + rfoot_pose_sensor[0];
                    com_sensor[1] =  - waist_pR.joint6.translation[1] + rfoot_pose_sensor[1];
                    com_sensor[2] =  - waist_pR.joint6.translation[2] + rfoot_pose_sensor[2];

                    support_flag = 1;   
                    //cout<<"left support!!!!!!!!!!!!1"<<endl;      

                }
                else
                {
                    ////left support

                    rfoot_pose_sensor[0] = lfoot_pose_sensor[0] + (waist_pR.joint6.translation[0] - waist_pL.joint6.translation[0]);
                    rfoot_pose_sensor[1] = lfoot_pose_sensor[1] + (waist_pR.joint6.translation[1] - waist_pL.joint6.translation[1]);
                    rfoot_pose_sensor[2] = lfoot_pose_sensor[2] + (waist_pR.joint6.translation[2] - waist_pL.joint6.translation[2]);
                    
                    //// support position
                    support_pos_sensor[0] = lfoot_pose_sensor[0];
                    support_pos_sensor[1] = lfoot_pose_sensor[1];
                    support_pos_sensor[2] = lfoot_pose_sensor[2];

                    /// COM
                    com_sensor[0] =  - waist_pL.joint6.translation[0] + lfoot_pose_sensor[0];
                    com_sensor[1] =  - waist_pL.joint6.translation[1] + lfoot_pose_sensor[1];
                    com_sensor[2] =  - waist_pL.joint6.translation[2] + lfoot_pose_sensor[2];

                    support_flag = 0;    
                // cout<<"right support!!!!!!!!!!!!1"<<endl;   
                }

            }


        }
        
//         com_sensor_hip[0] = (- waist_pL.joint6.translation[0] + lfoot_pose_sensor[0]) * Fz_ratio_r  + (- waist_pR.joint6.translation[0] + rfoot_pose_sensor[0]) * Fz_ratio_l;
//         com_sensor_hip[1] = (- waist_pL.joint6.translation[1] + lfoot_pose_sensor[1]) * Fz_ratio_r  + (- waist_pR.joint6.translation[1] + rfoot_pose_sensor[1]) * Fz_ratio_l;
//         com_sensor_hip[2] = (- waist_pL.joint6.translation[2] + lfoot_pose_sensor[2]) * Fz_ratio_r  + (- waist_pR.joint6.translation[2] + rfoot_pose_sensor[2]) * Fz_ratio_l; 

        com_sensor_hip[0] = butterworthLPFcomx.filter(com_sensor[0]);
        com_sensor_hip[1] = butterworthLPFcomy.filter(com_sensor[1]);
        com_sensor_hip[2] = butterworthLPFcomz.filter(com_sensor[2]);  
	
// 	com_sensor[0] = com_sensor_hip[0];
// 	com_sensor[1] = com_sensor_hip[1];
// 	com_sensor[2] = com_sensor_hip[2];
	
	
	theta_sensor[0] = body_imu.orientation.x * pi / 180 - theta_default(0,0);
	theta_sensor[1] = body_imu.orientation.y * pi / 180 - theta_default(1,0);
	theta_sensor[2] = body_imu.orientation.z * pi / 180;

        if (count_mpc > 1) 
        {
            comv_sensor[0] = (com_sensor_hip[0] - com_sensor_old[0]) /_dt;
            comv_sensor[1] = (com_sensor_hip[1] - com_sensor_old[1]) /_dt;
            comv_sensor[2] = (com_sensor_hip[2] - com_sensor_old[2]) /_dt;
            thetav_sensor[0] = (theta_sensor[0] - theta_sensor_old[0]) /_dt;
            thetav_sensor[1] = (theta_sensor[1] - theta_sensor_old[1]) /_dt;
            thetav_sensor[2] = (theta_sensor[2] - theta_sensor_old[2]) /_dt;  
        }

        comv_butterworth[0] = butterworthLPFx.filter(comv_sensor[0]);
        comv_butterworth[1] = butterworthLPFy.filter(comv_sensor[1]);
        comv_butterworth[2] = butterworthLPFz.filter(comv_sensor[2]);

        if (count_mpc > 1)
        {
            coma_sensor[0] = (comv_butterworth[0] - comv_sensor_old[0]) /_dt;
            coma_sensor[1] = (comv_butterworth[1] - comv_sensor_old[1]) /_dt;
            coma_sensor[2] = (comv_butterworth[2] - comv_sensor_old[2]) /_dt;
	    	  	  
            thetaa_sensor[0] = (thetav_sensor[0] - thetav_sensor_old[0]) /_dt;
            thetaa_sensor[1] = (thetav_sensor[1] - thetav_sensor_old[1]) /_dt;
            thetaa_sensor[2] = (thetav_sensor[2] - thetav_sensor_old[2]) /_dt;  	    

        }

        coma_butterworth[0] = butterworthLPFcomax.filter(coma_sensor[0]);
        coma_butterworth[1] = butterworthLPFcomay.filter(coma_sensor[1]);
        coma_butterworth[2] = butterworthLPFcomaz.filter(coma_sensor[2]);

        thetaa_butterworth[0] = butterworthLPFthetaax.filter(thetaa_sensor[0]);
        thetaa_butterworth[1] = butterworthLPFthetaay.filter(thetaa_sensor[1]);
        thetaa_butterworth[2] = butterworthLPFthetaaz.filter(thetaa_sensor[2]);        

        zmp_sensor[0] = zmp_ankle2waist_.x + com_sensor[0];
        zmp_sensor[1] = zmp_ankle2waist_.y + com_sensor[1];  
	zmp_sensor[2] = support_pos_sensor[2];
	
	omega_sensor = sqrt( (_g) /(com_sensor[2] - support_pos_sensor[2]) );
	dcm_sensor[0] = com_sensor[0] + comv_butterworth[0]/omega_sensor;
	dcm_sensor[1] = com_sensor[1] + comv_butterworth[1]/omega_sensor;	
	

        rfz[0] = rfz[1]; 
        rfz[1] = rfz[2]; 
        rfz[2] = rfz[3]; 
        rfz[3] = rfz[4]; 
        lfz[0] = lfz[1]; 
        lfz[1] = lfz[2]; 
        lfz[2] = lfz[3]; 
        lfz[3] = lfz[4];

        com_sensor_old[0] = com_sensor_hip[0];
        com_sensor_old[1] = com_sensor_hip[1];
        com_sensor_old[2] = com_sensor_hip[2];
        comv_sensor_old[0] = comv_butterworth[0];
        comv_sensor_old[1] = comv_butterworth[1];
        comv_sensor_old[2] = comv_butterworth[2];		
	
        theta_sensor_old[0] = theta_sensor[0];
        theta_sensor_old[1] = theta_sensor[1];
        theta_sensor_old[2] = theta_sensor[2];  	
 
        thetav_sensor_old[0] = thetav_sensor[0];
        thetav_sensor_old[1] = thetav_sensor[1];
        thetav_sensor_old[2] = thetav_sensor[2];

//         butter_worth(0) = coma_butterworth[0];
//         butter_worth(1) = coma_butterworth[1];
//         butter_worth(2) = coma_butterworth[2];
//         butter_worth(3) = comv_butterworth[0];
//         butter_worth(4) = comv_butterworth[1];
//         butter_worth(5) = comv_butterworth[2];
//         butter_worth(6) = thetaa_butterworth[0];
//         butter_worth(7) = thetaa_butterworth[1];
//         butter_worth(8) = thetaa_butterworth[2];
	
        butter_worth(0) = coma_sensor[0];
        butter_worth(1) = coma_sensor[1];
        butter_worth(2) = coma_sensor[2];
        butter_worth(3) = comv_butterworth[0];
        butter_worth(4) = comv_butterworth[1];
        butter_worth(5) = comv_butterworth[2];
        butter_worth(6) = thetav_sensor[0];
        butter_worth(7) = thetav_sensor[1];
        butter_worth(8) = thetav_sensor[2];
        
        butter_worth(9) = com_sensor[0];
        butter_worth(10) = com_sensor[1];
        butter_worth(11) = com_sensor[2]; 
        
        butter_worth(12) = zmp_sensor[0];
        butter_worth(13) = zmp_sensor[1];
        butter_worth(14) = zmp_sensor[2];

        butter_worth(15) = theta_sensor[0];
        butter_worth(16) = theta_sensor[1];
        butter_worth(17) = theta_sensor[2];
        
        butter_worth(18) = thetaa_sensor[0];
        butter_worth(19) = thetaa_sensor[1];
        butter_worth(20) = thetaa_sensor[2]; 
               
        butter_worth(21) = rfz[4];
        butter_worth(22) = lfz[4];   	
	
        
        return butter_worth;
    }



}






