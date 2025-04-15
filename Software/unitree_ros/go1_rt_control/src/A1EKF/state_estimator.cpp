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
#include "yaml.h"
#include <ros/package.h> 
#include <geometry_msgs/Twist.h>  

using namespace Eigen;
using namespace std;

// namespace gait{
    StateestimatorClass::StateestimatorClass()
    {
        ////// ***********  end-effectors ********************////
        config_set();
        fz_double = gait::mass * gait::_g /3;
        fz_limit = gait::force_z_limt;
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

        Fz_ratio_l = 0;
        Fz_ratio_r = 0;        

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

        
        com_sensor_hip[0] = 0;
        com_sensor_hip[1] = 0;
        com_sensor_hip[2] = 0;

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


        FR_leg_offset_x = 0.1881;
        FL_leg_offset_x = 0.1881;
        RR_leg_offset_x = -0.1881;
        RL_leg_offset_x = -0.1881;

        FR_leg_offset_y = RR_leg_offset_y = -0.12675;
        FL_leg_offset_y = RL_leg_offset_y = 0.12675;
        // FR_thigh_y = RR_thigh_y = -0.08;
        // FL_thigh_y = RL_thigh_y = 0.08;   

        ratio_FR = 0;
        ratio_FL = 0; 
        ratio_RR = 0;
        ratio_RL = 0;     
	

    }

    StateestimatorClass::~StateestimatorClass()
    {

    }


    void StateestimatorClass::config_set()
    {   
        /////load default parameter from the yaml.file
        ///////////////////  yaml code . ///////// 
        // YAML::Node config = YAML::LoadFile("/home/pi/go1_catkin/src/unitree_ros_to_real/unitree_legged_real/config/config.yaml");
        YAML::Node config = YAML::LoadFile("/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/go1_rt_control/config/config.yaml");

        _dt =  config["dt_rt_loop1"].as<double>();
    }


    //////merely kinematic-based
    Eigen::Matrix<double,23,1> StateestimatorClass::state_estimator(int gait_mode, int& support_flag, Eigen::Vector3d FR_foot_relative_mea, Eigen::Vector3d FL_foot_relative_mea,Eigen::Vector3d RR_foot_relative_mea, Eigen::Vector3d RL_foot_relative_mea,
                                                   Eigen::Matrix<double,5,1> footforce_FR, Eigen::Matrix<double,5,1> footforce_FL,Eigen::Matrix<double,5,1> footforce_RR,Eigen::Matrix<double,5,1> footforce_RL,
                                                   double *FRfoot_pose, double *FLfoot_pose, double *RRfoot_pose, double *RLfoot_pose, double *support_pos_sensor, double *com_sensor, double *dcm_sensor, double omega_sensor)
    {
        Eigen::Matrix<double,23,1> butter_worth;
        
        if(gait_mode==102) //troting
        {
            rfz[4] = (footforce_FL(4,0) + footforce_RR(4,0))/2;
            lfz[4] = (footforce_FR(4,0) + footforce_RL(4,0))/2;    

            ratio_FR = sqrt(pow(RL_leg_offset_x,2) + pow(RL_leg_offset_y,2))/(sqrt(pow(RL_leg_offset_x,2) + pow(RL_leg_offset_y,2)) + sqrt(pow(FR_leg_offset_x,2) + pow(FR_leg_offset_y,2)));
            ratio_RL  = 1 - ratio_FR;     

            ratio_RR = sqrt(pow(FL_leg_offset_x,2) + pow(FL_leg_offset_y,2))/(sqrt(pow(FL_leg_offset_x,2) + pow(FL_leg_offset_y,2)) + sqrt(pow(RR_leg_offset_x,2) + pow(RR_leg_offset_y,2)));
            ratio_FL  = 1 - ratio_RR;               
        }
        else
        {
            rfz[4] = (footforce_FR(4,0) + footforce_RR(4,0))/2;
            lfz[4] = (footforce_FL(4,0) + footforce_RL(4,0))/2;

            ratio_FR = sqrt(pow(RR_leg_offset_x,2) + pow(RR_leg_offset_y,2))/(sqrt(pow(RR_leg_offset_x,2) + pow(RR_leg_offset_y,2)) + sqrt(pow(FR_leg_offset_x,2) + pow(FR_leg_offset_y,2)));
            ratio_RR  = 1 - ratio_RR;     

            ratio_RL = sqrt(pow(FL_leg_offset_x,2) + pow(FL_leg_offset_y,2))/(sqrt(pow(FL_leg_offset_x,2) + pow(FL_leg_offset_y,2)) + sqrt(pow(RL_leg_offset_x,2) + pow(RL_leg_offset_y,2)));
            ratio_FL  = 1 - ratio_RL;               
        }

        Fz_ratio_l = lfz[4] / (rfz[4] + lfz[4]);
        Fz_ratio_r = rfz[4] / (rfz[4] + lfz[4]);

        if (((rfz[4] > fz_limit)&&(rfz[3] > fz_limit)&&(lfz[3] > fz_limit)&&(lfz[4] > fz_limit)))
        {
            ////left support
            if(gait_mode==102)
            {
                // sometimes, misjudge the support, zero support force
                if(footforce_RL(4,0) + footforce_FR(4,0)<10)
                {
                    footforce_RL(4,0) = footforce_FR(4,0) = 10;
                }
                com_sensor[0] =  (-FR_foot_relative_mea(0,0)  + FRfoot_pose[0]) * (footforce_RL(4,0))/(footforce_RL(4,0) + footforce_FR(4,0)) + (-RL_foot_relative_mea(0,0)  + RLfoot_pose[0])* (footforce_FR(4,0))/(footforce_RL(4,0) + footforce_FR(4,0));
                com_sensor[1] =  (-FR_foot_relative_mea(1,0)  + FRfoot_pose[1]) * (footforce_RL(4,0))/(footforce_RL(4,0) + footforce_FR(4,0)) + (-RL_foot_relative_mea(1,0)  + RLfoot_pose[1])* (footforce_FR(4,0))/(footforce_RL(4,0) + footforce_FR(4,0));
                com_sensor[2] =  (-FR_foot_relative_mea(2,0)  + FRfoot_pose[2]) * (footforce_RL(4,0))/(footforce_RL(4,0) + footforce_FR(4,0)) + (-RL_foot_relative_mea(2,0)  + RLfoot_pose[2])* (footforce_FR(4,0))/(footforce_RL(4,0) + footforce_FR(4,0));

                FLfoot_pose[0] = com_sensor[0] + FL_foot_relative_mea(0,0);
                FLfoot_pose[1] = com_sensor[1] + FL_foot_relative_mea(1,0);
                FLfoot_pose[2] = com_sensor[2] + FL_foot_relative_mea(2,0);

                RRfoot_pose[0] = com_sensor[0] + RR_foot_relative_mea(0,0);
                RRfoot_pose[1] = com_sensor[1] + RR_foot_relative_mea(1,0);
                RRfoot_pose[2] = com_sensor[2] + RR_foot_relative_mea(2,0);   

                //// support position
                support_pos_sensor[0] = FRfoot_pose[0]* (footforce_RL(4,0))/(footforce_RL(4,0) + footforce_FR(4,0)) + RLfoot_pose[0]* (footforce_FR(4,0))/(footforce_RL(4,0) + footforce_FR(4,0));
                support_pos_sensor[1] = FRfoot_pose[1]* (footforce_RL(4,0))/(footforce_RL(4,0) + footforce_FR(4,0)) + RLfoot_pose[1]* (footforce_FR(4,0))/(footforce_RL(4,0) + footforce_FR(4,0));
                support_pos_sensor[2] = FRfoot_pose[2]* (footforce_RL(4,0))/(footforce_RL(4,0) + footforce_FR(4,0)) + RLfoot_pose[2]* (footforce_FR(4,0))/(footforce_RL(4,0) + footforce_FR(4,0));  

                // com_sensor[0] =  (-FR_foot_relative_mea(0,0)  + FRfoot_pose[0]) * ratio_FR + (-RL_foot_relative_mea(0,0)  + RLfoot_pose[0])* ratio_RL;
                // com_sensor[1] =  (-FR_foot_relative_mea(1,0)  + FRfoot_pose[1]) * ratio_FR + (-RL_foot_relative_mea(1,0)  + RLfoot_pose[1])* ratio_RL;
                // com_sensor[2] =  (-FR_foot_relative_mea(2,0)  + FRfoot_pose[2]) * ratio_FR + (-RL_foot_relative_mea(2,0)  + RLfoot_pose[2])* ratio_RL;

                // FLfoot_pose[0] = com_sensor[0] + FL_foot_relative_mea(0,0);
                // FLfoot_pose[1] = com_sensor[1] + FL_foot_relative_mea(1,0);
                // FLfoot_pose[2] = com_sensor[2] + FL_foot_relative_mea(2,0);

                // RRfoot_pose[0] = com_sensor[0] + RR_foot_relative_mea(0,0);
                // RRfoot_pose[1] = com_sensor[1] + RR_foot_relative_mea(1,0);
                // RRfoot_pose[2] = com_sensor[2] + RR_foot_relative_mea(2,0);   

                // //// support position
                // support_pos_sensor[0] = FRfoot_pose[0]* ratio_FR + RLfoot_pose[0]* ratio_RL;
                // support_pos_sensor[1] = FRfoot_pose[1]* ratio_FR + RLfoot_pose[1]* ratio_RL;
                // support_pos_sensor[2] = FRfoot_pose[2]* ratio_FR + RLfoot_pose[2]* ratio_RL;                 

            }

            support_flag = 0; 
        }
        else
        {
            // if ((rfz[0] > fz_limit)&&(rfz[1] > fz_limit)&&(rfz[2] > fz_limit)&&(rfz[3] > fz_limit)&&(rfz[4] > fz_limit)&&(support_flag == 0)&&(lfz[4] <= fz_double)) ///change to be right support
            if ((rfz[0] > fz_limit)&&(rfz[1] > fz_limit)&&(rfz[2] > fz_limit)&&(rfz[3] > fz_limit)&&(rfz[4] > fz_limit)&&(support_flag == 0)&&(lfz[4] <= fz_double)) ///change to be right support
            {
                /// COM
                if(gait_mode==102)
                {            
                    if(footforce_FL(4,0) + footforce_RR(4,0)<10)
                    {
                        footforce_FL(4,0) = footforce_RR(4,0) = 10;
                    }                         
                    com_sensor[0] =  (-FL_foot_relative_mea(0,0)  + FLfoot_pose[0]) * (footforce_RR(4,0))/(footforce_FL(4,0) + footforce_RR(4,0)) + (-RR_foot_relative_mea(0,0)  + RRfoot_pose[0])* (footforce_FL(4,0))/(footforce_FL(4,0) + footforce_RR(4,0));
                    com_sensor[1] =  (-FL_foot_relative_mea(1,0)  + FLfoot_pose[1]) * (footforce_RR(4,0))/(footforce_FL(4,0) + footforce_RR(4,0)) + (-RR_foot_relative_mea(1,0)  + RRfoot_pose[1])* (footforce_FL(4,0))/(footforce_FL(4,0) + footforce_RR(4,0));
                    com_sensor[2] =  (-FL_foot_relative_mea(2,0)  + FLfoot_pose[2]) * (footforce_RR(4,0))/(footforce_FL(4,0) + footforce_RR(4,0)) + (-RR_foot_relative_mea(2,0)  + RRfoot_pose[2])* (footforce_FL(4,0))/(footforce_FL(4,0) + footforce_RR(4,0));

                    FRfoot_pose[0] = com_sensor[0] + FR_foot_relative_mea(0,0);
                    FRfoot_pose[1] = com_sensor[1] + FR_foot_relative_mea(1,0);
                    FRfoot_pose[2] = com_sensor[2] + FR_foot_relative_mea(2,0);

                    RLfoot_pose[0] = com_sensor[0] + RL_foot_relative_mea(0,0);
                    RLfoot_pose[1] = com_sensor[1] + RL_foot_relative_mea(1,0);
                    RLfoot_pose[2] = com_sensor[2] + RL_foot_relative_mea(2,0);  
                    // //// support position
                    support_pos_sensor[0] =  (FLfoot_pose[0]) * (footforce_RR(4,0))/(footforce_FL(4,0) + footforce_RR(4,0)) + (RRfoot_pose[0])* (footforce_FL(4,0))/(footforce_FL(4,0) + footforce_RR(4,0));
                    support_pos_sensor[1] =  (FLfoot_pose[1]) * (footforce_RR(4,0))/(footforce_FL(4,0) + footforce_RR(4,0)) + (RRfoot_pose[1])* (footforce_FL(4,0))/(footforce_FL(4,0) + footforce_RR(4,0));
                    support_pos_sensor[2] =  (FLfoot_pose[2]) * (footforce_RR(4,0))/(footforce_FL(4,0) + footforce_RR(4,0)) + (RRfoot_pose[2])* (footforce_FL(4,0))/(footforce_FL(4,0) + footforce_RR(4,0));

                    // com_sensor[0] =  (-FL_foot_relative_mea(0,0)  + FLfoot_pose[0]) * ratio_FL + (-RR_foot_relative_mea(0,0)  + RRfoot_pose[0])* ratio_RR;
                    // com_sensor[1] =  (-FL_foot_relative_mea(1,0)  + FLfoot_pose[1]) * ratio_FL + (-RR_foot_relative_mea(1,0)  + RRfoot_pose[1])* ratio_RR;
                    // com_sensor[2] =  (-FL_foot_relative_mea(2,0)  + FLfoot_pose[2]) * ratio_FL + (-RR_foot_relative_mea(2,0)  + RRfoot_pose[2])* ratio_RR;

                    // FRfoot_pose[0] = com_sensor[0] + FR_foot_relative_mea(0,0);
                    // FRfoot_pose[1] = com_sensor[1] + FR_foot_relative_mea(1,0);
                    // FRfoot_pose[2] = com_sensor[2] + FR_foot_relative_mea(2,0);

                    // RLfoot_pose[0] = com_sensor[0] + RL_foot_relative_mea(0,0);
                    // RLfoot_pose[1] = com_sensor[1] + RL_foot_relative_mea(1,0);
                    // RLfoot_pose[2] = com_sensor[2] + RL_foot_relative_mea(2,0);  
                    // // //// support position
                    // support_pos_sensor[0] =  (FLfoot_pose[0]) * ratio_FL + (RRfoot_pose[0])* ratio_RR;
                    // support_pos_sensor[1] =  (FLfoot_pose[1]) * ratio_FL + (RRfoot_pose[1])* ratio_RR;
                    // support_pos_sensor[2] =  (FLfoot_pose[2]) * ratio_FL + (RRfoot_pose[2])* ratio_RR;                    
                }
                support_flag = 1;     
                //cout<<"left support!!!!!!!!!!!!1"<<endl;  

            }
            else
            {
                if (((lfz[0] <= fz_limit)||(lfz[1] <= fz_limit)||(lfz[2] <= fz_limit)||(lfz[3] <= fz_limit)||(lfz[4] <= fz_limit))&&((support_flag == 1)||((rfz[4] >= fz_double)&&((rfz[3] >= fz_double))))) ///right support
                // if (((lfz[2] <= fz_limit)||(lfz[3] <= fz_limit)||(lfz[4] <= fz_limit))&&((support_flag == 1)||((rfz[4] >= fz_double)&&((rfz[3] >= fz_double))))) ///right support
                {
                    if(gait_mode==102)
                    {
                        if(footforce_FL(4,0) + footforce_RR(4,0)<10)
                        {
                            footforce_FL(4,0) = footforce_RR(4,0) = 10;
                        }                              
                        com_sensor[0] =  (-FL_foot_relative_mea(0,0)  + FLfoot_pose[0]) * (footforce_RR(4,0))/(footforce_FL(4,0) + footforce_RR(4,0)) + (-RR_foot_relative_mea(0,0)  + RRfoot_pose[0])* (footforce_FL(4,0))/(footforce_FL(4,0) + footforce_RR(4,0));
                        com_sensor[1] =  (-FL_foot_relative_mea(1,0)  + FLfoot_pose[1]) * (footforce_RR(4,0))/(footforce_FL(4,0) + footforce_RR(4,0)) + (-RR_foot_relative_mea(1,0)  + RRfoot_pose[1])* (footforce_FL(4,0))/(footforce_FL(4,0) + footforce_RR(4,0));
                        com_sensor[2] =  (-FL_foot_relative_mea(2,0)  + FLfoot_pose[2]) * (footforce_RR(4,0))/(footforce_FL(4,0) + footforce_RR(4,0)) + (-RR_foot_relative_mea(2,0)  + RRfoot_pose[2])* (footforce_FL(4,0))/(footforce_FL(4,0) + footforce_RR(4,0));

                        FRfoot_pose[0] = com_sensor[0] + FR_foot_relative_mea(0,0);
                        FRfoot_pose[1] = com_sensor[1] + FR_foot_relative_mea(1,0);
                        FRfoot_pose[2] = com_sensor[2] + FR_foot_relative_mea(2,0);

                        RLfoot_pose[0] = com_sensor[0] + RL_foot_relative_mea(0,0);
                        RLfoot_pose[1] = com_sensor[1] + RL_foot_relative_mea(1,0);
                        RLfoot_pose[2] = com_sensor[2] + RL_foot_relative_mea(2,0); 

                        //// support position
                        support_pos_sensor[0] =  (FLfoot_pose[0]) * (footforce_RR(4,0))/(footforce_FL(4,0) + footforce_RR(4,0)) + (RRfoot_pose[0])* (footforce_FL(4,0))/(footforce_FL(4,0) + footforce_RR(4,0));
                        support_pos_sensor[1] =  (FLfoot_pose[1]) * (footforce_RR(4,0))/(footforce_FL(4,0) + footforce_RR(4,0)) + (RRfoot_pose[1])* (footforce_FL(4,0))/(footforce_FL(4,0) + footforce_RR(4,0));
                        support_pos_sensor[2] =  (FLfoot_pose[2]) * (footforce_RR(4,0))/(footforce_FL(4,0) + footforce_RR(4,0)) + (RRfoot_pose[2])* (footforce_FL(4,0))/(footforce_FL(4,0) + footforce_RR(4,0));

                        // com_sensor[0] =  (-FL_foot_relative_mea(0,0)  + FLfoot_pose[0]) * ratio_FL + (-RR_foot_relative_mea(0,0)  + RRfoot_pose[0])* ratio_RR;
                        // com_sensor[1] =  (-FL_foot_relative_mea(1,0)  + FLfoot_pose[1]) * ratio_FL + (-RR_foot_relative_mea(1,0)  + RRfoot_pose[1])* ratio_RR;
                        // com_sensor[2] =  (-FL_foot_relative_mea(2,0)  + FLfoot_pose[2]) * ratio_FL + (-RR_foot_relative_mea(2,0)  + RRfoot_pose[2])* ratio_RR;

                        // FRfoot_pose[0] = com_sensor[0] + FR_foot_relative_mea(0,0);
                        // FRfoot_pose[1] = com_sensor[1] + FR_foot_relative_mea(1,0);
                        // FRfoot_pose[2] = com_sensor[2] + FR_foot_relative_mea(2,0);

                        // RLfoot_pose[0] = com_sensor[0] + RL_foot_relative_mea(0,0);
                        // RLfoot_pose[1] = com_sensor[1] + RL_foot_relative_mea(1,0);
                        // RLfoot_pose[2] = com_sensor[2] + RL_foot_relative_mea(2,0);  
                        // // //// support position
                        // support_pos_sensor[0] =  (FLfoot_pose[0]) * ratio_FL + (RRfoot_pose[0])* ratio_RR;
                        // support_pos_sensor[1] =  (FLfoot_pose[1]) * ratio_FL + (RRfoot_pose[1])* ratio_RR;
                        // support_pos_sensor[2] =  (FLfoot_pose[2]) * ratio_FL + (RRfoot_pose[2])* ratio_RR;  


                    }
                    support_flag = 1;   
                    //cout<<"left support!!!!!!!!!!!!1"<<endl;      

                }
                else
                {
                    ////left support
                    if(gait_mode==102)
                    {
                        if(footforce_RL(4,0) + footforce_FR(4,0)<10)
                        {
                            footforce_RL(4,0) = footforce_FR(4,0) = 10;
                        }                        
                        com_sensor[0] =  (-FR_foot_relative_mea(0,0)  + FRfoot_pose[0]) * (footforce_RL(4,0))/(footforce_RL(4,0) + footforce_FR(4,0)) + (-RL_foot_relative_mea(0,0)  + RLfoot_pose[0])* (footforce_FR(4,0))/(footforce_RL(4,0) + footforce_FR(4,0));
                        com_sensor[1] =  (-FR_foot_relative_mea(1,0)  + FRfoot_pose[1]) * (footforce_RL(4,0))/(footforce_RL(4,0) + footforce_FR(4,0)) + (-RL_foot_relative_mea(1,0)  + RLfoot_pose[1])* (footforce_FR(4,0))/(footforce_RL(4,0) + footforce_FR(4,0));
                        com_sensor[2] =  (-FR_foot_relative_mea(2,0)  + FRfoot_pose[2]) * (footforce_RL(4,0))/(footforce_RL(4,0) + footforce_FR(4,0)) + (-RL_foot_relative_mea(2,0)  + RLfoot_pose[2])* (footforce_FR(4,0))/(footforce_RL(4,0) + footforce_FR(4,0));

                        FLfoot_pose[0] = com_sensor[0] + FL_foot_relative_mea(0,0);
                        FLfoot_pose[1] = com_sensor[1] + FL_foot_relative_mea(1,0);
                        FLfoot_pose[2] = com_sensor[2] + FL_foot_relative_mea(2,0);

                        RRfoot_pose[0] = com_sensor[0] + RR_foot_relative_mea(0,0);
                        RRfoot_pose[1] = com_sensor[1] + RR_foot_relative_mea(1,0);
                        RRfoot_pose[2] = com_sensor[2] + RR_foot_relative_mea(2,0);  

                        //// support position
                        support_pos_sensor[0] = FRfoot_pose[0]* (footforce_RL(4,0))/(footforce_RL(4,0) + footforce_FR(4,0)) + RLfoot_pose[0]* (footforce_FR(4,0))/(footforce_RL(4,0) + footforce_FR(4,0));
                        support_pos_sensor[1] = FRfoot_pose[1]* (footforce_RL(4,0))/(footforce_RL(4,0) + footforce_FR(4,0)) + RLfoot_pose[1]* (footforce_FR(4,0))/(footforce_RL(4,0) + footforce_FR(4,0));
                        support_pos_sensor[2] = FRfoot_pose[2]* (footforce_RL(4,0))/(footforce_RL(4,0) + footforce_FR(4,0)) + RLfoot_pose[2]* (footforce_FR(4,0))/(footforce_RL(4,0) + footforce_FR(4,0)); 

                        // com_sensor[0] =  (-FR_foot_relative_mea(0,0)  + FRfoot_pose[0]) * ratio_FR + (-RL_foot_relative_mea(0,0)  + RLfoot_pose[0])* ratio_RL;
                        // com_sensor[1] =  (-FR_foot_relative_mea(1,0)  + FRfoot_pose[1]) * ratio_FR + (-RL_foot_relative_mea(1,0)  + RLfoot_pose[1])* ratio_RL;
                        // com_sensor[2] =  (-FR_foot_relative_mea(2,0)  + FRfoot_pose[2]) * ratio_FR + (-RL_foot_relative_mea(2,0)  + RLfoot_pose[2])* ratio_RL;

                        // FLfoot_pose[0] = com_sensor[0] + FL_foot_relative_mea(0,0);
                        // FLfoot_pose[1] = com_sensor[1] + FL_foot_relative_mea(1,0);
                        // FLfoot_pose[2] = com_sensor[2] + FL_foot_relative_mea(2,0);

                        // RRfoot_pose[0] = com_sensor[0] + RR_foot_relative_mea(0,0);
                        // RRfoot_pose[1] = com_sensor[1] + RR_foot_relative_mea(1,0);
                        // RRfoot_pose[2] = com_sensor[2] + RR_foot_relative_mea(2,0);   

                        // //// support position
                        // support_pos_sensor[0] = FRfoot_pose[0]* ratio_FR + RLfoot_pose[0]* ratio_RL;
                        // support_pos_sensor[1] = FRfoot_pose[1]* ratio_FR + RLfoot_pose[1]* ratio_RL;
                        // support_pos_sensor[2] = FRfoot_pose[2]* ratio_FR + RLfoot_pose[2]* ratio_RL;                                                                 
                    }
                    support_flag = 0;   
                }

            }


        }
        


        com_sensor_hip[0] = butterworthLPFcomx.filter(com_sensor[0]);
        com_sensor_hip[1] = butterworthLPFcomy.filter(com_sensor[1]);
        com_sensor_hip[2] = butterworthLPFcomz.filter(com_sensor[2]);  

        


        // if (count_ros > 1) 
        // {
            comv_sensor[0] = (com_sensor_hip[0] - com_sensor_old[0]) /_dt;
            comv_sensor[1] = (com_sensor_hip[1] - com_sensor_old[1]) /_dt;
            comv_sensor[2] = (com_sensor_hip[2] - com_sensor_old[2]) /_dt;
            // thetav_sensor[0] = (theta_sensor[0] - theta_sensor_old[0]) /_dt;
            // thetav_sensor[1] = (theta_sensor[1] - theta_sensor_old[1]) /_dt;
            // thetav_sensor[2] = (theta_sensor[2] - theta_sensor_old[2]) /_dt;  
        // }

        comv_butterworth[0] = butterworthLPFx.filter(comv_sensor[0]);
        comv_butterworth[1] = butterworthLPFy.filter(comv_sensor[1]);
        comv_butterworth[2] = butterworthLPFz.filter(comv_sensor[2]);

        // if (count_ros > 1)
        // {
            coma_sensor[0] = (comv_butterworth[0] - comv_sensor_old[0]) /_dt;
            coma_sensor[1] = (comv_butterworth[1] - comv_sensor_old[1]) /_dt;
            coma_sensor[2] = (comv_butterworth[2] - comv_sensor_old[2]) /_dt;
	    	  	  
            // thetaa_sensor[0] = (thetav_sensor[0] - thetav_sensor_old[0]) /_dt;
            // thetaa_sensor[1] = (thetav_sensor[1] - thetav_sensor_old[1]) /_dt;
            // thetaa_sensor[2] = (thetav_sensor[2] - thetav_sensor_old[2]) /_dt;  	    

        // }

        coma_butterworth[0] = butterworthLPFcomax.filter(coma_sensor[0]);
        coma_butterworth[1] = butterworthLPFcomay.filter(coma_sensor[1]);
        coma_butterworth[2] = butterworthLPFcomaz.filter(coma_sensor[2]);

        // thetaa_butterworth[0] = butterworthLPFthetaax.filter(thetaa_sensor[0]);
        // thetaa_butterworth[1] = butterworthLPFthetaay.filter(thetaa_sensor[1]);
        // thetaa_butterworth[2] = butterworthLPFthetaaz.filter(thetaa_sensor[2]);        

        // zmp_sensor[0] = zmp_ankle2waist_.x + com_sensor[0];
        // zmp_sensor[1] = zmp_ankle2waist_.y + com_sensor[1];  
        // zmp_sensor[2] = support_pos_sensor[2];
        
        // omega_sensor = sqrt( (gait::_g) /(com_sensor[2] - support_pos_sensor[2]) );
        // dcm_sensor[0] = com_sensor[0] + comv_butterworth[0]/omega_sensor;
        // dcm_sensor[1] = com_sensor[1] + comv_butterworth[1]/omega_sensor;	
	

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
	
        // theta_sensor_old[0] = theta_sensor[0];
        // theta_sensor_old[1] = theta_sensor[1];
        // theta_sensor_old[2] = theta_sensor[2];  	
 
        // thetav_sensor_old[0] = thetav_sensor[0];
        // thetav_sensor_old[1] = thetav_sensor[1];
        // thetav_sensor_old[2] = thetav_sensor[2];
	
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
        
        // butter_worth(12) = zmp_sensor[0];
        // butter_worth(13) = zmp_sensor[1];
        // butter_worth(14) = zmp_sensor[2];

        butter_worth(15) = theta_sensor[0];
        butter_worth(16) = theta_sensor[1];
        butter_worth(17) = theta_sensor[2];
        
        butter_worth(18) = thetaa_sensor[0];
        butter_worth(19) = thetaa_sensor[1];
        butter_worth(20) = thetaa_sensor[2]; 
               
        butter_worth(21) = rfz[4];
        butter_worth(22) = lfz[4];   	
	    
        // std::cout<<"kine_est"<<std::endl;
        
        return butter_worth;
    }



// }






