#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <sys/types.h>
#include <iostream>
#include <time.h>
#include <fstream>   
#include <string>  
#include <cassert>
#include <vector>
#include "eigen3/Eigen/Dense" //for hardware experiments
#include "math.h"
#include "whole_body_dynamics/dynmics_compute.h"
#include "/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/src/go1_rt_control/src/Robotpara/robot_const_para_config.h"
#include "gait_dynamic.h"


using namespace Eigen;
using namespace std;

///// gains variation
double  kpp0_det;
double  kdp0_det;
double  kpp1_det;
double  kdp1_det;          
double  kpp2_det;
double  kdp2_det;

void state_feed_sub_operation(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int jx = 0; jx<100; jx++)
    {  
        state_feedback_receieved(jx) = msg->position[jx]; 
    }   

    // receieved from the real-time loop;
    for(int j=0; j<3; j++)
    {
        FR_foot_Homing(j,0) = msg->position[j];   //  FR Leg position;
    }         
    for(int j=0; j<3; j++)
    {
        FL_foot_Homing(j,0) = msg->position[3+j];   
    }
    for(int j=0; j<3; j++)
    {
        RR_foot_Homing(j,0) = msg->position[6+j];  
    }
    for(int j=0; j<3; j++)
    {
        RL_foot_Homing(j,0) = msg->position[9+j];   
    } 
    for(int j=0; j<3; j++)
    {
        com_sensor[j] = msg->position[12+j];  
    }
    for(int j=0; j<3; j++)
    {
        comv_sensor[j] = msg->position[15+j];  
    }
    for(int j=0; j<3; j++)
    {
        theta_sensor[j] = msg->position[18+j];  
    }
    for(int j=0; j<3; j++)
    {
        thetav_sensor[j] = msg->position[21+j];  
    }

    for(int j=0; j<3; j++)
    {
        FR_sensor[j] = msg->position[24+j];  
    }
    for(int j=0; j<3; j++)
    {
        FL_sensor[j] = msg->position[27+j];  
    }
    for(int j=0; j<3; j++)
    {
        RR_sensor[j] = msg->position[28+j];  
    }
    for(int j=0; j<3; j++)
    {
        RL_sensor[j] = msg->position[31+j];  
    }  

    gait_start =   msg->position[99];    
        
}

void base_pos_fb_controler()
{
    ////// PD-type CoM position control//////
    if(right_support == 0) ////left support
    {
        switch (gait_mode)
            {
            case 101:  ////biped walking
                FR_swing = true;
                RR_swing = true;
                FL_swing = false;
                RL_swing = false;
                support_pos_sensor[0] = (FL_sensor[0] + RL_sensor[0])/2;
                support_pos_sensor[1] = (FL_sensor[1] + RL_sensor[1])/2;
                support_pos_sensor[2] = (FL_sensor[2] + RL_sensor[2])/2;
                support_pos_des[0] = (FL_foot_des[0] + RL_foot_des[0])/2;
                support_pos_des[1] = (FL_foot_des[1] + RL_foot_des[1])/2;
                support_pos_des[2] = (FL_foot_des[2] + RL_foot_des[2])/2; 

                break;
            case 102:  ///troting
                FR_swing = false;
                RL_swing = false;
                FL_swing = true;
                RR_swing = true;
                support_pos_sensor[0] = (FR_sensor[0] + RL_sensor[0])/2;
                support_pos_sensor[1] = (FR_sensor[1] + RL_sensor[1])/2;
                support_pos_sensor[2] = (FR_sensor[2] + RL_sensor[2])/2;
                support_pos_des[0] = (FR_foot_des[0] + RL_foot_des[0])/2;
                support_pos_des[1] = (FR_foot_des[1] + RL_foot_des[1])/2;
                support_pos_des[2] = (FR_foot_des[2] + RL_foot_des[2])/2;


                break;
            case 103:  ///gallop: alter the  x-y direction
                FR_swing = true;
                FL_swing = true;
                RR_swing = false;
                RL_swing = false;                    

                break;            
            default:


                break;
            } 
    }
    else
    {
        if (right_support == 1)
        {
            switch (gait_mode)
                {
                case 101:  ////biped walking
                    FR_swing = false;
                    RR_swing = false;
                    FL_swing = true;
                    RL_swing = true;
                    support_pos_sensor[0] = (FR_sensor[0] + RR_sensor[0])/2;
                    support_pos_sensor[1] = (FR_sensor[1] + RR_sensor[1])/2;
                    support_pos_sensor[2] = (FR_sensor[2] + RR_sensor[2])/2;
                    support_pos_des[0] = (FR_foot_des[0] + RR_foot_des[0])/2;
                    support_pos_des[1] = (FR_foot_des[1] + RR_foot_des[1])/2;
                    support_pos_des[2] = (FR_foot_des[2] + RR_foot_des[2])/2; 

                    break;
                case 102:  ///troting
                    FR_swing = true;
                    RL_swing = true;
                    FL_swing = false;
                    RR_swing = false;      

                    support_pos_sensor[0] = (FL_sensor[0] + RR_sensor[0])/2;
                    support_pos_sensor[1] = (FL_sensor[1] + RR_sensor[1])/2;
                    support_pos_sensor[2] = (FL_sensor[2] + RR_sensor[2])/2;
                    support_pos_des[0] = (FL_foot_des[0] + RR_foot_des[0])/2;
                    support_pos_des[1] = (FL_foot_des[1] + RR_foot_des[1])/2;
                    support_pos_des[2] = (FL_foot_des[2] + RR_foot_des[2])/2;                                                              

                    break;
                case 103:  ///gallop: alter the  x-y direction
                    FR_swing = false;
                    FL_swing = false;
                    RR_swing = true;
                    RL_swing = true;                    

                    break;            
                default:
                    break;
                }
        }
        else
        {
            FR_swing = false;
            FL_swing = false;
            RR_swing = false;
            RL_swing = false;  

            //// assuming right support in trotting mode; 
            support_pos_sensor[0] = (FL_sensor[0] + RR_sensor[0])/2;
            support_pos_sensor[1] = (FL_sensor[1] + RR_sensor[1])/2;
            support_pos_sensor[2] = (FL_sensor[2] + RR_sensor[2])/2;
            support_pos_des[0] = (FL_foot_des[0] + RR_foot_des[0])/2;
            support_pos_des[1] = (FL_foot_des[1] + RR_foot_des[1])/2;
            support_pos_des[2] = (FL_foot_des[2] + RR_foot_des[2])/2;               
        }                
        
    }
    


    body_relative_supportv_sensor[0] = (com_sensor[0] - support_pos_sensor[0] - body_relative_support_sensor_old[0])/dtx;
    body_relative_supportv_sensor[1] = (com_sensor[1] - support_pos_sensor[1] - body_relative_support_sensor_old[1])/dtx;
    body_relative_supportv_sensor[2] = (com_sensor[2] - support_pos_sensor[2] - body_relative_support_sensor_old[2])/dtx;
    if(global_com_feedback >0.5)
    {
        w_pos_m[0] = w_pos_m_kp[0] * (body_p_des[0] + (com_sensor[0])) + w_pos_m_kd[0] * (0 - comv_sensor[0]); /// x;
        w_pos_m[1] = w_pos_m_kp[1] * (body_p_des[1] - (com_sensor[1])) + w_pos_m_kd[1] * (0-comv_sensor[1]);  /// y;
        w_pos_m[2] = w_pos_m_kp[2] * (body_p_des[2] - (com_sensor[2])) + w_pos_m_kd[2] * (0-comv_sensor[2]); /// z:
    } 
    else
    {
        w_pos_m[0] = w_pos_m_kp[0] * (body_p_des[0] - support_pos_des[0] - (com_sensor[0] - support_pos_sensor[0])) + w_pos_m_kd[0] * (0-body_relative_supportv_sensor[0]); /// x;
        w_pos_m[1] = w_pos_m_kp[1] * (body_p_des[1] - support_pos_des[1] - (com_sensor[1] - support_pos_sensor[1])) + w_pos_m_kd[1] * (0-body_relative_supportv_sensor[1]);  /// y;
        w_pos_m[2] = w_pos_m_kp[2] * (body_p_des[2] - support_pos_des[2] - (com_sensor[2] - support_pos_sensor[2])) + w_pos_m_kd[2] * (0-body_relative_supportv_sensor[2]); /// z:   
    }   
    w_pos_m_filter[0] = butterworthLPF31.filter(w_pos_m[0]);
    w_pos_m_filter[1] = butterworthLPF32.filter(w_pos_m[1]);
    w_pos_m_filter[2] = butterworthLPF33.filter(w_pos_m[2]);
    if ((w_pos_m_filter[0] >= com_pos_max))
    {
        w_pos_m_filter[0] = com_pos_max;
    }
    else
    {
        if ((w_pos_m_filter[0] <= com_pos_min))
        {
            w_pos_m_filter[0] = com_pos_min;
        }
    }

    if ((w_pos_m_filter[1] >= com_pos_max))
    {
        w_pos_m_filter[1] = com_pos_max;
    }
    else
    {
        if ((w_pos_m_filter[1] <= com_pos_min))
        {
            w_pos_m_filter[1] = com_pos_min;
        }
    } 
    if ((w_pos_m_filter[2] >= 2*com_pos_max))
    {
        w_pos_m_filter[2] = 2*com_pos_max;
    }
    else
    {
        if ((w_pos_m_filter[2] <= 2*com_pos_min))
        {
            w_pos_m_filter[2] = 2*com_pos_min;
        }
    } 

    w_rpy_m[0] = w_rpy_m_kp[0] * (body_r_des[0]- theta_sensor[0]) + w_rpy_m_kp[0] * (0-thetav_sensor[0]); /// x;
    w_rpy_m[1] = w_rpy_m_kp[1] * (body_r_des[1]- theta_sensor[1]) + w_rpy_m_kp[1] * (0-thetav_sensor[1]);  /// y;
    w_rpy_m[2] = w_rpy_m_kp[2] * (body_p_des[2]- theta_sensor[2]) + w_rpy_m_kp[2] * (0-thetav_sensor[2]); /// z:
    w_rpy_m_filter[0] = butterworthLPF34.filter(w_rpy_m[0]);
    w_rpy_m_filter[1] = butterworthLPF35.filter(w_rpy_m[1]);
    w_rpy_m_filter[2] = butterworthLPF36.filter(w_rpy_m[2]);
    if ((w_rpy_m_filter[0] >= com_rpy_max))
    {
        w_rpy_m_filter[0] = com_rpy_max;
    }
    else
    {
        if ((w_rpy_m_filter[0] <= com_rpy_min))
        {
            w_rpy_m_filter[0] = com_rpy_min;
        }
    }
    if ((w_rpy_m_filter[1] >= com_rpy_max))
    {
        w_rpy_m_filter[1] = com_rpy_max;
    }
    else
    {
        if ((w_rpy_m_filter[1] <= com_rpy_min))
        {
            w_rpy_m_filter[1] = com_rpy_min;
        }
    } 
    if ((w_rpy_m_filter[2] >= com_rpy_max))
    {
        w_rpy_m_filter[2] = com_rpy_max;
    }
    else
    {
        if ((w_rpy_m_filter[2] <= com_rpy_min))
        {
            w_rpy_m_filter[2] = com_rpy_min;
        }
    }                        
                  
}


void base_acc_ff_controler()
{
    ////// PD-type CoM acceleration control: for Grf compensation//////
    ///// note the state-estimation error would caused undesired acceleration behaviour
    if(global_com_feedback >0.5)
    {
        coma_des[0] = kpp[0] * (body_p_des(0,0) - com_sensor[0]) + kdp[0]*(0 - comv_sensor[0]);
        coma_des[1] = kpp[1] * (body_p_des(1,0)  - com_sensor[1]) + kdp[1]*(0 - comv_sensor[1]);
        coma_des[2] = kpp[2] * (body_p_des(2,0)  - com_sensor[2]) + kdp[2]*(0 - comv_sensor[2]); 
    }
    else
    {
        coma_des[0] = kpp[0] * (body_p_des[0] - support_pos_des[0] - (com_sensor[0] - support_pos_sensor[0])) + kdp[0]*(0 - comv_sensor[0]);
        coma_des[1] = kpp[1] * (body_p_des[1] - support_pos_des[1] - (com_sensor[1] - support_pos_sensor[1])) + kdp[1]*(0 - comv_sensor[1]);
        coma_des[2] = kpp[2] * (body_p_des[2] - support_pos_des[2] - (com_sensor[2] - support_pos_sensor[2])) + kdp[2]*(0 - comv_sensor[2]);        
    }
 
    theta_acc_des[0] = kpw[0] * (body_r_des(0,0) - theta_sensor[0]) + kdw[0]*(0 - thetav_sensor[0]);
    theta_acc_des[1] = kpw[1] * (body_r_des(1,0) - theta_sensor[1]) + kdw[1]*(0 - thetav_sensor[1]);
    theta_acc_des[2] = kpw[2] * (body_r_des(2,0) - theta_sensor[2]) + kdw[2]*(0 - thetav_sensor[2]); 
}

void Gain_offset_callback(const geometry_msgs::Twist::ConstPtr &msgIn) {
    if(grf_start)
    {
        kpp0_det = msgIn->linear.x;
        kdp0_det = msgIn->linear.y;
        kpp1_det = msgIn->linear.z;
        kdp1_det = msgIn->angular.x;
        kpp2_det = msgIn->angular.y;
        kdp2_det = msgIn->angular.z;

        kpp[0] +=  kpp0_det;
        kdp[0] +=  kdp0_det;

        kpp[1] +=  kpp1_det;
        kdp[1] +=  kdp1_det;          

        kpp[2] +=  kpp2_det;
        kdp[2] +=  kdp2_det; 

        kpw[0] +=  10*kpp0_det;
        kdw[0] +=  10*kdp0_det;

        kpw[1] +=  10*kpp1_det;
        kdw[1] +=  10*kdp1_det;          

        kpw[2] +=  10*kpp2_det;
        kdw[2] +=  10*kdp2_det;

        if(count_old % 10 ==0) 
        {
            std::cout<<"kpw[0]:"<<kpw[0]<<endl; 
            std::cout<<"kdw[0]:"<<kdw[0]<<endl; 
            std::cout<<"kpw[1]:"<<kpw[1]<<endl; 
            std::cout<<"kdw[1]:"<<kdw[1]<<endl;           
            std::cout<<"kpw[2]:"<<kpw[2]<<endl; 
            std::cout<<"kdw[2]:"<<kdw[2]<<endl;
        }            

 

        // swing_kp(0,0) +=  kpp0_det;
        // swing_kd(0,0) +=  kdp0_det;

        // swing_kp(1,1) +=  kpp1_det;
        // swing_kd(1,1) +=  kdp1_det;          

        // swing_kp(2,2) +=  kpp2_det;
        // swing_kd(2,2) +=  kdp2_det;

        // std::cout<<"swing_kp(0,0):"<<swing_kp(0,0)<<endl; 
        // std::cout<<"swing_kd(0,0):"<<swing_kd(0,0)<<endl;  
        // std::cout<<"swing_kp(1,1):"<<swing_kp(1,1)<<endl; 
        // std::cout<<"swing_kd(1,1):"<<swing_kd(1,1)<<endl;           
        // std::cout<<"swing_kp(2,2):"<<swing_kp(2,2)<<endl; 
        // std::cout<<"swing_kd(2,2):"<<swing_kd(2,2)<<endl; 
    }

}

void config_set()
{   
    /////load default parameter from the yaml.file
    ///////////////////  yaml code . ///////// 
    YAML::Node config = YAML::LoadFile("/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/config/config.yaml");


    kpp[0] = config["kpp0"].as<double>();
    kpp[1] = config["kpp1"].as<double>();
    kpp[2] = config["kpp2"].as<double>();
    kdp[0] = config["kdp0"].as<double>();
    kdp[1] = config["kdp1"].as<double>();
    kdp[2] = config["kdp2"].as<double>();
    kpw[0] = config["kpw0"].as<double>();
    kpw[1] = config["kpw1"].as<double>();
    kpw[2] = config["kpw2"].as<double>();
    kdw[0] = config["kdw0"].as<double>();
    kdw[1] = config["kdw1"].as<double>();
    kdw[2] = config["kdw2"].as<double>();


    // swing_kp(0,0) = config["swingkp0"].as<double>();
    // swing_kp(1,1) = config["swingkp1"].as<double>();
    // swing_kp(2,2) = config["swingkp2"].as<double>();
    // swing_kd(0,0) = config["swingkd0"].as<double>();
    // swing_kd(1,1) = config["swingkd1"].as<double>();
    // swing_kd(2,2) = config["swingkd2"].as<double>();

    body_p_Homing_Retarget(0,0)  = config["body_p_Homing_Retarget0"].as<double>();
    body_p_Homing_Retarget(1,0)  = config["body_p_Homing_Retarget1"].as<double>();
    body_p_Homing_Retarget(2,0)  = config["body_p_Homing_Retarget2"].as<double>();

    clear_height = config["foot_clear_height"].as<double>();


    w_pos_m_kp[0] = config["w_pos_m_kp0"].as<double>();
    w_pos_m_kp[1] = config["w_pos_m_kp1"].as<double>();
    w_pos_m_kp[2] = config["w_pos_m_kp2"].as<double>();
    w_pos_m_kd[0] = config["w_pos_m_kd0"].as<double>();
    w_pos_m_kd[1] = config["w_pos_m_kd1"].as<double>();
    w_pos_m_kd[2] = config["w_pos_m_kd2"].as<double>();
    w_rpy_m_kp[0] = config["w_rpy_m_kp0"].as<double>();
    w_rpy_m_kp[1] = config["w_rpy_m_kp1"].as<double>();
    w_rpy_m_kp[2] = config["w_rpy_m_kp2"].as<double>();
    w_rpy_m_kd[0] = config["w_rpy_m_kd0"].as<double>();
    w_rpy_m_kd[1] = config["w_rpy_m_kd1"].as<double>();
    w_rpy_m_kd[2] = config["w_rpy_m_kd2"].as<double>();

    com_pos_max = config["com_pos_max2"].as<double>();
    com_pos_min = config["com_pos_min2"].as<double>();
    com_rpy_max = config["com_rpy_max2"].as<double>();
    com_rpy_min = config["com_rpy_min2"].as<double>();

    global_com_feedback = config["global_com_feedback1"].as<double>();
    dtx = config["dt_grf1"].as<double>();
    cout<<"dtx_grf_update:"<<dtx<<endl;

}


int main(int argc, char *argv[])
{

    ros::init(argc,argv,"grf");
    ros::NodeHandle nh;



    Rfoot_location_feedback.setZero();

    Lfoot_location_feedback.setZero();

    body_p_des.setZero();
    body_r_des.setZero();
    FR_foot_des.setZero(); 
    FL_foot_des.setZero(); 
    RR_foot_des.setZero(); 
    RL_foot_des.setZero(); 
    F_sum.setZero(); 
    comv_des.setZero(); 
    thetav_des.setZero(); 
    Momentum_sum << 0.0168352186, 0.0004636141, 0.0002367952,
                    0.0004636141, 0.0656071082, 3.6671e-05, 
                    0.0002367952, 3.6671e-05,   0.0742720659;

    coma_des.setZero();
    theta_acc_des.setZero();
    config_set();

    gait_mode = 102; 
    right_support =2; 
    y_offset =0;

    joint2simulation.position.resize(100);
    state_feedback_receieved.setZero();
    wbc_gait_result.setZero();
    wbc_gait_result(2) = wbc_gait_result(5) = wbc_gait_result(8) = wbc_gait_result(11) = gait::mass * gait::_g /4;

    gait_start = 0;
    grf_start = false;

    count_old = 0;   
    mpc_stop = 0;
    n_point = (int) 1/dtx;
    dynamic_count = 0;
    dynamic_count_period = 0;

   for(int i = 0; i < 3; i++){
        comv_des[i] = 0;
        coma_des[i] = 0;
        theta_acc_des[i] = 0;
        comv_sensor[i] = 0;
        com_sensor[i] = 0; 
        com_des_pre[0] = 0;
        theta_des_pre[0] = 0;
        w_pos_m[i] = 0;    
        w_rpy_m[i] = 0;   
        w_pos_m_filter[i] = 0;    
        w_rpy_m_filter[i] = 0;             

        FR_sensor[i] = 0;
        FL_sensor[i] = 0;
        RR_sensor[i] = 0;
        RL_sensor[i] = 0;
        body_relative_support_sensor_old[i] =  com_sensor[i] - support_pos_sensor[i];
        body_relative_supportv_sensor[i] = 0;
    }

    FR_swing = false;
    FL_swing = false; 
    RR_swing = false; 
    RL_swing = false;

    double f_sample_comx1 = 1/dtx;
	double fcutoff_comx1 = 5;
	double fcutoff_comx2 = 20;
	butterworthLPF31.init(f_sample_comx1,fcutoff_comx2);
	butterworthLPF32.init(f_sample_comx1,fcutoff_comx2);
	butterworthLPF33.init(f_sample_comx1,fcutoff_comx2);
	butterworthLPF34.init(f_sample_comx1,fcutoff_comx2);
	butterworthLPF35.init(f_sample_comx1,fcutoff_comx2);
	butterworthLPF36.init(f_sample_comx1,fcutoff_comx2);
	butterworthLPF37.init(f_sample_comx1,fcutoff_comx2);
    FR_foot_desx = 0;
    FR_foot_desy = 0;
    FR_foot_desz = 0;
    FL_foot_desx = 0;
    FL_foot_desy = 0;
    FL_foot_desz = 0;

    
    com_des_pre[2] = body_p_Homing_Retarget(2,0);


    ros::Subscriber state_feedback_subscribe_ = nh.subscribe<sensor_msgs::JointState>("/go1_gait_parameters", 10,state_feed_sub_operation);
    ros::Subscriber gain_tune = nh.subscribe("/Base_offset", 1, Gain_offset_callback);
    ros::Publisher gait_grf_pub_ = nh.advertise<sensor_msgs::JointState>("/go1/Grf_opt", 10);

    int pub_rate  = (int) (round(1/dtx));

    ros::Rate grf_rate(pub_rate);

    
    // ros::Duration duratione_des(dtx);

    while (ros::ok())
    {   
        ros::Time start = ros::Time::now(); 
        count_old +=1;
        
        if (gait_start > 0)
        {
            grf_start =true;   
        }
        else
        {
            if(count_old % 50 ==0)
            {
                cout<<"wait for grf node starting!! sending default values"<<endl;
            }            
        }

        if (grf_start)
        {
           dynamic_count +=1;
           ////generate gait trajectories////should be read from the NLP node./// not now
            //////////===========dyanmic walking COM movement test/////////////////////////
            body_p_des(0,0) = body_p_Homing_Retarget(0,0) + 0.00 * sin(8*M_PI*dynamic_count/n_point);
            body_p_des(1,0) = body_p_Homing_Retarget(1,0) + 0.00 * sin(8*M_PI*dynamic_count/n_point);
            body_p_des(2,0) = body_p_Homing_Retarget(2,0) + 0.00 *(sin(8*M_PI*dynamic_count/n_point));
            body_r_des(0,0) = 0.00 * sin(8*M_PI*dynamic_count/n_point);
            body_r_des(1,0) = 0.00 * sin(8*M_PI*dynamic_count/n_point);
            body_r_des(2,0) = 0.00 * sin(8*M_PI*dynamic_count/n_point); 
            
            comv_des(0,0) = -0.00 * cos(8*M_PI*dynamic_count/n_point) * (8*M_PI/n_point);
            comv_des(1,0) = -0.00 * cos(8*M_PI*dynamic_count/n_point) * (8*M_PI/n_point);
            comv_des(2,0) = -0.00 * cos(8*M_PI*dynamic_count/n_point) * (8*M_PI/n_point);

            thetav_des(0,0) = -0.00 * cos(8*M_PI*dynamic_count/n_point) * (8*M_PI/n_point);
            thetav_des(1,0) = -0.00 * cos(8*M_PI*dynamic_count/n_point) * (8*M_PI/n_point);
            thetav_des(2,0) = -0.00 * cos(8*M_PI*dynamic_count/n_point) * (8*M_PI/n_point);   

            // right_support = 2;  
            /////////////////////// leg movement test//////////////////////
            // FR_foot_desx = abs(0.04 * sin(8*M_PI*dynamic_count/n_point))-0.01;
            // FR_foot_desy = abs(0.02 * sin(8*M_PI*dynamic_count/n_point))-0.01;
            FR_foot_desz = (clear_height * sin(8*M_PI*dynamic_count/n_point))-0.01;
            // FL_foot_desx = abs(0.04 * sin(8*M_PI*dynamic_count/n_point + M_PI))-0.01;
            // FL_foot_desy = abs(0.02 * sin(8*M_PI*dynamic_count/n_point + M_PI))-0.01;
            //FL_foot_desz = (clear_height * sin(8*M_PI*dynamic_count/n_point + M_PI))-0.01;                       
            if((FR_foot_desz<=0))
            {
                FR_foot_desz = 0;
            }
            if(FL_foot_desz<=0)
            {
                FL_foot_desz = 0;
            }

            if(FR_foot_desx <=0)
            {
                FR_foot_desx = 0;
            }

            if(FR_foot_desy <=0)
            {
                FR_foot_desy = 0;
            }                       

            dynamic_count_period = floor(dynamic_count * 2/n_point ); 
            FR_foot_desx *= pow((-1), dynamic_count_period % 2);
            FR_foot_desy *= pow((-1), dynamic_count_period % 2);

            if((FR_foot_desz<=0))
            {
                if(FL_foot_desz<=0)
                {
                    right_support = 2;
                }                           
                else
                {
                    right_support = 0;
                }

            }
            else
            {
                right_support = 1;
            }

            FR_foot_des(0,0) = FR_foot_Homing(0,0) + FR_foot_desx;
            FR_foot_des(1,0) = FR_foot_Homing(1,0) + FR_foot_desy;
            FR_foot_des(2,0) = FR_foot_Homing(2,0) + FR_foot_desz;
            RL_foot_des(0,0) = RL_foot_Homing(0,0) + FR_foot_desx;
            RL_foot_des(1,0) = RL_foot_Homing(1,0) + FR_foot_desy;
            RL_foot_des(2,0) = RL_foot_Homing(2,0) + FR_foot_desz;

            FL_foot_des(0,0) = FL_foot_Homing(0,0) + FL_foot_desx;
            FL_foot_des(1,0) = FL_foot_Homing(1,0) + FL_foot_desy;
            FL_foot_des(2,0) = FL_foot_Homing(2,0) + FL_foot_desz;
            RR_foot_des(0,0) = RR_foot_Homing(0,0) + FL_foot_desx;
            RR_foot_des(1,0) = RR_foot_Homing(1,0) + FL_foot_desy;
            RR_foot_des(2,0) = RR_foot_Homing(2,0) + FL_foot_desz;

            ////// body pose feedback: obtained from the real-time feedback
            base_pos_fb_controler();          
            body_p_des(0,0) +=  w_pos_m_filter[0];
            body_p_des(1,0) +=  w_pos_m_filter[1];
            body_p_des(2,0) +=  w_pos_m_filter[2];
            body_r_des(0,0) +=  w_rpy_m_filter[0];
            body_r_des(1,0) +=  w_rpy_m_filter[1];
            body_r_des(2,0) +=  w_rpy_m_filter[2];
            if (dynamic_count>1)
            {
                comv_des(0,0) = (body_p_des(0,0) - com_des_pre[0]) /dtx;
                comv_des(1,0) = (body_p_des(1,0) - com_des_pre[1]) /dtx;
                comv_des(2,0) = (body_p_des(2,0) - com_des_pre[2]) /dtx;

                thetav_des(0,0) = (body_r_des(0,0) - theta_des_pre[0]) /dtx;
                thetav_des(1,0) = (body_r_des(1,0) - theta_des_pre[1]) /dtx;
                thetav_des(2,0) = (body_r_des(2,0) - theta_des_pre[2]) /dtx;                           
            }
            base_acc_ff_controler(); 

            // ///////////////===========================================///////////////////////////
            // //////=================== QP-based force distribution ================//////////////////
            F_sum(0) = gait::mass * coma_des(0,0);
            F_sum(1) = gait::mass * coma_des(1,0);
            F_sum(2) = gait::mass * (gait::_g + coma_des(2,0));
            ///////==== momentum check !!!!!!!!!!!!!!!!!!!!!!!! =================
            F_sum(3,0) = Momentum_sum(0,0) * theta_acc_des(0,0) + Momentum_sum(0,1) * theta_acc_des(1,0) + Momentum_sum(0,2) * theta_acc_des(2,0);
            F_sum(4,0) = Momentum_sum(1,0) * theta_acc_des(0,0) + Momentum_sum(1,1) * theta_acc_des(1,0) + Momentum_sum(1,2) * theta_acc_des(2,0);
            F_sum(5,0) = Momentum_sum(2,0) * theta_acc_des(0,0) + Momentum_sum(2,1) * theta_acc_des(1,0) + Momentum_sum(2,2) * theta_acc_des(2,0);
            
            Dynam.force_opt(body_p_des,FR_foot_des, FL_foot_des, RR_foot_des, RL_foot_des,
                            F_sum, gait_mode, right_support, y_offset);

            wbc_gait_result.block<3,1>(0,0) = (1 * (Dynam.grf_opt.block<3,1>(0,0)));
            wbc_gait_result.block<3,1>(3,0) = (1 * (Dynam.grf_opt.block<3,1>(3,0)));
            wbc_gait_result.block<3,1>(6,0) = (1 * (Dynam.grf_opt.block<3,1>(6,0)));
            wbc_gait_result.block<3,1>(9,0) = (1 * (Dynam.grf_opt.block<3,1>(9,0)));            
            wbc_gait_result.block<3,1>(12,0) = body_p_des;
            wbc_gait_result.block<3,1>(15,0) = body_r_des;
            wbc_gait_result.block<3,1>(18,0) = FR_foot_des;
            wbc_gait_result.block<3,1>(21,0) = FL_foot_des;        
            wbc_gait_result.block<3,1>(24,0) = RR_foot_des;
            wbc_gait_result.block<3,1>(27,0) = RL_foot_des;          
        }
        else
        {
            if(count_old>10)
            {
                // ////////===========dyanmic walking COM movement test/////////////////////////
                body_p_des(0,0) = body_p_Homing_Retarget(0,0) + 0.00 * sin(8*M_PI*dynamic_count/n_point);
                body_p_des(1,0) = body_p_Homing_Retarget(1,0) + 0.00 * sin(8*M_PI*dynamic_count/n_point);
                body_p_des(2,0) = body_p_Homing_Retarget(2,0) + 0.00 *(sin(8*M_PI*dynamic_count/n_point));
                body_r_des(0,0) = 0.00 * sin(8*M_PI*dynamic_count/n_point);
                body_r_des(1,0) = 0.00 * sin(8*M_PI*dynamic_count/n_point);
                body_r_des(2,0) = 0.00 * sin(8*M_PI*dynamic_count/n_point); 
                
                comv_des(0,0) = -0.00 * cos(8*M_PI*dynamic_count/n_point) * (8*M_PI/n_point);
                comv_des(1,0) = -0.00 * cos(8*M_PI*dynamic_count/n_point) * (8*M_PI/n_point);
                comv_des(2,0) = -0.00 * cos(8*M_PI*dynamic_count/n_point) * (8*M_PI/n_point);

                thetav_des(0,0) = -0.00 * cos(8*M_PI*dynamic_count/n_point) * (8*M_PI/n_point);
                thetav_des(1,0) = -0.00 * cos(8*M_PI*dynamic_count/n_point) * (8*M_PI/n_point);
                thetav_des(2,0) = -0.00 * cos(8*M_PI*dynamic_count/n_point) * (8*M_PI/n_point);   


                FR_foot_des(0,0) = FR_foot_Homing(0,0) + FR_foot_desx;
                FR_foot_des(1,0) = FR_foot_Homing(1,0) + FR_foot_desy;
                FR_foot_des(2,0) = FR_foot_Homing(2,0) + FR_foot_desz;
                RL_foot_des(0,0) = RL_foot_Homing(0,0) + FR_foot_desx;
                RL_foot_des(1,0) = RL_foot_Homing(1,0) + FR_foot_desy;
                RL_foot_des(2,0) = RL_foot_Homing(2,0) + FR_foot_desz;

                FL_foot_des(0,0) = FL_foot_Homing(0,0) + FL_foot_desx;
                FL_foot_des(1,0) = FL_foot_Homing(1,0) + FL_foot_desy;
                FL_foot_des(2,0) = FL_foot_Homing(2,0) + FL_foot_desz;
                RR_foot_des(0,0) = RR_foot_Homing(0,0) + FL_foot_desx;
                RR_foot_des(1,0) = RR_foot_Homing(1,0) + FL_foot_desy;
                RR_foot_des(2,0) = RR_foot_Homing(2,0) + FL_foot_desz;

                // ///////////////===========================================///////////////////////////
                // //////=================== QP-based force distribution ================//////////////////
                F_sum(0) = gait::mass * coma_des(0,0);
                F_sum(1) = gait::mass * coma_des(1,0);
                F_sum(2) = gait::mass * (gait::_g + coma_des(2,0));
                ///////==== momentum check !!!!!!!!!!!!!!!!!!!!!!!! =================
                F_sum(3,0) = Momentum_sum(0,0) * theta_acc_des(0,0) + Momentum_sum(0,1) * theta_acc_des(1,0) + Momentum_sum(0,2) * theta_acc_des(2,0);
                F_sum(4,0) = Momentum_sum(1,0) * theta_acc_des(0,0) + Momentum_sum(1,1) * theta_acc_des(1,0) + Momentum_sum(1,2) * theta_acc_des(2,0);
                F_sum(5,0) = Momentum_sum(2,0) * theta_acc_des(0,0) + Momentum_sum(2,1) * theta_acc_des(1,0) + Momentum_sum(2,2) * theta_acc_des(2,0);
                
                Dynam.force_opt(body_p_des,FR_foot_des, FL_foot_des, RR_foot_des, RL_foot_des,
                                F_sum, gait_mode, right_support, y_offset);

                wbc_gait_result.block<3,1>(0,0) = (1 * (Dynam.grf_opt.block<3,1>(0,0)));
                wbc_gait_result.block<3,1>(3,0) = (1 * (Dynam.grf_opt.block<3,1>(3,0)));
                wbc_gait_result.block<3,1>(6,0) = (1 * (Dynam.grf_opt.block<3,1>(6,0)));
                wbc_gait_result.block<3,1>(9,0) = (1 * (Dynam.grf_opt.block<3,1>(9,0)));            
                wbc_gait_result.block<3,1>(12,0) = body_p_des;
                wbc_gait_result.block<3,1>(15,0) = body_r_des;
                wbc_gait_result.block<3,1>(18,0) = FR_foot_des;
                wbc_gait_result.block<3,1>(21,0) = FL_foot_des;        
                wbc_gait_result.block<3,1>(24,0) = RR_foot_des;
                wbc_gait_result.block<3,1>(27,0) = RL_foot_des;      
            }
      
        }


        for(int i=0;i<3;i++)
        {
            // body_relative_support_des_old[i] =  body_p_des[i] - support_pos_des[i];
            body_relative_support_sensor_old[i] =  com_sensor[i] - support_pos_sensor[i];
        }

        com_des_pre[0] = body_p_des[0];
        com_des_pre[1] = body_p_des[1];
        com_des_pre[2] = body_p_des[2];

        theta_des_pre[0] = body_r_des[0];
        theta_des_pre[1] = body_r_des[1];
        theta_des_pre[2] = body_r_des[2];

        ros::Time end = ros::Time::now();
	
        ros::Duration duration = end -start;

        double t_slow_grf = duration.toSec();	

        wbc_gait_result(98,0) = t_slow_grf;

        for (int jx = 0; jx<100; jx++)
        {
          joint2simulation.position[jx] = wbc_gait_result(jx,0);
        }
        
        joint2simulation.header.stamp = ros::Time::now();
         

    
        gait_grf_pub_.publish(joint2simulation);
        ros::spinOnce(); 
	    //// for simulation /////
        grf_rate.sleep();
        /* code */
    }

    


    return 0;
}


