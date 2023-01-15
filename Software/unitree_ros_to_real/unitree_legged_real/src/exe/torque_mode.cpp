/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "convert.h"
#include "sensor_msgs/JointState.h"
#include <Eigen/Dense>
#include "go1_const.h"
#include "geometry_msgs/Twist.h"


using namespace UNITREE_LEGGED_SDK;

float qDes[3]={0};
float dqDes[3] = {0}; 
float dqDes_old[3] = {0}; 
int ctrl_estimation = 1000;


template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

double jointLinearInterpolation(double initPos, double targetPos, double rate, int j)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    dqDes[j] = 0.8*dqDes_old[j] + 0.2*(p - qDes[j])/(1.0/ctrl_estimation);
    dqDes_old[j] = dqDes[j];
    return p;
}


void keyboard_model_callback(const geometry_msgs::Twist::ConstPtr &msgIn) {
    
    if((msgIn->linear.x == 1))
    {
        printf("===========Switch to STAND_UP state==========");
    }
    if((msgIn->linear.x == 2))
    {
        printf("===========Switch to DYNAMIC WALKING state==========");
    } 
    printf("msgIn->linear.x: %f \n", msgIn->linear.x);  
}


template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(ctrl_estimation);

    ros::Publisher gait_data_pub; // for data_analysis
    ros::Publisher gait_data_pubx;  


    long motiontime=0;
    float torque = 0;
    float torque_L = 0;

    TCmd SendLowLCM = {0};
    TState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;

    sensor_msgs::JointState joint2simulation, joint2simulationx;
    joint2simulation.position.resize(100);
    joint2simulationx.position.resize(100);    

    bool initiated_flag = false;  // initiate need time
    int count = 0;   

    int rate_count = 0;
    int sin_count = 0;
    float qInit[3]={0};

    double Torque_ff = 0;
    double Torque_ff_L = 0;

    double k_spring = 3;
    double k_p_rest = -1.3;

    
    ////first configure
    // float sin_mid_q[3] = {0.0, 1.2, -2.0};
    ////second configure
    float sin_mid_q[3] = {0.0, 1.2, -1.3};
    double sin_joint1, sin_joint2;

    bool FF_enable = true; ///// Befor setting True, make sure the spring is engaged!!!!!!!!!!!!
    
    double torq_kp, torq_kd, torq_ki;
    torq_kp = 0;
    torq_kd = 0;
    torq_ki = 0;
    ////500 Hz
    // if(FF_enable)
    // {
    //     //  feedback plus feedforward
    //     torq_kp = 8;
    //     torq_kd = 0.05;
    // }
    // else
    // {
    //     // // only feedback
    //     torq_kp = 12;
    //     torq_kd = 0.25;
    // }

    ////// 1K hz 
    if(FF_enable)
    {
        //  feedback plus feedforward
        torq_kp = 7;
        torq_kd = 0.3;
        torq_ki = 0.05;
    }
    else
    {
        // // only feedback
        torq_kp = 8;
        torq_kd = 0.3;
        torq_ki = 0.05;
    }
    double torque_err_intergration = 0;
    Eigen::Matrix<double, 500,1> torque_err;
    torque_err.setZero();


    roslcm.SubscribeState();
    gait_data_pub = n.advertise<sensor_msgs::JointState>("go1_gait_data",10);
    gait_data_pubx = n.advertise<sensor_msgs::JointState>("go1_gait_datax",10);       
    ros::Subscriber robot_mode_cmd = n.subscribe("/Robot_mode", 1, keyboard_model_callback); //// for robot mode selectio

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
    }

    while (ros::ok()){
        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);
        // printf("FR foot force: %f\n",  RecvLowROS.footForce[0]);
        std::cout<<"Fr foot force:"<<RecvLowROS.footForce[0]<<std::endl;


        // gravity compensation
        SendLowROS.motorCmd[FR_0].tau = -0.65f;
        SendLowROS.motorCmd[FL_0].tau = +0.65f;
        SendLowROS.motorCmd[RR_0].tau = -0.65f;
        SendLowROS.motorCmd[RL_0].tau = +0.65f;
        
        if(initiated_flag == true){
            motiontime++;            
            if( motiontime >= 0){
                // first, get record initial position
                if( motiontime >= 0 && motiontime < 500){
                    qInit[0] = RecvLowROS.motorState[FR_0].q;
                    qInit[1] = RecvLowROS.motorState[FR_1].q;
                    qInit[2] = RecvLowROS.motorState[FR_2].q;

                    qDes[0] = qInit[0];
                    qDes[1] = qInit[1];
                    qDes[2] = qInit[2];

                }
                if( motiontime >= 500 && motiontime < 900){
                    // printf("%f %f %f\n", );
                    rate_count++;
                    double rate = rate_count/200.0;                       // needs count to 200

                    qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate, 0);
                    qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate, 1);
                    qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate, 2);
                }
                // last, do sine wave
                if( motiontime >= 900){
                    sin_count++;
                    sin_joint1 = 0.4 * sin(3*M_PI*sin_count/1000.0);
                    sin_joint2 = -0.3 * sin(1.8*M_PI*sin_count/1000.0);
                    qDes[0] = sin_mid_q[0];
                    qDes[1] = sin_mid_q[1];
                    //first configure
                    qDes[2] = sin_mid_q[2] + sin_joint2;
                    dqDes[2] = -0.3 * (1.8*M_PI/1000.0) * cos(1.8*M_PI*sin_count/1000.0);
                }

                if(qDes[2]<=go1_Calf_min)
                {
                    qDes[2]=go1_Calf_min;
                }
                else if (qDes[2]>=go1_Calf_max)
                {
                    qDes[2]=go1_Calf_max;
                }

                //// FR_2
                torque_err.block<499,1>(0,0) = torque_err.block<499,1>(1,0);

                torque_err(499,0) = qDes[2] - RecvLowROS.motorState[FR_2].q;

                torque_err_intergration = 0;
                for(int ij=0; ij<500; ij++)
                {
                   torque_err_intergration += torque_err(ij,0);
                } 
                 
                torque = (qDes[2] - RecvLowROS.motorState[FR_2].q)*torq_kp + (0 - RecvLowROS.motorState[FR_2].dq)*torq_kd + torque_err_intergration*torq_ki;
                
                if(qDes[2]<=k_p_rest)
                {
                    Torque_ff = k_spring * (qDes[2] - (k_p_rest));
                }
                else
                {
                    Torque_ff = 0;
                }
                
                if(FF_enable)
                {
                    torque += Torque_ff;
                }
                
                //// FL_2
                torque_L = (qDes[2] - RecvLowROS.motorState[FL_2].q)*torq_kp + (0 - RecvLowROS.motorState[FL_2].dq)*torq_kd;
                
                if(qDes[2]<=k_p_rest)
                {
                    Torque_ff_L = k_spring * (qDes[2] - (k_p_rest));
                }
                else
                {
                    Torque_ff_L = 0;
                }
                
                if(FF_enable)
                {
                    torque_L += Torque_ff_L;
                }



                if(torque > 3.0f) torque = 3.0f;
                if(torque < -3.0f) torque = -3.0f;

                if(torque_L > 3.0f) torque_L = 3.0f;
                if(torque_L < -3.0f) torque_L = -3.0f;


                SendLowROS.motorCmd[FR_2].q = PosStopF;
                SendLowROS.motorCmd[FR_2].dq = VelStopF;
                SendLowROS.motorCmd[FR_2].Kp = 0;
                SendLowROS.motorCmd[FR_2].Kd = 0;
                SendLowROS.motorCmd[FR_2].tau = torque;




                // SendLowROS.motorCmd[FL_2].q = PosStopF;
                // SendLowROS.motorCmd[FL_2].dq = VelStopF;
                // SendLowROS.motorCmd[FL_2].Kp = 0;
                // SendLowROS.motorCmd[FL_2].Kd = 0;
                // SendLowROS.motorCmd[FL_2].tau = torque_L;


            }


            

        }


        ///********************* data saving ************************************///////
        joint2simulation.header.stamp = ros::Time::now();
        joint2simulationx.header.stamp = ros::Time::now();
        ////
        for(int j=0; j<12; j++){
                joint2simulation.position[j] = SendLowROS.motorCmd[j].q; // desired joint angles; 
                if((j==2)||(j==5))
                {
                    joint2simulation.position[j] = qDes[2]; // desired joint angles; 
                }
        }
        for(int j=0; j<12; j++)
        {
            joint2simulation.position[12+j] = RecvLowROS.motorState[j].q;   // measured joint angles;
        } 


        // joint torque desired 
        for(int j=0; j<12; j++)
        {
            joint2simulation.position[78+j] = SendLowROS.motorCmd[j].tau;
        }

        //// torque measured
        for(int j=0; j<12; j++)
        {
            joint2simulationx.position[j] = RecvLowROS.motorState[j].tauEst;
        }
        
        /////// measured current
        for(int j=0; j<12; j++)
        {
            joint2simulationx.position[12+j] = RecvLowROS.motorState[j].tauEst;
        }        

        joint2simulationx.position[12] = Torque_ff;

        gait_data_pub.publish(joint2simulation);
        gait_data_pubx.publish(joint2simulationx);

        /////sending command ////////////
        SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
        roslcm.Send(SendLowLCM);

        ros::spinOnce();
        loop_rate.sleep();

        count++;
        if(count > 500){
            count = 500;
            initiated_flag = true;
        }

    }
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "torque_ros_mode");

    UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
}