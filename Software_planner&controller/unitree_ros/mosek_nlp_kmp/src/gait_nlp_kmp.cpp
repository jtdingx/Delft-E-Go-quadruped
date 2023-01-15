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
// #include "NLPRTControl/NLPRTControlClass.h"
//#include "/home/pi/go1_catkin/src/unitree_ros_to_real/unitree_legged_real/src/go1_rt_control/src/Robotpara/robot_const_para_config.h"
#include "/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/go1_rt_control/src/Robotpara/robot_const_para_config.h"
#include <geometry_msgs/Twist.h>  
#include "NLPRTControl/MpcRTControlClass.h"
#include "yaml.h"
#include <ros/package.h> 
#include <geometry_msgs/Twist.h>  


using namespace Eigen;
using namespace std;
using namespace gait;


sensor_msgs::JointState joint2simulation;

// NLPRTControlClass nlp_planner;
MpcRTControlClass tvo_nlp_planner;


bool mpc_start = false;

Eigen::Vector3d Rfoot_location_feedback;

Eigen::Vector3d Lfoot_location_feedback;

Eigen::Matrix<double,18,1> state_est;
Eigen::Matrix<double,25,1> state_feedback_receieved;
double step_length;
double step_width;

Matrix<double,100,1> mpc_gait_result;

extern double count;
extern double count_old;
double dt_mpc;

void state_feed_sub_operation(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int jx = 0; jx<25; jx++)
    {
        
        state_feedback_receieved(jx) = msg->position[jx]; 
        if (jx<18)
        {
            state_est(jx) = state_feedback_receieved(jx+1);
        }

    }
    
    Lfoot_location_feedback(0) = msg->position[19];
    Lfoot_location_feedback(1) = msg->position[20];
    Lfoot_location_feedback(2) = msg->position[21];
    
    Rfoot_location_feedback(0) = msg->position[22];
    Rfoot_location_feedback(1) = msg->position[23];
    Rfoot_location_feedback(2) = msg->position[24];    
}


void step_parameters_callback(const geometry_msgs::Twist::ConstPtr &msgIn) {
    if(mpc_start =true)
    {
        // step_ = msgIn->linear.x;
        // kdp0_det = msgIn->linear.y;
        // kpp1_det = msgIn->linear.z;
        // kdp1_det = msgIn->angular.x;
        // kpp2_det = msgIn->angular.y;
        // kdp2_det = msgIn->angular.z;

        step_length +=  0.01*msgIn->linear.x; /// W: forward
        // step_length +=  0.01*msgIn->linear.y;////S: backward
        if(step_length>0.07)
        {
           step_length>0.07;
        }
        else
        {
            if(step_length<-0.07)
            {
                step_length<-0.07;
            }
        }


        step_width +=  0.01*msgIn->linear.y;///A: leftward
        // step_width +=  0.01*msgIn->angular.x; ///D:rightward        
        if(step_width>0.05)
        {
           step_width>0.05;
        }
        else
        {
            if(step_width<-0.05)
            {
                step_width<-0.05;
            }
        }
        std::cout<<"step_length:"<<step_length<<endl; 
        std::cout<<"step_width:"<<step_width<<endl; 
    }

}





void keyboard_subscribe_sub_operation(const geometry_msgs::Twist::ConstPtr &msg)
{
    printf("linear x: %f\n",msg->linear.x); 
    printf("linear y: %f\n",msg->linear.y); 
}


void config_set()
{   
    /////load default parameter from the yaml.file
    ///////////////////  yaml code . ///////// 
    // YAML::Node config = YAML::LoadFile("/home/pi/go1_catkin/src/unitree_ros_to_real/unitree_legged_real/config/config.yaml");
    YAML::Node config = YAML::LoadFile("/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/go1_rt_control/config/config.yaml");

    dt_mpc =  config["dt_slow_mpc"].as<double>();
    std::cout<<"dt_mpc:"<<dt_mpc<<std::endl;


}



int main(int argc, char *argv[])
{

    ros::init(argc,argv,"mpc");
    ros::NodeHandle nh;

    joint2simulation.position.resize(100);
    state_est.setZero();
    state_feedback_receieved.setZero();
    mpc_gait_result.setZero();
    config_set();
    int count = 0;
    int count_old = 0;   
    double mpc_stop = 0;


    ros::Subscriber state_feedback_subscribe_ = nh.subscribe<sensor_msgs::JointState>("/rt2nrt/state", 10,state_feed_sub_operation);
    ros::Subscriber keyboard_subscribe_ = nh.subscribe<geometry_msgs::Twist>("/Robot_mode", 10,keyboard_subscribe_sub_operation);
    ros::Subscriber step_parameters_cmd = nh.subscribe("/Base_offset", 1, step_parameters_callback);    
    ros::Publisher mpc_gait_pub_ = nh.advertise<sensor_msgs::JointState>("/MPC/Gait", 10);

    int pub_rate  = (int) (round(1/dt_mpc));

    ros::Rate nrt_rate(pub_rate);
    step_length = 0.00;
    step_width = 0;

    
    ros::Duration duratione_des(dt_mpc);
    while (ros::ok())
    {   
        ros::Time start = ros::Time::now(); 

        if (state_feedback_receieved(0) > 0)
        {
            mpc_start =true;
            count += 1;
        }

        ///// input: obtained from the ros::subscriber
        /////////// time interval
        /////////// estimated_statex
        /////////// _Rfoot_location_feedback / _Lfoot_location_feedback
        //if ((mpc_stop <1)&&(count > count_old))
        if ((mpc_stop <1))
        {
        //   mpc_gait_result = nlp_planner.WalkingReactStepping(count,mpc_start,state_est,Rfoot_location_feedback, Lfoot_location_feedback);
             mpc_gait_result = tvo_nlp_planner.WalkingReactStepping(count,mpc_start,state_est,Rfoot_location_feedback, Lfoot_location_feedback,step_length,step_width); 
	  
        }

        mpc_stop = mpc_gait_result(97,0);

        ros::Time end = ros::Time::now();
	
        ros::Duration duration = end -start;

        double t_slow_mpc = duration.toSec();	

        mpc_gait_result(98,0) = t_slow_mpc;

        for (int jx = 0; jx<100; jx++)
        {
          joint2simulation.position[jx] = mpc_gait_result(jx,0);
        }
        
        joint2simulation.header.stamp = ros::Time::now();
         

        


        mpc_gait_pub_.publish(joint2simulation);

        //count_old = (int) state_feedback_receieved(0,0);
 
        ros::spinOnce(); 
        
	
	
	    //// for simulation /////
        nrt_rate.sleep();
        /* code */
    }

    


    return 0;
}


