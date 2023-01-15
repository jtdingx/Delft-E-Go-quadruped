/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
Edited by Jiatao Ding, email: jtdingx@gmail.com
************************************************************************/

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include "body.h"
#include "Kinematics.h"


using namespace std;
using namespace Eigen;


Kinematicclass::Kinematicclass()
{
    FR_leg_offset_x = 0.1881;
    FL_leg_offset_x = 0.1881;
    RR_leg_offset_x = -0.1881;
    RL_leg_offset_x = -0.1881;

    FR_leg_offset_y = RR_leg_offset_y = -0.04675;
    FL_leg_offset_y = RL_leg_offset_y = 0.04675;
    FR_thigh_y = RR_thigh_y = -0.08;
    FL_thigh_y = RL_thigh_y = 0.08;
    FR_thigh_length = FL_thigh_length = RR_thigh_length = RL_thigh_length =  -0.213;
    FR_calf_length = FL_calf_length = RR_calf_length = RL_calf_length = -0.213;

    FR_feet.setZero();
    FR_feet.setZero();
    FR_feet.setZero();
    FR_feet.setZero();

    Jacobian_kin.setZero();

    pos_hip.setZero();pos_thigh.setZero();pos_calf.setZero();pos_feet.setZero();

    lamda = 0.5;

    pos_cal.setZero(); det_pos.setZero(); det_angle.setZero(); q_des.setZero();        
}


Kinematicclass::~Kinematicclass()
{
}

// locally
Eigen::Matrix<double, 3,1> Kinematicclass::Forward_kinematics(Eigen::Matrix<double, 3,1> q_joint, int feet_flag)
{   
    if (feet_flag ==0)
    {
        leg_offset_x = FR_leg_offset_x;
        leg_offset_y = FR_leg_offset_y;
        thigh_y = FR_thigh_y;
        thigh_length = FR_thigh_length;
        calf_length = FR_calf_length;
    }
    else
    {
        if (feet_flag == 1)
        {
            leg_offset_x = FL_leg_offset_x;
            leg_offset_y = FL_leg_offset_y;
            thigh_y = FL_thigh_y;
            thigh_length = FL_thigh_length;
            calf_length = FL_calf_length;                 
        }
        else
        {
            if (feet_flag == 2)
            {
                leg_offset_x = RR_leg_offset_x;
                leg_offset_y = RR_leg_offset_y;
                thigh_y = RR_thigh_y;
                thigh_length = RR_thigh_length;
                calf_length = RR_calf_length;
            }
            else
            {
                leg_offset_x = RL_leg_offset_x;
                leg_offset_y = RL_leg_offset_y;
                thigh_y = RL_thigh_y;
                thigh_length = RL_thigh_length;
                calf_length = RL_calf_length;  
            }
        }
    }
    

    double q_hip = q_joint(0,0);
    double q_thigh = q_joint(1,0);
    double q_calf = q_joint(2,0);


    pos_hip << leg_offset_x,
               leg_offset_y,
               0;

    pos_thigh << leg_offset_x,
                 leg_offset_y + thigh_y*cos(q_hip),
                 thigh_y*sin(q_hip);   


    pos_calf << leg_offset_x + thigh_length*sin(q_thigh),
                leg_offset_y + thigh_y*cos(q_hip) - thigh_length*cos(q_thigh)*sin(q_hip),
                thigh_y*sin(q_hip) + thigh_length*cos(q_hip)*cos(q_thigh);


    pos_feet << leg_offset_x + calf_length*(cos(q_calf)*sin(q_thigh) + cos(q_thigh)*sin(q_calf)) + thigh_length*sin(q_thigh),
                leg_offset_y + calf_length*(sin(q_calf)*sin(q_hip)*sin(q_thigh) - cos(q_calf)*cos(q_thigh)*sin(q_hip)) + thigh_y*cos(q_hip) - thigh_length*cos(q_thigh)*sin(q_hip),
                thigh_y*sin(q_hip) + calf_length*(cos(q_calf)*cos(q_hip)*cos(q_thigh) - cos(q_hip)*sin(q_calf)*sin(q_thigh)) + thigh_length*cos(q_hip)*cos(q_thigh);



    Jacobian_kin(0,0) = 0;  
    Jacobian_kin(0,1) = calf_length*(cos(q_calf)*cos(q_thigh) - sin(q_calf)*sin(q_thigh)) + thigh_length*cos(q_thigh); 
    Jacobian_kin(0,2) = calf_length*(cos(q_calf)*cos(q_thigh) - sin(q_calf)*sin(q_thigh));
    Jacobian_kin(1,0) = - thigh_y*sin(q_hip) - calf_length*(cos(q_calf)*cos(q_hip)*cos(q_thigh) - cos(q_hip)*sin(q_calf)*sin(q_thigh)) - thigh_length*cos(q_hip)*cos(q_thigh);
    Jacobian_kin(1,1) = calf_length*(cos(q_calf)*sin(q_hip)*sin(q_thigh) + cos(q_thigh)*sin(q_calf)*sin(q_hip)) + thigh_length*sin(q_hip)*sin(q_thigh);
    Jacobian_kin(1,2) = calf_length*(cos(q_calf)*sin(q_hip)*sin(q_thigh) + cos(q_thigh)*sin(q_calf)*sin(q_hip));
    Jacobian_kin(2,0) = calf_length*(sin(q_calf)*sin(q_hip)*sin(q_thigh) - cos(q_calf)*cos(q_thigh)*sin(q_hip)) + thigh_y*cos(q_hip) - thigh_length*cos(q_thigh)*sin(q_hip);
    Jacobian_kin(2,1) = - calf_length*(cos(q_calf)*cos(q_hip)*sin(q_thigh) + cos(q_hip)*cos(q_thigh)*sin(q_calf)) - thigh_length*cos(q_hip)*sin(q_thigh);
    Jacobian_kin(2,2) = -calf_length*(cos(q_calf)*cos(q_hip)*sin(q_thigh) + cos(q_hip)*cos(q_thigh)*sin(q_calf));


    return pos_feet;
}

// global
Eigen::Matrix<double, 3,1> Kinematicclass::Forward_kinematics_g(Eigen::Matrix<double, 3,1> body_P, Eigen::Matrix<double, 3,1> body_R, Eigen::Matrix<double, 3,1> q_joint, int feet_flag)
{   
    if (feet_flag ==0)
    {
        leg_offset_x = FR_leg_offset_x;
        leg_offset_y = FR_leg_offset_y;
        thigh_y = FR_thigh_y;
        thigh_length = FR_thigh_length;
        calf_length = FR_calf_length;
    }
    else
    {
        if (feet_flag == 1)
        {
            leg_offset_x = FL_leg_offset_x;
            leg_offset_y = FL_leg_offset_y;
            thigh_y = FL_thigh_y;
            thigh_length = FL_thigh_length;
            calf_length = FL_calf_length;                 
        }
        else
        {
            if (feet_flag == 2)
            {
                leg_offset_x = RR_leg_offset_x;
                leg_offset_y = RR_leg_offset_y;
                thigh_y = RR_thigh_y;
                thigh_length = RR_thigh_length;
                calf_length = RR_calf_length;
            }
            else
            {
                leg_offset_x = RL_leg_offset_x;
                leg_offset_y = RL_leg_offset_y;
                thigh_y = RL_thigh_y;
                thigh_length = RL_thigh_length;
                calf_length = RL_calf_length;  
            }
        }
    }
    

    double q_hip = q_joint(0,0);
    double q_thigh = q_joint(1,0);
    double q_calf = q_joint(2,0);

    double body_px = body_P(0,0); 
    double body_py = body_P(1,0);
    double body_pz = body_P(2,0); 
    double body_r = body_R(0,0); 
    double body_p = body_R(1,0); 
    double body_y = body_R(2,0);    


    pos_hip << body_px - leg_offset_y*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r)) + leg_offset_x*cos(body_p)*cos(body_y),
               body_py + leg_offset_y*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y)) + leg_offset_x*cos(body_p)*sin(body_y),
               body_pz - leg_offset_x*sin(body_p) + leg_offset_y*cos(body_p)*sin(body_r);

    pos_thigh << body_px - thigh_y*(cos(q_hip)*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r)) - sin(q_hip)*(sin(body_r)*sin(body_y) + cos(body_r)*cos(body_y)*sin(body_p))) - leg_offset_y*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r)) + leg_offset_x*cos(body_p)*cos(body_y),
                 body_py + thigh_y*(cos(q_hip)*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y)) - sin(q_hip)*(cos(body_y)*sin(body_r) - cos(body_r)*sin(body_p)*sin(body_y))) + leg_offset_y*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y)) + leg_offset_x*cos(body_p)*sin(body_y),
                 body_pz - leg_offset_x*sin(body_p) + thigh_y*(cos(body_p)*cos(body_r)*sin(q_hip) + cos(body_p)*cos(q_hip)*sin(body_r)) + leg_offset_y*cos(body_p)*sin(body_r);   


    pos_calf << body_px - thigh_y*(cos(q_hip)*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r)) - sin(q_hip)*(sin(body_r)*sin(body_y) + cos(body_r)*cos(body_y)*sin(body_p))) + thigh_length*(cos(q_thigh)*(cos(q_hip)*(sin(body_r)*sin(body_y) + cos(body_r)*cos(body_y)*sin(body_p)) + sin(q_hip)*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r))) + cos(body_p)*cos(body_y)*sin(q_thigh)) - leg_offset_y*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r)) + leg_offset_x*cos(body_p)*cos(body_y),
                body_py + thigh_y*(cos(q_hip)*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y)) - sin(q_hip)*(cos(body_y)*sin(body_r) - cos(body_r)*sin(body_p)*sin(body_y))) - thigh_length*(cos(q_thigh)*(cos(q_hip)*(cos(body_y)*sin(body_r) - cos(body_r)*sin(body_p)*sin(body_y)) + sin(q_hip)*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y))) - cos(body_p)*sin(body_y)*sin(q_thigh)) + leg_offset_y*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y)) + leg_offset_x*cos(body_p)*sin(body_y),
                body_pz - thigh_length*(sin(body_p)*sin(q_thigh) - cos(q_thigh)*(cos(body_p)*cos(body_r)*cos(q_hip) - cos(body_p)*sin(body_r)*sin(q_hip))) - leg_offset_x*sin(body_p) + thigh_y*(cos(body_p)*cos(body_r)*sin(q_hip) + cos(body_p)*cos(q_hip)*sin(body_r)) + leg_offset_y*cos(body_p)*sin(body_r);


    pos_feet << body_px - thigh_y*(cos(q_hip)*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r)) - sin(q_hip)*(sin(body_r)*sin(body_y) + cos(body_r)*cos(body_y)*sin(body_p))) + thigh_length*(cos(q_thigh)*(cos(q_hip)*(sin(body_r)*sin(body_y) + cos(body_r)*cos(body_y)*sin(body_p)) + sin(q_hip)*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r))) + cos(body_p)*cos(body_y)*sin(q_thigh)) + calf_length*(cos(q_calf)*(cos(q_thigh)*(cos(q_hip)*(sin(body_r)*sin(body_y) + cos(body_r)*cos(body_y)*sin(body_p)) + sin(q_hip)*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r))) + cos(body_p)*cos(body_y)*sin(q_thigh)) - sin(q_calf)*(sin(q_thigh)*(cos(q_hip)*(sin(body_r)*sin(body_y) + cos(body_r)*cos(body_y)*sin(body_p)) + sin(q_hip)*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r))) - cos(body_p)*cos(body_y)*cos(q_thigh))) - leg_offset_y*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r)) + leg_offset_x*cos(body_p)*cos(body_y),
                body_py + thigh_y*(cos(q_hip)*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y)) - sin(q_hip)*(cos(body_y)*sin(body_r) - cos(body_r)*sin(body_p)*sin(body_y))) - thigh_length*(cos(q_thigh)*(cos(q_hip)*(cos(body_y)*sin(body_r) - cos(body_r)*sin(body_p)*sin(body_y)) + sin(q_hip)*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y))) - cos(body_p)*sin(body_y)*sin(q_thigh)) - calf_length*(cos(q_calf)*(cos(q_thigh)*(cos(q_hip)*(cos(body_y)*sin(body_r) - cos(body_r)*sin(body_p)*sin(body_y)) + sin(q_hip)*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y))) - cos(body_p)*sin(body_y)*sin(q_thigh)) - sin(q_calf)*(sin(q_thigh)*(cos(q_hip)*(cos(body_y)*sin(body_r) - cos(body_r)*sin(body_p)*sin(body_y)) + sin(q_hip)*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y))) + cos(body_p)*cos(q_thigh)*sin(body_y))) + leg_offset_y*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y)) + leg_offset_x*cos(body_p)*sin(body_y),
                body_pz - thigh_length*(sin(body_p)*sin(q_thigh) - cos(q_thigh)*(cos(body_p)*cos(body_r)*cos(q_hip) - cos(body_p)*sin(body_r)*sin(q_hip))) - leg_offset_x*sin(body_p) + thigh_y*(cos(body_p)*cos(body_r)*sin(q_hip) + cos(body_p)*cos(q_hip)*sin(body_r)) - calf_length*(cos(q_calf)*(sin(body_p)*sin(q_thigh) - cos(q_thigh)*(cos(body_p)*cos(body_r)*cos(q_hip) - cos(body_p)*sin(body_r)*sin(q_hip))) + sin(q_calf)*(cos(q_thigh)*sin(body_p) + sin(q_thigh)*(cos(body_p)*cos(body_r)*cos(q_hip) - cos(body_p)*sin(body_r)*sin(q_hip)))) + leg_offset_y*cos(body_p)*sin(body_r);

    Jacobian_kin(0,0) = thigh_y*(cos(q_hip)*(sin(body_r)*sin(body_y) + cos(body_r)*cos(body_y)*sin(body_p)) + sin(q_hip)*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r))) + calf_length*(cos(q_calf)*cos(q_thigh)*(cos(q_hip)*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r)) - sin(q_hip)*(sin(body_r)*sin(body_y) + cos(body_r)*cos(body_y)*sin(body_p))) - sin(q_calf)*sin(q_thigh)*(cos(q_hip)*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r)) - sin(q_hip)*(sin(body_r)*sin(body_y) + cos(body_r)*cos(body_y)*sin(body_p)))) + thigh_length*cos(q_thigh)*(cos(q_hip)*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r)) - sin(q_hip)*(sin(body_r)*sin(body_y) + cos(body_r)*cos(body_y)*sin(body_p)));
    Jacobian_kin(0,1) = - thigh_length*(sin(q_thigh)*(cos(q_hip)*(sin(body_r)*sin(body_y) + cos(body_r)*cos(body_y)*sin(body_p)) + sin(q_hip)*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r))) - cos(body_p)*cos(body_y)*cos(q_thigh)) - calf_length*(cos(q_calf)*(sin(q_thigh)*(cos(q_hip)*(sin(body_r)*sin(body_y) + cos(body_r)*cos(body_y)*sin(body_p)) + sin(q_hip)*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r))) - cos(body_p)*cos(body_y)*cos(q_thigh)) + sin(q_calf)*(cos(q_thigh)*(cos(q_hip)*(sin(body_r)*sin(body_y) + cos(body_r)*cos(body_y)*sin(body_p)) + sin(q_hip)*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r))) + cos(body_p)*cos(body_y)*sin(q_thigh)));
    Jacobian_kin(0,2) = -calf_length*(cos(q_calf)*(sin(q_thigh)*(cos(q_hip)*(sin(body_r)*sin(body_y) + cos(body_r)*cos(body_y)*sin(body_p)) + sin(q_hip)*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r))) - cos(body_p)*cos(body_y)*cos(q_thigh)) + sin(q_calf)*(cos(q_thigh)*(cos(q_hip)*(sin(body_r)*sin(body_y) + cos(body_r)*cos(body_y)*sin(body_p)) + sin(q_hip)*(cos(body_r)*sin(body_y) - cos(body_y)*sin(body_p)*sin(body_r))) + cos(body_p)*cos(body_y)*sin(q_thigh)));
    Jacobian_kin(1,0) = - thigh_y*(cos(q_hip)*(cos(body_y)*sin(body_r) - cos(body_r)*sin(body_p)*sin(body_y)) + sin(q_hip)*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y))) - calf_length*(cos(q_calf)*cos(q_thigh)*(cos(q_hip)*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y)) - sin(q_hip)*(cos(body_y)*sin(body_r) - cos(body_r)*sin(body_p)*sin(body_y))) - sin(q_calf)*sin(q_thigh)*(cos(q_hip)*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y)) - sin(q_hip)*(cos(body_y)*sin(body_r) - cos(body_r)*sin(body_p)*sin(body_y)))) - thigh_length*cos(q_thigh)*(cos(q_hip)*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y)) - sin(q_hip)*(cos(body_y)*sin(body_r) - cos(body_r)*sin(body_p)*sin(body_y)));
    Jacobian_kin(1,1) = thigh_length*(sin(q_thigh)*(cos(q_hip)*(cos(body_y)*sin(body_r) - cos(body_r)*sin(body_p)*sin(body_y)) + sin(q_hip)*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y))) + cos(body_p)*cos(q_thigh)*sin(body_y)) + calf_length*(cos(q_calf)*(sin(q_thigh)*(cos(q_hip)*(cos(body_y)*sin(body_r) - cos(body_r)*sin(body_p)*sin(body_y)) + sin(q_hip)*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y))) + cos(body_p)*cos(q_thigh)*sin(body_y)) + sin(q_calf)*(cos(q_thigh)*(cos(q_hip)*(cos(body_y)*sin(body_r) - cos(body_r)*sin(body_p)*sin(body_y)) + sin(q_hip)*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y))) - cos(body_p)*sin(body_y)*sin(q_thigh)));
    Jacobian_kin(1,2) = calf_length*(cos(q_calf)*(sin(q_thigh)*(cos(q_hip)*(cos(body_y)*sin(body_r) - cos(body_r)*sin(body_p)*sin(body_y)) + sin(q_hip)*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y))) + cos(body_p)*cos(q_thigh)*sin(body_y)) + sin(q_calf)*(cos(q_thigh)*(cos(q_hip)*(cos(body_y)*sin(body_r) - cos(body_r)*sin(body_p)*sin(body_y)) + sin(q_hip)*(cos(body_r)*cos(body_y) + sin(body_p)*sin(body_r)*sin(body_y))) - cos(body_p)*sin(body_y)*sin(q_thigh))); 
    Jacobian_kin(2,0) = thigh_y*(cos(body_p)*cos(body_r)*cos(q_hip) - cos(body_p)*sin(body_r)*sin(q_hip)) - calf_length*(cos(q_calf)*cos(q_thigh)*(cos(body_p)*cos(body_r)*sin(q_hip) + cos(body_p)*cos(q_hip)*sin(body_r)) - sin(q_calf)*sin(q_thigh)*(cos(body_p)*cos(body_r)*sin(q_hip) + cos(body_p)*cos(q_hip)*sin(body_r))) - thigh_length*cos(q_thigh)*(cos(body_p)*cos(body_r)*sin(q_hip) + cos(body_p)*cos(q_hip)*sin(body_r));
    Jacobian_kin(2,1) = - thigh_length*(cos(q_thigh)*sin(body_p) + sin(q_thigh)*(cos(body_p)*cos(body_r)*cos(q_hip) - cos(body_p)*sin(body_r)*sin(q_hip))) - calf_length*(cos(q_calf)*(cos(q_thigh)*sin(body_p) + sin(q_thigh)*(cos(body_p)*cos(body_r)*cos(q_hip) - cos(body_p)*sin(body_r)*sin(q_hip))) - sin(q_calf)*(sin(body_p)*sin(q_thigh) - cos(q_thigh)*(cos(body_p)*cos(body_r)*cos(q_hip) - cos(body_p)*sin(body_r)*sin(q_hip))));
    Jacobian_kin(2,2) = -calf_length*(cos(q_calf)*(cos(q_thigh)*sin(body_p) + sin(q_thigh)*(cos(body_p)*cos(body_r)*cos(q_hip) - cos(body_p)*sin(body_r)*sin(q_hip))) - sin(q_calf)*(sin(body_p)*sin(q_thigh) - cos(q_thigh)*(cos(body_p)*cos(body_r)*cos(q_hip) - cos(body_p)*sin(body_r)*sin(q_hip))));


    return pos_feet;
}


// locally 
Eigen::Matrix<double, 3,1> Kinematicclass::Inverse_kinematics(Eigen::Matrix<double, 3,1> pos_des, Eigen::Matrix<double, 3,1> q_ini,int feet_flag)
{
    

    pos_cal= Forward_kinematics(q_ini, feet_flag);

    q_des = q_ini;
    //cout <<"q_des"<< q_des.transpose()<<endl;



    for(int j=0; j<10;j++)
    {
        det_pos = pos_des-pos_cal;
        det_angle = lamda * (Jacobian_kin).inverse() * det_pos;

        if (det_angle.maxCoeff() < 0.0001)
        {
            break;
        }
        else
        {
            q_des(0,0)+= det_angle(0,0);
            q_des(1,0)+= det_angle(1,0);
            q_des(2,0)+= det_angle(2,0);

            pos_cal= Forward_kinematics(q_des, feet_flag);

        }
    }

    return q_des;


}

/// global
Eigen::Matrix<double, 3,1> Kinematicclass::Inverse_kinematics_g(Eigen::Matrix<double, 3,1> body_P, Eigen::Matrix<double, 3,1> body_R, Eigen::Matrix<double, 3,1> pos_des, Eigen::Matrix<double, 3,1> q_ini,int feet_flag)
{
    

    pos_cal= Forward_kinematics_g(body_P, body_R, q_ini, feet_flag);

    q_des = q_ini;
    //cout <<"q_des"<< q_des.transpose()<<endl;



    for(int j=0; j<15;j++)
    {
        det_pos = pos_des-pos_cal;
        det_angle = lamda * (Jacobian_kin).inverse() * det_pos;

        if (abs(pow(det_pos(0,0),2) + pow(det_pos(1,0),2) +pow(det_pos(2,0),2) ) <= 0.000001)
        {
            break;
        }
        else
        {
            q_des(0,0)+= det_angle(0,0);
            q_des(1,0)+= det_angle(1,0);
            q_des(2,0)+= det_angle(2,0);

            pos_cal= Forward_kinematics_g(body_P, body_R, q_des, feet_flag);

        }
    }

    return q_des;


}

