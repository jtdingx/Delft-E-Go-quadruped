/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "body.h"

namespace unitree_model {

ros::Publisher servo_pub[12];
unitree_legged_msgs::LowCmd lowCmd;
unitree_legged_msgs::LowState lowState;

// These parameters are only for reference.
// Actual patameters need to be debugged if you want to run on real robot.
void paramInit()
{   

    ////posotion mode test
    // for(int i=0; i<4; i++){
    //     lowCmd.motorCmd[i*3+0].mode = 0x0A;
    //     lowCmd.motorCmd[i*3+0].Kp = 70;
    //     lowCmd.motorCmd[i*3+0].dq = 0;
    //     lowCmd.motorCmd[i*3+0].Kd = 3;
    //     lowCmd.motorCmd[i*3+0].tau = 0;
    //     lowCmd.motorCmd[i*3+1].mode = 0x0A;
    //     lowCmd.motorCmd[i*3+1].Kp = 180;
    //     lowCmd.motorCmd[i*3+1].dq = 0;
    //     lowCmd.motorCmd[i*3+1].Kd = 8;
    //     lowCmd.motorCmd[i*3+1].tau = 0;
    //     lowCmd.motorCmd[i*3+2].mode = 0x0A;
    //     lowCmd.motorCmd[i*3+2].Kp = 300;
    //     lowCmd.motorCmd[i*3+2].dq = 0;
    //     lowCmd.motorCmd[i*3+2].Kd = 15;
    //     lowCmd.motorCmd[i*3+2].tau = 0;
    // }
    // for(int i=0; i<12; i++){
    //     lowCmd.motorCmd[i].q = lowState.motorState[i].q;
    // }

    ////torque mode test
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 15;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 0.1;
        lowCmd.motorCmd[i*3+0].q = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 15;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 0.1;
        lowCmd.motorCmd[i*3+1].q = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 15;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 0.1;
        lowCmd.motorCmd[i*3+2].q = 0;
    }
    for(int i=0; i<12; i++){
        lowCmd.motorCmd[i].tau = lowState.motorState[i].tauEst;
    }    
}

void stand()
{   
    double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                      0.0, 0.77, -1.0, -0.0, 0.77, -1.0};
    moveAllPosition(pos, 2*1000);
}

void motion_init()
{
    paramInit();
    //stand();
}

void sendServoCmd()
{
    for(int m=0; m<12; m++){
        servo_pub[m].publish(lowCmd.motorCmd[m]);
    }
    ros::spinOnce();
}

void moveAllPosition(double* targetPos, double duration)
{
    double pos[12] ,lastPos[12], percent, vel[12], tau[12];
    double T_kd, T_kp;
    T_kd = 0.01;
    T_kp = 15;
    for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;
    for(int i=1; i<=duration; i++){
        if(!ros::ok()) break;
        percent = (double)i/duration;
        for(int j=0; j<12; j++){
            // lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
            
            pos[j] = lastPos[j]*(1-percent) + targetPos[j]*percent;
            vel[j] = (pos[j] - targetPos[j])/0.001;
            lowCmd.motorCmd[j].tau = T_kp*(pos[j] - lowState.motorState[j].q) +T_kd * (vel[j] - lowState.motorState[j].dq); 
        }


        lowCmd.motorCmd[0].tau = -0.5f;
        lowCmd.motorCmd[3].tau = 0.5f;
        lowCmd.motorCmd[6].tau = -0.5f;
        lowCmd.motorCmd[9].tau = 0.5f;

        

        sendServoCmd();

        
    }
}


}
