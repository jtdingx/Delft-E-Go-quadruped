/*************************************************************
 * Copyright: Walker Team, UBTech
 * Auther: chunyu.chen
 * Build Date: 08/08/2018
 * Modify Date:
 *************************************************************/

#ifndef MOTOR_GAIT_API_H
#define MOTOR_GAIT_API_H

#include "servo_ctrl/ecatservoctrl.hpp"
//#include "../../servo_control/inc/ecatservoctrl.hpp"
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <map>
#include <cfloat>
#include <boost/system/system_error.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

/**
 * @namespace gait
 */
namespace gait{

    typedef enum {
        LHipYaw = 0,
        LHipRoll,
        LHipPitch,
        LKneePitch,
        LAnklePitch,
        LAnkleRoll,

        RHipYaw,
        RHipRoll,
        RHipPitch,
        RKneePitch,
        RAnklePitch,
        RAnkleRoll
    }LegJointName;

    /**
     * @class MotorGaitApi
     */
    class MotorGaitApi{
    public:
        /**
         * @brief constructor
         */
        MotorGaitApi();

        /**
         * @brief destructor
         */
        ~MotorGaitApi();

        /**
         * @brief Motor Read Api
         * @param msg_out: measured encoder info
         */

        bool ApiRead(sensor_msgs::JointState &msgs_out);

        /**
         * @brief motor write joint data
         * @param mode
         * @param msgs_in
         * @return
         */
        bool ApiWrite(const ServoCtrlMode mode, const sensor_msgs::JointState msgs_in);
        /**
         * @brief Api init
         * @return
         */
        bool ApiInit();
    private:

        //motor ctrl api from servo ctrl
        EcatServoCtrl *ctrl;
        ServoInfoPtrMap legs_in, legs_out;
        ServoInfoPtr out_servo;

    };
}

#endif //MOTOR_GAIT_API_H
