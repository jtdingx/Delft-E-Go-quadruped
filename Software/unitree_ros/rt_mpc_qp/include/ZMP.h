/*************************************************************
 * Copyright: Walker Team, UBTech
 * Auther: chunyu.chen
 * Build Date: 02/08/2018
 * Modify Date:
 *************************************************************/
#ifndef ZMP_H
#define ZMP_H

//#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "kinematics.h"
#include "sensor_msgs/JointState.h"
#include "robot_const_para_config.h"
#include <vector>
#include <algorithm>

/**
 * @namespace gait
 */
namespace gait{

    typedef struct {
        double fx;
        double fy;
        double fz;
        double tx;
        double ty;
        double tz;
    }ForceTorque;

    typedef struct {
        double x;
        double y;
    }Position2D;

    typedef struct {
        Position2D absolute_pos;
        Position2D relative_waist;
        Position2D l_relative_waist;
        Position2D r_relative_waist;
        Position2D l_absolute_pos;
        Position2D r_absolute_pos;
    }ZMPDataStruct;
    /**
     * @file
     * @brief ZMP computing
     * @class ZMP
     */
    class ZMP{
    public:
        /**
         * @brief constructor
         */
        ZMP();

        /**
         * @brief destructor
         */
        ~ZMP();

        /**
         * @name ZMPMeasured
         * @brief computing ZMP by data from F/T sensor
         * @param input pos_foot2waist including the distance between left or right,left x,y, right x,y
         *              foot and waist in x and y direction
         *              left_ft and right_ft are the data from F/T sensor
         *              offset is the between foot and F/T sensor middle.
         * @param output ZMP including relative waist ,left foot and right foot in x and y direction.
         */
        bool ZMPMeasured(const sensor_msgs::JointState joint_encoder, const geometry_msgs::Wrench& left_ft,
        const geometry_msgs::Wrench& right_ft, const double offset, ZMPDataStruct *out_zmp);

        bool ZMPMeasuredPose(const sensor_msgs::JointState joint_encoder, const geometry_msgs::Wrench& left_ft,
        const geometry_msgs::Wrench& right_ft, const double offset, const Pose& IMU ,ZMPDataStruct *out_zmp);

        /**
         * @name zmpJittersEliminationAverage
         * @brief used to solve the issue that robot's trembling when it's standing
         * @method it used to computing the average bewteen max and min during the Elimination's Cycle
         * @param input raw data, Elimination's Cycle, Elimination's Threshold
         * @param output ZMP including relative waist ,left foot and right foot in x and y direction.
         */
        double zmpJittersEliminationAverage(const double raw_data, const int zmp_size, const double threshold);

        /**
         * @name zmpAbsolutePos
         * @brief
         * @param
         * @param
         */
        bool zmpAbsolutePos(const sensor_msgs::JointState joint_encoder, const geometry_msgs::Wrench& left_ft,
        const geometry_msgs::Wrench& right_ft, const double offset, const Pose& IMU, ZMPDataStruct *out_zmp);


    public:
        Walker_Leg pL, pR;
        Kinematics kinematic_zmp_;

        //0--double on, 1--left on, 2--right on, 3--fly phase
        int stand_status, stand_status_hold;

        std::vector<double> zmp_jitters_elimination_;
    };
}

#endif //ZMP_H
