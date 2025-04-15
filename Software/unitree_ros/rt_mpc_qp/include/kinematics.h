/*************************************************************
 * Copyright: Walker Team, UBTech
 * Auther: chunyu.chen
 * Created Date: 01/08/2018
 * Modify Date:
 *************************************************************/

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "robot_const_para_config.h"
#include <math.h>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
//#include "gait_state_switch.h"

/**
 * @namespace gait
 */
namespace gait{

    typedef enum {
        DOUBLE_LEGS_STAND = 0,
        LEFT_LEG_STAND,
        RIGHT_LEG_STAND
    }LegStandStatus;

    typedef struct {
        double orientation[9];
        double translation[3];
    }Frame;

    typedef struct {
        Frame joint1;
        Frame joint2;
        Frame joint3;
        Frame joint4;
        Frame joint5;
        Frame joint6;
        Frame foot;
        Frame waist;
        double jacobian[36];
    }Walker_Leg;

    typedef struct {
        double roll;
        double pitch;
        double yaw;
    }Pose;

    /**
     * @class Kinematics
     */
    class Kinematics{
    public:
        /**
         * @brief constructor
         */
        Kinematics();

        /**
         * @brief destructor
         */
        ~Kinematics();

        /**
         * @name
         * @brief
         * @param input phase including only right leg stand,only left leg stand and both legs stand
         *              Ltheta_encoder[] left leg's encoders, Rtheta_encoder[] right leg's encoder         *
         * @param output pL and pR the joint's para, the distance between ankle and waist.
         */
        bool FK_leg(const int phase, const sensor_msgs::JointState& joint_encoder,
                    Walker_Leg *pL, Walker_Leg *pR);

        /**
         * @name
         * @brief
         * @param input
         * @param output
         */
        bool FK_body2foot_leg(const sensor_msgs::JointState& joint_encoder, Pose *imu,
                              Walker_Leg *pL, Walker_Leg *pR);

        /**
         * @name
         * @brief
         * @param input
         * @param output
         */
        bool IK_leg(double Lfoot_XYZRPY[], double Waist_XYZRPY[], double Rfoot_XYZRPY[], double Ltheta[], double Rtheta[]);

    private:
        /**
         * @name
         * @brief
         * @param input
         * @param output
         */
        void Frame_init(double RPY[], double x, double y, double z, Frame &frame);

        /**
         * @name
         * @brief
         * @param input
         * @param output
         */
        void Jacobian(int phase, Walker_Leg *pR, Walker_Leg *pL);

        /**
         * @name Cross
         * @brief
         * @param input
         * @param output
         */

        void Cross(double u[], double v[], double new_vec[]);

        /**
         * @name
         * @brief
         * @param input
         * @param output
         */
        void RPYtoRotm(double RPY[], double rotm[]);

        /**
         * @name
         * @brief
         * @param input
         * @param output
         */
        void Matrix_Multiplication(double a[], double b[], int m,
                                   int n, int k, double c[]);

        /**
         * @name
         * @brief
         * @param inputs
         * @param outputs
         */
        void Matrix_Substraction(double a[], double b[], int n, double c[]);

        /**
         * @name
         * @brief
         * @param input
         * @param output
         */
        void Matrix_TransPose(double* Matrix, int m, int n, double* Matrix_TransPose);

        /**
         * @name
         * @brief
         * @param input
         * @param output
         */
        double norm_2(double v[]);

        /**
         * @name
         * @brief
         * @param input
         * @return
         */
        int sign(double x);

    };

    /**
     * @name triple
     */

    double triple(double x0, double x1, double v0, double v1, double T, double t);
}//namespace gait

#endif //KINEMATICS_H
