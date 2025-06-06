//
// Created by shuoy on 10/19/21.
//

#ifndef Go1_CPP_UTILS_H
#define Go1_CPP_UTILS_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/src/mosek_nlp_kmp/src/FORCEMPC_mit/Go1Params.h"

class Utils {
public:
    // compare to Eigen's default eulerAngles
    // this function returns yaw angle within -pi to pi
    static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);
    static Eigen::Matrix3d skew(Eigen::Vector3d vec);
    static Eigen::Matrix3d pseudo_inverse(const Eigen::Matrix3d &mat);
    static double cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2);
    static Eigen::Matrix3d Rx(double angle);
    static Eigen::Matrix3d Ry(double angle);
    static Eigen::Matrix3d Rz(double angle);
    static Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d theta);
    static Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d R);

};

class BezierUtils {
    // TODO: allow degree change? may not be necessary, we can stick to degree 4
public:
    BezierUtils () {
        curve_constructed = false;
        bezier_degree = 4;
    }
    // set of functions create bezier curves, get points, reset
    Eigen::Vector3d get_foot_pos_curve(float t,
                                       Eigen::Vector3d foot_pos_start,
                                       Eigen::Vector3d foot_pos_final,
                                       double terrain_pitch_angle);

    // bool reset_foot_pos_curve() {curve_constructed = false;}
private:
    double bezier_curve(double t, const std::vector<double> &P);

    bool curve_constructed;
    float bezier_degree;
};

#endif //Go1_CPP_UTILS_H
