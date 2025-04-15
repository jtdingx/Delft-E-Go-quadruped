//
// Created by shuoy on 11/1/21.
//

#ifndef Go1_CPP_Go1BASICEKF_H
#define Go1_CPP_Go1BASICEKF_H

#include "Robotpara/Go1Params.h"
#include "Robotpara/Go1CtrlStates.h"
#include "utils/Utils.h"
#include "Go1legKinematics/Go1Kinematics.h"

// state estimator parameters
#define STATE_SIZE 18
#define MEAS_SIZE 28
// #define PROCESS_NOISE_PIMU 0.01
// #define PROCESS_NOISE_VIMU 0.01
// #define PROCESS_NOISE_PFOOT 0.01
// #define SENSOR_NOISE_PIMU_REL_FOOT 0.001
// #define SENSOR_NOISE_VIMU_REL_FOOT 0.01
// #define SENSOR_NOISE_ZFOOT 0.01


#define FOOT_FILTER_WINDOW_SIZE 5
// implement a basic error state KF to estimate robot pose
// assume orientation is known from a IMU (state.root_rot_mat)
class Go1BasicEKF {
public:
    Go1BasicEKF ();
    Go1BasicEKF (bool assume_flat_ground_);
    void init_state(Go1CtrlStates& state);
    void config_set();

    // Eigen::Matrix<double,6,1> update_low_state(Go1CtrlStates& state,Eigen::Matrix<double, 3,1> IMU_angle,Eigen::Matrix<double, 4,1> IMU_quaternion,Eigen::Matrix<double, 3,1> IMU_gyroscope,
    // Eigen::Matrix<double, 3,1> IMU_accelerometer, Eigen::Matrix<double,12,1> q, Eigen::Matrix<double,12,1> dq, Eigen::Matrix<double, 4,1> FSR, double dt, int simulation_mode);
    
    void update_estimation(Go1CtrlStates& state, double dt, int simulation_mode, Eigen::Vector3d gra_acc);
    bool is_inited() {return filter_initialized;}
private:

    double PROCESS_NOISE_PIMU;
    double PROCESS_NOISE_VIMU;
    double PROCESS_NOISE_PFOOT;
    double SENSOR_NOISE_PIMU_REL_FOOT;
    double SENSOR_NOISE_VIMU_REL_FOOT;
    double SENSOR_NOISE_ZFOOT;
    double fz_limit;


    bool filter_initialized = false;
    // state
    // 0 1 2 pos 3 4 5 vel 6 7 8 foot pos FL 9 10 11 foot pos FR 12 13 14 foot pos RL 15 16 17 foot pos RR
    Eigen::Matrix<double, STATE_SIZE, 1> x; // estimation state
    Eigen::Matrix<double, STATE_SIZE, 1> xbar; // estimation state after process update
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P; // estimation state covariance
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar; // estimation state covariance after process update
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> A; // estimation state transition
    Eigen::Matrix<double, STATE_SIZE, 3> B; // estimation state transition
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q; // estimation state transition noise

    // observation
    // 0 1 2   FL pos residual
    // 3 4 5   FR pos residual
    // 6 7 8   RL pos residual
    // 9 10 11 RR pos residual
    // 12 13 14 vel residual from FL
    // 15 16 17 vel residual from FR
    // 18 19 20 vel residual from RL
    // 21 22 23 vel residual from RR
    // 24 25 26 27 foot height
    Eigen::Matrix<double, MEAS_SIZE, 1> y; //  observation
    Eigen::Matrix<double, MEAS_SIZE, 1> yhat; // estimated observation
    Eigen::Matrix<double, MEAS_SIZE, 1> error_y; // estimated observation
    Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y; // S^-1*error_y
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> C; // estimation state observation
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> SC; // S^-1*C
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R; // estimation state observation noise
    // helper matrices
    Eigen::Matrix<double, 3, 3> eye3; // 3x3 identity
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S; // Innovation (or pre-fit residual) covariance
    Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> K; // kalman gain


    bool assume_flat_ground = false;

    // variables to process foot force
    double smooth_foot_force[4];
    double estimated_contacts[4];

    // Go1 hardware switch foot order
    Eigen::Matrix<int, NUM_DOF, 1> swap_joint_indices;
    Eigen::Matrix<int, NUM_LEG, 1> swap_foot_indices;

    // Go1 hardware foot force filter
    Eigen::Matrix<double, NUM_LEG, FOOT_FILTER_WINDOW_SIZE> foot_force_filters;
    Eigen::Matrix<int, NUM_LEG, 1> foot_force_filters_idx;
    Eigen::Matrix<double, NUM_LEG, 1> foot_force_filters_sum;

    ////
    // add leg kinematics
    // the leg kinematics is relative to body frame, which is the center of the robot
    // following are some parameters that defines the transformation between IMU frame(b) and robot body frame(r)
    Eigen::Vector3d p_br;
    Eigen::Matrix3d R_br;
    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    double leg_offset_x[4] = {};
    double leg_offset_y[4] = {};
    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    double motor_offset[4] = {};
    double upper_leg_length[4] = {};
    double lower_leg_length[4] = {};
    std::vector<Eigen::VectorXd> rho_fix_list;
    std::vector<Eigen::VectorXd> rho_opt_list;
    Go1Kinematics go1_kin;

};


#endif //Go1_CPP_Go1BASICEKF_H
