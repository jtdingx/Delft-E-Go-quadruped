#ifndef JUMPING_RL_H
#define JUMPING_RL_H

#include <stdio.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
// #include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <chrono>
#include <ctime> 
#include "Filter/butterworth_filter.h"
#include "Filter/butterworthLPF.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <string>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "kinematics/Kinematics.h"
#include "convert.h"
#include "sensor_msgs/JointState.h"
#include "utils/Utils.h"
#include <torch/torch.h>
#include <torch/script.h>
#include "Go1EKF/Go1BasicEKF.h"
#include "Go1legKinematics/Go1Kinematics.h"


using namespace UNITREE_LEGGED_SDK;

const float pi=3.1415926;

void clip(double& n, double lower, double upper);
int kbhit(void);
Eigen::MatrixXd wrap_to_pi(Eigen::MatrixXd angles);
Eigen::MatrixXd AngleDifference( Eigen::MatrixXd angles1, Eigen::MatrixXd angles2);


const std::string repo_path = "/home/vassil/TU_Delft/Internship_quadruped/quadrupedal_loco/catkin_ws/src/quadrupedal_loco";
const std::string policies_path = repo_path + "/unitree_ros_to_real/unitree_legged_real/policies";

class Quadruped{
    public:
    Kinematicclass Kine;

    // -------
    ros::NodeHandle n;
    ros::Publisher data_pub; // for data_analysis
    sensor_msgs::JointState data2pub;


    double start_time;
    char input_character;
    int motiontime = 0;
    int flight_started_time = 0;
    double rate;
    double rate_count = 0;
    double rate_count_jump = 0;
    bool reset_counter_cont_jump = false;

    torch::jit::script::Module policy;
    
    int hist_len = 20;
    int num_obs = 1014; // Size of observation buffer (and neural network input)
    bool switch_order_legs = true; // Switch order of joints from FR,FL,RR,RL to FL,FR,RL,RR
    double obs_clip = 100;
    double action_clip = 100;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,Eigen::RowMajor>  base_lin_vel_history; 
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,Eigen::RowMajor>  base_ang_vel_history;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,Eigen::RowMajor>  dof_pos_history;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,Eigen::RowMajor>  dof_vel_history;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,Eigen::RowMajor>  actions_history;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,Eigen::RowMajor>  contact_history;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,Eigen::RowMajor>  error_quat_history;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,Eigen::RowMajor>  base_quat_history;
    Eigen::VectorXd commands;
    Eigen::VectorXd projected_gravity;


    Eigen::Matrix<double,12,1> actions;
    Eigen::Matrix<double,12,1> prevActions;
    Eigen::Matrix<double,12,1> actions_filtered_store;
    Eigen::Matrix<double,12,1> actions_unfiltered_store;
    double action_scale = 0.25;
    double hip_scale_reduction = 0.5;

    double Kp_policy = 20;
    double Kd_policy = 0.5;

    double action_filter_freq = 0.456;//0.816;

    Eigen::MatrixXd reference_trajectory;
    int rate_traj = 0;

    bool has_jumped_bool = 0;

    struct{
        double lin_vel;
        double ang_vel;
        double dof_pos;
        double dof_vel;
        double contact;
        double error_quat;
        double commands;
    } obs_scales;


    // Filters:
    // butterworthLPF butterworthLPF1,butterworthLPF2,butterworthLPF3,butterworthLPF4,butterworthLPF5,butterworthLPF6;
    butterworthLPF butterworthBaseVel[3];
    butterworthLPF butterworthBaseAngVel[3];
    butterworthLPF butterworthTorque[12];
    // butterworthLPF butterworthFcontact[4];
    butterworthLPF butterworthBaseAcc[3];

    double fcutoff_vel;
    double fcutoff_tor;
    double fcutoff_contact;
    double fcutoff_acc;
    double sample_freq;
    
    bool landed = false;
    bool flight_started = false;
    ros::Time t0;
    ros::Time t1;
    ros::Duration flight_time;


    Eigen::Matrix<double,3,1> base_pos;
    Eigen::Matrix<double,3,1> base_pos_prev;
    Eigen::Matrix<double,3,1> base_pos_ini;
    Eigen::Matrix<double,3,1> base_rpy_ini;

    Eigen::Matrix<double,3,1> base_pos_flight;
    Eigen::Matrix<double,3,1> base_vel_flight;
    Eigen::Matrix<double,3,1> base_vel_imu;
    Eigen::MatrixXd base_vel_imu_local;

    double accelerometer_bias[3] = {0.,0.,0.};
    Eigen::Vector3d accelerometer_ave;
    std::string gait_status;
    int n_count_calibration=0;
    int stand_up_count=0;
    Go1Kinematics go1_kin;
    std::vector<Eigen::VectorXd> rho_fix_list;
    std::vector<Eigen::VectorXd> rho_opt_list;
    double leg_offset_x[4];
    double leg_offset_y[4];
    double motor_offset[4];
    double upper_leg_length[4] = {};
    double lower_leg_length[4] = {};


    Eigen::Matrix<double,3,1> base_vel_flight_imu;
    Eigen::Matrix<double,3,1> base_vel_landing;



    Eigen::Matrix<double,3,1> base_acc;
    Eigen::Matrix<double,3,1> base_pos_imu;
    Eigen::Matrix<double,3,1> base_pos_landing;
    Eigen::Matrix<double,3,1> base_acc_local;
    Eigen::Matrix<double,3,1> base_acc_prev;

    Eigen::Matrix<double,3,1> base_pos_standing;


    Eigen::Quaterniond base_quat;
    Eigen::MatrixXd base_quat_arr;
    Eigen::MatrixXd error_quat;
    Eigen::Vector3d base_rpy; 
    Eigen::Vector3d base_rpy_prev;
    Eigen::Matrix<double,3,1> base_rpy_offset;


    Eigen::Matrix<double,12,1> q_mea;
    Eigen::Matrix<double,3,1> base_vel;
    Eigen::Matrix<double,3,1> base_ang_vel;
    Eigen::Matrix<double,12,1> dq_mea;
    Eigen::Matrix<double,3,3> R_base;
    Eigen::Matrix<double,12,1> tau_mea;

    Eigen::Matrix<double,12,1> qDes_policy;
    Eigen::Matrix<double,12,1> Torque_ff_GRF_data;


    Eigen::Matrix<double,4,1> contact;
    Eigen::Matrix<double,4,1> F_contact_force;


    double Kp_hip = 60;
    double Kp_thigh = 40;
    double Kp_calf = 60;
    double Kd = 1.0;
    float Kp_joint[12] = {0,0,0,0,0,0,0,0,0,0,0,0};//{10,10,12,10,10,12,10,10,12,10,10,12};
    float Kd_joint[12] = {0,0,0,0,0,0,0,0,0,0,0,0};//{1,1,1,1,1,1,1,1,1,1,1,1};


    float mass;
    Eigen::Matrix<double,3,1> grav;
    Eigen::Matrix<double,3,1> grav_vector;

    float DEFAULT_HIP_ANGLE = 0.0;
    float DEFAULT_THIGH_ANGLE =  0.7220;
    float DEFAULT_CALF_ANGLE = -1.4441;
 

    Eigen::Matrix<double,12,1> q_lower_limits;
    
    Eigen::Matrix<double,12,1> q_upper_limits;
    
    Eigen::Matrix<double,12,1> q0;


    Safety safe;
    // UDP udp(8090,"192.168.123.161",8082,sizeof(LowCmd).sizeof(LowState));
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};

    float dt = 0.001;
    float policy_dt = 0.02;
    float est_dt = 0.001;

    int decimation = 4;

    Eigen::Matrix<double,12,1> qInit;

    Eigen::Matrix<double,12,1> qDes; // Current desired angle

    // Desired homing pose
    Eigen::Matrix<double,12,1>  qHomingPose;

    Quadruped(uint8_t level, double step_size, double policy_step_size);

    void loadPolicy(std::string policy_name);
    void checkFlight();
    void updateStates(LowState RecvLowROS);
    void publishData();
    void resetStateHistory();
    void computeStateHistory();
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    Eigen::MatrixXd actionFilter(Eigen::MatrixXd actions);
    LowCmd prepareCommand(LowCmd SendLowROS,Eigen::Matrix<double,12,1> qDes, Eigen::Matrix<double, 12, 1> tau);
    Eigen::Matrix<double,12,1> changeLegOrder(Eigen::Matrix<double,12,1> vector);
    Eigen::Matrix<double,12,1> unscaleActions(Eigen::Matrix<double,12,1> actions);
    void upload_reference(const std::string& reference_file, Eigen::MatrixXd& mat);


    // void standUp(Eigen::Matrix<double,12,1>& qDes, int rate_count, double total_steps, const float qInit[12], const float homing_pose_q[12]);
    void standUp(Eigen::Matrix<double,12,1>& qDes, int rate_count, double total_steps, const Eigen::Matrix<double,12,1> qInit, const Eigen::Matrix<double,12,1> qHomingPose);

    void changePIDgains(double Kp_hip,double Kp_thigh, double Kp_calf, double K_d);
    void switchFBtoFF(int rate_count_jump, double Kp_hip_ini,double Kp_thigh_ini,double Kp_calf_ini, double Kd_ini);


    Eigen::VectorXd computeObservations();
    Eigen::Matrix<double,12,1> policyStep(Eigen::VectorXd obs);
    Eigen::Matrix<double,12,1> safetyPolicy(Eigen::Matrix<double,12,1> qDes_policy);
    int CheckPositionProtect(Eigen::Matrix<double,12,1>& qDes_policy, LowState state, double soft_limit);
    
    private:
    template <typename T> 
    Eigen::MatrixXd shiftHistory(Eigen::MatrixXd history, T new_data);
    
};



#endif