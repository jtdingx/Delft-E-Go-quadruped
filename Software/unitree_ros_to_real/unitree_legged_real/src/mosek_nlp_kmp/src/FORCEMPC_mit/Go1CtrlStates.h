//
// Created by shuoy on 10/18/21.
//

#ifndef Go1_CPP_Go1CTRLSTATES_H
#define Go1_CPP_Go1CTRLSTATES_H

#include <Eigen/Dense>

#include "Go1Params.h"

class Go1CtrlStates {
public:
    // this init function sets all variables to that used in orbit.issac.Go1 controller
    Go1CtrlStates(){reset();};

    void reset() {
        stance_leg_control_type = 1;
        use_terrain_adapt = 1;
        movement_mode = 1;
        counter_per_gait = 120 * 2;
        counter_per_swing = 120;
        counter = 0;
        gait_counter.setZero();
        gait_counter_speed.setZero();
        gait_type = 1;
        gait_type_last = 1;
        // init gait counter
        // TODO: add other gait patterns?
        gait_counter_reset();

        root_pos_d.setZero();
        root_euler_d.setZero();
        root_lin_vel_d.setZero();
        root_ang_vel_d.setZero();

        robot_mass = 13;
        Go1_trunk_inertia << 0.0168352186, 0.0, 0.0,
                0.0, 0.0656071082, 0.0,
                0.0, 0.0, 0.0742720659;
        // this initialization is matlab style
        default_foot_pos << 0.17, 0.17, -0.17, -0.17,
                0.15, -0.15, 0.15, -0.15,
                -0.35, -0.35, -0.35, -0.35;

        q_weights.resize(13);
        r_weights.resize(12);


        //// too much body inclination
        // q_weights << 10.0, 10.0, 1000.0,
        //         10.0, 10.0, 20000.0,
        //         100.0, 100.0, 2000.0,
        //         10000.0, 10000.0, 10000.0,
        //         0.0; ///// theta_r, theta_p, theta_y, x, y, z,omega_r, omega_p, omega_y, vel_x, vel_y, vel_z, gravity;
        
        // //// tstep = 0.3
        // q_weights << 100.0, 100.0, 1000.0,
        //         100.0, 100.0, 1000.0,
        //         1000.0, 1000.0, 5000.0,
        //         100.0, 100.0, 1000.0,
        //         0.0; ///// theta_r, theta_p, theta_y, x, y, z,omega_r, omega_p, omega_y, vel_x, vel_y, vel_z, gravity;

        //// tstep = 0.25
        q_weights << 1000.0, 1000.0, 20000.0,
                100.0, 100.0, 2000.0,
                100.0, 100.0, 2000.0,
                1000.0, 1000.0, 2000.0,
                0.0; ///// theta_r, theta_p, theta_y, x, y, z,omega_r, omega_p, omega_y, vel_x, vel_y, vel_z, gravity;


        r_weights << 5e-5, 5e-5, 1e-5,
                5e-5, 5e-5, 1e-5,
                5e-5, 5e-5, 1e-5,
                5e-5, 5e-5, 1e-5;
        
        r_weights *=5000;

        root_pos.setZero();
        root_quat.setIdentity();
        root_euler.setZero();
        root_rot_mat.setIdentity();
        root_rot_mat_z.setZero();
        root_lin_vel.setZero();
        root_ang_vel.setZero();
        root_acc.setZero();

        foot_force.setZero();

        joint_pos.setZero();
        joint_vel.setZero();

        walking_surface_height_tmp = 0;
        walking_surface_height = 0;
        walking_surface_fit_count = 0;

        foot_pos_target_world.setZero();
        foot_pos_target_abs.setZero();
        foot_pos_target_rel.setZero();
        foot_pos_start.setZero();
        foot_pos_world.setZero();
        foot_pos_abs.setZero();
        foot_pos_rel.setZero();
        foot_pos_abs_mpc.setZero();
        foot_pos_rel_last_time.setZero();
        foot_pos_target_last_time.setZero();
        foot_pos_cur.setZero();
        foot_pos_recent_contact.setZero();
        foot_vel_world.setZero();
        foot_vel_abs.setZero();
        foot_vel_rel.setZero();
        j_foot.setIdentity();

        for (int i = 0; i < NUM_LEG; ++i) {
            contacts[i] = false;
            plan_contacts[i] = false;
            early_contacts[i] = false;
        }

        gait_counter_speed << 2, 2, 2, 2;

        double kp_foot_x = 300.0;
        double kp_foot_y = 400.0;
        double kp_foot_z = 400.0;

        double kd_foot_x = 8.0;
        double kd_foot_y = 8.0;
        double kd_foot_z = 8.0;

        kp_foot <<
                kp_foot_x, kp_foot_x, kp_foot_x, kp_foot_x,
                kp_foot_y, kp_foot_y, kp_foot_y, kp_foot_y,
                kp_foot_z, kp_foot_z, kp_foot_z, kp_foot_z;
        kd_foot <<
                kd_foot_x, kd_foot_x, kd_foot_x, kd_foot_x,
                kd_foot_y, kd_foot_y, kd_foot_y, kd_foot_y,
                kd_foot_z, kd_foot_z, kd_foot_z, kd_foot_z;

        km_foot = Eigen::Vector3d(0.1, 0.1, 0.1);

        kp_linear = Eigen::Vector3d(1000.0, 1000.0, 1000.0);
        kd_linear = Eigen::Vector3d(200.0, 70.0, 120.0);
        kp_angular = Eigen::Vector3d(650.0, 35.0, 1.0);
        kd_angular = Eigen::Vector3d(4.5, 4.5, 30.0);

        torques_gravity << 0.80, 0, 0, -0.80, 0, 0, 0.80, 0, 0, -0.80, 0, 0;
        joint_torques.setZero();

        power_level = 5;
    }



    void gait_counter_reset() {
        if (gait_type == 1) {
            gait_counter << 0, 120, 120, 0;
        }
    }

    // variables
    int stance_leg_control_type; // 0: QP, 1: MPC
    int movement_mode;  // 0: standstill, 1: start to locomote
    int use_terrain_adapt; 
    double control_dt = MAIN_UPDATE_FREQUENCY / 1000.0;

    // period of one gait cycle
    double plan_dt;
    int counter_per_plan;
    double counter_per_gait;
    double counter_per_swing;
    int counter;
    Eigen::Vector4d gait_counter;
    Eigen::Vector4d gait_counter_speed;

    int gait_type;
    int gait_type_last;

    // control target
    Eigen::Vector3d root_pos_d;
    Eigen::Vector3d root_euler_d;
    Eigen::Vector3d root_lin_vel_d;
    Eigen::Vector3d root_lin_vel_d_world;
    Eigen::Vector3d root_ang_vel_d;
    Eigen::Vector3d root_ang_vel_d_world;

    Eigen::Matrix<double, MPC_STATE_DIM, 1> mpc_states;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> mpc_states_d;

    // important kinematics constants
    double robot_mass;

    Eigen::Matrix3d Go1_trunk_inertia;

    Eigen::Matrix<double, 3, NUM_LEG> default_foot_pos;

    // MPC parameters
    Eigen::VectorXd q_weights;
    Eigen::VectorXd r_weights;

    // terrain related
    double terrain_pitch_angle;  // the estimated terrain angle on pitch direction

    // important kinematics variables
    Eigen::Vector3d root_pos;
    Eigen::Quaterniond root_quat;
    Eigen::Vector3d root_euler;
    Eigen::Matrix3d root_rot_mat;
    Eigen::Matrix3d root_rot_mat_z;
    Eigen::Vector3d root_lin_vel;
    Eigen::Vector3d root_ang_vel;
    Eigen::Vector3d root_acc;

    Eigen::Vector4d foot_force;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
    Eigen::Vector4d isaac_contact_flag;

    Eigen::Matrix<double, NUM_DOF, 1> joint_pos;
    Eigen::Matrix<double, NUM_DOF, 1> joint_vel;

    double walking_surface_height_tmp;
    double walking_surface_height;
    int walking_surface_fit_count;

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_world; // in the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_abs; // in a frame which centered at the robot frame's origin but parallels to the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_rel; // in the robot frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_start;

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_world; // in the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs; // in a frame which centered at the robot frame's origin but parallels to the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel; // in the robot frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs_mpc;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel_last_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_last_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_recent_contact;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_world;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_abs;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_rel;
    Eigen::Matrix<double, 12, 12> j_foot;

    bool contacts[NUM_LEG];         // flag to decide leg in the stance/swing mode
    bool plan_contacts[NUM_LEG];    // planed flag for stance/swing mode
    bool early_contacts[NUM_LEG];   // true if foot hit objects during swing

    // controller variables
    double kp_lin_x;
    double kd_lin_x;
    double kf_lin_x;
    double kp_lin_y;
    double kd_lin_y;
    double kf_lin_y;

    Eigen::Matrix<double, NUM_DOF_PER_LEG, NUM_LEG> kp_foot;
    Eigen::Matrix<double, NUM_DOF_PER_LEG, NUM_LEG> kd_foot;
    Eigen::Matrix<double, NUM_DOF_PER_LEG, 1> km_foot;

    double kp_linear_lock_x, kp_linear_lock_y;
    Eigen::Vector3d kp_linear;
    Eigen::Vector3d kd_linear;
    Eigen::Vector3d kp_angular;
    Eigen::Vector3d kd_angular;

    Eigen::Matrix<double, NUM_DOF, 1> torques_gravity;
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;

    // IMU sensor data
    Eigen::Vector3d imu_acc;
    Eigen::Vector3d imu_ang_vel;

    // state estimation
    Eigen::Matrix<double, 4,1> estimated_contacts;;  // true if the estimator thinks the foot has contact
    Eigen::Vector3d estimated_root_pos;
    Eigen::Vector3d estimated_root_vel;

    // hardware
    int power_level;
};

#endif //Go1_CPP_Go1CTRLSTATES_H
