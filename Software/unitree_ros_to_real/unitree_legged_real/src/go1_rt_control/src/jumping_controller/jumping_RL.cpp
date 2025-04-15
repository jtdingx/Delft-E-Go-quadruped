#include <string>
#include "jumping_RL.h"
#include "geometry_msgs/Twist.h"
#include "go1_const.h"
#include "Robotpara/robot_const_para_config.h"
#include "sensor_msgs/JointState.h"
#include <chrono>
#include <stdio.h>
#include <sys/ioctl.h> // For FIONREAD
#include <termios.h>
#include <stdbool.h>
#include <unistd.h>
// Keyboard input check.
int kbhit(void) {
    static bool initflag = false;
    static const int STDIN = 0;

    if (!initflag) {
        // Use termios to turn off line buffering
        struct termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initflag = true;
    }

    int nbbytes;
    ioctl(STDIN, FIONREAD, &nbbytes);  // 0 is STDIN
    return nbbytes;
}

// Clip a number between a lower and upper bound.
void clip(double& n, double lower, double upper) {
  n = std::max(lower, std::min(n, upper));
}


// Interpolate between two joint angles with a given rate.
double jointLinearInterpolation(double initPos, double targetPos, double rate, int j)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

Eigen::MatrixXd wrap_to_pi(Eigen::MatrixXd angles){
    for (int i=0; i<3; i++){
        double angle = angles(i,0);
        angle = fmod(angle,2*pi);
        if (angle > pi){
            angle -= 2*pi;
        }
        angles(i,0) = angle;
    }
    return angles;
}

Eigen::MatrixXd AngleDifference( Eigen::MatrixXd angles1, Eigen::MatrixXd angles2){
    
    Eigen::MatrixXd diffs(3,1);

    for (int i=0; i<3; i++){
        double angle1 = angles1(i,0);
        double angle2 = angles2(i,0);

        double diff = fmod(( angle2 - angle1 + pi ) , (2*pi)) - pi;
        diff = diff < -pi ? diff + (2*pi) : diff;
        diffs(i,0) = diff;
    }
    return diffs;
}
// Multiply two quaternions.
Eigen::MatrixXd quat_mul(Eigen::MatrixXd q1, Eigen::MatrixXd q2)
{
    
    float ww = (q1(2,0) + q1(0,0)) * (q2(0,0) + q2(1,0));
    float yy = (q1(3,0) - q1(1,0)) * (q2(3,0) + q2(2,0));
    float zz = (q1(3,0) + q1(1,0)) * (q2(3,0) - q2(2,0));
    float xx = ww + yy + zz;
    float qq = 0.5 * (xx + (q1(2,0) - q1(0,0)) * (q2(0,0) - q2(1,0)));
    float w = qq - ww + (q1(2,0) - q1(1,0)) * (q2(1,0) - q2(2,0));
    float x = qq - xx + (q1(0,0) + q1(3,0)) * (q2(0,0) + q2(3,0));
    float y = qq - yy + (q1(3,0) - q1(0,0)) * (q2(1,0) + q2(2,0));
    float z = qq - zz + (q1(2,0) + q1(1,0)) * (q2(3,0) - q2(0,0));

    Eigen::MatrixXd q(4,1);
    q << x, y, z, w;

    return q;
}

// Conjugate of a quaternion.
Eigen::MatrixXd quat_conjugate(Eigen::MatrixXd q)
{
    Eigen::MatrixXd q_conj(4,1);
    q_conj << -q(0,0), -q(1,0), -q(2,0), q(3,0);

    return q_conj;
}

// Logarithmic map of a quaternion.
Eigen::MatrixXd quat_log(Eigen::MatrixXd q)
{
    double v_norm = q.block<3,1>(0,0).norm();
    double q_norm = q.norm();
    double tolerance = 1e-17;
    Eigen::MatrixXd q_dist(4,1);

    if (q_norm < tolerance)
    {
        std::cout << "ERROR: Quaternion is undefined" << std::endl;
        q_dist.setZero();
        return q_dist;
    }

    if (v_norm < tolerance)
    {
        // std::cout << "Quaternion is zero" << std::endl;
        q_dist.setZero();
        q_dist(3,0) = 1;

        return q_dist;
    }

    Eigen::VectorXd v = q.block<3,1>(0,0) / v_norm;
    Eigen::VectorXd vector = v * std::acos(q(3,0) / q_norm); 
    q_dist << vector, log(q_norm);

    return q_dist;

}

// Distance between two quaternions.
Eigen::MatrixXd quat_distance(Eigen::MatrixXd q1, Eigen::MatrixXd q2)
{
    Eigen::MatrixXd q1_conj(4,1);

    q1_conj = quat_conjugate(q1);

    return quat_log(quat_mul(q1_conj, q2));

}

Quadruped::Quadruped(uint8_t level, double step_size, double policy_step_size): safe(LeggedType::Go1), udp(level){//, 8090, "192.168.123.10", 8007){
    udp.InitCmdData(cmd);
    // Initialise and set some parameters and variables:

    // --- Setup data publisher:
    data_pub = n.advertise<sensor_msgs::JointState>("go1_gait_data",10);

    data2pub.position.resize(200);

    // Filters initialisation
    fcutoff_vel = 10; // Cutoff frequency for velocity
    fcutoff_tor = 50; // Cutoff frequency for torque
    fcutoff_acc = 50;
    dt = step_size;
    est_dt = dt;
    sample_freq = std::ceil(1.0/dt); // 1kHz
    std::cout << "Sample freq: " << sample_freq << std::endl;

    policy_dt = policy_step_size;
    decimation = std::ceil(policy_dt/dt);
    std::cout << "Policy freq: " << 1.0/policy_dt << std::endl;
    std::cout << "Decimation: " << decimation << std::endl;

    for (int i=0;i<3;i++){
        butterworthBaseVel[i].init(sample_freq,fcutoff_vel);
    }

    for (int i=0;i<3;i++){
        butterworthBaseAngVel[i].init(sample_freq,fcutoff_vel);
    }


    for (int i=0;i<12;i++){
        butterworthTorque[i].init(sample_freq,fcutoff_tor);
    }

    for (int i=0;i<3;i++){
        butterworthBaseAcc[i].init(sample_freq,fcutoff_acc);
    }

    
    base_pos.setZero();         // Real position of the base/body of the robot.
    base_quat.w() = 1;          // Real orientation of the base/body of the robot (as a quaternion).
    R_base = base_quat.normalized().toRotationMatrix();
    q_mea.setZero();                // Real joint angle of the robot joints.
    dq_mea.setZero();               // Real joint velocity of the robot joints.
    qDes.setZero();
    base_rpy_offset.setZero();
    base_pos_landing.setZero();
    base_pos_standing.setZero();

    base_vel.setZero();
    base_ang_vel.setZero();

    grav_vector << 0.0,0.0,-1.0;
    grav = grav_vector * 9.81;

    base_acc.setZero();
    base_acc_local.setZero();
    base_acc_prev.setZero();
    base_vel_imu.setZero();
    base_pos_imu.setZero();



    // Set nominal angles:
    q0 << DEFAULT_HIP_ANGLE,DEFAULT_THIGH_ANGLE,DEFAULT_CALF_ANGLE,
          DEFAULT_HIP_ANGLE,DEFAULT_THIGH_ANGLE,DEFAULT_CALF_ANGLE,
          DEFAULT_HIP_ANGLE,DEFAULT_THIGH_ANGLE,DEFAULT_CALF_ANGLE,
          DEFAULT_HIP_ANGLE,DEFAULT_THIGH_ANGLE,DEFAULT_CALF_ANGLE;

    // The stand up pose is the same as the nominal pose:
    qHomingPose = q0;

    qInit.setZero();

    // Joint angle limits:
    q_lower_limits << -0.863,-0.686,-2.818,
                    -0.863,-0.686,-2.818,
                    -0.863,-0.686,-2.818,
                    -0.863,-0.686,-2.818;
    
    q_upper_limits << 0.863,4.501,-0.888,
                    0.863,4.501,-0.888,
                    0.863,4.501,-0.888,
                    0.863,4.501,-0.888;

    error_quat.resize(4,1);
    error_quat << 0.0,0.0,0.0,1.0;

    base_quat_arr.resize(4,1);
    base_quat_arr << 0.0,0.0,0.0,1.0;

    // Resize the state histories accordingly and zero them:
    base_lin_vel_history.resize(hist_len,3);
    base_lin_vel_history.setZero();
    base_ang_vel_history.resize(hist_len,3);
    base_ang_vel_history.setZero();
    dof_pos_history.resize(hist_len,12);
    error_quat_history.resize(hist_len,4);
    base_quat_history.resize(hist_len,4);

    for (int i=0; i<hist_len; i++){
        dof_pos_history.block(i,0,1,12) = q0.transpose();
        error_quat_history.block(i,0,1,4) = error_quat.transpose();
        base_quat_history.block(i,0,1,4) = base_quat_arr.transpose();
    }
    dof_pos_history.setZero();

    dof_vel_history.resize(hist_len,12);
    dof_vel_history.setZero();
    actions_history.resize(hist_len,12);
    actions_history.setZero();
    contact_history.resize(hist_len,4);
    contact_history.setOnes();

    resetStateHistory();

    projected_gravity.resize(3);
    projected_gravity = grav_vector;

    commands.resize(13);
    commands << 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // commands <<  0.5, -0.30,  0.0,  0.0,  0.0, -0.2669,  0.9637,  0.0, 0.0,  0.0,  0.0,  0.0,  0.0;
    actions.setZero();
    prevActions.setZero();
    actions_unfiltered_store.setZero();
    actions_filtered_store.setZero();
    qDes_policy.setZero();
    qDes_policy = q0;
    Torque_ff_GRF_data.setZero();


    obs_scales.lin_vel = 2.0;
    obs_scales.ang_vel = 0.25;
    obs_scales.dof_pos = 1.0;
    obs_scales.dof_vel = 0.05;
    obs_scales.error_quat = 1.0;
    

    // Load the policy:
    std::string policy_name="default";
    policy_name = "policy_Jul03_11-09-51_.pt";

    ROS_INFO("Loading policy: %s", policy_name.c_str());
    loadPolicy(policy_name);

    // decimation = int((1./50.) / dt);


}

// Receive a message from the robot through UDP.
void Quadruped::UDPRecv()
{  
    udp.Recv();
}

// Send a command to the robot through UDP.
void Quadruped::UDPSend()
{  
    udp.Send();
}

void Quadruped::gait_pose_callback(LowState RecvLowROS,double dt, int j)
{
    /// FL, FR, RL, RR;
    Go1_ctrl_states.joint_pos[0] = RecvLowROS.motorState[3].q;
    Go1_ctrl_states.joint_vel[0] = RecvLowROS.motorState[3].dq;
    Go1_ctrl_states.joint_pos[1] = RecvLowROS.motorState[4].q;
    Go1_ctrl_states.joint_vel[1] = RecvLowROS.motorState[4].dq;
    Go1_ctrl_states.joint_pos[2] = RecvLowROS.motorState[5].q;
    Go1_ctrl_states.joint_vel[2] = RecvLowROS.motorState[5].dq;

    Go1_ctrl_states.joint_pos[3] = RecvLowROS.motorState[0].q;
    Go1_ctrl_states.joint_vel[3] = RecvLowROS.motorState[0].dq;
    Go1_ctrl_states.joint_pos[4] = RecvLowROS.motorState[1].q;
    Go1_ctrl_states.joint_vel[4] = RecvLowROS.motorState[1].dq;
    Go1_ctrl_states.joint_pos[5] = RecvLowROS.motorState[2].q;
    Go1_ctrl_states.joint_vel[5] = RecvLowROS.motorState[2].dq;

    Go1_ctrl_states.joint_pos[6] = RecvLowROS.motorState[9].q;
    Go1_ctrl_states.joint_vel[6] = RecvLowROS.motorState[9].dq;   
    Go1_ctrl_states.joint_pos[7] = RecvLowROS.motorState[10].q;
    Go1_ctrl_states.joint_vel[7] = RecvLowROS.motorState[10].dq;
    Go1_ctrl_states.joint_pos[8] = RecvLowROS.motorState[11].q;
    Go1_ctrl_states.joint_vel[8] = RecvLowROS.motorState[11].dq;

    Go1_ctrl_states.joint_pos[9] = RecvLowROS.motorState[6].q;
    Go1_ctrl_states.joint_vel[9] = RecvLowROS.motorState[6].dq;
    Go1_ctrl_states.joint_pos[10] = RecvLowROS.motorState[7].q;
    Go1_ctrl_states.joint_vel[10] = RecvLowROS.motorState[7].dq;
    Go1_ctrl_states.joint_pos[11] = RecvLowROS.motorState[8].q;
    Go1_ctrl_states.joint_vel[11] = RecvLowROS.motorState[8].dq;

    // Go1_ctrl_states.foot_force[0] = (footforce_fl.mean()); 
    // Go1_ctrl_states.foot_force[1] = (footforce_fr.mean()); 
    // Go1_ctrl_states.foot_force[2] = (footforce_rl.mean()); 
    // Go1_ctrl_states.foot_force[3] = (footforce_rr.mean()); 

    Go1_ctr_states.foot_force[0] = F_contact_force(1,0);
    Go1_ctr_states.foot_force[1] = F_contact_force(0,0);
    Go1_ctr_states.foot_force[2] = F_contact_force(3,0);
    Go1_ctr_states.foot_force[3] = F_contact_force(2,0);

    // Go1_ctrl_states.foot_force[0] = (footforce_fl(4,0)); 
    // Go1_ctrl_states.foot_force[1] = (footforce_fr(4,0)); 
    // Go1_ctrl_states.foot_force[2] = (footforce_rl(4,0)); 
    // Go1_ctrl_states.foot_force[3] = (footforce_rr(4,0));   

    Go1_ctrl_states.imu_acc = Eigen::Vector3d(
            acc_x.CalculateAverage(RecvLowROS.imu.accelerometer[0]),
            acc_y.CalculateAverage(RecvLowROS.imu.accelerometer[1]),
            acc_z.CalculateAverage(RecvLowROS.imu.accelerometer[2])
    );
    Go1_ctrl_states.imu_ang_vel = Eigen::Vector3d(
            gyro_x.CalculateAverage(RecvLowROS.imu.gyroscope[0]),
            gyro_y.CalculateAverage(RecvLowROS.imu.gyroscope[1]),
            gyro_z.CalculateAverage(RecvLowROS.imu.gyroscope[2])
    );            
    rot_imu_acc = R_base * Go1_ctrl_states.imu_acc;

    /////// mean value of the imu_linear acc, angular velocity;//// detect the IMU bias
    if(gait_status=="STAND_INIT_STATUS") /////// before calibration /// initialize the EKF estimator
    {
           Go1_ctrl_states.estimated_root_pos[0] =  base_pos(0,0);
           Go1_ctrl_states.estimated_root_pos[1] =   base_pos(1,0);
           Go1_ctrl_states.estimated_root_pos[2] =   base_pos(2,0);
           Go1_ctrl_states.estimated_root_vel[0] =  base_vel(0,0);
           Go1_ctrl_states.estimated_root_vel[1] =  base_vel(1,0);
           Go1_ctrl_states.estimated_root_vel[2] =  base_vel(2,0);           
    }
    else{
        if(gait_status == "STANDING" && rate_count_jump == 0) //// in the calibration process;
        {
            accelerometer_bias[0] += rot_imu_acc[0];
            accelerometer_bias[1] += rot_imu_acc[1];
            accelerometer_bias[2] += rot_imu_acc[2];            

            n_count_calibration += 1;  
        }
        else
        {
            if(rate_count_jump == 1)
            {
                accelerometer_ave(0) = -accelerometer_bias[0] / n_count_calibration;
                accelerometer_ave(1) = -accelerometer_bias[1] / n_count_calibration;
                accelerometer_ave(2) = -accelerometer_bias[2] / n_count_calibration;


            }

        }
    }



    // calculate several useful variables
    // euler should be roll pitch yaw
    Go1_ctrl_states.root_quat = base_quat;
    ///// built in function is not right
    // Go1_ctrl_states.root_rot_mat = Go1_ctrl_states.root_quat.normalized().toRotationMatrix();
    // Go1_ctrl_states.root_euler = Utils::quat_to_euler(Go1_ctrl_states.root_quat);
    Go1_ctrl_states.root_rot_mat = R_base;
    Go1_ctrl_states.root_euler = base_rpy;

    double yaw_angle = Go1_ctrl_states.root_euler[2];

    Go1_ctrl_states.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

    Go1_ctrl_states.root_ang_vel = Go1_ctrl_states.root_rot_mat * Go1_ctrl_states.imu_ang_vel;


    //////*****************************************main loop **************************************************////
    // TODO: we call estimator update here, be careful the runtime should smaller than the HARDWARE_UPDATE_FREQUENCY,i.e., 2ms
    // state estimation
    int simulation_mode = 2;
    if (rate_count_jump > 0){
        go1_estimate.update_estimation(Go1_ctrl_states, dt,simulation_mode, accelerometer_ave);
    }
    else if (!go1_estimate.is_inited() || rate_count_jump == 0) {
        go1_estimate.init_state(Go1_ctrl_states); ///// 
    }



    // FL, FR, RL, RR
    // use estimation base pos and vel to get foot pos and foot vel in world frame
    for (int i = 0; i < NUM_LEG; ++i) {
        Go1_ctrl_states.foot_pos_rel.block<3, 1>(0, i) = go1_kin.fk(
                Go1_ctrl_states.joint_pos.segment<3>(3 * i),
                rho_opt_list[i], rho_fix_list[i]);
        Go1_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i) = go1_kin.jac(
                Go1_ctrl_states.joint_pos.segment<3>(3 * i),
                rho_opt_list[i], rho_fix_list[i]);
        Eigen::Matrix3d tmp_mtx = Go1_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i);
        Eigen::Vector3d tmp_vec = Go1_ctrl_states.joint_vel.segment<3>(3 * i);
        Go1_ctrl_states.foot_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

        Go1_ctrl_states.foot_pos_abs.block<3, 1>(0, i) =
                Go1_ctrl_states.root_rot_mat * Go1_ctrl_states.foot_pos_rel.block<3, 1>(0, i);
        Go1_ctrl_states.foot_vel_abs.block<3, 1>(0, i) =
                Go1_ctrl_states.root_rot_mat * Go1_ctrl_states.foot_vel_rel.block<3, 1>(0, i);

        // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
        // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
        // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
        Go1_ctrl_states.foot_pos_world.block<3, 1>(0, i) =
                Go1_ctrl_states.foot_pos_abs.block<3, 1>(0, i) + Go1_ctrl_states.root_pos;
        Go1_ctrl_states.foot_vel_world.block<3, 1>(0, i) =
                Go1_ctrl_states.foot_vel_abs.block<3, 1>(0, i) + Go1_ctrl_states.root_lin_vel;
    }


}



// Update the internal stored states of the robot through given LowState message.
void Quadruped::updateStates(LowState RecvLowROS){


    base_pos_prev = base_pos;
    base_rpy_prev = base_rpy;

    for (int j = 0; j < 4; j++) {
        F_contact_force(j,0) = RecvLowROS.footForce[j];
    }

    // Detect contact (if the force is above a threshold, we consider the foot to be in contact):
    double contact_detection_threshold = 180.0; // Choose a threshold value for detecting contact - we used around 160 in the tests.

    for (int foot=0; foot < 4; foot++){
        if (F_contact_force(foot,0) <= contact_detection_threshold){
            contact(foot,0) = 0;
        }
        else{
            contact(foot,0) = 1;
        }
    }
    // -----------------

    // Get the joint positions, velocities and torques:
    for (int i=0;i<12;i++){
        q_mea(i,0) = RecvLowROS.motorState[i].q;
        dq_mea(i,0) = RecvLowROS.motorState[i].dq;
        tau_mea(i,0) =  RecvLowROS.motorState[i].tauEst;

    }

    // Get the base orientation:
    base_quat = Eigen::Quaterniond(RecvLowROS.imu.quaternion[0],
                                    RecvLowROS.imu.quaternion[1],
                                    RecvLowROS.imu.quaternion[2],
                                    RecvLowROS.imu.quaternion[3]); 

    base_quat = base_quat.normalized();

    Eigen::Matrix<double,3,1> base_pos_0;
    base_pos_0.setZero();

    Eigen::Matrix<double,3,1> base_rpy_0;
    base_rpy_0.setZero();


    // Subtract the offset from the base orientation (to zero the initial yaw):
    base_rpy = Utils::quat_to_euler(base_quat) - base_rpy_offset;
 
    // base_rpy = wrap_to_pi(base_rpy);

    Eigen::Quaterniond quat_offset;
    quat_offset =  Eigen::AngleAxisd(base_rpy(0,0), Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(base_rpy(1,0), Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(base_rpy(2,0), Eigen::Vector3d::UnitZ());

    R_base = quat_offset.toRotationMatrix();
    base_quat = quat_offset; //Set base_quat to the offset one.
    // std::cout << "R_Base: " << R_base << std::endl;
    // Add IMU acceleration:
    double alpha = 1;
    for (int i=0;i<3;i++){
        // base_acc_local(i,0) = (1-alpha)*base_acc_local(i,0) + alpha*RecvLowROS.imu.accelerometer[i];//butterworthBase_acc_vel[i].filter(RecvLowROS.imu.accelerometer[i]);
        base_acc_local(i,0) = butterworthBaseAcc[i].filter(RecvLowROS.imu.accelerometer[i]);
    }
    base_acc = R_base * base_acc_local + grav;


    leg_offset_x[0] = 0.1881;
    leg_offset_x[1] = 0.1881;
    leg_offset_x[2] = -0.1881;
    leg_offset_x[3] = -0.1881;
    leg_offset_y[0] = 0.04675;
    leg_offset_y[1] = -0.04675;
    leg_offset_y[2] = 0.04675;
    leg_offset_y[3] = -0.04675;
    motor_offset[0] = 0.08;
    motor_offset[1] = -0.08;
    motor_offset[2] = 0.08;
    motor_offset[3] = -0.08;
    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = 0.213;
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = LOWER_LEG_LENGTH;


    for (int i = 0; i < NUM_LEG; i++) {
        Eigen::VectorXd rho_fix(5);
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
        Eigen::VectorXd rho_opt(3);
        rho_opt << 0.0, 0.0, 0.0;
        rho_fix_list.push_back(rho_fix);
        rho_opt_list.push_back(rho_opt);
    }

    
    Eigen::Matrix<double,4,3> foot_poses;

    // First get the foot poses relative to the (0,0,0) frame
    foot_poses.block<1,3>(0,0) = Kine.Forward_kinematics_g(base_pos_0, base_rpy_0,q_mea.block<3,1>(0,0), 0);
    foot_poses.block<1,3>(1,0)  = Kine.Forward_kinematics_g(base_pos_0, base_rpy_0,q_mea.block<3,1>(3,0), 1);
    foot_poses.block<1,3>(2,0)  = Kine.Forward_kinematics_g(base_pos_0, base_rpy_0,q_mea.block<3,1>(6,0), 2);
    foot_poses.block<1,3>(3,0)  = Kine.Forward_kinematics_g(base_pos_0, base_rpy_0,q_mea.block<3,1>(9,0), 3);

    // Then find the base_pos relative to the (0,0,0) frame
    base_pos(0,0) = (- foot_poses(0,0) - foot_poses(1,0) - foot_poses(2,0) - foot_poses(3,0) )/4.0;
    base_pos(1,0) = (- foot_poses(0,1) - foot_poses(1,1) - foot_poses(2,1) - foot_poses(3,1) )/4.0;
    base_pos(2,0) = (- foot_poses(0,2) - foot_poses(1,2) - foot_poses(2,2) - foot_poses(3,2) )/4.0;


    // Compute base lin and ang velocities:
    // base_vel(0,0) = butterworthLPF1.filter((base_pos(0,0) - base_pos_prev(0,0)) / dt);
    // base_vel(1,0) = butterworthLPF2.filter((base_pos(1,0) - base_pos_prev(1,0)) / dt);
    // base_vel(2,0) = butterworthLPF3.filter((base_pos(2,0) - base_pos_prev(2,0)) / dt);

    for (int i=0;i<3;i++){
        base_vel(i,0) = butterworthBaseVel[i].filter((base_pos(i,0) - base_pos_prev(i,0)) / dt);
    }

    // base_ang_vel(0,0) = butterworthLPF4.filter((base_rpy(0) - base_rpy_prev(0)) / dt);
    // base_ang_vel(1,0) = butterworthLPF5.filter((base_rpy(1) - base_rpy_prev(1)) / dt);
    // base_ang_vel(2,0) = butterworthLPF6.filter((base_rpy(2) - base_rpy_prev(2)) / dt);
    // Eigen::MatrixXd base_ang_vel_raw = AngleDifference(base_rpy, base_rpy_prev);
    // for (int i=0;i<3;i++){
    //     base_ang_vel(i,0) = butterworthBaseAngVel[i].filter(base_ang_vel_raw(i,0) / dt);
    // }
    for (int i=0;i<3;i++){
        base_ang_vel(i,0) = butterworthBaseAngVel[i].filter((base_rpy(i) - base_rpy_prev(i)) / dt);
    }

    for (int i=0;i<3;i++){
        clip(base_ang_vel(i,0),-3.0,3.0);
    }
    
    for (int i=0;i<3;i++){
        if (landed){
            base_vel_imu(i,0) = base_vel(i,0);
        } 
        else if (rate_count_jump==0 || reset_counter_cont_jump){
            base_vel_imu(i,0) = 0.0;
        }
        else {
            base_vel_imu(i,0) = base_vel_imu(i,0) + (base_acc(i,0))*est_dt;
        }
    }   

    base_vel_imu_local = R_base.inverse()*base_vel_imu;

    if (!flight_started){
        base_pos_imu = base_pos;
    }
    else {
        for (int j=0;j<3;j++){
            base_pos_imu(j,0) = base_pos_imu(j,0) + base_vel_imu(j,0)*est_dt; //+ 0.5*(base_acc(j,0))*pow(est_dt,2);
        }    
    }

    
    // Compute the error quaternion:
    Eigen::MatrixXd q1(4,1);
    Eigen::MatrixXd q2(4,1);
    // Convert base quat into Eigen matrix:
    q1 << base_quat.x(), base_quat.y(), base_quat.z(), base_quat.w();
    // Take the desired quaternion:
    q2 << commands(3), commands(4), commands(5), commands(6);
    // error_quat = quat_distance(q1,q2);
    error_quat << 0.0,0.0,0.0,1.0; // OVERWRITING FOR NOW;

    base_quat_arr << base_quat.x(), base_quat.y(), base_quat.z(), base_quat.w();

    // Compute the projected gravity vector (i.e. gravity vector in base frame):
    projected_gravity << R_base.inverse()*grav_vector;
};

// Shifts the history by one row to the bottom and adds the new data to the top row.
template <typename T>
Eigen::MatrixXd Quadruped::shiftHistory(Eigen::MatrixXd history, T new_data){

    int hist_len = history.rows();
    // Create a copy of the history which to shift (Eigen has weird behaviour with in-place shifting)
    Eigen::MatrixXd history_updated = history;

    history_updated.bottomRows(hist_len-1) = history.topRows(hist_len-1);
    
    // Make sure the dimensions match:
    if (history.topRows(1).size() != new_data.cols()){
        new_data.transposeInPlace();
    }

    history_updated.topRows(1) << new_data;
    return history_updated;
}

// Reset the state histories to zero.
void Quadruped::resetStateHistory(){
    dof_pos_history.setZero();
    dof_vel_history.setZero();

    error_quat_history.setZero();
    base_quat_history.setZero();

    actions_history.setZero();
    contact_history.setZero();

    base_lin_vel_history.setZero();
    base_ang_vel_history.setZero();
}
// Compute the state histories using the current state and shifting the previous state histories.
void Quadruped::computeStateHistory(){

    // Compute base vel in local frame:

    base_lin_vel_history = shiftHistory<Eigen::MatrixXd>(base_lin_vel_history, base_vel_imu_local);
    // Ang vel is already in local frame (comes from IMU)
    base_ang_vel_history = shiftHistory<Eigen::MatrixXd>(base_ang_vel_history, base_ang_vel);

    Eigen::MatrixXd q_mea_ordered = q_mea;
    Eigen::MatrixXd dq_mea_ordered = dq_mea;
    Eigen::MatrixXd contact_ordered = contact;

    // Fix the order of the joint angles/velocities if needed:
    if (switch_order_legs){
        q_mea_ordered << q_mea.block<3,1>(3,0), q_mea.block<3,1>(0,0), q_mea.block<3,1>(9,0), q_mea.block<3,1>(6,0);
        dq_mea_ordered << dq_mea.block<3,1>(3,0), dq_mea.block<3,1>(0,0), dq_mea.block<3,1>(9,0), dq_mea.block<3,1>(6,0);
        contact_ordered << contact(1,0), contact(0,0), contact(3,0), contact(2,0);
    }

    dof_pos_history = shiftHistory<Eigen::MatrixXd>(dof_pos_history, q_mea_ordered);
    dof_vel_history = shiftHistory<Eigen::MatrixXd>(dof_vel_history, dq_mea_ordered);
    actions_history = shiftHistory<Eigen::MatrixXd>(actions_history, actions);
    contact_history = shiftHistory<Eigen::MatrixXd>(contact_history, contact_ordered);
    error_quat_history = shiftHistory<Eigen::MatrixXd>(error_quat_history, error_quat);
    base_quat_history = shiftHistory<Eigen::MatrixXd>(base_quat_history, base_quat_arr);



}        

// Compute and return the observations buffer using the flattened state histories.
Eigen::VectorXd Quadruped::computeObservations(){
   

    Eigen::VectorXd obs_buffer;
    obs_buffer.resize(num_obs);


    // Flatten the state histories:
    Eigen::Map<const Eigen::VectorXd> base_lin_vel_flattened(base_lin_vel_history.data(), base_lin_vel_history.size());
    Eigen::Map<const Eigen::VectorXd> base_ang_vel_flattened(base_ang_vel_history.data(), base_ang_vel_history.size());
    Eigen::Map<const Eigen::VectorXd> dof_pos_flattened(dof_pos_history.data(), dof_pos_history.size());
    Eigen::Map<const Eigen::VectorXd> dof_vel_flattened(dof_vel_history.data(), dof_vel_history.size());
    Eigen::Map<const Eigen::VectorXd> actions_flattened(actions_history.data(), actions_history.size());
    Eigen::Map<const Eigen::VectorXd> contact_flattened(contact_history.data(), contact_history.size());
    Eigen::Map<const Eigen::VectorXd> error_quat_flattened(error_quat_history.data(), error_quat_history.size());
    Eigen::Map<const Eigen::VectorXd> base_quat_flattened(base_quat_history.data(), base_quat_history.size());

    Eigen::MatrixXd q0_stacked = q0.replicate(hist_len,1);
    Eigen::Map<const Eigen::VectorXd> q0_flattened(q0_stacked.data(),q0_stacked.size());


    // has_jumped_bool = 1; // OVERWRITING THIS FOR NOW!!!
    
    // std:cout << "contact_flattened" << contact_flattened << std::endl;
    // std::cout << "dof_pos_flattened" << dof_pos_flattened << std::endl;
    // std::cout << "base_quat_flattened: " << base_quat_flattened << std::endl;
    // std::cout << "actions: " << actions_flattened << std::endl;
    // Concatenate the flattened state histories: 
    obs_buffer << base_lin_vel_flattened * obs_scales.lin_vel,
                  base_ang_vel_flattened * obs_scales.ang_vel,
                //   projected_gravity,
                  (dof_pos_flattened - q0_flattened) * obs_scales.dof_pos,
                  dof_vel_flattened * obs_scales.dof_vel,
                  actions_flattened,
                //   error_quat_flattened * obs_scales.error_quat,
                  base_quat_flattened * obs_scales.error_quat,
                  commands,
                  has_jumped_bool,
                  contact_flattened;

    obs_buffer = obs_buffer.cwiseMin(obs_clip).cwiseMax(-obs_clip);

    return obs_buffer;
}

// Load the policy.
void Quadruped::loadPolicy(std::string policy_name){

    std::string policy_path = policies_path + "/" + policy_name; 
    try {
        // Deserialize the ScriptModule from a file using torch::jit::load().
        policy = torch::jit::load(policy_path);
        std::cout << "Policy loaded.\n";
    }
    catch (const c10::Error& e) {
        std::cerr << "error loading the model\n";
        throw;
    }
    
}

// Step the policy with the given observation and obtain the vector of actions.
Eigen::Matrix<double,12,1> Quadruped::policyStep(Eigen::VectorXd obs){


    // Convert Eigen double vector to torch double tensor.
    torch::Tensor input_tensor = torch::from_blob(obs.data(), {1, num_obs}, at::kDouble).clone();

    // Convert torch double tensor to torchscript float IValue
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input_tensor.to(torch::kFloat));

    // Run the policy:
    torch::Tensor output_tensor = policy.forward(inputs).toTensor();

    // convert network output to Eigen double vector. 
    Eigen::Matrix<double,12,1> actions(output_tensor.to(torch::kDouble).data_ptr<double>());

    // Store in class variable actions:
    actions = actions.cwiseMin(action_clip).cwiseMax(-action_clip);

    return actions;
}

Eigen::Matrix<double,12,1> Quadruped::changeLegOrder(Eigen::Matrix<double,12,1> vector){
    Eigen::Matrix<double,12,1> vector_reshaped;
    vector_reshaped << vector.block<3,1>(3,0), vector.block<3,1>(0,0), vector.block<3,1>(9,0), vector.block<3,1>(6,0);

    return vector_reshaped;
}

void Quadruped::publishData(){
    data2pub.header.stamp = ros::Time::now();// - start_time;

    for (int j=0;j<3;j++){
        // data2pub.name[j] = "base_pos";
        data2pub.position[j] = base_pos(j,0);
    }

    // if (landing_motion){
    //     data2pub.position[0] += base_pos_landing(0,0);
    // }
    for (int j=0;j<3;j++){
        // data2pub.name[3+j] = "base_vel";
        data2pub.position[3+j] = base_vel(j,0);
    }

    for (int j=0;j<3;j++){
        // data2pub.name[6+j] = "base_rpy";
        data2pub.position[6+j] = base_rpy(j,0);
    }

    for (int j=0;j<3;j++){
        // data2pub.name[9+j] = "base_ang_vel";
        data2pub.position[9+j] = base_ang_vel(j,0);
    }

    for (int j=0;j<12;j++){
        // data2pub.name[12+j] = "q_mea";
        data2pub.position[12+j] = q_mea(j,0);
    }

    for (int j=0;j<12;j++){
        // data2pub.name[24+j] = "dq_mea";
        data2pub.position[24+j] = dq_mea(j,0);
    }

    for (int j=0;j<4;j++){
        // data2pub.name[36+j] = "F_contact_force";
        data2pub.position[36+j] = F_contact_force(j,0);
    }

    for (int j=0;j<12;j++){
        // data2pub.name[40+j] = "Torque_ff_GRF";
        data2pub.position[40+j] = Torque_ff_GRF_data(j,0);
    }

    for (int j=0;j<12;j++){
        // data2pub.name[52+j] = "F_GRF";
        data2pub.position[67+j] = qDes(j,0);
    }

    data2pub.position[101] = flight_started;
    data2pub.position[102] = landed;

    for (int j=0;j<12;j++){
        data2pub.position[103+j] = tau_mea(j,0);
        
    }

    for (int j=0;j<3;j++){
        // std::cout << "Elapse flight time: " << flight_time << std::endl;
        // data2pub.position[128+j] = base_pos_flight(j,0) + base_vel_flight(j,0)*flight_time.toSec() + grav(j,0)*0.5*pow(flight_time.toSec(),2);
        data2pub.position[128+j] = base_pos_imu(j,0);
    }

    // data2pub.position[132] = k_hip;
    // data2pub.position[133] = k_thigh;
    // data2pub.position[134] = k_calf;

    for (int j=0;j<3;j++){
        data2pub.position[135+j] = base_vel_imu(j,0);
        data2pub.position[138+j] = base_acc(j,0);
    }

    
    if (flight_started){
        
            // std::cout << "Elapse flight time: " << flight_time << std::endl;
        data2pub.position[141] = base_pos_flight(2,0) + base_vel_flight(2,0)*flight_time.toSec() + grav(2,0)*0.5*pow(flight_time.toSec(),2);
    }

    else{
        data2pub.position[141] = base_pos(2,0);
        
    }

    data2pub.position[142] = est_dt*1e3;

    for (int j=0;j<12;j++){

        data2pub.position[143+j] = qDes_policy(j,0);
    }

    for (int j=0;j<4;j++){
        data2pub.position[155+j] = contact(j,0);
    }

     for (int j=0;j<12;j++){

        data2pub.position[159+j] = actions_unfiltered_store(j,0);
    }   

     for (int j=0;j<12;j++){

        data2pub.position[171+j] = actions_filtered_store(j,0);
    }   
    
    for (int j=0;j<3;j++){

        data2pub.position[183+j] = base_vel_imu_local(j,0);
    }

    data_pub.publish(data2pub);

}

// Linearly interpolate between initial pose and the desired homing pose (qDes is passed by reference).
void Quadruped::standUp(Eigen::Matrix<double,12,1>& qDes, int rate_count, double total_steps, const Eigen::Matrix<double,12,1> qInit, const Eigen::Matrix<double,12,1> qHomingPose){
    rate = pow(std::min(rate_count/total_steps,1.0),2); 
    for(int j=0; j<12;j++)
    {
        qDes(j,0) = jointLinearInterpolation(qInit(j,0), qHomingPose(j,0), rate, 0);
    }
}

Eigen::Matrix<double,12,1> Quadruped::unscaleActions(Eigen::Matrix<double,12,1> actions){
    Eigen::Matrix<double,12,1> scaled_actions;
    // Multiply all actions by the action scale.
    scaled_actions = actions*action_scale;
    // For hip joints apply additional reduction:
    for (int i=0; i<4; i++){
        scaled_actions(3*i,0) = scaled_actions(3*i,0)*hip_scale_reduction;
    }
    return scaled_actions;
}

void Quadruped::switchFBtoFF(int rate_count_jump, double Kp_hip_ini,double Kp_thigh_ini,double Kp_calf_ini, double Kd_ini){
            
    double Kp_hip,Kp_thigh,Kp_calf, Kd;

    Kp_hip = Kp_hip_ini - std::min(rate_count_jump/500.0, 1.0)*Kp_hip_ini;
    Kp_thigh = Kp_thigh_ini - std::min(rate_count_jump/500.0, 1.0)*Kp_thigh_ini;
    Kp_calf = Kp_calf_ini - std::min(rate_count_jump/500.0, 1.0)*Kp_calf_ini;

    Kd = Kd_ini - std::min(rate_count_jump/500.0, 1.0)*Kd_ini; 

    changePIDgains(Kp_hip,Kp_thigh,Kp_calf,Kd);


}

// Change the PID gains for the joints.
void Quadruped::changePIDgains(double Kp_hip=0.0,double Kp_thigh=0.0,double Kp_calf=0.0, double Kd=0.0){
    for(int j=0; j<4; j++)
    {
        Kp_joint[j*3] = Kp_hip;
        Kp_joint[j*3+1] = Kp_thigh;
        Kp_joint[j*3+2] = Kp_calf;

        Kd_joint[j*3] = Kd; 
        Kd_joint[j*3+1] = Kd; 
        Kd_joint[j*3+2] = Kd; 
    }

}

// Check if the robot is in flight and if it has landed.
void Quadruped::checkFlight(){
    
    if (contact.sum() <= 2 && !flight_started && rate_count_jump > 5 && base_vel(2,0) > 0.5){
        std::cout << "FLIGHT DETECTED: " << motiontime << std::endl;

        flight_started = true;
        // landed = false;
        flight_started_time = motiontime;
        base_pos_flight = base_pos;
        base_pos_flight(1,0) = 0.0;

        base_vel_flight = base_vel;
        base_vel_flight_imu = base_vel_imu;
    }

    if (flight_started && !landed){
        t1 = ros::Time::now();
        flight_time = t1 - t0;
    }
    if (contact.sum() > 2 && flight_started && !landed && (motiontime - flight_started_time) > 5){
        std::cout << "LANDING DETECTED: " << motiontime << std::endl;

        landed = true;
        has_jumped_bool = 1;        
        // flight_started = false;
    }

}

Eigen::Matrix<double,12,1> Quadruped::safetyPolicy(Eigen::Matrix<double,12,1> qDes_policy)
{
    Eigen::Matrix<double,12,1> remainder_lower_limit, remainder_upper_limit;

    remainder_lower_limit = qDes_policy - q_lower_limits;
    remainder_upper_limit = qDes_policy - q_upper_limits;

    for (int i=0; i<12; i++){
        clip(remainder_lower_limit(i,0), -100.0, 0.0);
        clip(remainder_upper_limit(i,0), 0.0, 100.0);
    }

    remainder_lower_limit = remainder_lower_limit.cwiseMin(0.0).cwiseMax(-100.0);
    remainder_upper_limit = remainder_upper_limit.cwiseMin(100.0).cwiseMax(0.0);

    return remainder_lower_limit + remainder_upper_limit;

}

int Quadruped::CheckPositionProtect(Eigen::Matrix<double,12,1>& qDes_policy, LowState state, double soft_limit)
{
    int protect = 0;
    for (int i=0; i<12; i++)
    {
        if (state.motorState[i].q < (q_lower_limits(i,0) + soft_limit) || state.motorState[i].q > (q_upper_limits(i,0) - soft_limit) )
        {   
            clip(qDes_policy(i,0), q_lower_limits(i,0), q_upper_limits(i,0));
            protect = 1;
        }
    }
    return protect;


}

// Exponential Moving Average (EMA) filter for the actions.
Eigen::MatrixXd Quadruped::actionFilter(Eigen::MatrixXd actions){
    Eigen::MatrixXd actions_filtered;
    actions_filtered.setZero();

    // Run the filter:
    actions_filtered = prevActions*(1-action_filter_freq) + actions*action_filter_freq;

    // Store the filtered actions for the next iteration:
    prevActions = actions_filtered;

    return actions_filtered;
}

// The main robot control loop.
void Quadruped::RobotControl()
{

    auto t0_dt = std::chrono::high_resolution_clock::now();
    motiontime++;

    std::string phase = "None";
    int phase_val = 0; // Associated phase_value (for switch case)

    Eigen::Matrix<double, 12,1> Torque_ff;
    Torque_ff.setZero();

    udp.GetRecv(state);
    // Update the states of the robot (q,dq,base_pos and base_quat, F_contact)
    updateStates(state);
    gait_pose_callback(state,dt,motiontime);

    // // OVERWRITE SOME VALUES FOR TESTING:
    // double HI = 0.2; // set HI and LO according to your problem.
    // double LO = -0.2;
    // double range= HI-LO;
    // Eigen::MatrixXd noise = Eigen::MatrixXd::Random(12,1); // 3x3 Matrix filled with random numbers between (-1,1)
    // noise = (noise + Eigen::MatrixXd::Constant(12,1,1.))*range/2.; // add 1 to the matrix to have values between 0 and 2; multiply with range/2
    // noise = (noise + Eigen::MatrixXd::Constant(12,1,LO)); //set LO as the lower bound (offset)
    // q_mea = q0;
    // contact.setOnes();


    float init_time = 2; // Time to initialise (in seconds)
    float stand_up_time = 3; // Time to stand up (in seconds)


    float phase_duration[2] = {};

    phase_duration[0] = int(init_time / dt); // Initialisation duration (in steps)
    phase_duration[1] = int(stand_up_time / dt); // Stand up duration (in steps)

    if(motiontime <= phase_duration[0]){
        phase = "initialise";
        phase_val = 1;
        gait_status = "STAND_INIT_STATUS";
    }
    // Move to the homing pose and maintain it until 'f' is pressed afterwards:
    else if (motiontime <= (phase_duration[0] + phase_duration[1]) || input_character != 'f'){
        phase="stand_up";
        phase_val = 2;
    }
    // After 'f' is pressed (and enough time for stand up has passed), start the jumping motion:
    else if (motiontime >= (phase_duration[0] + phase_duration[1]) && input_character == 'f'){
        phase="jumping";
        phase_val = 3;
    }

    else{
        throw std::invalid_argument("Motiontime doesn't match any phase.");
    }


    switch (phase_val)
    {
    case 1:{ // I.e. initialise
        // Set the gains to zero while initialising else it could cause a very jerky motion.
        changePIDgains(0.0,0.0,0.0,0.0);
        // Record initial states:
        for(int j=0; j<12;j++)
        {
            qInit(j,0) = state.motorState[j].q;
            qDes(j,0) = qInit(j,0);
        }
        base_pos_ini = base_pos;
        base_rpy_ini = base_rpy;

        if( motiontime == phase_duration[0]){
            std::cout << "Press Enter to Stand up..." << std::endl;
            std::cin.get();
            // input_char_stand_up = keyCheck();
            
            changePIDgains(Kp_hip,Kp_thigh,Kp_calf,Kd);
        }
        break;
    }

    case 2:{ // I.e. stand up
        double stand_up_rate = (phase_duration[1] - phase_duration[0]);
        double total_steps = phase_duration[1];

        // Stand up (passing qDes as a reference and updating its values):
        if (motiontime < (phase_duration[0] + phase_duration[1])){
            rate_count++;
            standUp(qDes, rate_count, total_steps, qInit, qHomingPose);
        }

        // After enough time has passed:
        if (motiontime >=(phase_duration[0] + phase_duration[1])){
            gait_status = "STANDING";
            // Record the standing pose:
            base_pos_standing(0,0) = base_pos(0,0);
            base_pos_standing(1,0) = base_pos(1,0);
            base_pos_standing(2,0) = base_pos(2,0);


            // Maintain the standing pose while waiting for the user to press 'f' to start jumping:
            if (motiontime ==(phase_duration[0] + phase_duration[1])){
                base_rpy_offset(2,0) = base_rpy(2,0);
                printf("Press 'f' to switch to policy: ");
            }
            if (kbhit()) {
                input_character = getchar();
                printf("\nChar received:%c\n", input_character);
                }
            else {
                // printf(".");
                fflush(stdout);
                usleep(1);
            }
            // ----------------------
        }
        // computeStateHistory();
        
        break;
    }

    case 3: {// I.e. jumping


        // Switch to the policy gains after the stand up phase:
        if (rate_count_jump == 0){
            changePIDgains(Kp_policy,Kp_policy,Kp_policy,Kd_policy);
            // std::cout << "Press 'd' to switch to start the jump: " << std::endl;
        }

        rate_count_jump++;

        if (rate_count_jump < 400.0){
            has_jumped_bool = 1;
        }
        else if (rate_count_jump == 400.0){
            has_jumped_bool = 0;
        }



        // if (rate_count_jump == 3500.0){
        //     resetStateHistory();
        //     reset_counter_cont_jump = true;
        //     std::cout << "Jump again" << std::endl;
        // }
        // if (rate_count_jump > 3500 && rate_count_jump < 3900.0){
        //     reset_counter_cont_jump = false;
        //     has_jumped_bool = 1;
        // }
        // else if (rate_count_jump == 3900.0){
        //     has_jumped_bool = 0;
        // }


        // Check flight:
        checkFlight();

        // This is unused for now:

        // if (kbhit()) {
        //         input_character = getchar();
        //         printf("\nChar received:%c\n", input_character);
        //         }
        // else {
        //     // printf(".");
        //     fflush(stdout);
        //     usleep(1);
        // }

        // if (input_character == 'd'){
        //     std::cout << "Switching to jump" << std::endl;
        // }

        // Only update the command every decimation steps:
        if (motiontime%decimation == 0){

            // Compute the state history (for the observation buffer):
            computeStateHistory();

 
            Eigen::VectorXd obs_buffer;
            obs_buffer = computeObservations();
            actions = policyStep(obs_buffer);

            // Reshape actions to match the order of the legs:
            Eigen::Matrix<double, 12,1> actions_local = actions;  
            if (switch_order_legs){
                actions_local = changeLegOrder(actions_local);
            }

            actions_unfiltered_store = actions_local; // For plotting

            // Filter them with the EMA filter:
            actions_local = actionFilter(actions_local);

            // For the first few steps dont pass the actions (let the robot settle)
            if (rate_count_jump < 2){
                actions_local.setZero();
            }

            actions_filtered_store = actions_local; // For plotting
            
            // Unscale the actions:
            actions_local = unscaleActions(actions_local);

            // Desired joint angle is default angles + unscaled actions:
            qDes_policy = q0 + actions_local;


            // for (int i=0; i<4; i++){
            //     clip(qDes_policy(i*3+0,0),-1.0471975512,1.0471975512);
            //     clip(qDes_policy(i*3+1,0),-0.663225115758,2.96705972839);
            //     clip(qDes_policy(i*3+2,0),-2.72271363311,-0.837758040957);
            // }
                // ---------------------- Safety Version 3 ----------------------

            // Like version 2 but instead of braking with PositionProtect it only limits the cmd;

            int safety_check = CheckPositionProtect(qDes_policy, state, 0.087); // This: once the protection is triggered, all motors are taken out of servo mode, or as it is called Braking mode
            if (safety_check){
                std::cout << "Triggered check" << std::endl;
            }

        }
        // Save as qDes to give to the motors:
        qDes = qDes_policy;
        
        // No feedforward torque:

        Torque_ff = Kp_policy*(qDes - q_mea) - tau_mea;
        for (int i=0; i<4; i++){
            clip(Torque_ff(i*3 + 0,0),-5.0,5.0); // Hip torque limit
            clip(Torque_ff(i*3 + 1,0),-5.0,5.0); // Thigh torque limit
            clip(Torque_ff(i*3 + 2,0),-10.0,10.0); // Calf torque limit
        }

        Torque_ff_GRF_data = Torque_ff;
        Torque_ff.setZero();

        break;
    }
    
    }
   

    cmd = prepareCommand(cmd,qDes, Torque_ff);

    // int res1 = safe.PowerProtect(cmd, state, 10);
    // int res1 = safe.PowerProtect(cmd, state, 6);

    // ---------------------- Safety Version 1 ----------------------

    // This clips the joint angle in the command based on the limits:
    
    // safe.PositionLimit(cmd); // This limits the joint angle in the cmd (basically clip).

    // ---------------------- Safety Version 2 ----------------------
    // If the joint angle is too close to the limit, it brakes the motor.

    // int safety_check = safe.PositionProtect(cmd, state, 0.087); // This: once the protection is triggered, all motors are taken out of servo mode, or as it is called Braking mode
    // if (safety_check == -1){
    //     safe.PositionLimit(cmd);
    // }


    // ---------------------- Safety Version 4 ----------------------
    
    // This uses the position controller up to the joint limits and then uses a
    // torque command for the remainder.

    // Eigen::Matrix<double,12,1> remaining_q;
    // remaining_q = safetyPolicy(qDes_policy);
    // Torque_ff_GRF = Kp_policy*(remaining_q);
    // cmd = prepareCommand(cmd,qDes, Torque_ff_GRF);
    // safe.PositionLimit(cmd); // This limits the joint angle in the cmd (basically clip).

    // ----------------------- Safety ----------------------


    /// send controls to robot
    udp.SetSend(cmd);

    publishData();



    // auto t1_dt = std::chrono::high_resolution_clock::now();

    // std::chrono::duration<double, std::milli> estimated_dt = t1_dt - t0_dt;

    // est_dt = estimated_dt.count()*1e-3;

    // // if (est_dt > .05){
    // //     est_dt = dt;
    // // }
    // // else if (est_dt < dt){
    // //     est_dt = dt;
    // // }

    // est_dt = dt;


    
}

// Prepare a command in ROS format to be published.
LowCmd Quadruped::prepareCommand(LowCmd SendLowROS,Eigen::Matrix<double, 12,1> qDes,  Eigen::Matrix<double, 12,1> torque_ff){
    
    for(int j=0; j<12;j++)
    {   
    for (int i=0; i<4; i++){
        clip(torque_ff(i*3 + 0,0),-23.0,23.0); // Hip torque limit
        clip(torque_ff(i*3 + 1,0),-23.0,23.0); // Thigh torque limit
        clip(torque_ff(i*3 + 2,0),-30.0,30.0); // Calf torque limit
    }

        // std::cout << "Clamped torque is: " << torque_ff(j,0) << std::endl;
        ////// joint-level + torque
        SendLowROS.motorCmd[j].q = qDes(j,0);
        SendLowROS.motorCmd[j].dq = 0; //
        SendLowROS.motorCmd[j].Kp = Kp_joint[j];
        SendLowROS.motorCmd[j].Kd = Kd_joint[j];
        SendLowROS.motorCmd[j].tau = 0;//torque_ff(j, 0);         
    }

    return SendLowROS;
}


int main(int argc, char *argv[]){

    ros::init(argc, argv, "torque_jumping_controller");
    

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    

    double dt = 0.005; // Change the operational period (in sec).
    double policy_dt = 0.02; // Change the policy period (in sec).

    Quadruped robot(LOWLEVEL,dt,policy_dt);

    // InitEnvironment();
    LoopFunc loop_control("control_loop", robot.dt,    boost::bind(&Quadruped::RobotControl, &robot));
    LoopFunc loop_udpSend("udp_send",     robot.dt, 3, boost::bind(&Quadruped::UDPSend,      &robot));
    LoopFunc loop_udpRecv("udp_recv",     robot.dt, 3, boost::bind(&Quadruped::UDPRecv,      &robot));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    ros::spin();


    return 0;

}

