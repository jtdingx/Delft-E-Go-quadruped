//
// Created by shuoy on 11/1/21.
// Tested by Jiatao Ding 
//

#include "Go1BasicEKF.h"
#include "yaml.h"

Go1BasicEKF::Go1BasicEKF () {
    // constructor
    config_set();
    eye3.setIdentity();
    // C is fixed: measurement matrix,
    C.setZero();
    for (int i=0; i<NUM_LEG; ++i) {
        C.block<3,3>(i*3,0) = -eye3;  //-pos
        C.block<3,3>(i*3,6+i*3) = eye3;  //foot pos
        C.block<3,3>(NUM_LEG*3+i*3,3) = eye3;  // vel
        C(NUM_LEG*6+i,6+i*3+2) = 1;  // height z of foot
    }

    // Q R are fixed
    Q.setIdentity();
    Q.block<3,3>(0,0) = PROCESS_NOISE_PIMU*eye3;               // position transition
    Q.block<3,3>(3,3) = PROCESS_NOISE_VIMU*eye3;               // velocity transition
    for (int i=0; i<NUM_LEG; ++i) {
        Q.block<3,3>(6+i*3,6+i*3) = PROCESS_NOISE_PFOOT*eye3;  // foot position transition
    }

    R.setIdentity();
    for (int i=0; i<NUM_LEG; ++i) {
        R.block<3,3>(i*3,i*3) = SENSOR_NOISE_PIMU_REL_FOOT*eye3;                        // fk estimation
        R.block<3,3>(NUM_LEG*3+i*3,NUM_LEG*3+i*3) = SENSOR_NOISE_VIMU_REL_FOOT*eye3;      // vel estimation
        R(NUM_LEG*6+i,NUM_LEG*6+i) = SENSOR_NOISE_ZFOOT;                               // height z estimation
    }

    // set A to identity
    A.setIdentity();

    // set B to zero
    B.setZero();

    assume_flat_ground = true;


    // init leg kinematics
    // set leg kinematics related parameters
    // body_to_Go1_body
    p_br = Eigen::Vector3d(-0.2293, 0.0, -0.067);
    R_br = Eigen::Matrix3d::Identity();
    // leg order: 0-FL  1-FR  2-RL  3-RR
    leg_offset_x[0] = 0.1881;
    leg_offset_x[1] = 0.1881;
    leg_offset_x[2] = -0.1881;
    leg_offset_x[3] = -0.1881;
    leg_offset_y[0] = 0.04675;
    leg_offset_y[1] = -0.04675;
    leg_offset_y[2] = 0.04675;
    leg_offset_y[3] = -0.04675;
    motor_offset[0] = 0.0838;
    motor_offset[1] = -0.0838;
    motor_offset[2] = 0.0838;
    motor_offset[3] = -0.0838;
    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = 0.213;
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = 0.213;

    for (int i = 0; i < NUM_LEG; i++) {
        Eigen::VectorXd rho_fix(5);
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
        Eigen::VectorXd rho_opt(3);
        rho_opt << 0.0, 0.0, 0.0;
        rho_fix_list.push_back(rho_fix);
        rho_opt_list.push_back(rho_opt);
    }

    //init swap order, very important
    swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
    swap_foot_indices << 1, 0, 3, 2;

    // Go1 hardware foot force filter reset
    foot_force_filters.setZero();
    foot_force_filters_idx.setZero();
    foot_force_filters_sum.setZero();    

}

Go1BasicEKF::Go1BasicEKF (bool assume_flat_ground_):Go1BasicEKF() {
    // constructor
    assume_flat_ground = assume_flat_ground_;
    // change R according to this flag, if we do not assume the robot moves on flat ground,
    // then we cannot infer height z using this way
    if (assume_flat_ground == false) {
        for (int i=0; i<NUM_LEG; ++i) {
            R(NUM_LEG*6+i,NUM_LEG*6+i) = 1e5;   // height z estimation not reliable
        }
    }
}

void Go1BasicEKF::config_set()
{   
    /////load default parameter from the yaml.file
    ///////////////////  yaml code . ///////// 
    YAML::Node config = YAML::LoadFile("/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/config/config.yaml");

    fz_limit = config["fz_limit1"].as<double>();

    PROCESS_NOISE_PIMU = config["PROCESS_NOISE_PIMU"].as<double>();
    PROCESS_NOISE_VIMU = config["PROCESS_NOISE_VIMU"].as<double>();
    PROCESS_NOISE_PFOOT = config["PROCESS_NOISE_PFOOT"].as<double>();
    SENSOR_NOISE_PIMU_REL_FOOT = config["SENSOR_NOISE_PIMU_REL_FOOT"].as<double>();
    SENSOR_NOISE_VIMU_REL_FOOT = config["SENSOR_NOISE_VIMU_REL_FOOT"].as<double>();
    SENSOR_NOISE_ZFOOT = config["SENSOR_NOISE_ZFOOT"].as<double>();

    //std::cout<<std::endl;

}



// /// update low robot state using the sensor values //
// Eigen::Matrix<double,6,1> Go1BasicEKF::update_low_state(Go1CtrlStates& state,Eigen::Matrix<double, 3,1> IMU_angle,Eigen::Matrix<double, 4,1> IMU_quaternion,Eigen::Matrix<double, 3,1> IMU_gyroscope,
//     Eigen::Matrix<double, 3,1> IMU_accelerometer, Eigen::Matrix<double,12,1> q, Eigen::Matrix<double,12,1> dq, Eigen::Matrix<double, 4,1> FSR, double dt, int simulation_mode) {

//         // fill data to state, notice the order in state is FR, FL, RR, RL
//         // fill data to state, notice the order in state is FL, FR, RL, RR
//         /* TODO: fill data */
//         /////// note that the quaternion is not good
//         if(simulation_mode==0)
//         {
//             state.root_quat = Eigen::Quaterniond(IMU_quaternion[1],
//                                                  IMU_quaternion[2],
//                                                  IMU_quaternion[3],
//                                                  IMU_quaternion[0]); 
//             // // state.root_rot_mat = state.root_quat.normalized().toRotationMatrix(); ////// normalize()
//             // state.root_euler = Utils::quat_to_euler(state.root_quat);

//             state.root_euler = IMU_angle;
//             state.root_rot_mat = Utils::Rz(state.root_euler(2)) *Utils::Ry(state.root_euler(1)) *Utils::Rx(state.root_euler(0)); ///rotation matrix
//         }
//         else{
//             state.root_quat = Eigen::Quaterniond(IMU_quaternion[0],
//                                                       IMU_quaternion[1],
//                                                       IMU_quaternion[2],
//                                                       IMU_quaternion[3]);            
//             /////// fix one bug//// quaternion to rotationmatrix!!!!!!!!!!!!!!!!!!! debug here !!!!!!!!!!!!!!!!!!!!!!!!!!!!
//             state.root_rot_mat = state.root_quat.normalized().toRotationMatrix(); ////// normalize()
//             state.root_euler = Utils::quat_to_euler(state.root_quat);
//             // std::cout<<"euler angle: "<< state.root_euler.transpose()<<std::endl;
//         }


//         double yaw_angle = state.root_euler[2];

//         state.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());
//         // state.root_pos     | do not fill
//         // state.root_lin_vel | do not fill

//         state.imu_acc = IMU_accelerometer;
//         // state.imu_acc[0] *= -1;
//         // state.imu_acc[1] *= -1;

//         state.imu_ang_vel = IMU_gyroscope;
//         state.root_ang_vel = state.root_rot_mat * state.imu_ang_vel;

//         // joint states

//         for (int i = 0; i < NUM_DOF; ++i) {
//             int swap_i = swap_joint_indices(i);
//             state.joint_vel[i] = dq[swap_i];

//             state.joint_pos[i] = q[swap_i];
//         }

//         // foot force, add a filter here: using the moving average filter in the outside loop
//         // for (int i = 0; i < NUM_LEG; ++i) {
//         //     int swap_i = swap_foot_indices(i);
//         //     double value = static_cast<double>(FSR[swap_i]);
//         //     foot_force_filters_sum[i] -= foot_force_filters(i, foot_force_filters_idx[i]);
//         //     foot_force_filters(i, foot_force_filters_idx[i]) = value;
//         //     foot_force_filters_sum[i] += value;
//         //     foot_force_filters_idx[i]++;
//         //     foot_force_filters_idx[i] %= FOOT_FILTER_WINDOW_SIZE;
//         //     state.foot_force[i] = foot_force_filters_sum[i] / static_cast<double>(FOOT_FILTER_WINDOW_SIZE);
//         // }

//         /////// filter 
//         for (int i = 0; i < NUM_LEG; ++i) {
//             int swap_i = swap_foot_indices(i);
//             double value = static_cast<double>(FSR[swap_i]);
//             state.foot_force[i] = FSR[swap_i];
//         }


//         //////*****************************************main loop **************************************************////
//         // TODO: we call estimator update here, be careful the runtime should smaller than the HARDWARE_UPDATE_FREQUENCY,i.e., 2ms
//         // state estimation
//         if (!is_inited()) {
//             init_state(state);
//         } else {
//             update_estimation(state, dt,simulation_mode,gra_acc);
//         }
        
//         //////check, probably the kinematic is not the same
//         // FL, FR, RL, RR
//         // use estimation base pos and vel to get foot pos and foot vel in world frame
//         for (int i = 0; i < NUM_LEG; ++i) {
//             state.foot_pos_rel.block<3, 1>(0, i) = go1_kin.fk(
//                     state.joint_pos.segment<3>(3 * i),
//                     rho_opt_list[i], rho_fix_list[i]);
//             state.j_foot.block<3, 3>(3 * i, 3 * i) = go1_kin.jac(
//                     state.joint_pos.segment<3>(3 * i),
//                     rho_opt_list[i], rho_fix_list[i]);
//             Eigen::Matrix3d tmp_mtx = state.j_foot.block<3, 3>(3 * i, 3 * i);
//             Eigen::Vector3d tmp_vec = state.joint_vel.segment<3>(3 * i);
//             state.foot_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

//             state.foot_pos_abs.block<3, 1>(0, i) =
//                     state.root_rot_mat * state.foot_pos_rel.block<3, 1>(0, i);
//             state.foot_vel_abs.block<3, 1>(0, i) =
//                     state.root_rot_mat * state.foot_vel_rel.block<3, 1>(0, i);

//             // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
//             // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
//             // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
//             state.foot_pos_world.block<3, 1>(0, i) =
//                     state.foot_pos_abs.block<3, 1>(0, i) + state.root_pos;
//             state.foot_vel_world.block<3, 1>(0, i) =
//                     state.foot_vel_abs.block<3, 1>(0, i) + state.root_lin_vel;
//         }
//         return x.segment<6>(0);
// }






void Go1BasicEKF::init_state(Go1CtrlStates& state) {

    filter_initialized = true;
    P.setIdentity();
    P = P * 3;

    // set initial value of x
    x.setZero();
    //x.segment<3>(0) = Eigen::Vector3d(0, 0, 0.3);
    x.segment<3>(0) = state.estimated_root_pos;
    x.segment<3>(3) = state.estimated_root_vel;
    for (int i = 0; i < NUM_LEG; ++i) {
        Eigen::Vector3d fk_pos = state.foot_pos_rel.block<3, 1>(0, i);
        x.segment<3>(6 + i * 3) = state.root_rot_mat * fk_pos + x.segment<3>(0);
    }
}

void Go1BasicEKF::update_estimation(Go1CtrlStates& state, double dt, int simulation_mode, Eigen::Vector3d gra_acc) {
    // update A B using latest dt: the linear model: x_{k+1} = x_k + dt * v_k, v_{k+1} = v_k + dt * a_k
    A.block<3, 3>(0, 3) = dt * eye3;  ////// A is the so-caled F matrix in Kalman-Bayesian-Filter-in-python book
    B.block<3, 3>(3, 0) = dt * eye3;
    //B.block<3, 3>(0, 0) = 0.5*pow(dt,2) * eye3; /////// modified by jiatao
    Eigen::Vector3d u;
    u.setZero();

    // control input u is Ra + ag ####### linear acceleration
    if(simulation_mode==0) /////pybullet
    {
        u = state.root_rot_mat * state.imu_acc;
    }
    else ///// imu acceleraton plus gravity
    {
        if(simulation_mode==1)
        {
          u = state.root_rot_mat * state.imu_acc + Eigen::Vector3d(0, 0, -9.80);
        }
        else
        {
            u = state.root_rot_mat * state.imu_acc + gra_acc;
        }
        
    }
    
    // process update: predict 
    xbar = A*x + B*u ; /////state prediction
    Pbar = A * P * A.transpose() + Q; ///update belief
     
    ///////// update the kf value according to the contact state////////// 
    ////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!=======///////////////////////////////////////////
    // contact estimation, ///// the 100.0 should be carefully tuned to test if the contact estimator is reliable enough
    if (state.movement_mode == 0) {  // stand
        for (int i = 0; i < NUM_LEG; ++i) estimated_contacts[i] = 1.0;
    } else {  // walk
        for (int i = 0; i < NUM_LEG; ++i) {
            if(simulation_mode == 0)/////pybullet
            {
                estimated_contacts[i] = std::min(std::max((state.foot_force(i)) / (1.0 - 0.0), 0.0), 1.0); ////// judge contact
            }
            else if (simulation_mode == 1)
            {
                estimated_contacts[i] = std::min(std::max((state.foot_force(i)) / (10.0 - 0.0), 0.0), 1.0); ////// judge contact
            }
            else /////hardware: tune 100 to the real threshold
            {
                estimated_contacts[i] = std::min(std::max(pow((state.foot_force(i)) / (300 - 0.0),2), 0.0), 1.0); ////// judge contact

                if(estimated_contacts[i]<0.5)
                {
                   estimated_contacts[i] = 0;
                } 
                else
                {
                    estimated_contacts[i] = 1;
                }
            }
            
            
        }
    }

    // put estimated values back to Go1CtrlStates& state
    for (int i = 0; i < NUM_LEG; ++i) {
        if (estimated_contacts[i] < 0.5) {
            state.estimated_contacts[i] = 0;
        } else {
            state.estimated_contacts[i] = 1;
        }
    }

    if(state.estimated_contacts.maxCoeff()<0.5) ///all in flight phase
    {
        x = xbar;
        //std::cout<<"in the air!! only IMU"<<std::endl;
    }
    else
    {
        // update Q/////// the parameters are quite confusing ......
        Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt / 20.0 * eye3;
        Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * 9.8 / 20.0 * eye3;
        // update Q R for legs not in contact
        for (int i = 0; i < NUM_LEG; ++i) {
            Q.block<3, 3>(6 + i * 3, 6 + i * 3)
                    =
                    (1 + (1 - estimated_contacts[i]) * 1e3) * dt * PROCESS_NOISE_PFOOT * eye3;  // foot position transition
            // for estimated_contacts[i] == 1, Q = 0.002
            // for estimated_contacts[i] == 0, Q = 1001*Q

            R.block<3, 3>(i * 3, i * 3)
                    = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_PIMU_REL_FOOT *
                    eye3;                       // fk estimation
            R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3)
                    = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_VIMU_REL_FOOT * eye3;      // vel estimation
            if (assume_flat_ground) {
                R(NUM_LEG * 6 + i, NUM_LEG * 6 + i)
                        = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_ZFOOT;       // height z estimation
            }
        }
        
        // measurement construction
        // yhat = C*xbar; /////// C is the so-called H matrix in in Kalman-Bayesian-Filter-in-python book

        //    leg_v = (-J_rf*av-skew(omega)*p_rf);
        //    r((i-1)*3+1:(i-1)*3+3) = body_v - R_er*leg_v;
        // actual measurement: in this way, there is always delay in the actual measurement
        for (int i=0; i<NUM_LEG; ++i) {
            Eigen::Vector3d fk_pos = state.foot_pos_rel.block<3,1>(0,i);
            y.block<3,1>(i*3,0) = state.root_rot_mat*fk_pos;   // fk estimation
            Eigen::Vector3d leg_v = -state.foot_vel_rel.block<3,1>(0,i) - Utils::skew(state.imu_ang_vel)*fk_pos;
            y.block<3,1>(NUM_LEG*3+i*3,0) =
                    (1.0-estimated_contacts[i])*x.segment<3>(3) +  estimated_contacts[i]*state.root_rot_mat*leg_v;      // vel estimation

            y(NUM_LEG*6+i) =
                    (1.0-estimated_contacts[i])*(x(2)+fk_pos(2)) + estimated_contacts[i]*0;                               // height z estimation
        }
        
        ///// state error
        ////error_y = y - yhat;
        //// state error
        error_y = y - C*xbar;

        //// preparing for the Kalman gain
        S = C * Pbar *C.transpose() + R;
        S = 0.5*(S+S.transpose());
        
        Serror_y = S.fullPivHouseholderQr().solve(error_y); //////  S^{-1} * error_y

        x = xbar + Pbar * C.transpose() * Serror_y; //////updated x

        SC = S.fullPivHouseholderQr().solve(C); //////// S^{-1} * C
        P = Pbar - Pbar * C.transpose() * SC * Pbar;
        P = 0.5 * (P + P.transpose());

        // reduce position drift
        if (P.block<2, 2>(0, 0).determinant() > 1e-6) {
            P.block<2, 16>(0, 2).setZero();
            P.block<16, 2>(2, 0).setZero();
            P.block<2, 2>(0, 0) /= 10.0;
        }
    }

//    std::cout << x.transpose() <<std::endl;
    state.estimated_root_pos = x.segment<3>(0);
    state.estimated_root_vel = x.segment<3>(3);

    state.root_pos = x.segment<3>(0);
    state.root_lin_vel = x.segment<3>(3);



    // return x.segment<6>(0);
}