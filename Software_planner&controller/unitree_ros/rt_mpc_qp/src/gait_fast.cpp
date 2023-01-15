#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <sys/types.h>
#include <time.h>
///// add necessarey head file
#include <fstream>   
#include <string>  
#include <cassert>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "math.h"
#include "FastMPC/PRMPCClass.h"
#include "/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/go1_rt_control/src/Robotpara/robot_const_para_config.h"
//#include "execution_timer.h"
#include "yaml.h"
#include <ros/package.h> 
#include <geometry_msgs/Twist.h>  


using namespace Eigen;
using namespace std;

sensor_msgs::JointState joint2simulationx;
sensor_msgs::JointState state_to_MPC; /// 1 flag + 18state+3right_leg+3left_foot;

double footvx_max = 1;
double footvx_min = -1;
double footvy_max = 1;
double footvy_min = -1;
double footvz_max = 2;
double footvz_min = -2;
bool mpc_start = false;

Eigen::Matrix<double,25,1> state_feedback;
Matrix<double,100,1> low_mpc_gait;
int mpc_gait_flag,  mpc_gait_flag_old;
Matrix<double,51,1> low_mpc_gait_inte;
Matrix<double,25,1> control_gait;

PRMPCClass rpympc;

Eigen::Matrix<double, 21,1> rpy_mpc_body, rfoot_inter, lfoot_inter, bodytheta_inter, rftheta_inter, lftheta_inter, zmp_inter, dcm_inter, comacc_inter;

Eigen::Vector3d COM_in1, COM_in2, COMxyz_ref, COMv_ref, COM_ref2;
Eigen::Vector3d FootL_in1, FootL_in2, FootL_ref, FootLv_ref, FootL_ref2;
Eigen::Vector3d FootR_in1, FootR_in2, FootR_ref, FootRv_ref, FootR_ref2;
Eigen::Vector3d body_in1, body_in2, body_ref, bodyv_ref, body_ref2;
Eigen::Vector3d rfootrpy_in1, rfootrpy_in2, rfootrpy_ref, rfootrpyv_ref, rfootrpy_ref2;
Eigen::Vector3d lfootrpy_in1, lfootrpy_in2, lfootrpy_ref, lfootrpyv_ref, lfootrpy_ref2;
Eigen::Vector3d COMacc_in1, COMacc_in2, COMacc_ref, COMaccv_ref, COMacc_ref2;

Eigen::Vector3d zmp_in1, zmp_in2, zmpxyz_ref, zmpv_ref, zmp_ref2;
Eigen::Vector3d dcm_in1, dcm_in2, dcmxyz_ref, dcmv_ref, dcm_ref2;

Eigen::Vector3d PelvisPos, body_thetax, LeftFootPosx,RightFootPosx;
Eigen::Vector3d F_L, F_R, M_L, M_R;
Eigen::Vector3d LeftFootRPY, RightFootRPY;

Eigen::Matrix<double, 2, 5 > zmp_mpc_ref, rfoot_mpc_ref, lfoot_mpc_ref, bodyangle_mpc_ref;
Eigen::Matrix<double, 3, 5 > comacc_mpc_ref;
Eigen::Matrix<double, 4, 1 > bodyangle_state;
Eigen::Matrix<double, 14, 1 > bodyangle_mpc;


int count_in_rt_loop;
int count_in_rt_loopx;
int count_in_rt_loop_restart;
int count_in_rt_mpc;
int count_in_rt_mpcx;
int count_in_rt_mpc_restart;
int count_in_rt_start;
int count_inteplotation;
int t_int;
int n_t_int;
double dt_mpc_fast;
double dt_mpc_slow;
int mpc_fast_frequency;
double _mass;
double _j_ini;
double _Zsc;
double _ggg;
double z_c;
double dtx;


bool _stopwalking;
Eigen::Matrix<double, 30, 1> foorpr_gen;
Eigen::Matrix<double, 6, 1> foorpr_kmp;
Eigen::Matrix<double, 9, 1> Nrtfoorpr_gen;
Eigen::Matrix<double, 30, 1> foortheta_gen;
double _height_offset_timex;
double rfootx, rfooty,rfootz,lfootx, lfooty,lfootz;

void nrt_gait_sub_operation(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int jx = 0; jx<100; jx++)
    {
        low_mpc_gait(jx) = msg->position[jx]; 
    }
    mpc_gait_flag = low_mpc_gait(26);
    
    Nrtfoorpr_gen = low_mpc_gait.block<9,1>(86, 0);
    
}


void control_gait_sub_operation(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int jx = 0; jx<25; jx++)
    {
        control_gait(jx) = msg->position[jx]; 
    }
    
    /// state feedback
    for (int jx = 1; jx<25; jx++)
    {
        state_feedback(jx) = msg->position[jx]; 
    }  
    
    bodyangle_state(0) = state_feedback(10);
    bodyangle_state(1) = state_feedback(11);
    bodyangle_state(2) = state_feedback(13);
    bodyangle_state(3) = state_feedback(14);    
    
}


void xget_position_interpolation()
{
  count_inteplotation += 1;

  /// using the current state: 2 *_ dt time delay .....
  if (t_int >2)
  { 
    
  ///// modified: using the receieve MPC data /////// smooth trajectory            
/*    rpy_mpc_body = rpympc.XGetSolution_position_mod1(count_inteplotation, dt_mpc_fast,COM_in1,COM_in2,COMxyz_ref,COM_ref2);///  4^th order
    comacc_inter = rpympc.XGetSolution_position_mod1(count_inteplotation, dt_mpc_fast,COMacc_in1,COMacc_in2,COMacc_ref,COMacc_ref2);
    lfoot_inter = rpympc.XGetSolution_position_mod1(count_inteplotation, dt_mpc_fast,FootL_in1,FootL_in2,FootL_ref,FootL_ref2);  ///
    rfoot_inter = rpympc.XGetSolution_position_mod1(count_inteplotation, dt_mpc_fast,FootR_in1,FootR_in2,FootR_ref,FootR_ref2);  ///
    rftheta_inter = rpympc.XGetSolution_position_mod1(count_inteplotation, dt_mpc_fast,rfootrpy_in1,rfootrpy_in2,rfootrpy_ref,rfootrpy_ref2);  ///
    lftheta_inter = rpympc.XGetSolution_position_mod1(count_inteplotation, dt_mpc_fast,lfootrpy_in1,lfootrpy_in2,lfootrpy_ref,lfootrpy_ref2);  ///
    zmp_inter = rpympc.XGetSolution_position_mod1(count_inteplotation, dt_mpc_fast,zmp_in1,zmp_in2,zmpxyz_ref,zmp_ref2);  ///
    dcm_inter = rpympc.XGetSolution_position_mod1(count_inteplotation, dt_mpc_fast,dcm_in1,dcm_in2,dcmxyz_ref,dcm_ref2);  ///
  */  
    rpy_mpc_body = rpympc.XGetSolution_position_mod3(count_inteplotation, dt_mpc_fast,COM_in1,COM_in2,COMxyz_ref,COM_ref2);///  4^th order
    comacc_inter = rpympc.XGetSolution_position_mod3(count_inteplotation, dt_mpc_fast,COMacc_in1,COMacc_in2,COMacc_ref,COMacc_ref2);
    // lfoot_inter = rpympc.XGetSolution_position_mod3(count_inteplotation, dt_mpc_fast,FootL_in1,FootL_in2,FootL_ref,FootL_ref2);  ///
    // rfoot_inter = rpympc.XGetSolution_position_mod3(count_inteplotation, dt_mpc_fast,FootR_in1,FootR_in2,FootR_ref,FootR_ref2);  ///
    // rftheta_inter = rpympc.XGetSolution_position_mod3(count_inteplotation, dt_mpc_fast,rfootrpy_in1,rfootrpy_in2,rfootrpy_ref,rfootrpy_ref2);  ///
    // lftheta_inter = rpympc.XGetSolution_position_mod3(count_inteplotation, dt_mpc_fast,lfootrpy_in1,lfootrpy_in2,lfootrpy_ref,lfootrpy_ref2);  ///
    zmp_inter = rpympc.XGetSolution_position_mod3(count_inteplotation, dt_mpc_fast,zmp_in1,zmp_in2,zmpxyz_ref,zmp_ref2);  ///
    dcm_inter = rpympc.XGetSolution_position_mod3(count_inteplotation, dt_mpc_fast,dcm_in1,dcm_in2,dcmxyz_ref,dcm_ref2);  ///
  }
  

  /////// 
  if (count_inteplotation % n_t_int ==0){
    //// using the last two mpc_data
    COM_in1 = COM_in2;
    COM_in2 = COMxyz_ref;  

    FootL_in1 = FootL_in2;
    FootL_in2 = FootL_ref;  

    FootR_in1 = FootR_in2;
    FootR_in2 = FootR_ref;

    rfootrpy_in1 = rfootrpy_in2;
    rfootrpy_in2 = rfootrpy_ref;   

    lfootrpy_in1 = lfootrpy_in2;
    lfootrpy_in2 = lfootrpy_ref;      


    zmp_in1 = zmp_in2;
    zmp_in2 = zmpxyz_ref; 

    dcm_in1 = dcm_in2;
    dcm_in2 = dcmxyz_ref; 

    COMacc_in1 = COMacc_in2;
    COMacc_in2 = COMacc_ref;      

    if (mpc_gait_flag > mpc_gait_flag_old) //mpc update
    {
      COMxyz_ref(0) = low_mpc_gait(0);
      COMxyz_ref(1) = low_mpc_gait(1);
      COMxyz_ref(2) = low_mpc_gait(2);

      COMv_ref(0) = low_mpc_gait(36);
      COMv_ref(1) = low_mpc_gait(37);
      COMv_ref(2) = low_mpc_gait(38); 
      COM_ref2(0) = COMxyz_ref(0) + COMv_ref(0) * dt_mpc_slow;
      COM_ref2(1) = COMxyz_ref(1) + COMv_ref(1) * dt_mpc_slow;
      COM_ref2(2) = COMxyz_ref(2) + COMv_ref(2) * dt_mpc_slow;


      COMacc_ref(0) = low_mpc_gait(39);
      COMacc_ref(1) = low_mpc_gait(40);
      COMacc_ref(2) = low_mpc_gait(41);

      COMacc_ref2(0) = low_mpc_gait(80);
      COMacc_ref2(1) = low_mpc_gait(81);
      COMacc_ref2(2) = low_mpc_gait(82);


      FootL_ref(0) = low_mpc_gait(6);
      FootL_ref(1) = low_mpc_gait(7);
      FootL_ref(2) = low_mpc_gait(8);

      FootLv_ref(0) = low_mpc_gait(49);
      FootLv_ref(1) = low_mpc_gait(50);
      FootLv_ref(2) = low_mpc_gait(51); 
      FootL_ref2(0) = FootL_ref(0) + FootLv_ref(0) * dt_mpc_slow;
      FootL_ref2(1) = FootL_ref(1) + FootLv_ref(1) * dt_mpc_slow;
      FootL_ref2(2) = FootL_ref(2) + FootLv_ref(2) * dt_mpc_slow;

      FootR_ref(0) = low_mpc_gait(9);
      FootR_ref(1) = low_mpc_gait(10);
      FootR_ref(2) = low_mpc_gait(11);

      FootRv_ref(0) = low_mpc_gait(46);
      FootRv_ref(1) = low_mpc_gait(47);
      FootRv_ref(2) = low_mpc_gait(48); 
      FootR_ref2(0) = FootR_ref(0) + FootRv_ref(0) * dt_mpc_slow;
      FootR_ref2(1) = FootR_ref(1) + FootRv_ref(1) * dt_mpc_slow;
      FootR_ref2(2) = FootR_ref(2) + FootRv_ref(2) * dt_mpc_slow;

      zmpxyz_ref(0) = low_mpc_gait(12);
      zmpxyz_ref(1) = low_mpc_gait(13);

      zmp_ref2(0) = low_mpc_gait(42);
      zmp_ref2(1) = low_mpc_gait(43);

      dcmxyz_ref(0) = low_mpc_gait(34);
      dcmxyz_ref(1) = low_mpc_gait(35);

      dcm_ref2(0) = low_mpc_gait(44);
      dcm_ref2(1) = low_mpc_gait(45);    


      rfootrpy_ref(0) = low_mpc_gait(31);
      rfootrpy_ref(1) = low_mpc_gait(32);
      rfootrpy_ref(2) = low_mpc_gait(33);

      rfootrpyv_ref(0) = low_mpc_gait(58);
      rfootrpyv_ref(1) = low_mpc_gait(59);
      rfootrpyv_ref(2) = low_mpc_gait(60); 
      rfootrpy_ref2(0) = rfootrpy_ref(0) + rfootrpyv_ref(0) * dt_mpc_slow;
      rfootrpy_ref2(1) = rfootrpy_ref(1) + rfootrpyv_ref(1) * dt_mpc_slow;
      rfootrpy_ref2(2) = rfootrpy_ref(2) + rfootrpyv_ref(2) * dt_mpc_slow;

      lfootrpy_ref(0) = low_mpc_gait(28);
      lfootrpy_ref(1) = low_mpc_gait(29);
      lfootrpy_ref(2) = low_mpc_gait(30);

      lfootrpyv_ref(0) = low_mpc_gait(61);
      lfootrpyv_ref(1) = low_mpc_gait(62);
      lfootrpyv_ref(2) = low_mpc_gait(63); 
      lfootrpy_ref2(0) = lfootrpy_ref(0) + lfootrpyv_ref(0) * dt_mpc_slow;
      lfootrpy_ref2(1) = lfootrpy_ref(1) + lfootrpyv_ref(1) * dt_mpc_slow;
      lfootrpy_ref2(2) = lfootrpy_ref(2) + lfootrpyv_ref(2) * dt_mpc_slow;                

    }
    else  /// in case that the mpc data is not updated
    {

      COMxyz_ref(0) = low_mpc_gait(0);
      COMxyz_ref(1) = low_mpc_gait(1);
      COMxyz_ref(2) = low_mpc_gait(2);

      COMv_ref(0) = low_mpc_gait(36);
      COMv_ref(1) = low_mpc_gait(37);
      COMv_ref(2) = low_mpc_gait(38);               
      COMxyz_ref(0) += COMv_ref(0) * dt_mpc_slow;
      COMxyz_ref(1) += COMv_ref(1) * dt_mpc_slow;
      COMxyz_ref(2) += COMv_ref(2) * dt_mpc_slow;
      COMv_ref(0) += low_mpc_gait(39) * dt_mpc_slow;
      COMv_ref(1) += low_mpc_gait(40) * dt_mpc_slow;
      COMv_ref(2) += low_mpc_gait(41) * dt_mpc_slow; 
      COM_ref2(0) = COMxyz_ref(0) + COMv_ref(0) * dt_mpc_slow;
      COM_ref2(1) = COMxyz_ref(1) + COMv_ref(1) * dt_mpc_slow;
      COM_ref2(2) = COMxyz_ref(2) + COMv_ref(2) * dt_mpc_slow; 


      COMacc_ref(0) = low_mpc_gait(80);
      COMacc_ref(1) = low_mpc_gait(81);
      COMacc_ref(2) = low_mpc_gait(82);      
      COMacc_ref2(0) = low_mpc_gait(83);
      COMacc_ref2(1) = low_mpc_gait(84);
      COMacc_ref2(2) = low_mpc_gait(85);  


      FootL_ref(0) = low_mpc_gait(6);
      FootL_ref(1) = low_mpc_gait(7);
      FootL_ref(2) = low_mpc_gait(8);

      FootLv_ref(0) = low_mpc_gait(49);
      FootLv_ref(1) = low_mpc_gait(50);
      FootLv_ref(2) = low_mpc_gait(51); 
      FootL_ref(0) += FootLv_ref(0) * dt_mpc_slow;
      FootL_ref(1) += FootLv_ref(1) * dt_mpc_slow;
      FootL_ref(2) += FootLv_ref(2) * dt_mpc_slow;

      FootLv_ref(0) += low_mpc_gait(55) * dt_mpc_slow;
      FootLv_ref(1) += low_mpc_gait(56) * dt_mpc_slow;
      FootLv_ref(2) += low_mpc_gait(57) * dt_mpc_slow; 
      FootL_ref2(0) = FootL_ref(0) + FootLv_ref(0) * dt_mpc_slow;
      FootL_ref2(1) = FootL_ref(1) + FootLv_ref(1) * dt_mpc_slow;
      FootL_ref2(2) = FootL_ref(2) + FootLv_ref(2) * dt_mpc_slow;

      FootR_ref(0) = low_mpc_gait(9);
      FootR_ref(1) = low_mpc_gait(10);
      FootR_ref(2) = low_mpc_gait(11);

      FootRv_ref(0) = low_mpc_gait(46);
      FootRv_ref(1) = low_mpc_gait(47);
      FootRv_ref(2) = low_mpc_gait(48); 
      FootR_ref(0) += FootRv_ref(0) * dt_mpc_slow;
      FootR_ref(1) += FootRv_ref(1) * dt_mpc_slow;
      FootR_ref(2) += FootRv_ref(2) * dt_mpc_slow;

      FootRv_ref(0) += low_mpc_gait(52) * dt_mpc_slow;
      FootRv_ref(1) += low_mpc_gait(53) * dt_mpc_slow;
      FootRv_ref(2) += low_mpc_gait(54) * dt_mpc_slow; 
      FootR_ref2(0) = FootR_ref(0) + FootRv_ref(0) * dt_mpc_slow;
      FootR_ref2(1) = FootR_ref(1) + FootRv_ref(1) * dt_mpc_slow;
      FootR_ref2(2) = FootR_ref(2) + FootRv_ref(2) * dt_mpc_slow;

      zmpxyz_ref(0) = low_mpc_gait(42);
      zmpxyz_ref(1) = low_mpc_gait(43);
      zmp_ref2(0) = low_mpc_gait(76);
      zmp_ref2(1) = low_mpc_gait(77); 
      // zmp_ref2(0) = 2 * low_mpc_gait(42) - low_mpc_gait(12);
      // zmp_ref2(1) = 2 * low_mpc_gait(43) - low_mpc_gait(13);

      dcmxyz_ref(0) = low_mpc_gait(44);
      dcmxyz_ref(1) = low_mpc_gait(45);

      dcm_ref2(0) = low_mpc_gait(78);
      dcm_ref2(1) = low_mpc_gait(79); 
      // dcm_ref2(0) = 2 * low_mpc_gait(44) - low_mpc_gait(34);
      // dcm_ref2(1) = 2 * low_mpc_gait(45) - low_mpc_gait(35); 


      rfootrpy_ref(0) = low_mpc_gait(31);
      rfootrpy_ref(1) = low_mpc_gait(32);
      rfootrpy_ref(2) = low_mpc_gait(33);

      rfootrpyv_ref(0) = low_mpc_gait(58);
      rfootrpyv_ref(1) = low_mpc_gait(59);
      rfootrpyv_ref(2) = low_mpc_gait(60); 
      rfootrpy_ref(0) += rfootrpyv_ref(0) * dt_mpc_slow;
      rfootrpy_ref(1) += rfootrpyv_ref(1) * dt_mpc_slow;
      rfootrpy_ref(2) += rfootrpyv_ref(2) * dt_mpc_slow;

      rfootrpyv_ref(0) += low_mpc_gait(64) * dt_mpc_slow;
      rfootrpyv_ref(1) += low_mpc_gait(65) * dt_mpc_slow;
      rfootrpyv_ref(2) += low_mpc_gait(66) * dt_mpc_slow; 
      rfootrpy_ref2(0) = rfootrpy_ref(0) + rfootrpyv_ref(0) * dt_mpc_slow;
      rfootrpy_ref2(1) = rfootrpy_ref(1) + rfootrpyv_ref(1) * dt_mpc_slow;
      rfootrpy_ref2(2) = rfootrpy_ref(2) + rfootrpyv_ref(2) * dt_mpc_slow;

      lfootrpy_ref(0) = low_mpc_gait(28);
      lfootrpy_ref(1) = low_mpc_gait(29);
      lfootrpy_ref(2) = low_mpc_gait(30);

      lfootrpyv_ref(0) = low_mpc_gait(61);
      lfootrpyv_ref(1) = low_mpc_gait(62);
      lfootrpyv_ref(2) = low_mpc_gait(63); 
      lfootrpy_ref(0) += lfootrpyv_ref(0) * dt_mpc_slow;
      lfootrpy_ref(1) += lfootrpyv_ref(1) * dt_mpc_slow;
      lfootrpy_ref(2) += lfootrpyv_ref(2) * dt_mpc_slow;

      lfootrpyv_ref(0) += low_mpc_gait(67) * dt_mpc_slow;
      lfootrpyv_ref(1) += low_mpc_gait(68) * dt_mpc_slow;
      lfootrpyv_ref(2) += low_mpc_gait(69) * dt_mpc_slow; 
      lfootrpy_ref2(0) = lfootrpy_ref(0) + lfootrpyv_ref(0) * dt_mpc_slow;
      lfootrpy_ref2(1) = lfootrpy_ref(1) + lfootrpyv_ref(1) * dt_mpc_slow;
      lfootrpy_ref2(2) = lfootrpy_ref(2) + lfootrpyv_ref(2) * dt_mpc_slow;
    }

    count_inteplotation = 0;
    mpc_gait_flag_old = mpc_gait_flag;
  } 
}

void config_set()
{   
    /////load default parameter from the yaml.file
    ///////////////////  yaml code . ///////// 
    // YAML::Node config = YAML::LoadFile("/home/pi/go1_catkin/src/unitree_ros_to_real/unitree_legged_real/config/config.yaml");
    YAML::Node config = YAML::LoadFile("/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/go1_rt_control/config/config.yaml");

    dt_mpc_slow =  config["dt_slow_mpc"].as<double>();
    z_c =  config["body_p_Homing_Retarget2"].as<double>();
    dtx =  config["dt_rt_loop1"].as<double>();
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"rtmpc");

    ros::NodeHandle nh;
    config_set();
    joint2simulationx.position.resize(100);
    state_to_MPC.position.resize(25);
    state_feedback.setZero();
    low_mpc_gait.setZero();   
    mpc_gait_flag = 0;
    mpc_gait_flag_old = 0;
    low_mpc_gait_inte.setZero(); 
    COM_in1.setZero(); 
    COM_in2.setZero(); 
    COMxyz_ref.setZero(); 
    COM_ref2.setZero(); 
    COM_in1(2) = COM_in2(2) = COMxyz_ref(2) = COM_ref2(2) = z_c; 

    COMv_ref.setZero(); 

    COMacc_in1.setZero();  
    COMacc_in2.setZero();  
    COMacc_ref.setZero();  
    COMaccv_ref.setZero();  
    COMacc_ref2.setZero();     

    FootL_in1.setZero();  
    FootL_in2.setZero();  
    FootL_ref.setZero();  
    FootL_ref2.setZero();  
    //FootL_in1(1) = FootL_in2(1)= FootL_ref(1) = FootL_ref2(1) = gait::RobotParaClass_HALF_HIP_WIDTH; 
    FootLv_ref.setZero(); 

    FootR_in1.setZero();  
    FootR_in2.setZero();  
    FootR_ref.setZero();  
    //FootR_in1(1) = FootR_in2(1)= FootR_ref(1) = FootR_ref2(1) = -gait::RobotParaClass_HALF_HIP_WIDTH; 
    FootRv_ref.setZero(); 

    zmp_in1.setZero(); zmp_in2.setZero(); zmpxyz_ref.setZero(); zmpv_ref.setZero(); zmp_ref2.setZero();
    dcm_in1.setZero(); dcm_in2.setZero(); dcmxyz_ref.setZero(); dcmv_ref.setZero(); dcm_ref2.setZero();

    body_in1.setZero(); body_in2.setZero(); body_ref.setZero(); bodyv_ref.setZero(); body_ref2;
    rfootrpy_in1.setZero(); rfootrpy_in2.setZero(); rfootrpy_ref.setZero(); rfootrpyv_ref.setZero(); rfootrpy_ref2.setZero();
    lfootrpy_in1.setZero(); lfootrpy_in2.setZero(); lfootrpy_ref.setZero(); lfootrpyv_ref.setZero(); lfootrpy_ref2.setZero();



    PelvisPos.setZero();  
    body_thetax.setZero();  
    LeftFootPosx.setZero(); 
    RightFootPosx.setZero();  
    dcmxyz_ref.setZero();  
    F_L.setZero();  
    F_R.setZero(); 
    M_L.setZero();  
    M_R.setZero(); 
    LeftFootRPY.setZero();  
    RightFootRPY.setZero(); 
    
    rpy_mpc_body.setZero();
    rpy_mpc_body(2) = COM_ref2(2);

    rfoot_inter.setZero();
    rfoot_inter(1) = FootR_ref2(1);

    lfoot_inter.setZero();
    lfoot_inter(1) = FootL_ref2(1);

    bodytheta_inter.setZero();

    rftheta_inter.setZero(); 
    lftheta_inter.setZero();

    zmp_inter.setZero(); 
    dcm_inter.setZero();

    zmp_mpc_ref.setZero(); rfoot_mpc_ref.setZero(); lfoot_mpc_ref.setZero(); bodyangle_mpc_ref.setZero(); comacc_mpc_ref.setZero();
    bodyangle_mpc.setZero();
    bodyangle_state.setZero();

    ros::Subscriber nrt_mpc_gait_subscribe_ = nh.subscribe<sensor_msgs::JointState>("/MPC/Gait", 10,nrt_gait_sub_operation);
    ros::Subscriber control_gait_subscribe_ = nh.subscribe<sensor_msgs::JointState>("/control2rtmpc/state", 10,control_gait_sub_operation);

    ros::Publisher rt_gait_pub_ = nh.advertise<sensor_msgs::JointState>("/rtMPC/traj", 10);
    ros::Publisher rt_to_nrt_pub_ = nh.advertise<sensor_msgs::JointState>("/rt2nrt/state", 10);

     count_in_rt_loop = 0;
     count_in_rt_loopx = 0;
     count_in_rt_loop_restart = 0;
     count_in_rt_mpc = 0;
     count_in_rt_mpcx = 0;
     count_in_rt_mpc_restart = 0;

    count_inteplotation = 0;
    t_int = 0;
    dt_mpc_fast = gait::dt_mpc_fast; 
    mpc_fast_frequency = (int) round(1/dt_mpc_fast);

    
    n_t_int = (int) floor(dt_mpc_slow /dt_mpc_fast);  


    _mass = gait::mass;
    _j_ini = gait::J_ini;

    _Zsc = 0;
    _ggg = gait::g;

    ros::Rate rate(mpc_fast_frequency);

    rpympc.Initialize();

    _stopwalking = false;
    foorpr_gen.setZero();
    foorpr_kmp.setZero();
    
    // for (int jjj=0; jjj<5; jjj++ )
    // {
    //   foorpr_gen(1+6*jjj,0) = - gait::RobotParaClass_HALF_HIP_WIDTH;
    //   foorpr_gen(4+6*jjj,0) = gait::RobotParaClass_HALF_HIP_WIDTH;   
    // }
    
    // foorpr_kmp(1) = - gait::RobotParaClass_HALF_HIP_WIDTH;
    // foorpr_kmp(4) = gait::RobotParaClass_HALF_HIP_WIDTH;
    
    foortheta_gen.setZero();
    _height_offset_timex = gait::time_set;
    
    ros::Duration duratione_des(dt_mpc_fast);
    while (ros::ok())
    {   

	      ros::Time start = ros::Time::now(); 
	    
        //// note that the control_gait is obtained from the main_control_loop. to test the function of rt_mpc_qp node, set control_gait(0) = 1;
        //control_gait(0) = 1;
        if (control_gait(0)>0)
        {
            /// following block can be executed by main_control_loop
            if(mpc_gait_flag_old > mpc_gait_flag)
            {
                count_in_rt_loop_restart = count_in_rt_loopx;
                rpympc.Initialize();

            }
            count_in_rt_loopx += 1;
            count_in_rt_loop = count_in_rt_loopx - count_in_rt_loop_restart;
            

            t_int += (int) floor(count_in_rt_loop/n_t_int);

            state_feedback(0,0) = t_int;

            //////state_estiamtion: obtained by the dob algorithm;
            for (int jx = 0; jx<25; jx++)
            {
                state_to_MPC.position[jx] = state_feedback(jx,0);
            }        
            
            rt_to_nrt_pub_.publish(state_to_MPC);


            // /////////// trajectory interpolation  ///////////////////////// 
            if ( mpc_gait_flag>0) /// receieved data from the low MPC;
            {
                
                if(mpc_gait_flag_old > mpc_gait_flag)
                {
                    count_in_rt_mpc_restart = count_in_rt_mpcx;
                }                
                count_in_rt_mpcx += 1;

                count_in_rt_mpc = count_in_rt_mpcx - count_in_rt_mpc_restart;
                
                xget_position_interpolation();
		
                if ( count_in_rt_mpc * dt_mpc_fast > _height_offset_timex)
                {
                    int count_in_rt_mpc_foot;
                    count_in_rt_mpc_foot = count_in_rt_mpc - (int) _height_offset_timex/dt_mpc_fast;
                    
                  foorpr_gen = rpympc.Foot_trajectory_solve_mod2(count_in_rt_mpc_foot, _stopwalking,Nrtfoorpr_gen);
                  for (int jjj=0; jjj<5; jjj++ )
                  {
                    foorpr_gen(0+6*jjj,0) += low_mpc_gait(18);
                    foorpr_gen(1+6*jjj,0) += low_mpc_gait(19);
                    foorpr_gen(2+6*jjj,0) += low_mpc_gait(20);

                    foorpr_gen(3+6*jjj,0) += low_mpc_gait(21);
                    foorpr_gen(4+6*jjj,0) += low_mpc_gait(22);
                    foorpr_gen(5+6*jjj,0) += low_mpc_gait(23);    
                  }
                  foortheta_gen = rpympc.XGetSolution_Foot_rotation(count_in_rt_mpc_foot, dt_mpc_fast);
                  
                }
                
                if ((int) low_mpc_gait(27,0) % 2 == 0)           //odd:left support
                {
                    _Zsc = lfoot_inter(2);
                    zmpxyz_ref(2) = _Zsc;
                }
                else
                {
                    _Zsc = rfoot_inter(2);
                    zmpxyz_ref(2) = _Zsc;
                }

                for (int jxx=0; jxx<5; jxx++)
                {
                    if (jxx ==0)
                    {
                        zmp_mpc_ref(0,jxx) = zmp_inter(0,0); 
                        zmp_mpc_ref(1,jxx) = zmp_inter(1,0);
                        
			                  // /// using the data from nrt_mpc
                        // rfoot_mpc_ref(0,jxx) = rfoot_inter(0,0); 
                        // rfoot_mpc_ref(1,jxx) = rfoot_inter(1,0); 
                        // lfoot_mpc_ref(0,jxx) = lfoot_inter(0,0); 
                        // lfoot_mpc_ref(1,jxx) = lfoot_inter(1,0); 
                        // body_thetax(0) = bodyangle_mpc_ref(0,jxx) = (rftheta_inter(0,0) + lftheta_inter(0,0))/5;
                        // body_thetax(1) = bodyangle_mpc_ref(1,jxx) = (rftheta_inter(1,0) + lftheta_inter(1,0))/5;
			

			                   
                        rfoot_mpc_ref(0,jxx) = foorpr_gen(jxx*6);
                        rfoot_mpc_ref(0,jxx) = foorpr_gen(jxx*6+1);
                        lfoot_mpc_ref(0,jxx) = foorpr_gen(jxx*6+3); 
                        lfoot_mpc_ref(1,jxx) = foorpr_gen(jxx*6+4); 			

                        // body_thetax(0) = bodyangle_mpc_ref(0,jxx) = (foortheta_gen(jxx*6,0) + foortheta_gen(jxx*6+3,0))/5;
                        // body_thetax(1) = bodyangle_mpc_ref(1,jxx) = (foortheta_gen(jxx*6+1,0) + foortheta_gen(jxx*6+4,0))/5;	
                        body_thetax(2) = (foortheta_gen(jxx*6+2,0) + foortheta_gen(jxx*6+5,0))/2;		
			
                        comacc_mpc_ref(2,jxx) = comacc_inter(2,0); 
                    }
                    else
                    {
                        zmp_mpc_ref(0,jxx) = zmp_inter(8+3*jxx-2,0); 
                        zmp_mpc_ref(1,jxx) = zmp_inter(8+3*jxx-1,0);
                        
                        // /// using the data from nrt_mpc
                        // rfoot_mpc_ref(0,jxx) = rfoot_inter(8+3*jxx-2,0); 
                        // rfoot_mpc_ref(1,jxx) = rfoot_inter(8+3*jxx-1,0); 
                        // lfoot_mpc_ref(0,jxx) = lfoot_inter(8+3*jxx-2,0); 
                        // lfoot_mpc_ref(1,jxx) = lfoot_inter(8+3*jxx-1,0);

                        rfoot_mpc_ref(0,jxx) = foorpr_gen(jxx*6);
                        rfoot_mpc_ref(0,jxx) = foorpr_gen(jxx*6+1);
                        lfoot_mpc_ref(0,jxx) = foorpr_gen(jxx*6+3); 
                        lfoot_mpc_ref(1,jxx) = foorpr_gen(jxx*6+4); 

                        // bodyangle_mpc_ref(0,jxx) = (foortheta_gen(jxx*6,0) + foortheta_gen(jxx*6+3,0))/5;
                        // bodyangle_mpc_ref(1,jxx) = (foortheta_gen(jxx*6+1,0) + foortheta_gen(jxx*6+4,0))/5; 

                        comacc_mpc_ref(2,jxx) = comacc_inter(8+3*jxx,0);          
                    }
                }


                ///// Body inclination mpc ////
                bodyangle_mpc = rpympc.body_theta_mpc(count_in_rt_mpc, bodyangle_state, zmp_mpc_ref,bodyangle_mpc_ref, rfoot_mpc_ref,lfoot_mpc_ref,comacc_mpc_ref,Nrtfoorpr_gen);


            }
        }
        else
        {
            // cout<<"fast mpc awaiting the rtcontrol_command!!"<<endl;
        }
        

        
        
        low_mpc_gait_inte(0,0) = rpy_mpc_body(0);
        low_mpc_gait_inte(1,0) = rpy_mpc_body(1);
        low_mpc_gait_inte(2,0) = rpy_mpc_body(2);
        // low_mpc_gait_inte(3,0) = body_thetax(0);
        // low_mpc_gait_inte(4,0) = body_thetax(1);
        low_mpc_gait_inte(3,0) = bodyangle_mpc(0);
        low_mpc_gait_inte(4,0) = bodyangle_mpc(1);        
        low_mpc_gait_inte(5,0) = body_thetax(2);
	

        //// polynomial leg trajectory
        low_mpc_gait_inte(6,0) = foorpr_gen(3);
        low_mpc_gait_inte(7,0) = foorpr_gen(4);
        low_mpc_gait_inte(8,0) = foorpr_gen(5);
        low_mpc_gait_inte(9,0) = foorpr_gen(0);
        low_mpc_gait_inte(10,0) = foorpr_gen(1);
        low_mpc_gait_inte(11,0) = foorpr_gen(2);
	
        // // //// kmp leg trajectory
        // low_mpc_gait_inte(6,0) = foorpr_kmp(3);
        // low_mpc_gait_inte(7,0) = foorpr_kmp(4);
        // low_mpc_gait_inte(8,0) = foorpr_kmp(5);
        // low_mpc_gait_inte(9,0) = foorpr_kmp(0);
        // low_mpc_gait_inte(10,0) = foorpr_kmp(1);
        // low_mpc_gait_inte(11,0) = foorpr_kmp(2);	
        if((low_mpc_gait_inte(6,0) - lfootx)/dt_mpc_fast > footvx_max)
        {
          low_mpc_gait_inte(6,0) = lfootx + footvx_max*dt_mpc_fast;
        }
        else
        {
          if((low_mpc_gait_inte(6,0) - lfootx)/dt_mpc_fast < footvx_min)
          {
            low_mpc_gait_inte(6,0) = lfootx + footvx_min*dt_mpc_fast;
          }
        }
        if((low_mpc_gait_inte(7,0) - lfooty)/dt_mpc_fast > footvy_max)
        {
          low_mpc_gait_inte(7,0) = lfooty + footvy_max*dt_mpc_fast;
        }
        else
        {
          if((low_mpc_gait_inte(7,0) - lfooty)/dt_mpc_fast < footvy_min)
          {
            low_mpc_gait_inte(7,0) = lfooty + footvy_min*dt_mpc_fast;
          }
        }
        if((low_mpc_gait_inte(8,0) - lfootz)/dt_mpc_fast > footvz_max)
        {
          low_mpc_gait_inte(8,0) = lfootz + footvz_max*dt_mpc_fast;
        }
        else
        {
          if((low_mpc_gait_inte(8,0) - lfootz)/dt_mpc_fast < footvz_min)
          {
            low_mpc_gait_inte(8,0) = lfootz + footvz_min*dt_mpc_fast;
          }
        }
        if((low_mpc_gait_inte(9,0) - rfootx)/dt_mpc_fast > footvx_max)
        {
          low_mpc_gait_inte(9,0) = rfootx + footvx_max*dt_mpc_fast;
        }
        else
        {
          if((low_mpc_gait_inte(9,0) - rfootx)/dt_mpc_fast < footvx_min)
          {
            low_mpc_gait_inte(9,0) = rfootx + footvx_min*dt_mpc_fast;
          }
        }
        if((low_mpc_gait_inte(10,0) - rfooty)/dt_mpc_fast > footvy_max)
        {
          low_mpc_gait_inte(10,0) = rfooty + footvy_max*dt_mpc_fast;
        }
        else
        {
          if((low_mpc_gait_inte(10,0) - rfooty)/dt_mpc_fast < footvy_min)
          {
            low_mpc_gait_inte(10,0) = rfooty + footvy_min*dt_mpc_fast;
          }
        }
        if((low_mpc_gait_inte(11,0) - rfootz)/dt_mpc_fast > footvz_max)
        {
          low_mpc_gait_inte(11,0) = rfootz + footvz_max*dt_mpc_fast;
        }
        else
        {
          if((low_mpc_gait_inte(11,0) - rfootz)/dt_mpc_fast < footvz_min)
          {
            low_mpc_gait_inte(11,0) = rfootz + footvz_min*dt_mpc_fast;
          }
        }        

        lfootx = low_mpc_gait_inte(6,0);
        lfooty = low_mpc_gait_inte(7,0); 
        lfootz = low_mpc_gait_inte(8,0);
        rfootx = low_mpc_gait_inte(9,0); 
        rfooty = low_mpc_gait_inte(10,0);
        rfootz = low_mpc_gait_inte(11,0);	
        
        low_mpc_gait_inte(12,0) = zmp_inter(0);
        low_mpc_gait_inte(13,0) = zmp_inter(1);
        low_mpc_gait_inte(14,0) = zmpxyz_ref(2);
        low_mpc_gait_inte(15,0) = F_L(0);
        low_mpc_gait_inte(16,0) = F_L(1);
        low_mpc_gait_inte(17,0) = F_L(2);
        low_mpc_gait_inte(18,0) = F_R(0);
        low_mpc_gait_inte(19,0) = F_R(1);
        low_mpc_gait_inte(20,0) = F_R(2);
        low_mpc_gait_inte(21,0) = M_L(0);
        low_mpc_gait_inte(22,0) = M_L(1);
        low_mpc_gait_inte(23,0) = M_L(2);
        low_mpc_gait_inte(24,0) = M_R(0);
        low_mpc_gait_inte(25,0) = M_R(1);
        low_mpc_gait_inte(26,0) = M_R(2);
        low_mpc_gait_inte(27,0) = low_mpc_gait(27,0);
        
//         low_mpc_gait_inte(28,0) = lftheta_inter(0);
//         low_mpc_gait_inte(29,0) = lftheta_inter(1);
//         low_mpc_gait_inte(30,0) = lftheta_inter(2);
//         low_mpc_gait_inte(31,0) = rftheta_inter(0);
//         low_mpc_gait_inte(32,0) = rftheta_inter(1);
//         low_mpc_gait_inte(33,0) = rftheta_inter(2);
        
        low_mpc_gait_inte(28,0) = foortheta_gen(3);
        low_mpc_gait_inte(29,0) = foortheta_gen(4);
        low_mpc_gait_inte(30,0) = foortheta_gen(5);
        low_mpc_gait_inte(31,0) = foortheta_gen(0);
        low_mpc_gait_inte(32,0) = foortheta_gen(1);
        low_mpc_gait_inte(33,0) = foortheta_gen(2);  

        low_mpc_gait_inte(34,0) = dcm_inter(0);
        low_mpc_gait_inte(35,0) = dcm_inter(1); 
        low_mpc_gait_inte(36,0) = bodyangle_mpc(0); 
        low_mpc_gait_inte(37,0) = bodyangle_mpc(1); 
        low_mpc_gait_inte(38,0) = bodyangle_mpc(2); 
        low_mpc_gait_inte(39,0) = bodyangle_mpc(3); 
        low_mpc_gait_inte(40,0) = bodyangle_mpc(4); 
        low_mpc_gait_inte(41,0) = bodyangle_mpc(5); 
        low_mpc_gait_inte(42,0) = bodyangle_mpc(6); 
        low_mpc_gait_inte(43,0) = bodyangle_mpc(7); 
        low_mpc_gait_inte(44,0) = bodyangle_mpc(8); 
        low_mpc_gait_inte(45,0) = bodyangle_mpc(9); 
        low_mpc_gait_inte(46,0) = bodyangle_mpc(10); 
        low_mpc_gait_inte(47,0) = bodyangle_mpc(11); 
        low_mpc_gait_inte(48,0) = bodyangle_mpc(12); 
        low_mpc_gait_inte(49,0) = bodyangle_mpc(13);         

	
        ros::Time end = ros::Time::now(); 
        
        ros::Duration duration = end -start;
        
        
        double t_fast_mpc = duration.toSec();	
	
        low_mpc_gait_inte(50,0) = t_fast_mpc;  

        for (int jx = 0; jx<36; jx++)
        {
          joint2simulationx.position[jx] = low_mpc_gait(jx,0);
        }

        for (int jx = 36; jx<=86; jx++)
        {
          joint2simulationx.position[jx] = low_mpc_gait_inte(jx-36,0);
        }  
        
        
        joint2simulationx.position[98] = (int) rpympc._tx_total/dtx;
        
        joint2simulationx.position[99] = count_in_rt_loop;
        
	
	
        joint2simulationx.header.stamp = ros::Time::now();
        rt_gait_pub_.publish(joint2simulationx);
         


        ros::Time end1 = ros::Time::now();
        ros::Duration duration1 = end1 -start;
        	
        
        ros::spinOnce();

        rate.sleep();

    }


    return 0;
}
