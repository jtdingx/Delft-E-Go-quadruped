/*****************************************************************************
MpcRTControlClass.cpp

Description:    source file of MpcRTControlClass

@Version:   1.0
@Author:    Jiatao ding (jtdingx@gmail.com)
@Release:   Tue 27 Jun 2017 09:33:32 AM CEST
@Update:    Tue 27 Jun 2017 09:33:37 AM CEST
*****************************************************************************/
#include "NLPRTControl/MpcRTControlClass.h"
#include "NLP/MPCClass.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>
#include <ros/ros.h>
#include "yaml.h"
#include <ros/package.h> 
#include <geometry_msgs/Twist.h>  
//#include "/usr/include/yaml-cpp/binary.h"



using namespace Eigen;
using namespace std;

MpcRTControlClass::MpcRTControlClass()
{

    config_set();
    std::string RobotPara_name = "go1";	
    gait_mode = 102; ///trotting; 101: pacing
    RobotPara_HALF_HIP_WIDTH = gait::RobotParaClass_HALF_HIP_WIDTH;
    RobotPara_dt = dt_mpc;
    RobotPara_g = clear_height;
    RobotPara_FOOT_WIDTH = gait::RobotPara_FOOT_WIDTH; 

    mpc._method_flag = 2;//for height opt:strategy: 0: reactive step; 1: reactive step+ body inclination; 2: reactive step+ body inclination+height variation;	
    mpc._robot_name = RobotPara_name;
    mpc._robot_mass = RobotPara_totalmass;
    mpc._lift_height = clear_height;
    mpc._gait_mode = gait_mode;
    mpc._hcom = RobotPara_Z_C;

  


  // initialization
  // input step parameters
  if(gait_mode!=102)
  {
    stepwidthinput = gait::RobotParaClass_HALF_HIP_WIDTH*2; 
  }
  
  // stepheightinput = 0.0;  	  
  mpc.FootStepInputs(stepwidthinput, steplengthinput, stepheightinput);
  
  // offline initialization
  mpc.Initialize();
    
  
  _refer_t_max = mpc.Get_maximal_number_reference();

 _t_int = 0;
  _t_walkdtime_flag = 0;
  _t_walkdtime_restart_flag = 0;
  _walkdtime1 =0;
 
 _stop_walking = false;
 _start_walking_again = false;
 
  // loop numbern generation
  _dtx = dt_mpc;    
  _walkdtime_max = mpc.Get_maximal_number(_dtx);
  _wal_max = _walkdtime_max;
  
    
  _estimated_state.setZero();
  
  _Rfoot_location_feedback.setZero();
  _Lfoot_location_feedback.setZero(); 
  
  _state_generate_interpo.setZero();

  _torso_angle.setZero();
   

//// parameters for local coordinate  
  _ppx = 0.01; _pdx = 0.0001;     _pix = 0.000001;
  _ppy = 0.01;  _pdy = 0.00001;    _piy = 0.0000001;
  _ppz = 0.01;  _pdz = 0.00001;   _piz = 0.00000001; 
  
  _ppthetax= 0.01; _pdthetax =0; _pithetax =0.0001;
  _ppthetay= 0.01; _pdthetay = 0;_pithetax =0.0001; 
  _ppthetaz= 0.1; _pdthetaz = 0.001;_pithetaz =0.0001;     
  
  
  _feedback_lamda = 0;

  IsStartWalk = false;


	PelvisPos.setZero();
	body_thetax.setZero();
	LeftFootPosx.setZero();
	RightFootPosx.setZero();
	zmp_ref.setZero();

	PelvisPos(2) = RobotPara_Z_C;
  PelvisPos_old.setZero();
  LeftFootPosx_old.setZero();
  RightFootPosx_old.setZero();


  if(gait_mode==102)
  {
    RightFootPosx(1) = 0;
    LeftFootPosx(1) = 0; 
  }
  else
  {
    RightFootPosx(1) = -RobotPara_HALF_HIP_WIDTH;
    LeftFootPosx(1) = RobotPara_HALF_HIP_WIDTH; 
  }

  

	PelvisPosv.setZero();
	PelvisPosa.setZero(); 


  LeftFootRPY.setZero();
  RightFootRPY.setZero();
  leg_rpy.setZero();
  dcm_ref.setZero();    

  mpc_rlfoot_traj.setZero(); 
  mpc_body.setZero();  
  CoM_squat.setZero();
  
  mpc_stop = 0;    

  right_support = 2;

  n_time_set = round(gait::time_set/dt_mpc);


}


Eigen::Matrix<double,100,1> MpcRTControlClass::WalkingReactStepping(int walkdtime, bool start_mpc ,Eigen::Matrix<double,18,1> estimated_statex, 
                                                                    Eigen::Vector3d _Rfoot_location_feedbackx, Eigen::Vector3d _Lfoot_location_feedbackx,
                                                                    double step_length_ref, double step_width_ref)

{
  /// update the maximal clock number;
  _walkdtime_max = mpc.Get_maximal_number(_dtx);



	_walkdtime1 = walkdtime;

  IsStartWalk = start_mpc;

	if(IsStartWalk)
	{
    if(_walkdtime1 <= n_time_set) ///adjust initial status;
    {
      j_count = 1;
      bjx1 = 1;
      tx = 1;
      td = 0;
    }
    else
    {
      _walkdtime1 = walkdtime - _t_walkdtime_restart_flag - n_time_set;
		///////normal walking ===============================
	      if(_walkdtime1 <= (_walkdtime_max))
	      {
          //// recycling:
          if(_walkdtime1 == (_walkdtime_max))  ///update the _t_walkdtime_restart_flag
          {
            _t_walkdtime_restart_flag += _walkdtime_max;
            cout<<"_t_walkdtime_restart_flag:"<<_t_walkdtime_restart_flag<<endl;
   
          }
      

          _t_int = _walkdtime1;
          if (_t_int >=0)
          {
              mpc_body = mpc.step_timing_opti_loop(_t_int, estimated_statex,_Rfoot_location_feedbackx,_Lfoot_location_feedbackx,
                                                   _feedback_lamda,_stop_walking,_t_walkdtime_restart_flag,_t_walkdtime_flag,
                                                   step_length_ref, step_width_ref);			    
              PelvisPos(0) = mpc_body(0)+PelvisPos_old(0);
              PelvisPos(1) = mpc_body(1)+PelvisPos_old(1);
              // PelvisPos(2) = mpc_body(2)+PelvisPos_old(2);
              PelvisPos(2) = mpc_body(2);              
              PelvisPosv(0) = mpc_body(3);
              PelvisPosv(1) = mpc_body(4);
              PelvisPosv(2) = mpc_body(5);
              PelvisPosa(0) = mpc_body(6);
              PelvisPosa(1) = mpc_body(7);
              PelvisPosa(2) = mpc_body(8);  					
              
              mpc_rlfoot_traj = mpc.Foot_trajectory_solve(_t_int, _stop_walking);	      
              RightFootPosx(0) = mpc_rlfoot_traj(0,0) + RightFootPosx_old(0);  
              RightFootPosx(1) = mpc_rlfoot_traj(1,0) + RightFootPosx_old(1); 
              RightFootPosx(2) = mpc_rlfoot_traj(2,0) + RightFootPosx_old(2);	
              LeftFootPosx(0) = mpc_rlfoot_traj(3,0) + LeftFootPosx_old(0);   
              LeftFootPosx(1) = mpc_rlfoot_traj(4,0) + LeftFootPosx_old(1);  
              LeftFootPosx(2) = mpc_rlfoot_traj(5,0) + LeftFootPosx_old(2);            
          }
          
          // ///// foot rpy ====================
          leg_rpy = mpc.XGetSolution_Foot_rotation(_walkdtime1,_dtx,_t_int);
          RightFootRPY(0) = leg_rpy(0,0);  RightFootRPY(1) = leg_rpy(1,0); RightFootRPY(2) = leg_rpy(2,0);	
          LeftFootRPY(0) = leg_rpy(3,0);   LeftFootRPY(1) = leg_rpy(4,0);  LeftFootRPY(2) = leg_rpy(5,0); 
          ///
          body_thetax(0) = (RightFootRPY(0) + LeftFootRPY(0));
          body_thetax(1) = (RightFootRPY(1) + LeftFootRPY(1));
          body_thetax(2) = (RightFootRPY(2) + LeftFootRPY(2));
          body_thetax(3) = (leg_rpy(9,0) + leg_rpy(6,0))/5;
          body_thetax(4) = (leg_rpy(10,0) + leg_rpy(7,0))/5;
          body_thetax(5) = (leg_rpy(11,0) + leg_rpy(8,0))/2;
          body_thetax(6) = (leg_rpy(15,0) + leg_rpy(12,0))/5;
          body_thetax(7) = (leg_rpy(16,0) + leg_rpy(13,0))/5;
          body_thetax(8) = (leg_rpy(17,0) + leg_rpy(14,0))/2;    
          
          right_support = mpc.right_support;
          
          zmp_ref(0) = mpc_body(9);
          zmp_ref(1) = mpc_body(10);
          dcm_ref(0) = mpc_body(11);
          dcm_ref(1) = mpc_body(12);    
          


          j_count = mpc._j_count;
          bjx1 = mpc._bjx1;
          if (bjx1>=1)
          {		    
            tx = mpc._tx(bjx1-1);
            td = mpc._td(bjx1-1);			  
          }
          else
          {		    
            tx = 0;
            td = 0;			  
          }
          _t_walkdtime_flag = _t_walkdtime_restart_flag;////last count
          //// recycling:
          if(_walkdtime1 == (_walkdtime_max))  ///update the position
          {
              PelvisPos_old(0) = PelvisPos(0);
              PelvisPos_old(1) = PelvisPos(1);
              PelvisPos_old(2) = PelvisPos(2);   
              RightFootPosx_old(0) = RightFootPosx(0);  
              RightFootPosx_old(1) = RightFootPosx(1); 
              RightFootPosx_old(2) = RightFootPosx(2);	
              LeftFootPosx_old(0) = LeftFootPosx(0);   
              LeftFootPosx_old(1) = LeftFootPosx(1);  
              LeftFootPosx_old(2) = LeftFootPosx(2); 
          }          
	      }
    }

  }
	else
	{
		j_count = 1;
		bjx1 = 1;
		
		tx = 1;
		td = 0;			
	}
	
    _state_generate_interpo(0,0) = PelvisPos(0);
    _state_generate_interpo(1,0) = PelvisPos(1);
    _state_generate_interpo(2,0) = PelvisPos(2);
    _state_generate_interpo(3,0) = body_thetax(0);
    _state_generate_interpo(4,0) = body_thetax(1);
    _state_generate_interpo(5,0) = body_thetax(2);
    _state_generate_interpo(6,0) = LeftFootPosx(0);
    _state_generate_interpo(7,0) = LeftFootPosx(1);
    _state_generate_interpo(8,0) = LeftFootPosx(2);
    _state_generate_interpo(9,0) = RightFootPosx(0);
    _state_generate_interpo(10,0) = RightFootPosx(1);
    _state_generate_interpo(11,0) = RightFootPosx(2);	  
    _state_generate_interpo(12,0) = (n_time_set+_t_walkdtime_restart_flag)*dt_mpc + mpc_body(38);  ////starting time of current step;
    _state_generate_interpo(13,0) = mpc_body(39);  ////reinitialization count;
    // _state_generate_interpo(12,0) = zmp_ref(0);
    // _state_generate_interpo(13,0) = zmp_ref(1);
    // _state_generate_interpo(14,0) = zmp_ref(2);
    // _state_generate_interpo(15,0) = F_L(0);
    // _state_generate_interpo(16,0) = F_L(1);
    // _state_generate_interpo(17,0) = F_L(2);
    // _state_generate_interpo(18,0) = F_R(0);
    // _state_generate_interpo(19,0) = F_R(1);
    // _state_generate_interpo(20,0) = F_R(2);
    // _state_generate_interpo(21,0) = M_L(0);
    // _state_generate_interpo(22,0) = M_L(1);
    // _state_generate_interpo(23,0) = M_L(2);
    // _state_generate_interpo(24,0) = M_R(0);
    // _state_generate_interpo(25,0) = M_R(1);
    // _state_generate_interpo(26,0) = M_R(2);
    _state_generate_interpo(27,0) = bjx1;
    _state_generate_interpo(28,0) = LeftFootRPY(0);
    _state_generate_interpo(29,0) = LeftFootRPY(1);
    _state_generate_interpo(30,0) = LeftFootRPY(2);
    _state_generate_interpo(31,0) = RightFootRPY(0);
    _state_generate_interpo(32,0) = RightFootRPY(1);
    _state_generate_interpo(33,0) = RightFootRPY(2);  
    _state_generate_interpo(34,0) = dcm_ref(0);
    _state_generate_interpo(35,0) = dcm_ref(1); 

    _state_generate_interpo(36,0) = PelvisPosv(0);
    _state_generate_interpo(37,0) = PelvisPosv(1);
    _state_generate_interpo(38,0) = PelvisPosv(2);
    _state_generate_interpo(39,0) = PelvisPosa(0);
    _state_generate_interpo(40,0) = PelvisPosa(1);
    _state_generate_interpo(41,0) = PelvisPosa(2);
    _state_generate_interpo(42,0) = mpc_body(13);  /// next two zmp 
    _state_generate_interpo(43,0) = mpc_body(14);
    _state_generate_interpo(44,0) = mpc_body(15);  /// next two dcm
    _state_generate_interpo(45,0) = mpc_body(16);    
    _state_generate_interpo(46,0) = mpc_rlfoot_traj(6);  /// foot velocity
    _state_generate_interpo(47,0) = mpc_rlfoot_traj(7);
    _state_generate_interpo(48,0) = mpc_rlfoot_traj(8);
    _state_generate_interpo(49,0) = mpc_rlfoot_traj(9);        
    _state_generate_interpo(50,0) = mpc_rlfoot_traj(10);          
    _state_generate_interpo(51,0) = mpc_rlfoot_traj(11);  
    _state_generate_interpo(52,0) = mpc_rlfoot_traj(12);  /// foot acceleration
    _state_generate_interpo(53,0) = mpc_rlfoot_traj(13);
    _state_generate_interpo(54,0) = mpc_rlfoot_traj(14);
    _state_generate_interpo(55,0) = mpc_rlfoot_traj(15);        
    _state_generate_interpo(56,0) = mpc_rlfoot_traj(16);          
    _state_generate_interpo(57,0) = mpc_rlfoot_traj(17);  

    _state_generate_interpo(58,0) = leg_rpy(6);  /// footrpy velocity
    _state_generate_interpo(59,0) = leg_rpy(7);
    _state_generate_interpo(60,0) = leg_rpy(8);
    _state_generate_interpo(61,0) = leg_rpy(9);        
    _state_generate_interpo(62,0) = leg_rpy(10);          
    _state_generate_interpo(63,0) = leg_rpy(11);  
    _state_generate_interpo(64,0) = leg_rpy(12);  /// footrpy acceleration
    _state_generate_interpo(65,0) = leg_rpy(13);
    _state_generate_interpo(66,0) = leg_rpy(14);
    _state_generate_interpo(67,0) = leg_rpy(15);        
    _state_generate_interpo(68,0) = leg_rpy(16);          
    _state_generate_interpo(69,0) = leg_rpy(17);  

    _state_generate_interpo(70,0) = body_thetax(3);  /// body_theata velocity
    _state_generate_interpo(71,0) = body_thetax(4);
    _state_generate_interpo(72,0) = body_thetax(5);
    _state_generate_interpo(73,0) = body_thetax(6);  /// body_theata acceleration    
    _state_generate_interpo(74,0) = body_thetax(7);          
    _state_generate_interpo(75,0) = body_thetax(8);  

    _state_generate_interpo(76,0) = mpc_body(17);  /// next three zmp 
    _state_generate_interpo(77,0) = mpc_body(18);
    _state_generate_interpo(78,0) = mpc_body(19);  /// next three dcm
    _state_generate_interpo(79,0) = mpc_body(20); 

    _state_generate_interpo(80,0) = mpc_body(21);  /// next two com acceleration 
    _state_generate_interpo(81,0) = mpc_body(22);
    _state_generate_interpo(82,0) = mpc_body(23);  
    _state_generate_interpo(83,0) = mpc_body(24);  /// next two com acceleration   
    _state_generate_interpo(84,0) = mpc_body(25);  
    _state_generate_interpo(85,0) = mpc_body(26);  
    //// generated step location/////
    _state_generate_interpo(86,0) = mpc_body(27); 
    _state_generate_interpo(87,0) = mpc_body(28);  
    _state_generate_interpo(88,0) = mpc_body(29);
    _state_generate_interpo(89,0) = mpc_body(30);  
    _state_generate_interpo(90,0) = mpc_body(31);  
    _state_generate_interpo(91,0) = mpc_body(32);  
    _state_generate_interpo(92,0) = mpc_body(33);      
    _state_generate_interpo(93,0) = mpc_body(34);  
    _state_generate_interpo(94,0) = mpc_body(35);   //// current step period/////   
    _state_generate_interpo(95,0) = mpc_body(36);  
    _state_generate_interpo(96,0) = mpc_body(37);    
    _state_generate_interpo(97,0) = mpc_stop;
    _state_generate_interpo(99,0) = right_support;

	return _state_generate_interpo;

}



void MpcRTControlClass::StartWalking()
{
  
  IsStartWalk = true;
  if (_stop_walking)
  {
    _start_walking_again = true;
  }

   _stop_walking = false;
  
}

void MpcRTControlClass::StopWalking()
{

    if (_t_int <10)
    {
//       _t_walkdtime_restart_flag = 0;
     IsStartWalk = false;  
/*     
     cout << "stop invalid"<<endl;  */
    }
    else
    {
      
      IsStartWalk = true;
      _stop_walking = true;  
    //   DPRINTF("========= stop walking enable=============\n");          
    }
      
  
}


void MpcRTControlClass::config_set()
{   
    /////load default parameter from the yaml.file
    ///////////////////  yaml code . ///////// 
    YAML::Node config = YAML::LoadFile("/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/config/config.yaml");
    // YAML::Node config = YAML::LoadFile("/home/jiatao/unitree_loco_catk/src/unitree_ros/go1_rt_control/config/config.yaml");

    dt_mpc =  config["dt_slow_mpc"].as<double>();
    steplengthinput = config["step_length"].as<double>();
    stepwidthinput = config["step_width"].as<double>();
    stepheightinput = config["step_height"].as<double>();
    clear_height = config["foot_clear_height"].as<double>();
    RobotPara_Tstep = config["t_period"].as<double>();
    RobotPara_Z_C = config["body_p_Homing_Retarget2"].as<double>(); 
    RobotPara_totalmass = config["total_mass"].as<double>(); 
    //std::cout<<"dt_mpc:"<<dt_mpc<<std::endl;


}



