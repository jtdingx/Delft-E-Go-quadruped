/*****************************************************************************
NLPRTControlClass.cpp

Description:    source file of NLPRTControlClass

@Version:   1.0
@Author:    Jiatao Ding
@Release:   Tue 27 Jun 2017 09:33:32 AM CEST
@Update:    Tue 27 Jun 2017 09:33:37 AM CEST
*****************************************************************************/
#include "NLPRTControl/NLPRTControlClass.h"
#include "/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/go1_rt_control/src/Robotpara/robot_const_para_config.h"
#include "NLP/NLPClass.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>
//#include "execution_timer.h"


using namespace Eigen;
using namespace std;

NLPRTControlClass::NLPRTControlClass()
{
    /// mpc flag: not using for the NLP metho
    nlp._method_flag = 2;//strategy: 0: reactive step; 1: reactive step+ body inclination; 2: reactive step+ body inclination+height variation;	
    
    /// KMP flag
    nlp._pvFlag_kmp = 1; /// 1: for pos and vel generration, 0: pos generation

    std::string RobotPara_name = "go1";	
    RobotPara_totalmass = 12;
    RobotPara_HALF_HIP_WIDTH = gait::RobotParaClass_HALF_HIP_WIDTH;
    
    RobotPara_dt = dt_nlp;
    
    RobotPara_Tstep = gait::t_period;
    RobotPara_Z_C = gait::RobotPara_Z_C;
    RobotPara_g = 9.8;
    RobotPara_FOOT_WIDTH = 0.03;   


    nlp._robot_name = RobotPara_name;
    nlp._robot_mass = RobotPara_totalmass;

    nlp._lift_height = 0.035;

    nlp.RobotPara_totalmass = RobotPara_totalmass;
    nlp.RobotPara_HALF_HIP_WIDTH = RobotPara_HALF_HIP_WIDTH;
    nlp.RobotPara_dt = RobotPara_dt;
    nlp.RobotPara_Tstep = RobotPara_Tstep;
    nlp.RobotPara_Z_C = RobotPara_Z_C;
    nlp.RobotPara_g = RobotPara_g;
    nlp.RobotPara_FOOT_WIDTH = RobotPara_FOOT_WIDTH;
    



    // initialization
    // input step parameters
    stepwidthinput = RobotPara_HALF_HIP_WIDTH*2; 
    if (RobotPara_name == "coman")
    {
        steplengthinput = 0.08;
    }
    else if (RobotPara_name == "go1")
    {
      steplengthinput = 0.05;
      //steplengthinput = 0.1;
    }
    else if (RobotPara_name == "cogimon")
    {
        steplengthinput = 0.15;
    } 
    //   else
    //   {DPRINTF("Errorrrrrrrr for IK\n");}
 	  
    nlp.FootStepInputs(stepwidthinput, steplengthinput, stepheightinput);

    // offline initialization
    nlp.Initialize();


    _t_int = 0;
    _t_walkdtime_flag = 0;
    _t_walkdtime_restart_flag = 0;
    _walkdtime1 =0;

    _stop_walking = false;
    _start_walking_again = false;

    // loop numbern generation
    _dtx = RobotPara_dt;    
    _walkdtime_max = nlp.Get_maximal_number(_dtx)+1;
    _wal_max = _walkdtime_max;

    _flag_walkdtime.setZero(_walkdtime_max);
    _stop_flag_walkdtime.setZero(_walkdtime_max);

    _estimated_state.setZero();
    _estimated_state_global.setZero(19,_walkdtime_max);

    _Rfoot_location_feedback.setZero();
    _Rfoot_location_feedback(1) = - RobotPara_HALF_HIP_WIDTH;
    _Lfoot_location_feedback.setZero(); 
    _Lfoot_location_feedback(1) = RobotPara_HALF_HIP_WIDTH;

    _state_generate_interpo.setZero();


    _COM_IN.setZero(3,_walkdtime_max);
    _COM_IN(2,0) = RobotPara_Z_C-_height_offsetx;
    _COM_IN(2,1) = RobotPara_Z_C-_height_offsetx; 
    _body_IN.setZero(3,_walkdtime_max);
    _FootR_IN.setZero(3,_walkdtime_max);
    _FootR_IN(1,0) = -RobotPara_HALF_HIP_WIDTH;
    _FootR_IN(1,1) = -RobotPara_HALF_HIP_WIDTH;
    _FootL_IN.setZero(3,_walkdtime_max);	
    _FootL_IN(1,0) = RobotPara_HALF_HIP_WIDTH;
    _FootL_IN(1,1) = RobotPara_HALF_HIP_WIDTH; 


    //// parameters for local coordinate  
    _ppx = 0.01;  _pdx = 0.0005;     _pix = 0.00001;
    _ppy = 0.01;  _pdy = 0.00001;    _piy = 0.0000001;
    _ppz = 0.01;  _pdz = 0.00001;   _piz = 0.00000001; 

    _ppthetax= 0.01; _pdthetax =0.00001; _pithetax =0.0001;
    _ppthetay= 0.1; _pdthetay = 0.00001;_pithetax =0.0001; 
    _ppthetaz= 0.1; _pdthetaz = 0.001;_pithetaz =0.0001;     

    _error_com_position.setZero(3);
    _error_torso_angle.setZero(3);

    _feedback_lamda = 0;

    /////*************leg***************************//////
    _kmp_leg_traje.setZero();  


    _F_r_nlp.setZero();
    _F_l_nlp.setZero();
    _M_r_nlp.setZero();
    _M_l_nlp.setZero();
    _ZMP_relax_nlp.setZero();


    COM_in1.setZero();  COM_in2.setZero();  COM_in3.setZero(); 
    body_in1.setZero();  body_in2.setZero();  body_in3.setZero(); 
    FootL_in1.setZero();  FootL_in2.setZero();  FootL_in3.setZero(); 
    FootR_in1.setZero();  FootR_in2.setZero();  FootR_in3.setZero();   



    IsStartWalk = true;
    PelvisPos.setZero();
    body_thetax.setZero();
    LeftFootPosx.setZero();
    RightFootPosx.setZero();
    zmp_ref.setZero();
    
    PelvisPos(2) = RobotPara_Z_C;
    RightFootPosx(1) = -RobotPara_HALF_HIP_WIDTH;
    LeftFootPosx(1) = RobotPara_HALF_HIP_WIDTH;   
    
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
    
}



Eigen::Matrix<double,100,1> NLPRTControlClass::WalkingReactStepping(int walkdtime, bool start_mpc ,Eigen::Matrix<double,18,1> estimated_statex, 
                                                                    Eigen::Vector3d _Rfoot_location_feedbackx, Eigen::Vector3d _Lfoot_location_feedbackx)
{
  
// this is the loop for normal walking
  _walkdtime1 = walkdtime - _t_walkdtime_restart_flag;
  // cout << "walkdtime1:"<<_walkdtime1<<endl;
  IsStartWalk = start_mpc;
  
  if(IsStartWalk)
  {
      
    if (!_start_walking_again)
    {
      //_state_generate_interpo(99,0) = _walkdtime1;
      
      if (_walkdtime1*_dtx<=_height_offset_time)
      {
        CoM_squat = nlp.X_CoM_position_squat(_walkdtime1, _dtx); 
        PelvisPos(0) = CoM_squat(0);
        PelvisPos(1) = CoM_squat(1);
        PelvisPos(2) = CoM_squat(2);
        PelvisPosv(0) = CoM_squat(3);
        PelvisPosv(1) = CoM_squat(4);
        PelvisPosv(2) = CoM_squat(5);
        PelvisPosa(0) = CoM_squat(6);
        PelvisPosa(1) = CoM_squat(7);
        PelvisPosa(2) = CoM_squat(8);        
        
        ZMPxy_realx = zmp_ref = (LeftFootPosx+RightFootPosx)/2;
        F_R.setZero();
        F_L.setZero(); 
        F_R(2) = 9.8/2*RobotPara_totalmass;
        F_L(2) = 9.8/2*RobotPara_totalmass;
        M_R.setZero();
        M_L.setZero();
        
        body_thetax.setZero();	  
        LeftFootPosx(0) =0;    LeftFootPosx(1) = RobotPara_HALF_HIP_WIDTH;   LeftFootPosx(2) =0;
        RightFootPosx(0) = 0;  RightFootPosx(1) = -RobotPara_HALF_HIP_WIDTH; RightFootPosx(2) = 0;
                    
        zmp_ref = dcm_ref = (LeftFootPosx+RightFootPosx)/2;
        mpc_body(13) = zmp_ref(0);
        mpc_body(14) = zmp_ref(1);
        mpc_body(15) = dcm_ref(0);
        mpc_body(16) = dcm_ref(1);
        
        j_count = 1;
        bjx1 = 1;
        
        tx = 1;
        td = 0;	        
      }
      else
      {   
        _walkdtime1 -= (int)_height_offset_time/_dtx;
        if(_walkdtime1 < _walkdtime_max)
        {
            _t_walkdtime_flag = _walkdtime1;	  		
            _t_int = _walkdtime1;		

            rt_nlp_gait(estimated_statex, _Rfoot_location_feedbackx,_Lfoot_location_feedbackx); 
        }
        else  //walking beyond time counter
        {
          right_support = 2;
            IsStartWalk = false;
            _t_walkdtime_restart_flag = walkdtime;	
            mpc_stop = 2;	
            cout << "slow nlp over"<<endl;
        }            
      }
    }
  }
  else
  { 
    /////keep the current state:double support phase		
    ZMPxy_realx = zmp_ref =(LeftFootPosx+RightFootPosx)/2;
    F_R.setZero();
    _F_r_nlp.setZero();
    F_L.setZero(); 
    _F_l_nlp.setZero();
    F_R(2) = _F_r_nlp(2) = 9.8/2*RobotPara_totalmass;
    F_L(2) = _F_l_nlp(2) = 9.8/2*RobotPara_totalmass;
    M_R.setZero();
    M_L.setZero();
    
    j_count = 1;
    bjx1 = 1;
    
    tx = 1;
    td = 0;	
    cout << "slow nlp not start"<<endl;
    
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
    _state_generate_interpo(12,0) = zmp_ref(0);
    _state_generate_interpo(13,0) = zmp_ref(1);
    _state_generate_interpo(14,0) = zmp_ref(2);
    _state_generate_interpo(15,0) = F_L(0);
    _state_generate_interpo(16,0) = F_L(1);
    _state_generate_interpo(17,0) = F_L(2);
    _state_generate_interpo(18,0) = F_R(0);
    _state_generate_interpo(19,0) = F_R(1);
    _state_generate_interpo(20,0) = F_R(2);
    _state_generate_interpo(21,0) = M_L(0);
    _state_generate_interpo(22,0) = M_L(1);
    _state_generate_interpo(23,0) = M_L(2);
    _state_generate_interpo(24,0) = M_R(0);
    _state_generate_interpo(25,0) = M_R(1);
    _state_generate_interpo(26,0) = M_R(2);
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
    _state_generate_interpo(94,0) = mpc_body(35);     
    _state_generate_interpo(95,0) = mpc_body(36);  
    _state_generate_interpo(96,0) = mpc_body(37);    
    _state_generate_interpo(97,0) = mpc_stop;
    _state_generate_interpo(99,0) = right_support;
  
  return _state_generate_interpo;  
 
}



void NLPRTControlClass::StartWalking()
{
  
  IsStartWalk = true;
  if (_stop_walking)
  {
    _start_walking_again = true;
    //DPRINTF("========= start walking again. =============\n");     
  }
  else
  {
    //DPRINTF("========= start walking-first time =============\n");   
  }
   _stop_walking = false;
  
}

void NLPRTControlClass::StopWalking()
{

    if (_t_int <10)
    {
     IsStartWalk = false;  
    }
    else
    {     
      IsStartWalk = true;
      _stop_walking = true;  
      //DPRINTF("========= stop walking enable=============\n");          
    }
      
      
  
}


void NLPRTControlClass::rt_nlp_gait(Eigen::Matrix<double,18,1> estimated_statex, Eigen::Vector3d _Rfoot_location_feedback, 
                                    Eigen::Vector3d _Lfoot_location_feedback)
{
    if (_t_int >=1)
    {
        _flag_walkdtime(_walkdtime1) = _t_int;

        if (_flag_walkdtime(_walkdtime1 -1) < _t_int)
        {	
            mpc_body = nlp.step_timing_opti_loop(_t_int,_estimated_state,_Rfoot_location_feedback,
                                                 _Lfoot_location_feedback,_feedback_lamda,_stop_walking);
            
            PelvisPos(0) = mpc_body(0);
            PelvisPos(1) = mpc_body(1);
            PelvisPos(2) = mpc_body(2);
            PelvisPosv(0) = mpc_body(3);
            PelvisPosv(1) = mpc_body(4);
            PelvisPosv(2) = mpc_body(5);
            PelvisPosa(0) = mpc_body(6);
            PelvisPosa(1) = mpc_body(7);
            PelvisPosa(2) = mpc_body(8);             
            
            //nlp.Foot_trajectory_solve(_t_int, _stop_walking);	
            mpc_rlfoot_traj = nlp.Foot_trajectory_solve_mod2(_t_int, _stop_walking); 
            RightFootPosx(0) = mpc_rlfoot_traj(0,0);  RightFootPosx(1) = mpc_rlfoot_traj(1,0); RightFootPosx(2) = mpc_rlfoot_traj(2,0);	
            LeftFootPosx(0) = mpc_rlfoot_traj(3,0);   LeftFootPosx(1) = mpc_rlfoot_traj(4,0);  LeftFootPosx(2) = mpc_rlfoot_traj(5,0);            
            ///////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   
        }
    }  

    if (_walkdtime1>=2){
        COM_in1 = _COM_IN.col(_walkdtime1-2);
        COM_in2 = _COM_IN.col(_walkdtime1-1);
        COM_in3 = _COM_IN.col(_walkdtime1);

        body_in1 = _body_IN.col(_walkdtime1-2);
        body_in2 = _body_IN.col(_walkdtime1-1);
        body_in3 = _body_IN.col(_walkdtime1);

        FootL_in1 = _FootL_IN.col(_walkdtime1-2);
        FootL_in2 = _FootL_IN.col(_walkdtime1-1);
        FootL_in3 = _FootL_IN.col(_walkdtime1);	

        FootR_in1 = _FootR_IN.col(_walkdtime1-2);
        FootR_in2 = _FootR_IN.col(_walkdtime1-1);
        FootR_in3 = _FootR_IN.col(_walkdtime1);		  

    }
    else{
        COM_in1.setZero();
        COM_in2.setZero();
        COM_in3 = _COM_IN.col(_walkdtime1);	

        body_in1.setZero();
        body_in2.setZero();
        body_in3 = _body_IN.col(_walkdtime1);	

        FootL_in1.setZero();
        FootL_in2.setZero();
        FootL_in3 = _FootL_IN.col(_walkdtime1);

        FootR_in1.setZero();
        FootR_in2.setZero();
        FootR_in3 = _FootR_IN.col(_walkdtime1);			  

    }

    // // polynomial swing leg trajectory
    //   comxyzx = PelvisPos = nlp.XGetSolution_CoM_position(_walkdtime1, _dtx,COM_in1,COM_in2,COM_in3);              
    //   thetaxyx = body_thetax = nlp.XGetSolution_body_inclination(_walkdtime1, _dtx, body_in1,body_in2,body_in3);	  
    //   Lfootxyzx = LeftFootPosx = nlp.XGetSolution_Foot_positionL(_walkdtime1, _dtx, FootL_in1,FootL_in2,FootL_in3);
    //   Rfootxyzx = RightFootPosx = nlp.XGetSolution_Foot_positionR(_walkdtime1, _dtx, FootR_in1,FootR_in2,FootR_in3);

    //cout<<"polynomial"<<endl;
    // /////*****************leg trajectory generated by KMP********************************/////
    // if (nlp._bjx1>=2)
    // {
    //   if (nlp._pvFlag_kmp > 0)
    //   {
	  //     _kmp_leg_traje = nlp.XGetSolution_Foot_position_KMP(_walkdtime1,_dtx,_t_int);
    //   }
    //   else
    //   {
	  //     _kmp_leg_traje = nlp.XGetSolution_Foot_position_KMP_faster(_walkdtime1,_dtx,_t_int);
    //   }
        		
    //   RightFootPosx(0) = _kmp_leg_traje(0);  RightFootPosx(1) = _kmp_leg_traje(1); RightFootPosx(2) = _kmp_leg_traje(2);	
    //   LeftFootPosx(0) = _kmp_leg_traje(3);   LeftFootPosx(1) = _kmp_leg_traje(4);  LeftFootPosx(2) = _kmp_leg_traje(5);
    //   Lfootxyzx = LeftFootPosx;   Rfootxyzx = RightFootPosx;        
    // }


    
    // ///// foot rpy ====================
    leg_rpy = nlp.XGetSolution_Foot_rotation(_walkdtime1,_dtx,_t_int);
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
    
    right_support = nlp.right_support;

    nlp.Zmp_distributor(_walkdtime1,_dtx);

    ZMPxy_realx = zmp_ref = nlp._ZMPxy_realx;
    F_R = _F_r_nlp = nlp._F_R;
    F_L = _F_l_nlp = nlp._F_L;
    M_R = _M_r_nlp = nlp._M_R;
    M_L = _M_l_nlp = nlp._M_L;
    
    zmp_ref(0) = mpc_body(9);
    zmp_ref(1) = mpc_body(10);
    dcm_ref(0) = mpc_body(11);
    dcm_ref(1) = mpc_body(12);    
    

    _ZMP_relax_nlp = nlp._zmp_rela_vari;

    j_count = nlp._j_count;
    bjx1 = nlp._bjx1;
    if (bjx1>=1)
    {		    
        tx = nlp._tx(bjx1-1);
        td = nlp._td(bjx1-1);			  
    }
    else
    {		    
        tx = 0;
        td = 0;			  
    }

    
    // ////// foot location modifiation considering rotation//////
    // double rh = fabs(tan(RightFootRPY(0,0))) *RobotPara_FOOT_WIDTH/2;
    // double lh = fabs(tan(LeftFootRPY(0,0))) * RobotPara_FOOT_WIDTH/2;
    // RightFootPosx(2,0) += rh;
    // LeftFootPosx(2,0) += lh;

    // store
    _COM_IN(0,_walkdtime1) = PelvisPos(0);
    _COM_IN(1,_walkdtime1) = PelvisPos(1);
    _COM_IN(2,_walkdtime1) = PelvisPos(2);
    _body_IN(0,_walkdtime1) = thetaxyx(0);
    _body_IN(1,_walkdtime1) = thetaxyx(1);
    _body_IN(2,_walkdtime1) = thetaxyx(2);	  
    _FootL_IN(0,_walkdtime1) = Lfootxyzx(0);
    _FootL_IN(1,_walkdtime1) = Lfootxyzx(1);
    _FootL_IN(2,_walkdtime1) = Lfootxyzx(2);	
    _FootR_IN(0,_walkdtime1) = Rfootxyzx(0);
    _FootR_IN(1,_walkdtime1) = Rfootxyzx(1);
    _FootR_IN(2,_walkdtime1) = Rfootxyzx(2);

}




