#include <termios.h>  
#include <signal.h>  
#include <math.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <sys/poll.h>  
#include <fcntl.h>
#include <sensor_msgs/Joy.h>
#include "ros/ros.h"
      
#include <boost/thread/thread.hpp>  
#include <ros/ros.h>  
#include <geometry_msgs/Twist.h>  
      
#define KEYCODE_w 0x77  
#define KEYCODE_a 0x61  
#define KEYCODE_s 0x73  
#define KEYCODE_d 0x64
#define KEYCODE_x 0x78
#define KEYCODE_r 0x72
#define KEYCODE_SPACE 0x20   
#define KEYCODE_Esc 0x1B
#define KEYCODE_j 0x6A
#define KEYCODE_k 0x6B
#define KEYCODE_f 102
#define KEYCODE_n 110
#define KEYCODE_t 116

#define KEYCODE_W 87
#define KEYCODE_A 65  
#define KEYCODE_S 83  
#define KEYCODE_D 68
#define KEYCODE_Q 81
#define KEYCODE_E 69
#define KEYCODE_I 73   
#define KEYCODE_L 76
#define KEYCODE_J 74
#define KEYCODE_K 75
#define KEYCODE_U 85
#define KEYCODE_O 79

#define KEYCODE_0 48
#define KEYCODE_1 49
#define KEYCODE_2 50
#define KEYCODE_3 51
#define KEYCODE_4 52
#define KEYCODE_5 53
#define KEYCODE_6 54
#define KEYCODE_7 55
#define KEYCODE_8 56
#define empty 100
      
using namespace std;

int kfd = 0;  
struct termios cooked, raw;  

bool Space=0,Re_start=0,Stop=0;
double Direction = 0;
double Speed = 0,Up = 0;
int buttons[17],i;
char out_char[6] = "hello";
int out_num = 0;

double Robot_mode_sign = 0;
double Dance_Pitch = 0, Dance_Roll = 0 , Dance_Yaw = 0;
double Dance_Zside = 0, Dance_Xside = 0 , Dance_Yside = 0;
double Robo_function , Push_num = 0,Push_num_old = 0;
double nav_sign = 0;//nav_sign=1,navigation mode
int    t_gait_cycle_set = 0;

void JoyReceived(const sensor_msgs::Joy& joy)
{
    for(i=0;i<17;i++)
    {
        buttons[i]=joy.buttons[i];
    }
}

void FunctionReceived(const geometry_msgs::Twist& Function_on)
{
	Robo_function =   Function_on.linear.x;
}

//ros::Subscriber sub = n.subscribe("joy", 10, JoyReceived);

//  以下为键盘控制程序: keyboard control 
class keyboard_controller  
{  
    private:  
        geometry_msgs::Twist teleopkey;
        geometry_msgs::Twist Robot_mode;
        geometry_msgs::Twist Base_offset;
        geometry_msgs::Twist cmd_to_joy;
        ros::NodeHandle n_;
        ros::Publisher pub_;   
        ros::Publisher pub1_;  
        ros::Publisher pub2_; 
        ros::Publisher pub3_;
//        ros::Subscriber sub1;
    public:  
    keyboard_controller()  
    {  
        pub_ = n_.advertise<geometry_msgs::Twist>("teleopkey", 1);  
        pub1_ = n_.advertise<geometry_msgs::Twist>("/Robot_mode", 10); 
        pub2_ = n_.advertise<geometry_msgs::Twist>("/Base_offset", 10); 
        pub3_ = n_.advertise<geometry_msgs::Twist>("cmd_to_joy", 1);
  //      sub1= n_.subscribe("Function_on", 10, FunctionReceived);
    }     
    ~keyboard_controller() { }  
    void keyboardLoop();               
};  

// Signal handler
// TODO does not send 'Q' to drone on Ctrl+C 
     
void quit(int sig) 
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}
     
int main(int argc, char** argv)  
{  
    ros::init(argc,argv,"teleop");  

    // ros::Rate loop_rate(1000);

    keyboard_controller teleop;  

    // loop_rate.sleep();

    signal(SIGINT, quit);

    teleop.keyboardLoop(); 

    

    ros::spin();  
        
    // Dance_Xside = 0;
    // Dance_Yside = 0;
    // Dance_Zside = 0;
    // Dance_Roll = 0;
    // Dance_Pitch = 0;
    // Dance_Yaw = 0;  
    // char_in = KEYCODE_SPACE; 

    return(0);  
}  
      
void keyboard_controller::keyboardLoop()  
{  
    char char_in;   
         
        // get the console in raw mode  
    tcgetattr(kfd, &cooked);  
    memcpy(&raw, &cooked, sizeof(struct termios));  
    raw.c_lflag &=~ (ICANON | ECHO);  
    raw.c_cc[VEOL] = 1;  
    raw.c_cc[VEOF] = 2;  
    tcsetattr(kfd, TCSANOW, &raw);  


    // Dance_Xside = 0;
    // Dance_Yside = 0;
    // Dance_Zside = 0;
    // Dance_Roll = 0;
    // Dance_Pitch = 0;
    // Dance_Yaw = 0;     
   
    for(;;)  
    {  
            // get the next event from the keyboard  
        if(read(kfd, &char_in, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        switch(char_in)  
        {  
            case KEYCODE_r:  
                Re_start = 1;  
                Direction=0;
                Speed=0;
                Up = 0;
                out_num = KEYCODE_r;
             break;
            case KEYCODE_SPACE:  
                Space = !Space; 
                out_num = KEYCODE_SPACE;
            break;
            case  KEYCODE_a:
                Direction=min(Direction+1,5.0);
//                cout<<KEYCODE_a<<endl;
                out_num = KEYCODE_a;
            break;
            case  KEYCODE_d:
	        	Direction=max(Direction-1,-5.0);
            out_num = KEYCODE_d;
            break;
	          case  KEYCODE_w:
	       	    Direction=Direction; 
	        	Speed=min(Speed+1,5.0);
            out_num = KEYCODE_w;
            break;
            case  KEYCODE_s:
	       	    Speed=max(Speed-1,-5.0);
	        	Direction=Direction; 
            out_num = KEYCODE_s;
            break;
            case  KEYCODE_j:  
                Direction=Direction; 
	        	Up=min(Up+1,10.0);
            out_num = KEYCODE_j;
            break;
            case  KEYCODE_k:  
                Direction=Direction; 
	        	Up=max(Up-1,-10.0);
            out_num = KEYCODE_k;
            break;
            case  KEYCODE_x:
	       	    Speed=0; 
                Up = 0;
                Direction=Direction; 
                out_num = KEYCODE_x;
            break;

    //==============  机器人模式 robot mode  ===============
            case  KEYCODE_0:
	       	    Robot_mode_sign = 0;
                Push_num    =   Push_num + 1;
                out_num = KEYCODE_0;
            break;
            case  KEYCODE_1:
	       	    Robot_mode_sign = 1;
                Push_num    =   Push_num + 1;
                Re_start = 0; 
                out_num = KEYCODE_1;
            break;
            case  KEYCODE_2:
	       	    Robot_mode_sign = 2;
                Push_num    =   Push_num + 1;
                Re_start = 0; 
                out_num = KEYCODE_2;
            break;
            case  KEYCODE_3:
	       	    Robot_mode_sign = 3;
              Re_start = 0;
                Push_num    =   Push_num + 1;
                out_num = KEYCODE_3;
            break;
            case  KEYCODE_4:
	       	    Robot_mode_sign = 4;
              Re_start = 0;
                Push_num    =   Push_num + 1;
                out_num = KEYCODE_4;
            break;
            case  KEYCODE_5:
	       	    Robot_mode_sign = 5;
              Re_start = 0;
                Push_num    =   Push_num + 1;
                out_num = KEYCODE_5;
            break;
            case  KEYCODE_6:
	       	    Robot_mode_sign = 6;
              Re_start = 0;
                Push_num    =   Push_num + 1;
                out_num = KEYCODE_6;
            break;
            case  KEYCODE_7:
              Robot_mode_sign = 7;
              Re_start = 0;
                Push_num    =   Push_num + 1;
                out_num = KEYCODE_7;
            break;
            case  KEYCODE_8:
              Robot_mode_sign = 8;
              Re_start = 0;
                Push_num    =   Push_num + 1;
                out_num = KEYCODE_8;
            break;

            case KEYCODE_n:
              nav_sign = nav_sign + 1;
              out_num = KEYCODE_n;
            break;
            case KEYCODE_t:
              t_gait_cycle_set = t_gait_cycle_set + 1;
              out_num = KEYCODE_t;
            break;


    //==============    跳舞指令 dancing position offest command    ======================
            // case  KEYCODE_A:
                 
            //   Dance_Yside +=  0.0005;
            //   out_num = KEYCODE_A;
            // break;
            // case  KEYCODE_D:
	       	  //   Dance_Yside +=  -0.0005;
            //   out_num = KEYCODE_D;
            // break;
            // case  KEYCODE_W:
	       	  //   Dance_Xside +=   0.0005;
            //   out_num = KEYCODE_W;
            // break;
            // case  KEYCODE_S:
	       	  //   Dance_Xside +=  - 0.0005;
            //   out_num = KEYCODE_S;
            // break;
            // case  KEYCODE_Q:
	       	  //   Dance_Zside += 0.0005;
            //   out_num = KEYCODE_Q;
            // break;
            // case  KEYCODE_E:
	       	  //   Dance_Zside +=  - 0.0005;
            //   out_num = KEYCODE_E;
            // break;
            // //===========   跳舞姿态角 dancing orientation command   ============
            // case  KEYCODE_J:
	       	  //   Dance_Roll +=  0.0005;
            //   out_num = KEYCODE_J;
            // break;
            // case  KEYCODE_L:
	       	  //   Dance_Roll +=  -0.0005;
            //   out_num = KEYCODE_L;
            // break;
            // case  KEYCODE_I:
	       	  //   Dance_Pitch +=   0.0005;
            //   out_num = KEYCODE_I;
            // break;
            // case  KEYCODE_K:
	       	  //   Dance_Pitch +=  - 0.0005;
            //   out_num = KEYCODE_K;
            // break;
            // case  KEYCODE_U:
	       	  //   Dance_Yaw +=   0.0005;
            //   out_num = KEYCODE_U;
            // break;
            // case  KEYCODE_O:
	       	  //   Dance_Yaw +=  - 0.0005;
            //   out_num = KEYCODE_O;
            // break;

//==============    跳舞指令 gains tuning    ======================
            case  KEYCODE_A:    
              Dance_Yside =  1;
              Dance_Xside = 0;
              Dance_Zside = 0;
              Dance_Roll = 0;
              Dance_Pitch = 0;
              Dance_Yaw = 0;

              out_num = KEYCODE_A;
            break;
            case  KEYCODE_D:
	       	    Dance_Yside =  -1;
              Dance_Xside = 0;
              Dance_Zside = 0;
              Dance_Roll = 0;
              Dance_Pitch = 0;
              Dance_Yaw = 0;              
              out_num = KEYCODE_D;
            break;
            case  KEYCODE_W:
	       	    Dance_Xside =   1;
	       	    Dance_Yside =  0;
              Dance_Zside = 0;
              Dance_Roll = 0;
              Dance_Pitch = 0;
              Dance_Yaw = 0;              
              out_num = KEYCODE_W;
            break;
            case  KEYCODE_S:
	       	    Dance_Xside =  - 1;
	       	    Dance_Yside =  0;
              Dance_Zside = 0;
              Dance_Roll = 0;
              Dance_Pitch = 0;
              Dance_Yaw = 0;               
              out_num = KEYCODE_S;
            break;
            case  KEYCODE_Q:
	       	    Dance_Zside = 1;
	       	    Dance_Xside =  0;
	       	    Dance_Yside =  0;
              Dance_Roll = 0;
              Dance_Pitch = 0;
              Dance_Yaw = 0;                
              out_num = KEYCODE_Q;
            break;
            case  KEYCODE_E:
	       	    Dance_Zside =  - 1;
	       	    Dance_Xside =  0;
	       	    Dance_Yside =  0;
              Dance_Roll = 0;
              Dance_Pitch = 0;
              Dance_Yaw = 0;               
              out_num = KEYCODE_E;
            break;
            //=========== gains tuning ============
            case  KEYCODE_J:
	       	    Dance_Roll =  1;
	       	    Dance_Zside =  0;
	       	    Dance_Xside =  0;
	       	    Dance_Yside =  0;
              Dance_Pitch = 0;
              Dance_Yaw = 0;                 
              out_num = KEYCODE_J;
            break;
            case  KEYCODE_L:
	       	    Dance_Roll =  -1;
	       	    Dance_Zside =  0;
	       	    Dance_Xside =  0;
	       	    Dance_Yside =  0;
              Dance_Pitch = 0;
              Dance_Yaw = 0;              
              out_num = KEYCODE_L;
            break;
            case  KEYCODE_I:
	       	    Dance_Pitch =   1;

	       	    Dance_Zside =  0;
	       	    Dance_Xside =  0;
	       	    Dance_Yside =  0;
	       	    Dance_Roll =  0;              
              Dance_Yaw = 0;                 
              out_num = KEYCODE_I;
            break;
            case  KEYCODE_K:
	       	    Dance_Pitch =  - 1;
	       	    Dance_Zside =  0;
	       	    Dance_Xside =  0;
	       	    Dance_Yside =  0;
	       	    Dance_Roll =  0;              
              Dance_Yaw = 0;               
              out_num = KEYCODE_K;
            break;
            case  KEYCODE_U:
	       	    Dance_Yaw =   1;
	       	    Dance_Pitch =  0;
	       	    Dance_Zside =  0;
	       	    Dance_Xside =  0;
	       	    Dance_Yside =  0;
	       	    Dance_Roll =  0;               
              out_num = KEYCODE_U;
            break;
            case  KEYCODE_O:
	       	    Dance_Yaw =  - 1;
	       	    Dance_Pitch =  0;
	       	    Dance_Zside =  0;
	       	    Dance_Xside =  0;
	       	    Dance_Yside =  0;
	       	    Dance_Roll =  0;               
              out_num = KEYCODE_O;
            break;


            // =============    停机命令    ==================
            case    KEYCODE_Esc:
                Stop=1;
                out_num = KEYCODE_Esc;
            break;
            case    KEYCODE_f:
                out_num = KEYCODE_f;
            break;
            default:  
                Space=Space;
                Direction=Direction;
 
                // ;
        }

/*
        if(Robo_function == 1)
        Robot_mode_sign = 0;
*/
//    std::cout << Stop <<" "<< Direction<< endl;

        cmd_to_joy.linear.x = out_num;
        pub3_.publish(cmd_to_joy);
                   
    teleopkey.linear.x  =   Space;
    teleopkey.linear.y  =   Direction;
    teleopkey.linear.z  =   Speed;
    teleopkey.angular.x =   Re_start;
    teleopkey.angular.y =   Stop;
    teleopkey.angular.z =   Up;
    pub_.publish(teleopkey);

    Robot_mode.linear.x = Robot_mode_sign;
    Robot_mode.linear.y = Push_num;
    Robot_mode.linear.z = nav_sign;
    Robot_mode.angular.x = t_gait_cycle_set;
    pub1_.publish(Robot_mode);

    Base_offset.linear.x  =   Dance_Xside;
    Base_offset.linear.y  =   Dance_Yside;
    Base_offset.linear.z  =   Dance_Zside;
    Base_offset.angular.x =   Dance_Roll;
    Base_offset.angular.y =   Dance_Pitch;
    Base_offset.angular.z =   Dance_Yaw;
    pub2_.publish(Base_offset);
    
    }  
} 