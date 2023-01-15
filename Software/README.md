1, "Unitree_ros" provides a general locomotion control framework for legged robots(taking the unitree go1-simulation for example)

Already tested on Ros Moledic/Noetic


# **Dependency**

Add catkin dependency, please make sure you already have successfully tested the "unitree_ros" repo from "https://github.com/unitreerobotics/unitree_ros".


# **Build and Run**

(1) Put the "unitree_ros" in you own workspcce "src/", then "catkin_make"

(1) load gazebo simulator using GO1 desription: 'roslaunch unitree_gazebo normal.launch rname:=go1 wname:=empty'

(2) start real-time control loop: ' rosrun go1_rt_control go1_servo'

(3) keyboard control: ' rosrun go1_rt_control go1_teleop_control'
        
         "1": go to homing pose
         
         "2": enter the "walking mode"
            
              then, try "W". "S", "A", "D",



================================================================================================================================

2, "unitree_ros_to_real" provides hardware implementation (run on the local computer)

The repo contains PEA force compensation, you can disable it by set "enable_spring:0" in the "config.yaml"

# **Dependency**

Make sure you already know how to control Go1 robot in the "Low-level" control mode ("https://github.com/unitreerobotics/unitree_ros_to_real")

# **Build and Run**

(1) Put the "unitree_legged_real" in you own workspcce "src/", then "catkin_make"

(1) start real robot in "lowlevel" control mode: 'roslaunch unitree_legged_real real.launch'

(2) start real-time control loop: ' rosrun unitree_legged_real go1_servo_control'

(3) keyboard control: ' rosrun unitree_legged_real go1_teleop_control'
        
         "1": go to homing pose
         
         "2": enter the "walking mode"
            
              then, try "W". "S", "A", "D",

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

One open source lib from "https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller" has been integrated, but not test yet.

