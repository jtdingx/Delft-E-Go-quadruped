cmake_minimum_required(VERSION 2.8.3)
project(go1_rt_control)

MESSAGE(STATUS "Project: ${PROJECT_NAME}")    
MESSAGE(STATUS "Project Directory: ${PROJECT_SOURCE_DIR}")

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
    unitree_legged_msgs
)

find_package(yaml-cpp REQUIRED)
INCLUDE_DIRECTORIES(${YAML_INCLUDE_DIR})
MESSAGE(STATUS "YAML_INCLUDE_DIR: ${YAML_INCLUDE_DIR}")


find_package(gazebo REQUIRED)

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
MESSAGE(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")


catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs 
)

FIND_PACKAGE ( Threads REQUIRED )
SET(SUB_SRC  src)
SET(SRC_PATH  ${PROJECT_SOURCE_DIR}/${SUB_SRC})

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${SRC_PATH}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Declare a C++ library
add_library(${PROJECT_NAME}
    src/body.cpp 
)

add_dependencies(${PROJECT_NAME} unitree_legged_msgs_gencpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${EXTRA_LIBS}
)

# add_library(unitreeFootContactPlugin SHARED plugin/foot_contact_plugin.cc)
# target_link_libraries(unitreeFootContactPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# add_library(unitreeDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
# target_link_libraries(unitreeDrawForcePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(go1_external_force 
               src/external_force.cpp)
target_link_libraries(go1_external_force 
                      ${catkin_LIBRARIES})


add_executable(go1_teleop_control
               src/teleop_control.cpp)

target_link_libraries(go1_teleop_control 
                      ${catkin_LIBRARIES})               
                      

add_executable(go1_servo 
               src/servo_control/servo.cpp
               src/Filter/butterworth_filter.cpp
               src/Filter/butterworthLPF.cpp
               src/kinematics/Kinematics.cpp
               src/Robotpara/robot_const_para_config.cpp
               src/whole_body_dynamics/dynmics_compute.cpp
               src/utils/Utils.cpp
               src/utils/EiQuadProg/EiQuadProg.cpp
               src/QP/QPBaseClass.cpp
            #    src/A1legKinematics/A1Kinematics.cpp
               src/A1EKF/A1BasicEKF.cpp
               src/A1EKF/state_estimator.cpp
               )

target_link_libraries(go1_servo 
                      ${PROJECT_NAME} ${catkin_LIBRARIES}
                      yaml-cpp)
