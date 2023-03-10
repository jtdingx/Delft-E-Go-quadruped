cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_real)
MESSAGE(STATUS "Project: ${PROJECT_NAME}")    
MESSAGE(STATUS "Project Directory: ${PROJECT_SOURCE_DIR}")


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    unitree_legged_msgs
    roslib
)

catkin_package()

# include_directories(
#     include
#     ${Boost_INCLUDE_DIR}
#     ${catkin_INCLUDE_DIRS}
# )

set(CMAKE_CXX_FLAGS "-O3")

find_package(yaml-cpp REQUIRED)

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
MESSAGE(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

SET(SUB_SRC  src/go1_rt_control/src)
SET(SRC_PATH  ${PROJECT_SOURCE_DIR}/${SUB_SRC})
MESSAGE(STATUS "SRC_PATH Directory: ${SRC_PATH}")

FIND_PACKAGE ( Threads REQUIRED )
SET(SUB_SRC_NLP  src/mosek_nlp_kmp/src)
SET(SRC_PATH_NLP  ${PROJECT_SOURCE_DIR}/${SUB_SRC_NLP})


set(YAML_INC_DIR unitree_ros_to_real/unitree_legged_real/include)
set(YAML_LIB_DIR unitree_ros_to_real/unitree_legged_real/lib)
 

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    # ${GAZEBO_INCLUDE_DIRS}
    ${SRC_PATH}
    ${SRC_PATH_NLP}
    /usr/local/include
)


include_directories(/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_legged_sdk/include)
link_directories(/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_legged_sdk/lib)
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_amd64.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

add_executable(lcm_server /home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_legged_sdk/examples/lcm_server.cpp)
target_link_libraries(lcm_server ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(lcm_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(position_lcm src/exe/position_mode.cpp)
target_link_libraries(position_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(position_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(velocity_lcm src/exe/velocity_mode.cpp)
# target_link_libraries(velocity_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(velocity_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(torque_lcm src/exe/torque_mode.cpp)
# target_link_libraries(torque_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(torque_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(walk_lcm src/exe/walk_mode.cpp)
# target_link_libraries(walk_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(walk_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(go1_teleop_control ${SRC_PATH}/teleop_control.cpp)
target_link_libraries(go1_teleop_control ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(go1_teleop_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

##########################for go1_servo generation######

add_executable(go1_servo_control 
${SRC_PATH}/torque_control_servo/torque_mode_udp.cpp
${SRC_PATH}/Filter/butterworth_filter.cpp
${SRC_PATH}/Filter/butterworthLPF.cpp
${SRC_PATH}/kinematics/Kinematics.cpp
${SRC_PATH}/Robotpara/robot_const_para_config.cpp
${SRC_PATH}/whole_body_dynamics/dynmics_compute.cpp
${SRC_PATH}/utils/Utils.cpp
${SRC_PATH}/body/body.cpp
${SRC_PATH}/QP/QPBaseClass.cpp
${SRC_PATH}/utils/EiQuadProg/EiQuadProg.cpp
${SRC_PATH}/A1EKF/A1BasicEKF.cpp
${SRC_PATH}/A1EKF/state_estimator.cpp
)

target_link_libraries(go1_servo_control ${EXTRA_LIBS} ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(go1_servo_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


########################## for nlp_mosek generation################
set(RTControl_DIRS ${SRC_PATH_NLP}/NLPRTControl)
set(NLP_DIRS ${SRC_PATH_NLP}/NLP)


FILE(GLOB SOURCES
	${RTControl_DIRS}/*.cpp
	${NLP_DIRS}/*.cpp
	${SRC_PATH}/Robotpara/robot_const_para_config.cpp
    ${SRC_PATH}/QP/QPBaseClass.cpp
    ${SRC_PATH}/utils/EiQuadProg/EiQuadProg.cpp
)
add_executable(mosek_nlp_kmp_node ${SRC_PATH_NLP}/gait_nlp_kmp.cpp
                ${SOURCES}
                )

add_dependencies(mosek_nlp_kmp_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)


 target_link_libraries(mosek_nlp_kmp_node
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_LIBRARIES}
   yaml-cpp
 )