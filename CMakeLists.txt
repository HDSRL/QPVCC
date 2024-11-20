cmake_minimum_required(VERSION 3.5)
project(A1_Robot VERSION 2.0.0)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3")

set(LL_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(COMPILE_RAISIM TRUE)
set(COMPILE_MUJOCO TRUE) # Have not implemented Mujoco for low level only. 

set(USE_QPSWIFT TRUE) # Alternatively, OSQP is used


if($ENV{USER} STREQUAL "kaveh" OR $ENV{USER} STREQUAL "unitree") # Here to prevent the need to change CMake every time you run exp. 
    set(SIMULATION FALSE)
else()
    set(MUJOCO_PATH /home/kavehakbarihamed/mujoco210) # Path to mujoco 210 <--- must be 210
    set(MUJOCO_LIB_PATH ${MUJOCO_PATH}/bin) # Path to the bin folder in mujoco
	set(SIMULATION TRUE)
endif()
# set(SIMULATION FALSE)

# ===== RaiSim and RaiSimOgre ===== #
find_package(raisim PATHS "/home/kavehakbarihamed/raisim/build/lib/cmake/raisim")
find_package(raisimOgre CONFIG PATHS "/home/kavehakbarihamed/raisim/build/share/raisimOgre/cmake")

# ===== General packages and UT SDk ===== #
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
#find_package(Boost REQUIRED COMPONENTS system thread)
find_library(LCM lcm HINTS /usr/local/lib)
include_directories(${EIGEN3_INCLUDE_DIRS})
set(UT_INC ${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk/include)
set(UT_LINK ${CMAKE_CURRENT_SOURCE_DIR}/unitree_legged_sdk/lib)
set(EXTRA_LIBS -pthread ${UT_LINK}/libunitree_legged_sdk_amd64.so)

# ===== QP SWIFT Library ===== #
add_library(optim_lib
    "${LL_DIR}/optimization/iSWIFT/src/Auxilary.c"
    "${LL_DIR}/optimization/iSWIFT/src/Prime.c"
    "${LL_DIR}/optimization/iSWIFT/src/timer.c"
    "${LL_DIR}/optimization/iSWIFT/ldl/src/ldl.c"
    "${LL_DIR}/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp")
target_include_directories(optim_lib PUBLIC
    ${LL_DIR}/optimization/iSWIFT/include
    ${LL_DIR}/optimization/iSWIFT/ldl/include
    ${LL_DIR}/optimization/iSWIFT/cpp_wrapper)

# ===== Dynamics Library ===== #
# Settings:
set(COMPILE_CORIOLIS FALSE) # Option to compile the coriolis terms. Not implemented in code though.
# set(ROBOT_DIR "Go1_wFoot")
set(ROBOT_DIR "A1_New_wFoot")
set(dyn_srcs 
    "${LL_DIR}/dynamics/${ROBOT_DIR}/A1_D_B_G.cpp"
    "${LL_DIR}/dynamics/${ROBOT_DIR}/A1_FK.cpp"
    "${LL_DIR}/dynamics/${ROBOT_DIR}/A1_FK.cpp"
    "${LL_DIR}/dynamics/${ROBOT_DIR}/A1_Jaco.cpp"
    "${LL_DIR}/dynamics/${ROBOT_DIR}/A1_Jaco_dot.cpp"
    "${LL_DIR}/dynamics/RobotModel.cpp")
if (COMPILE_CORIOLIS)
    set(dyn_srcs ${dyn_srcs} "${LL_DIR}/dynamics/${ROBOT_DIR}/A1_C.cpp")
endif()
add_library(dyn_lib ${dyn_srcs})
target_include_directories(dyn_lib PUBLIC
    ${LL_DIR}/dynamics
    ${LL_DIR}/dynamics/${ROBOT_DIR}
    ${LL_DIR}/util_include
    ${LL_DIR}/global_include)

# ===== Parameter library ===== #
add_library(params_lib
    "${LL_DIR}/params/Parameters.cpp")
target_include_directories(params_lib PUBLIC
    ${LL_DIR}/params
    ${LL_DIR}/global_include)

# ===== Controller Library ===== #
set(optimlibrary optim_lib)
add_definitions(-DUSE_QPSWIFT=1)

add_library(ctrl_lib
    "${LL_DIR}/src/LowLevelCtrl.cpp"
    "${LL_DIR}/src/VirtualConstraints.cpp"
    "${LL_DIR}/src/DataLog.cpp")
target_include_directories(ctrl_lib PUBLIC 
    ${LL_DIR}/include
    ${LL_DIR}/util_include
    ${LL_DIR}/global_include)
target_link_libraries(ctrl_lib PUBLIC 
    dyn_lib
    params_lib
    ${optimlibrary})

# ===== Planning and Contact ===== #
add_library(plan_lib
    "${LL_DIR}/MP_and_Con/src/ContactEst.cpp"
    "${LL_DIR}/MP_and_Con/src/MotionPlanner.cpp")
target_include_directories(plan_lib PUBLIC
    ${LL_DIR}/MP_and_Con/inc
    ${LL_DIR}/global_include
    ${LL_DIR}/util_include)

# ===== Wrapper Library ===== #
add_library(wrapper
    "${LL_DIR}/src/LocoWrapper.cpp")
target_link_libraries(wrapper PUBLIC
    ctrl_lib
    plan_lib)

# ===== High Level Library (basic test only) ===== #
# add_library(hl_lib
# 	"${LL_DIR}/src/forceUpdate.cpp"
# )


# ===== Simulation Executables ===== #
if(SIMULATION)
    add_definitions(-DRUNNING_SIM) # adds the toe offset to be used in simulation (exp needs no offset)
    if (raisimOgre_FOUND AND COMPILE_RAISIM)   
        include_directories(PUBLIC ${LL_DIR}/raisimOgre_include)
        file(COPY ${LL_DIR}/rsc DESTINATION ${CMAKE_BINARY_DIR})
    
        add_executable(Run_Sim src/A1_Sim.cpp)
        target_link_libraries(Run_Sim PRIVATE wrapper raisim::raisimOgre)
        target_compile_definitions(Run_Sim PRIVATE "-DEXAMPLE_ROBOT_RESOURCE_DIR=${LL_DIR}/rsc/")
    endif()

    # ===== Mujoco ===== #
    if (COMPILE_MUJOCO)
        add_library(mujoco_wrapper
            "MJ_Wrapper/MJ_Wrapper.cpp")
        target_include_directories(mujoco_wrapper PUBLIC 
            ${MUJOCO_PATH}/include
            "MJ_Wrapper")
        target_link_libraries(mujoco_wrapper PUBLIC
            ${MUJOCO_LIB_PATH}/libmujoco210.so
            libGL.so
            ${MUJOCO_LIB_PATH}/libglew.so
            ${MUJOCO_LIB_PATH}/libglfw.so.3)
        target_link_directories(mujoco_wrapper PUBLIC ${MUJOCO_LIB_PATH}/bin)

        # ===== Mujoco Simulation ===== #
        file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/rsc/A1/A1.xml DESTINATION ${CMAKE_BINARY_DIR})
        add_executable(Run_Mujoco src/Mujoco_Example.cpp)
        target_link_libraries(Run_Mujoco PRIVATE
            mujoco_wrapper
            wrapper)
        set_target_properties(Run_Mujoco
            PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
    endif()
endif()


# ==== Experiment Executables ==== #

if(NOT SIMULATION)
    add_executable(A1_Experiment src/A1_Exp.cpp)
    target_link_libraries(A1_Experiment PRIVATE wrapper ${EXTRA_LIBS} ${LCM} ${Boost_LIBRARIES})
    target_include_directories(A1_Experiment PRIVATE ${UT_INC})
    
    add_executable(A1_Exp_Joystick src/A1_Exp_Joystick.cpp)
    target_link_libraries(A1_Exp_Joystick PRIVATE wrapper ${EXTRA_LIBS} ${LCM} ${Boost_LIBRARIES})
    target_include_directories(A1_Exp_Joystick PRIVATE ${UT_INC})
    
    add_executable(restore src/restoreConfig.cpp)
    target_link_libraries(restore PRIVATE ${EXTRA_LIBS} ${LCM} ${Boost_LIBRARIES})
    target_include_directories(restore PRIVATE ${UT_INC})
endif()

# EOF


