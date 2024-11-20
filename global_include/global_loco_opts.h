#ifndef GLOBAL_LOCO_OPTS
#define GLOBAL_LOCO_OPTS

#define MY_PI 3.14159265359

#define simfreq_raisim 0.001        // maybe constant
#define simfreq 0.001               // maybe changable
// #define MPC_Hz 200
#define ctrlHz 1000                 // depends on simfreq
#define LL_Hz 1000                  // Low level control loop rate

#define TOTAL_DOF 18 // Total degrees of freedom for the system
#define TOTAL_IN  12 // Total system inputs

#define CTRL_HORIZ     10 // Length of the prediction horizon
#define NUM_RED_STATE  12 // Number of reduced order states
#define NUM_RED_INPUT  12 // Number of input forces to the MPC (4 legs *(x,y,z))
#define NUM_CTRL       12 // Number of actuators

#define FR_LEG 0
#define FL_LEG 1
#define RR_LEG 2
#define RL_LEG 3

#define STAND           0
#define POSE            1
#define TAP             2
#define INPLACE_WALK    3
#define INPLACE_TROT    4
#define WALK            5
#define TROT            6
#define PACE            7

#define POSE_X     0
#define POSE_Y     1
#define POSE_Z     2
#define POSE_ROLL  3
#define POSE_PITCH 4
#define POSE_YAW   5
#define POSE_COMB  6
#define POSE_DEMO  7
#define POSE_CMD  7

#ifdef RUNNING_SIM // DEFINE THIS IN THE CMAKE IF USING A SIMULATOR
    #define Z_TOE_OFFSET 0.019
#else 
    #define Z_TOE_OFFSET 0.012
#endif

#endif