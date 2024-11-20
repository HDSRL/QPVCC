#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Sparse"
#include <vector>
#include "string.h"

// Mujoco helper headers
#include "MJ_Wrapper.hpp"
#include "MJ_sensors.hpp"

// Control related Headers
#include "Filters.h"
#include "KinEstim.hpp"
#include "LocoWrapper.hpp"

std::vector<LocoWrapper*> Controllers;
kinEstStruct est_obj;
extern mjfSensor mjcb_sensor;

void mycontroller(const mjModel* m, mjData* d){
    static long ctrlTick = 0;
    LocoWrapper* loco_obj = Controllers[0];

    /////////////////////////////////////////////////////////////////////
    //////////////////////////// INITIALIZE
    /////////////////////////////////////////////////////////////////////
    size_t loco_kind = TROT;                        // Gait pattern to use
    size_t pose_kind = POSE_COMB;                  // Pose type to use (if loco_kind is set to POSE)
    size_t settling = 0.2*ctrlHz;                   // Settling down
    size_t duration = 0.8*ctrlHz;                   // Stand up
    size_t loco_start = settling + duration;        // Start the locomotion pattern

    double *tau;
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// UPDATE STATE
    /////////////////////////////////////////////////////////////////////
    Eigen::Matrix<double, 19, 1> jointPos = Eigen::Map< Eigen::Matrix<double, 19, 1> > (d->qpos,19,1);
    Eigen::Matrix<double, 18, 1> jointVel = Eigen::Map< Eigen::Matrix<double, 18, 1> > (d->qvel,18,1);

    Eigen::Matrix<double, 3, 3> rotE;
    Eigen::Matrix<double, 4, 1> quat = jointPos.block(3,0,4,1);
    quat_to_R(quat,rotE);
    double *rotMatrixDouble;
    rotMatrixDouble = rotE.data();

    double jpos[18], jvel[18];
    Eigen::Matrix<double, 3, 1> eul;
    quat_to_XYZ(quat,eul);
    for(size_t i=0; i<3; ++i){
        jpos[i] = jointPos(i);
        jvel[i] = jointVel(i);
        jpos[i+3] = eul(i);
        jvel[i+3] = jointVel(i+3);
    }
    for(size_t i=6; i<18; ++i){
        jpos[i] = jointPos(i+1);
        jvel[i] = jointVel(i);
    }
    
    A1_LowState state;
    A1_ParseSensors(m, d, &state);
    for(size_t i=0; i<12; ++i){
        jpos[i+6] = state.motorState[i].q;
        jvel[i+6] = state.motorState[i].dq;
    }   
    int footForce[4] = {0};
    const int *conInd = loco_obj->getConDes();
    kinEst(footForce,conInd,jpos,jvel,rotE,est_obj);

    /////////////////////////////////////////////////////////////////////
    //////////////////////////// CONTROL
    /////////////////////////////////////////////////////////////////////

    int force[4] = {0};
    // Update the desired torques from LL controller
    if(ctrlTick < settling){ // Settle down
        double temp[18] = {0};
        tau = temp;
        loco_obj->initStandVars(jointPos.block(0,0,3,1),jointPos(5),(int)duration);
    }
    else if(ctrlTick >= settling & ctrlTick <= loco_start){ // Start standing
        loco_obj->calcTau(jpos,jvel,rotMatrixDouble,force,STAND,ctrlTick);
        tau = loco_obj->getTorque();
    }
    else if(ctrlTick > loco_start){ // Start locomotion
        loco_obj->calcTau(jpos,jvel,rotMatrixDouble,force,loco_kind,ctrlTick);
        tau = loco_obj->getTorque();
    }

    for(int i=0; i<12;++i){
        d->ctrl[i] = tau[i+6];
    }

    ctrlTick++;
}

int main(int argc, char* argv[]){

    // start instance of Mujoco GUI
    std::string XML_File = "./A1.xml";
    MujocoWrapper* MJ = new MujocoWrapper(XML_File);

    // MuJoCo data structures
    mjModel* m = MJ->m;                // MuJoCo model
    mjData*  d = MJ->d;                // MuJoCo data
    mjvCamera  *cam = &MJ->cam;        // abstract camera
    mjvOption  *opt = &MJ->opt;        // visualization options
    mjvScene   *scn = &MJ->scn;        // abstract scene
    mjrContext *con = &MJ->con;        // custom GPU context
    GLFWwindow *window = MJ->window;   // pointer to the window

    Eigen::Matrix<mjtNum, 19, 1> pInit;
    pInit << 0, 0, 0.12, 1, 0, 0, 0, 0.0, M_PI/3, -2.6, 0.0, M_PI/3, -2.6, 0.0, M_PI/3, -2.6, 0.0, M_PI/3, -2.6;
    for(int i=0; i<19; ++i){
        d->qpos[i] = pInit(i);
    }
    d->userdata = 0;

    LocoWrapper* A1 = new LocoWrapper(argc,argv);
    Controllers.push_back(A1);

    // ================================== //
    // ============ Settings ============ //
    // ================================== //

    bool startPaused = false;       // Set to true to start in manual stepping (use right arrow to step)
    double simLength = 360;         // Length of simulation in seconds
    bool showMPC = true;            // Show the MPC body?
    bool record = false;            // Record?
    double fps = 30;                // Frames per second to record
    opt->flags[mjVIS_CONTACTFORCE] = false; // visualize contact forces (arrows)
    opt->flags[mjVIS_CONTACTPOINT] = true;  // visualize contact points
    scn->flags[mjRND_SKYBOX] = true;        // render skybox
    opt->flags[mjVIS_STATIC] = true;        // render ground

    // ================================== //
    // =========== Simulation =========== //
    // ================================== //
    cam->type = mjCAMERA_TRACKING;
    cam->trackbodyid = *m->cam_bodyid;

    mjcb_control = mycontroller;
    mjtNum simstart;
    MJ->oldRun = (startPaused) ? 0 : 1;
    while( !glfwWindowShouldClose(window) && d->time<simLength)
    {
        // Render 60 fps
        simstart = d->time;
        while( d->time - simstart < 1.0/60.0 && MJ->runSim==1){
            mj_step(m, d); // Step simulation
        }

        MJ->stepSim = 0;
        MJ->runSim = MJ->oldRun;

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, opt, NULL, cam, mjCAT_ALL, scn);
        mjr_render(viewport, scn, con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }
    if(window && !glfwWindowShouldClose(window)){
        glfwSetWindowShouldClose(window, 1);
        glfwPollEvents();
    }

    delete MJ;
    delete Controllers.back();

    return 1;
}