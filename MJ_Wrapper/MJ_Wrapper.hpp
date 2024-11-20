#ifndef MUJOCO_CPP_WRAPPER_H
#define MUJOCO_CPP_WRAPPER_H

#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "iostream"

class MujocoWrapper {
public: 
    MujocoWrapper(std::string modelPath);
    virtual ~MujocoWrapper(){
        mjv_freeScene(&scn);
        mjr_freeContext(&con);
        mj_deleteData(d);
        mj_deleteModel(m);
    };

    // MuJoCo data structures
    mjModel* m = NULL;                  // MuJoCo model
    mjData*  d = NULL;                  // MuJoCo data
    mjvCamera  cam;                     // abstract camera
    mjvOption  opt;                     // visualization options
    mjvScene   scn;                     // abstract scene
    mjrContext con;                     // custom GPU context
    GLFWwindow* window;
    int runSim = 1;
    int stepSim = 1;
    int oldRun = 1;

    void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
    void mouse_button(GLFWwindow* window, int button, int act, int mods);
    void mouse_move(GLFWwindow* window, double xpos, double ypos);
    void scroll(GLFWwindow* window, double xoffset, double yoffset);

    static void static_keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
    static void static_mouse_button(GLFWwindow* window, int button, int act, int mods);
    static void static_mouse_move(GLFWwindow* window, double xpos, double ypos);
    static void static_scroll(GLFWwindow* window, double xoffset, double yoffset);

private:

    // mouse interaction
    bool button_left = false;
    bool button_middle = false;
    bool button_right =  false;
    double lastx = 0;
    double lasty = 0;

};





#endif