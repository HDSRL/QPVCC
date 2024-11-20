#include "MJ_Wrapper.hpp"

using MJ = MujocoWrapper;

MJ::MujocoWrapper(std::string modelPath){

    // load and compile model
    char error[1000] = "Could not load the XML";
    m = mj_loadXML(&modelPath[0], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1200, 900, "Main Sim", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window,         MJ::static_keyboard);
    glfwSetCursorPosCallback(window,   MJ::static_mouse_move);
    glfwSetMouseButtonCallback(window, MJ::static_mouse_button);
    glfwSetScrollCallback(window,      MJ::static_scroll);
}

// GUI Callbacks
void MJ::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods){
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE ){
        mj_resetData(m, d);
        mj_forward(m, d);
    } else if (act==GLFW_PRESS && key==GLFW_KEY_ESCAPE){
        glfwSetWindowShouldClose(window, 1);
    } else if (act==GLFW_PRESS && key==GLFW_KEY_TAB){
        if (cam.type==mjCAMERA_TRACKING){
            cam.type = mjCAMERA_FREE;
            cam.trackbodyid = -1;
        } else{
            cam.type = mjCAMERA_TRACKING;
            cam.trackbodyid = *m->cam_bodyid;
        }
    } else if (act==GLFW_PRESS && key==GLFW_KEY_SPACE){
        runSim = (runSim==0) ? 1 : 0;
        oldRun = runSim;
    } else if (act==GLFW_PRESS && key==GLFW_KEY_RIGHT){
        oldRun = runSim;
        runSim = 1;
        stepSim = 1;
    }
}
void MJ::mouse_button(GLFWwindow* window, int button, int act, int mods){
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}
void MJ::mouse_move(GLFWwindow* window, double xpos, double ypos){
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}
void MJ::scroll(GLFWwindow* window, double xoffset, double yoffset){
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

void MJ::static_keyboard(GLFWwindow* window, int key, int scancode, int act, int mods){
    MJ* temp = reinterpret_cast<MJ*>(glfwGetWindowUserPointer(window));
    temp->keyboard(window,key,scancode,act,mods);
}
void MJ::static_mouse_button(GLFWwindow* window, int button, int act, int mods){
    MJ* temp = reinterpret_cast<MJ*>(glfwGetWindowUserPointer(window));
    temp->mouse_button(window,button,act,mods);
}
void MJ::static_mouse_move(GLFWwindow* window, double xpos, double ypos){
    MJ* temp = reinterpret_cast<MJ*>(glfwGetWindowUserPointer(window));
    temp->mouse_move(window,xpos,ypos);
}
void MJ::static_scroll(GLFWwindow* window, double xoffset, double yoffset){
    MJ* temp = reinterpret_cast<MJ*>(glfwGetWindowUserPointer(window));
    temp->scroll(window,xoffset,yoffset);
}

