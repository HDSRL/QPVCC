//
// Authror: Randy Fawcett on 12/2021.
//
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#include "raisim/OgreVis.hpp"
#include "randyImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp" // THIS IS WHERE cmd.vel AND cmd.pose ARE IMPLEMENTED (as globals)
#include "helper.hpp"
#include "helper2.hpp"
#include "Filters.h"

//HDSRL header
#include "LocoWrapper.hpp"

FiltStruct_f* filt = (FiltStruct_f*)malloc(sizeof(FiltStruct_f));


void setupCallback() {
    raisim::OgreVis *vis = raisim::OgreVis::get();

    /// light
    vis->getLight()->setDiffuseColour(1, 1, 1);
    vis->getLight()->setCastShadows(false);
    Ogre::Vector3 lightdir(-3,3,-0.5); // Light shines on ROBOTS top/front/right side
    // Ogre::Vector3 lightdir(-3,-3,-0.5); // Light shines on ROBOTS top/front/left side
    lightdir.normalise();
    vis->getLightNode()->setDirection({lightdir});
    vis->setCameraSpeed(300);

    vis->addResourceDirectory(raisim::loadResource("material"));
    vis->loadMaterialFile("myMaterials.material");

    vis->addResourceDirectory(vis->getResourceDir() + "/material/skybox/violentdays");
    vis->loadMaterialFile("violentdays.material");

    /// shdow setting
    vis->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
    vis->getSceneManager()->setShadowTextureSettings(2048, 3);

    /// scale related settings!! Please adapt it depending on your map size
    // beyond this distance, shadow disappears
    vis->getSceneManager()->setShadowFarDistance(10);
    // size of contact points and contact forces
    vis->setContactVisObjectSize(0.03, 0.6);
    // speed of camera motion in freelook mode
    vis->getCameraMan()->setTopSpeed(5);
}

void disturbance(std::vector<raisim::ArticulatedSystem *> A1,std::map<std::__cxx11::string, raisim::VisualObject> &objList,size_t start,size_t stop,size_t ctrlTick){
    Eigen::VectorXd pos = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1);
    Eigen::VectorXd vel = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    A1.back()->getState(pos, vel);
    
    // ====================== External Forces ====================== //
    raisim::Vec<3> extForce;
    raisim::Mat<3,3> rot;
    raisim::Vec<3> dir;
    if (ctrlTick>=start && ctrlTick<stop){
        extForce = {0,40,0}; // Pulse
        // extForce = {50*sin(4*ctrlTick*simfreq_raisim),0,0}; // Fwd Sine
        extForce = {0,20*sin(4*ctrlTick*simfreq_raisim),0}; // Lat Sine
    } else{
        extForce = {0,0,0};
    }
    A1.back()->setExternalForce(1,extForce);
    dir = extForce;
    dir /= dir.norm();
    raisim::zaxisToRotMat(dir, rot);
    objList["extForceArrow"].offset = {pos(0),pos(1),pos(2)};
    objList["extForceArrow"].scale = {0.2,0.2,0.01*extForce.norm()};
    objList["extForceArrow"].rotationOffset = rot;
}

void controller(std::vector<raisim::ArticulatedSystem *> A1, LocoWrapper *loco_obj, size_t controlTick) {
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// INITIALIZE
    /////////////////////////////////////////////////////////////////////
    static size_t loco_kind = TROT;                        // Gait pattern to use
    size_t pose_kind = POSE_CMD;                   // Pose type to use (if loco_kind is set to POSE)
    size_t settling = 0.2*ctrlHz;                   // Settling down
    size_t duration = 0.8*ctrlHz;                   // Stand up 
    size_t loco_start = settling + duration;        // Start the locomotion pattern

    double *tau;
    double jpos[18], jvel[18];
    Eigen::VectorXd jointTorqueFF = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    Eigen::VectorXd jointPosTotal = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1);
    Eigen::VectorXd jointVelTotal = Eigen::MatrixXd::Zero(TOTAL_DOF,1);    
    raisim::Mat<3,3> rotMat;
    Eigen::Matrix<double, 3, 1> eul;
    Eigen::Matrix<double, 4, 1> quat;

    /////////////////////////////////////////////////////////////////////
    //////////////////////////// UPDATE STATE
    /////////////////////////////////////////////////////////////////////
    A1.back()->getState(jointPosTotal, jointVelTotal);
    A1.back()->getBaseOrientation(rotMat);
    double rotMatrixDouble[9];
    for(size_t i=0;i<9;i++){
        rotMatrixDouble[i] = rotMat[i];
    }
    Eigen::Map< Eigen::Matrix<double, 3, 3> > rotE(rotMatrixDouble, 3, 3);
    jointVelTotal.segment(3,3) = rotE.transpose()*jointVelTotal.segment(3,3); // convert to body frame, like robot measurements

    quat = jointPosTotal.block(3,0,4,1);
    quat_to_XYZ(quat,eul);
    for(size_t i=0; i<3; ++i){
        jpos[i] = jointPosTotal(i);
        jvel[i] = jointVelTotal(i);
        jpos[i+3] = eul(i);
        jvel[i+3] = jointVelTotal(i+3);
    }
    for(size_t i=6; i<18; ++i){
        jpos[i] = jointPosTotal(i+1);
        jvel[i] = jointVelTotal(i);
    }

    int force[4] = {0};
    for(auto &con: A1.back()->getContacts()){
        int conInd = con.getlocalBodyIndex();
        force[conInd/3-1] = 500;
        force[conInd/3-1] = con.getNormal().e().norm();
    }

    
    float vel_temp[3] = {cmd.vel[0],cmd.vel[1],cmd.vel[2]};
    ////////////////////////////////////////////////////////////////////
    // Used for keyboard commands. Not recommended.                   // 
    // To use, you must set 'extCmd' to 1 in the walking params file. //
    ////////////////////////////////////////////////////////////////////
    //
    // discrete_butter_f(filt,vel_temp);
    // printf("Mode: %lu || Fwd Vel: %0.3f || Lat Vel: %0.3f || Yaw Vel: %0.3f || Pitch: %0.3f\n",loco_kind,vel_temp[0],vel_temp[1],vel_temp[2],cmd.pose[1]);
    // 
    // static bool change_prev = false;
    // if ((vel_temp[0]+vel_temp[1]+vel_temp[2])<0.02 && cmd.change_gait==1 && cmd.change_gait!=change_prev){
    //     if (loco_kind==TROT){
    //         std::cout<<"Transitioned to Pose"<<std::endl;
    //         loco_kind = POSE;
    //     }else{
    //         std::cout<<"Transitioned to Trot"<<std::endl;
    //         loco_kind = TROT;
    //     }
    // }
    // change_prev = cmd.change_gait;
    // cmd.change_gait = 0;

    /////////////////////////////////////////////////////////////////////
    //////////////////////////// CONTROL
    /////////////////////////////////////////////////////////////////////
    // Update the desired torques
    if(controlTick < settling){ // Settle down
        // loco_obj->posSetup(jointPosTotal.head(3));
        double temp[18] = {0};
        tau = temp;
        loco_obj->initStandVars(jointPosTotal.block(0,0,3,1),jointPosTotal(5),(int)duration);
    }
    else if(controlTick >= settling & controlTick <= loco_start){ // Start standing
        loco_obj->calcTau(jpos,jvel,rotMatrixDouble,force,STAND,controlTick);
        tau = loco_obj->getTorque();
    }
    else if(controlTick > loco_start){ // Start locomotion
        loco_obj->updateVel(vel_temp);
        loco_obj->updatePose(cmd.pose);
        loco_obj->calcTau(jpos,jvel,rotMatrixDouble,force,loco_kind,controlTick);
        tau = loco_obj->getTorque();
    }

    jointTorqueFF = Eigen::Map< Eigen::Matrix<double,18,1> >(tau,18);
    jointTorqueFF.block(0,0,6,1).setZero();

    // Set the desired torques
    A1.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    A1.back()->setGeneralizedForce(jointTorqueFF);
};

int main(int argc, char *argv[]) {
    // ============================================================ //
    // =================== SETUP RAISIM/VISUALS =================== //
    // ============================================================ //
    /// create raisim world
    raisim::World::setActivationKey(raisim::loadResource("activation_Randall_Fawcett.raisim"));
    raisim::World world;
    world.setTimeStep(simfreq_raisim);

    raisim::OgreVis *vis = raisim::OgreVis::get();

    /// these method must be called before initApp
    vis->setWorld(&world);
    vis->setWindowSize(1792, 1200); // Should be evenly divisible by 16!!
    vis->setImguiSetupCallback(imguiSetupCallback); // These 2 lines make the interactable gui visible
    vis->setImguiRenderCallback(imguiRenderCallBack);
    vis->setKeyboardCallback(raisimKeyboardCallback);
    vis->setSetUpCallback(setupCallback);
    vis->setAntiAliasing(2);

    /// starts visualizer thread
    vis->initApp();

    /// create raisim objects
    raisim::TerrainProperties terrainProperties;
    terrainProperties.frequency = 0.0;
    terrainProperties.zScale = 0.0;
    terrainProperties.xSize = 300.0;
    terrainProperties.ySize = 300.0;
    terrainProperties.xSamples = 50;
    terrainProperties.ySamples = 50;
    terrainProperties.fractalOctaves = 0;
    terrainProperties.fractalLacunarity = 0.0;
    terrainProperties.fractalGain = 0.0;

    raisim::HeightMap *ground = world.addHeightMap(0.0, 0.0, terrainProperties);
    vis->createGraphicalObject(ground, "terrain", "checkerboard_blue");
    world.setDefaultMaterial(0.8, 0.0, 0.0); //surface friction could be 0.8 or 1.0
    vis->addVisualObject("extForceArrow", "arrowMesh", "red", {0.0, 0.0, 0.0}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);

    // // WEIGHT VISUALIZATION FOR LCSS PAPER, KEEP FOR NOW.
    // // create raisim objects
    // double scale = 1;
    // vis->loadMeshFile("/home/kavehakbarihamed/raisim/workspace/A1_LL_Exp/rsc/Dumbell_5lb.STL", "weight1", false);
    // raisim::VisualObject *weightVis1 = vis->addVisualObject("weight_vis1", "weight1", "purple", {scale, scale, scale}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);
    // vis->loadMeshFile("/home/kavehakbarihamed/raisim/workspace/A1_LL_Exp/rsc/Dumbell_5lb.STL", "weight2", false);
    // raisim::VisualObject *weightVis2 = vis->addVisualObject("weight_vis2", "weight2", "purple", {scale, scale, scale}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);

    // raisim::Box *box = world.addBox(0.24,0.08,0.15,4.54);
    // box->setPosition(-0.005,0,0.25);
    // // vis->createGraphicalObject(box,"Payload","purple");

    // raisim::Box *box = world.addBox(0.04,0.04,0.01,20);
    // box->setPosition(0.,0,0.0);
    // // box->setPosition(-0.1,0,0.25);
    // vis->createGraphicalObject(box,"Payload","purple"); 

    auto& list = vis->getVisualObjectList();

    // ============================================================ //
    // ======================= SETUP Robot ======================== //
    // ============================================================ //
    std::vector<raisim::ArticulatedSystem*> A1;
    // A1.push_back(world.addArticulatedSystem(raisim::loadResource("Go1/Go1.urdf"))); // WHEN USING Go1, BE SURE TO CHANGE CMAKE TO USE CORRECT DYNAMICS
    A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified_new.urdf")));
    vis->createGraphicalObject(A1.back(), "A1");
    A1.back()->setGeneralizedCoordinate({0, 0, 0.12, 1, 0, 0, 0,
                                        0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6});
    A1.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    A1.back()->setName("A1_Robot");
    assignBodyColors(vis,"A1","collision","gray");

    LocoWrapper* loco_obj = new LocoWrapper(argc,argv);
    float a[3] = {1.00000000, -1.99555712, 0.99556697};
    float b[3] = {0.00000246, 0.00000492, 0.00000246};
    // float a[3] = {1.0, -1.14298050253990, 0.41280159809619};
    // float b[3] = {0.06745527388907, 0.13491054777814, 0.06745527388907};
    populate_filter_f(filt, a, b, 3, 2);


    // ============================================================ //
    // ================= VIEW AND RECORDING OPTIONS =============== //
    // ============================================================ //
    raisim::gui::showContacts = false;
    raisim::gui::showForces = false;
    raisim::gui::showCollision = false;
    raisim::gui::showBodies = true;

    std::string cameraview = "isoside";
    bool panX = true;                // Pan view with robot during walking (X direction)
    bool panY = false;                // Pan view with robot during walking (Y direction)
    bool record = false;             // Record?
    double startTime = 0*ctrlHz;    // Recording start time
    double simlength = 300*ctrlHz;   // Sim end time
    double fps = 30;            
    std::string directory = "/media/kavehakbarihamed/Data/A1_RaiSim_Outputs/LCSS_2021/";
    // std::string filename = "Payload_Inplace";
    std::string filename = "Payload_Trot_10cm_New";
    // std::string filename = "inplace_sim";

    // ============================================================ //
    // ========================= VIEW SETUP ======================= //
    // ============================================================ //
    // NOTE: Pi is defined in /dynamics/dynamicsSupportFunctions.h
    // NOTE: This section still needs some work. 
    if(cameraview == "iso"){
        // vis->getCameraMan()->getCamera()->setPosition(3, 3, 2);
        vis->getCameraMan()->getCamera()->setPosition(2, -1, 0.5);
        vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(5*Pi/6-Pi/2));
        vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
    }else if(cameraview == "isoside"){
        vis->getCameraMan()->getCamera()->setPosition(1.1, -2, 0.5);
        vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(4*Pi/6-Pi/2));
        vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
    }else if(cameraview == "side"){
        vis->getCameraMan()->getCamera()->setPosition(0, -2, 0.5);
        vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(0));
        vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
    }else if(cameraview == "front"){
        vis->getCameraMan()->getCamera()->setPosition(2, 0, 0.5);
        vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(Pi/2));
        vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
    }else if(cameraview == "top"){
        vis->getCameraMan()->getCamera()->setPosition(1, -3, 2.5);
        vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(1.0));
    }else{
        vis->getCameraMan()->getCamera()->setPosition(1, -3, 2.5);
        vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(1.0));
    }
    unsigned long mask = 0;
    if(raisim::gui::showBodies) mask |= raisim::OgreVis::RAISIM_OBJECT_GROUP;
    if(raisim::gui::showCollision) mask |= raisim::OgreVis::RAISIM_COLLISION_BODY_GROUP;
    if(raisim::gui::showContacts) mask |= raisim::OgreVis::RAISIM_CONTACT_POINT_GROUP;
    if(raisim::gui::showForces) mask |= raisim::OgreVis::RAISIM_CONTACT_FORCE_GROUP;
    vis->setVisibilityMask(mask);

    if(panX) raisim::gui::panViewX = panX;
    if(panY) raisim::gui::panViewY = panY;
    
    // ============================================================ //
    // ========================== RUN SIM ========================= //
    // ============================================================ //
    const std::string name = directory+filename+"_"+cameraview+".mp4";
    vis->setDesiredFPS(fps);
    long simcounter = 0;
    static bool added = false;
    while (!vis->getRoot()->endRenderingQueued() && simcounter <= simlength){

        size_t dist_start = 40000*ctrlHz;               // Start the disturbance (if any)
        size_t dist_stop  = dist_start+200;             // Stop the disturbance (if any)

        // FOR LCSS PAPER, KEEP FOR NOW.
        // if (simcounter>1500 & !added){
        //     box->setPosition(-0.1,0,0.30);
        //     added = true;
        // }
        // raisim::Vec<3> pos = box->getComPosition();
        // weightVis1->offset = {pos[0]-0.245/2,pos[1]-.078/2,pos[2]-0.078};
        // weightVis2->offset = {pos[0]-0.245/2,pos[1]-.078/2,pos[2]-0.015};
        // // std::cout<<pos[0]<<"\t"<<pos[1]<<"\t"<<pos[2]<<std::endl;

        controller(A1,loco_obj,simcounter);
        world.integrate();        
        
        if (simcounter%30 == 0)
            vis->renderOneFrame();
        
        if (!vis->isRecording() & record & simcounter>=startTime)
            vis->startRecordingVideo(name);
        
        auto currentPos = vis->getCameraMan()->getCamera()->getPosition();
        if (raisim::gui::panViewX){
            Eigen::VectorXd jointPosTotal(18 + 1);
            Eigen::VectorXd jointVelTotal(18);
            jointPosTotal.setZero();
            jointVelTotal.setZero();
            A1.back()->getState(jointPosTotal, jointVelTotal);
            if (cameraview=="front"){
                currentPos[0] = jointPosTotal(0)+2;
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            } else if(cameraview=="side"){
                currentPos[0] = jointPosTotal(0);
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            } else if(cameraview=="iso"){
                currentPos[0] = jointPosTotal(0)+2;
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            } else if(cameraview=="isoside"){
                currentPos[0] = jointPosTotal(0)+1.1;
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            }
        }
        if (raisim::gui::panViewY){
            Eigen::VectorXd jointPosTotal(18 + 1);
            Eigen::VectorXd jointVelTotal(18);
            jointPosTotal.setZero();
            jointVelTotal.setZero();
            A1.back()->getState(jointPosTotal, jointVelTotal);
            if (cameraview=="front"){
                currentPos[1] = jointPosTotal(1);
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            } else if(cameraview=="side"){
                currentPos[1] = jointPosTotal(1)-2;
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            } else if(cameraview=="iso"){
                currentPos[1] = jointPosTotal(1)-1;
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            } else if(cameraview=="isoside"){
                currentPos[1] = jointPosTotal(1)-2;
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            }
        }
        simcounter++;
    }

    // End recording if still recording
    if (vis->isRecording())
        vis->stopRecordingVideoAndSave();

    /// terminate the app
    vis->closeApp();

    delete loco_obj;
    clear_filter_f(filt);

    return 0;
}
