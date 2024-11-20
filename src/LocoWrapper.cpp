//
// Authror: Randy Fawcett on 12/2021.
//
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#include "LocoWrapper.hpp"
#include "iostream"

LocoWrapper::LocoWrapper(int argc, char *argv[]) : Parameters(argc,argv){

   std::string filename = ""; // empty string will produce no output file
    
    data = std::unique_ptr<DataLog>( new DataLog(filename) ); // make_unique DNE in c++11
    quad = new RobotModel();
    conEst = new ContactEst();
    LL = new LowLevelCtrl();
    VC = new VirtualConstraints();
    PP = new MotionPlanner();

    state = quad->getStatePointer();
    dyn = quad->getDynamicsPointer();
    kin = quad->getKinematicsPointer();
    con = conEst->getConInfoPointer();
    traj = PP->getTrajInfoPointer();
    vcon = VC->getVCPointer();
    ll = LL->getllPointer();
    
    locoTick = 0;
    maxPhase = 1.05;
}

LocoWrapper::~LocoWrapper(){
    delete quad;
    delete conEst;
    delete LL;
    delete VC;
    delete PP;
}

void LocoWrapper::calcTau(const double q[18], const double dq[18], const double R[9], const int force[4], size_t gait, size_t ctrlTick){
    phaseVar = getPhase(1.0*locoTick, 0.0, 1.0*traj->domLen);   // update phase variable
    quad->updateState(q,dq,R);                                  // update state


    float footPos[4] = {0};  // DUMMY VARS
    if (gait!=gaitTemp || (phaseVar>maxPhase && gait!=STAND) ){ 
        // Change domain immediately since gait changed
        conEst->forceDomChange();                                                       // force con->changeDomain=1 to plan properly
        // std::cout << "Time trigger: " << phaseVar << std::endl;
        phaseVar = 0;
        PP->planTraj(state, kin, conEst, gait, phaseVar, ctrlTick, &motion_params);     // plan trajectory
        conEst->updateConState(footPos,phaseVar,force);                                 // update contact detection
        locoTick = 0;
    }else {
        // Wait for impact to change domain
        conEst->updateConState(footPos,phaseVar,force);                                 // impact detection
        if (con->changeDomain==1 && gait!=STAND){
            locoTick = 0;
            // std::cout << "Contact trigger: " << phaseVar << std::endl;
            phaseVar = 0;
        }
        PP->planTraj(state, kin, conEst, gait, phaseVar, ctrlTick, &motion_params);     // plan trajectory
    }
    
    
    quad->updateSwingMatrices(con->ind,con->cnt);                                               // update the jacobians
    VC->updateVirtualConstraints(state, kin, traj, con, gait, phaseVar, &motion_params, ll);    // update VC's
    LL->calcTorque(state, dyn, kin, vcon, con, &ll_params);                                     // run low level controller
    data->writeData(state,vcon,traj,ll,ctrlTick,force);                                         // log relavent data

    locoTick += (ctrlHz)/LL_Hz;     // increment locoTick
    gaitTemp = gait;                // update the previous gait used
}
