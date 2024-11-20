#ifndef LOCO_WRAPPER
#define LOCO_WRAPPER

#include "global_loco_structs.hpp"
#include "Parameters.hpp"
#include "RobotModel.hpp"
#ifdef USE_OSQP
#include "LowLevelCtrl_OSQP.hpp"
#else
#include "LowLevelCtrl.hpp"
#endif
#include "VirtualConstraints.hpp"
#include "ContactEst.hpp"
#include "MotionPlanner.hpp"
#include "DataLog.hpp"

#include <memory>

class LocoWrapper : public Parameters
{
public:
    LocoWrapper(int argc, char *argv[]);
    virtual ~LocoWrapper();

    void calcTau(const double q[18], const double dq[18], const double R[9], const int force[4], size_t gait, size_t ctrlTick);
    double* getTorque(){return LL->getTorque();};
    const int* getConDes(){return con->des;};
    Eigen::Matrix<double, 18, 1> getJointPosCmd(){return ll->q;};
    Eigen::Matrix<double, 18, 1> getJointVelCmd(){return ll->dq;};
    void initStandVars(Eigen::Matrix<double,3,1> com, double yaw, double standTime){ PP->updateStandVars(com,yaw,standTime);};
    void updateDesiredForce(Eigen::Matrix<double, 12, 1> fDes){VC->setDesiredForce(fDes);};
    void updateVel(const float vel[3]){PP->setVel(vel);};
    void updatePose(const float pose[6]){PP->setPose(pose);};
    void updatePoseType(size_t poseType_){PP->setPoseType(poseType_);};

    // Pointers to structs
    const StateInfo *state;
    const DynamicsInfo *dyn;
    const KinematicsInfo *kin;
    const ContactInfo *con;
    const TrajInfo *traj;
    const VCInfo *vcon;
    const LLInfo *ll;

private:
    // size_t newDom = 0;
    size_t locoTick = 0;
    double phaseVar = 0;
    double maxPhase = 1.05;
    size_t gaitTemp = STAND;
    size_t forceDomainChange = 0;

    // Pointers to class objects
    std::unique_ptr<DataLog> data;
    RobotModel *quad;
    LowLevelCtrl *LL;
    VirtualConstraints *VC;
    ContactEst *conEst;
    MotionPlanner *PP;
};

inline double getPhase(double time, double time_0, double time_f){
    return (1.0*time-1.0*time_0)/(1.0*time_f-1.0*time_0);
};

#endif
