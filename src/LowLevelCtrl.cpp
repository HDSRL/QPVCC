//
// Authror: Randy Fawcett on 12/2021.
//
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#include "LowLevelCtrl.hpp"

LowLevelCtrl::LowLevelCtrl(){
    P_QP.setZero();
    c_QP.setZero();
    A_QP.setZero();
    b_QP.setZero();
    G_QP.setZero();
    h_QP.setZero();
}

LowLevelCtrl::~LowLevelCtrl(){
};

// Main function
void LowLevelCtrl::calcTorque(const StateInfo *state, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, LLP *params){
    // ====================================================================== //
    // =============================== Setup ================================ //
    // ====================================================================== //
    size_t useCLF = params->useCLF;
    size_t conDim = 3*con->cnt;
    size_t outDim = 6+3*(4-con->cnt);
    size_t numDec = conDim+TOTAL_IN+outDim+useCLF;

    // ====================================================================== //
    // ============================ Solve the QP ============================ //
    // ====================================================================== //
    solve_qpSWIFT(params, dyn, kin, vc, con, outDim, conDim, numDec, useCLF);
    // saturateTorque();
    Eigen::Map<Eigen::Matrix<double, 18, 1>> tau_eig(tau,18,1);

    // ====================================================================== //
    // ============================ Swing Leg PD ============================ //
    // ====================================================================== //
    if (conDim<12){
        Eigen::MatrixXd Delta_temp = (kin->Js*dyn->Dinv*kin->Js.transpose());
        Eigen::MatrixXd Delta = Delta_temp.inverse();
        Eigen::MatrixXd p_d = Eigen::MatrixXd::Zero(12-conDim,1);
        Eigen::MatrixXd v_d = Eigen::MatrixXd::Zero(12-conDim,1);
        Eigen::MatrixXd Kp = Eigen::MatrixXd::Identity(12-conDim,12-conDim);
        double Kd = 40;
        double wd = 40;
        size_t cnts = 0;
        for (size_t i=0; i<4; i++){
            if(con->ind[i]==0){
                p_d.block(cnts,0,3,1) = (vc->hd.block(6+cnts,0,3,1)-kin->toePos.block(0,i,3,1));
                v_d.block(cnts,0,3,1) = (vc->dhd.block(6+cnts,0,3,1)-kin->Jtoe.block(3*i,0,3,TOTAL_DOF)*state->dq);
                Kp.block(cnts,cnts,3,3).diagonal() = wd*wd*Delta.block(cnts,cnts,3,3).diagonal();
                cnts+=3;
            }
        }
        tau_eig += kin->Js.transpose()*(Kp*p_d + Kd*v_d);
    }

    // ====================================================================== //
    // ======================= Calculate Joint Angles ======================= //
    // ====================================================================== //
    ll.ddq = dyn->Dinv*(dyn->B*tau_eig.block(6,0,12,1)+kin->Jtoe.transpose()*ll.QP_force - dyn->H);
    ll.dq = state->dq+ll.ddq/LL_Hz;
    ll.q  = state->q+ll.dq/LL_Hz+0.5/(LL_Hz*LL_Hz)*ll.ddq;
    if(conDim!=12){
        swingInvKin(state, dyn, kin, vc, con, params);
        for(int i=0; i<4; ++i){
            if (con->ind[i]==0){
                // tau_eig.block(6+3*i,0,3,1) += 20*(ll.q.block(6+3*i,0,3,1)-state->q.block(6+3*i,0,3,1))
                //                            + 1*(ll.dq.block(6+3*i,0,3,1)-state->dq.block(6+3*i,0,3,1));
                // std::cout<<state->q.block(6+3*i,0,3,1).transpose()<<std::endl;
                // std::cout<<ll.q.block(6+3*i,0,3,1).transpose()<<"\n\n"; 
           }
       }
    }

}

// qpSwift functions
void LowLevelCtrl::solve_qpSWIFT(LLP *params, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF){
    // ====================================================================== //
    // ============================ Solve the QP ============================ //
    // ====================================================================== //
    cost(params, vc, con, outDim, conDim, numDec, useCLF);
    constraints(params, dyn, kin, vc, con, outDim, conDim, numDec, useCLF);
    optimOut = new double[numDec];

    iswiftQp_e(P_QP.block(0,0,numDec,numDec), c_QP.block(0,0,numDec,1),
               A_QP.block(0,0,conDim+outDim,numDec), b_QP.block(0,0,conDim+outDim,1),
               G_QP.block(0,0,5*con->cnt+2*TOTAL_IN+useCLF,numDec), h_QP.block(0,0,5*con->cnt+2*TOTAL_IN+useCLF,1),
               optimOut);
    
    // ====================================================================== //
    // ======================== Parse the QP solution ======================= //
    // ====================================================================== //
    int cnt = 0;
    ll.QP_force.setZero();
    for(int i=0; i<4; ++i){
        if(con->ind[i]==1){
            for(int j=0; j<3; ++j){
                ll.QP_force(3*i+j) = optimOut[cnt];
                cnt++;
            }
        }
    }
    for(int i=0; i<TOTAL_IN; ++i){
        tau[6+i] = optimOut[cnt];
        ll.tau[6+i] = optimOut[cnt];
        cnt++;
    }
    if(useCLF){
        dV = LfV+Veps;
        for(int i=0; i<outDim; ++i){
            dV += LgV(i)*optimOut[cnt];
            cnt++;
        }
    }
    ll.dV = dV;
    delete[] optimOut;
}

void LowLevelCtrl::cost(LLP *params, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF){
    // ====================================================================== //
    // ============================ Cost Function =========================== //
    // ====================================================================== //
    P_QP.block(0,0,conDim,conDim) = params->dfPen*Eigen::MatrixXd::Identity(conDim, conDim);
    P_QP.block(conDim,conDim,TOTAL_IN,TOTAL_IN) = params->tauPen*Eigen::MatrixXd::Identity(TOTAL_IN,TOTAL_IN);
    P_QP.block(conDim+TOTAL_IN,conDim+TOTAL_IN,outDim,outDim) = params->auxPen*Eigen::MatrixXd::Identity(outDim,outDim);
    if (useCLF){
        P_QP(numDec-1, numDec-1) = params->clfPen;
    }

    c_QP.setZero();
    Eigen::MatrixXd Fd(conDim,1);
    int cnt = 0;
    for (int i=0; i<4; ++i){
    	if (con->ind[i]==1){
    		Fd.block(cnt,0,3,1) = vc->fDes.block(3*i,0,3,1);
    		cnt+=3;
    	}
    }	
    
    c_QP.block(0,0,conDim,1) = -Fd*params->dfPen;
}

void LowLevelCtrl::constraints(LLP *params, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF){  
    double mu = params->mu;
    double kpGain = params->kp;
    double kdGain = params->kd;
    // Eigen::MatrixXd kpGain, kdGain;
    // kpGain.setIdentity(outDim,outDim);
    // kdGain.setIdentity(outDim,outDim);
    // kpGain *= params->kp;
    // kdGain *= params->kd;
    // kpGain.block(3,3,3,3) = Eigen::Matrix<double,3,3>::Identity()*400;
    // kdGain.block(3,3,3,3) = Eigen::Matrix<double,3,3>::Identity()*40;

    
    // ====================================================================== //
    // ======================== Equality Constraints ======================== //
    // ====================================================================== //
    A_QP.block(0,0,conDim+outDim,numDec-useCLF) <<  
            (kin->Jc)*(dyn->Dinv)*(kin->Jc.transpose()), (kin->Jc)*(dyn->Dinv)*(dyn->B), Eigen::MatrixXd::Zero(conDim,outDim),
            (vc->H0)*(dyn->Dinv)*(kin->Jc.transpose()), (vc->H0)*(dyn->Dinv)*(dyn->B), Eigen::MatrixXd::Identity(outDim,outDim);
    b_QP.block(0,0,conDim+outDim,1) << (kin->Jc)*(dyn->Dinv)*(dyn->H) - (kin->dJc),
                                     (-kpGain*(vc->y)-kdGain*(vc->dy)) + (vc->H0)*(dyn->Dinv)*(dyn->H) - (vc->dH0);

    // ====================================================================== //
    // ======================= Inequality Constraints ======================= //
    // ====================================================================== //
    // Friction Cone
    Eigen::Matrix<double, 5, 3> gc;
    G_QP.setZero();
    h_QP.setZero();
    gc <<  1,  0, -mu/sqrt(2),
          -1,  0, -mu/sqrt(2),
           0,  1, -mu/sqrt(2),
           0, -1, -mu/sqrt(2),
           0,  0,          -1;
    repdiag(gc,G_QP,con->cnt);

    // Bounds
    G_QP.block(5*con->cnt,conDim,TOTAL_IN,TOTAL_IN) <<  Eigen::MatrixXd::Identity(TOTAL_IN,TOTAL_IN);
    G_QP.block(5*con->cnt+TOTAL_IN,conDim,TOTAL_IN,TOTAL_IN) << -Eigen::MatrixXd::Identity(TOTAL_IN,TOTAL_IN);
    h_QP.block(5*con->cnt,0,TOTAL_IN,1) << sat,sat,sat,sat;
    h_QP.block(5*con->cnt+TOTAL_IN,0,TOTAL_IN,1) << sat,sat,sat,sat;
}

// General functions
void LowLevelCtrl::saturateTorque(){
    // Joint Torque Saturation
    for(size_t i=0; i<4; i++){
        tau[6+3*i]   =   tau[6+3*i]>sat[0] ? sat[0] : ( (  tau[6+3*i]<-1*sat[0]) ? -1*sat[0] :   tau[6+3*i] );
        tau[6+3*i+1] = tau[6+3*i+1]>sat[1] ? sat[1] : ( (tau[6+3*i+1]<-1*sat[1]) ? -1*sat[1] : tau[6+3*i+1] );
        tau[6+3*i+2] = tau[6+3*i+2]>sat[2] ? sat[2] : ( (tau[6+3*i+2]<-1*sat[2]) ? -1*sat[2] : tau[6+3*i+2] );
    }
}

void LowLevelCtrl::swingInvKin(const StateInfo *state, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, LLP *params){
    // ===== General Equations ===== //
    // dxde_h = dxde - dxh      \dot{x}_des w.r.t hip where \dot{x}_des is desired foot vel (all in global)
    //  xde_h =  xde -  xh      x_des w.r.t hip where x_des is desired foot pos (all in global)
    //   xe_h =   xe -  xh      x w.r.t hip where x is the current foor pos (all in global)

    size_t swDim = 3*(4-con->cnt);
    size_t conDim = 12-swDim;
    Eigen::MatrixXd dxde_h = Eigen::MatrixXd::Zero(swDim,1);
    Eigen::MatrixXd xde_h  = Eigen::MatrixXd::Zero(swDim,1);
    Eigen::MatrixXd xe_h   = Eigen::MatrixXd::Zero(swDim,1);
    Eigen::MatrixXd Jtemp  = Eigen::MatrixXd::Zero(3,TOTAL_DOF);
    Eigen::MatrixXd Jtheta = Eigen::MatrixXd::Zero(swDim,3);
    Eigen::MatrixXd Jq     = Eigen::MatrixXd::Zero(swDim,swDim);
    Eigen::MatrixXd dq     = Eigen::MatrixXd::Zero(swDim,1);
    size_t cnts = 0;
    for (int i=0;  i<4; ++i){
        if(con->ind[i]==0){
            dxde_h.block(cnts,0,3,1) = vc->dhd.block(6+cnts,0,3,1) - (kin->Jhip.block(3*i,0,3,TOTAL_DOF)*state->dq);
            xde_h.block(cnts,0,3,1)  = vc->hd.block(6+cnts,0,3,1) - kin->hipPos.block(0,i,3,1);
            xe_h.block(cnts,0,3,1)   = kin->toePos.block(0,i,3,1) - kin->hipPos.block(0,i,3,1);
            Jtemp.block(0,0,3,TOTAL_DOF) = kin->Jtoe.block(3*i,0,3,TOTAL_DOF) - kin->Jhip.block(3*i,0,3,TOTAL_DOF);
            Jtheta.block(cnts,0,3,3) = Jtemp.block(0,3,3,3);
            Jq.block(cnts,cnts,3,3)  = Jtemp.block(0,6+3*i,3,3);
            cnts+=3;
        }
    }
    dq = Jq.inverse()*( dxde_h + 20*(xde_h - xe_h )- Jtheta*state->dq.block(3,0,3,1)); //

    cnts = 0;
    for (int i=0; i<4; ++i){
        if(con->ind[i]==0){
            ll.dq.block(6+3*i,0,3,1) = dq.block(cnts,0,3,1);
            ll.q.block(6+3*i,0,3,1)  = state->q.block(6+3*i,0,3,1)+dq.block(cnts,0,3,1)/LL_Hz;
            cnts+=3;
        }
    }
}

