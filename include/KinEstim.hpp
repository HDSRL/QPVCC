#ifndef OTHERUTILS
#define OTHERUTILS

#include "math.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "A1_Dynamics.h"
#include "Transforms.hpp"
#include "global_loco_opts.h"

struct kinEstStruct
{
	double COM[3]= {0.0};
	double fr_prev[3] = {0.0};
	double fl_prev[3] = {0.0};
	double rr_prev[3] = {0.0};
	double rl_prev[3] = {0.0};
};

inline void kinEst(const int footForce[4], const int contactIndex[4], double q[18], double dq[18], Eigen::Matrix3d &R, kinEstStruct &estobj) {
    // ================================== //
	// ========= Con Estimator ========== //
	// ================================== //
    int actCon[4] = {0};
	float weightedCon[4];
	int thresh = 2000;
	float numContact = 0;
	for (int i=0; i<4; ++i){
		actCon[i] = (footForce[i]>thresh) ? 1 : 0;
		weightedCon[i] = actCon[i]+contactIndex[i];
		numContact += weightedCon[i];
	}

	// ================================== //
	// ========= Kin Estimator ========== //
	// ================================== //

	// toe pos
	double fr_toe[3], fl_toe[3], rl_toe[3], rr_toe[3];
	q[0] = 0; q[1] = 0; q[2] = 0;
	FK_FR_toe(fr_toe, q); FK_FL_toe(fl_toe, q);
	FK_RR_toe(rr_toe, q); FK_RL_toe(rl_toe, q);
	
	// update change in com pos
	double deltaPos[2] = {0.0};
	for(int i=0; i<2; ++i){
		deltaPos[i] += (estobj.fr_prev[i]-fr_toe[i])*weightedCon[0];
		deltaPos[i] += (estobj.fl_prev[i]-fl_toe[i])*weightedCon[1];
		deltaPos[i] += (estobj.rr_prev[i]-rr_toe[i])*weightedCon[2];
		deltaPos[i] += (estobj.rl_prev[i]-rl_toe[i])*weightedCon[3];
		deltaPos[i] /= numContact;
	}
	estobj.COM[0] += deltaPos[0];
	estobj.COM[1] += deltaPos[1];
	estobj.COM[2]  = -1.0*(fr_toe[2]*weightedCon[0]+fl_toe[2]*weightedCon[1]+rr_toe[2]*weightedCon[2]+rl_toe[2]*weightedCon[3])/numContact;
	
	for(int i=0; i<3; ++i){
		estobj.fr_prev[i] = fr_toe[i]; estobj.fl_prev[i] = fl_toe[i];
		estobj.rr_prev[i] = rr_toe[i]; estobj.rl_prev[i] = rl_toe[i];		
	}
	
	double Jfr_toe[54], Jfl_toe[54], Jrl_toe[54], Jrr_toe[54];
	double COM_vel[3] = {0,0,0};
	J_FR_toe(Jfr_toe, q); J_FL_toe(Jfl_toe, q);
	J_RR_toe(Jrr_toe, q); J_RL_toe(Jrl_toe, q);
	Eigen::Matrix<double,3,1> dq_temp = {dq[3],dq[4],dq[5]};
	toWorld(&dq[3],dq_temp,R);
	for (int i = 3; i < 18; ++i){
		COM_vel[0] -= (Jfr_toe[3*i+0]*weightedCon[0] + Jfl_toe[3*i+0]*weightedCon[1] + Jrr_toe[3*i+0]*weightedCon[2] + Jrl_toe[3*i+0]*weightedCon[3])*dq[i];
	 	COM_vel[1] -= (Jfr_toe[3*i+1]*weightedCon[0] + Jfl_toe[3*i+1]*weightedCon[1] + Jrr_toe[3*i+1]*weightedCon[2] + Jrl_toe[3*i+1]*weightedCon[3])*dq[i];
	 	COM_vel[2] -= (Jfr_toe[3*i+2]*weightedCon[0] + Jfl_toe[3*i+2]*weightedCon[1] + Jrr_toe[3*i+2]*weightedCon[2] + Jrl_toe[3*i+2]*weightedCon[3])*dq[i];
	}
	COM_vel[0] /= numContact;
	COM_vel[1] /= numContact;
	COM_vel[2] /= numContact;
	
	dq_temp = {dq[3],dq[4],dq[5]};
	toBody(&dq[3],dq_temp,R);

	// Set results
	q[0] = estobj.COM[0]; q[1] = estobj.COM[1]; q[2] = estobj.COM[2]+Z_TOE_OFFSET;
	dq[0] = COM_vel[0]; dq[1] = COM_vel[1]; dq[2] = COM_vel[2];
};


#endif