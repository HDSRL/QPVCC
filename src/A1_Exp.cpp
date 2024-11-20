//
// Authror: Randy Fawcett on 12/2021.
//
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#include "A1_Dynamics.h"
#include "LocoWrapper.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "timer.h"
#include "Transforms.hpp"
#include "Filters.h"

#include "stdio.h"
#include "fcntl.h"
#include "unistd.h"

#include "linux/input.h"
#include "sys/stat.h"
#include "fstream"

using namespace UNITREE_LEGGED_SDK;


class ExternalComm
{
public:
	ExternalComm() : udpComp(LOWLEVEL){
		fid = fopen("/home/kaveh/A1_exp_data/stateData_0003.csv", "w");

		// 60 Hz
		double ad[3] = {1.0, -1.47548044359265, 0.58691950806119};
		double bd[3] = {0.02785976611714, 0.05571953223427, 0.02785976611714};
		populate_filter_d(jointfilter,ad,bd,3,12);

		// 0.75 Hz
		float af[3] = {1.00000000,-1.99333570,0.99335783};
		float bf[3] = {0.00000553,0.00001107,0.00000553};
		populate_filter_f(remotefilter, af, bf, 3, 2);

		// 2 Hz 
		float aa[3] = {1.00000000,-1.98222893,0.98238545};
		float ba[3] = {0.00003913,0.00007826,0.00003913};
		populate_filter_f(angfilter, aa, ba, 3, 2);
		
	}
	virtual ~ExternalComm(){
		clear_filter_d(jointfilter);
		clear_filter_f(remotefilter);
		clear_filter_f(angfilter);
	}

	void Calc();

	std::unique_ptr<LocoWrapper> loco_obj;
	UDP udpComp;
	FILE *fid;

	long motiontime = 0;
	bool softFall = false;
	bool beginCommand = false;
	bool beginPose = false;
	bool setup = true;
	double q[18] = {0.0};
	double dq[18] = {0.0};
	double tauEst[12] = {0.0};

	FiltStruct_d* jointfilter  = (FilterStructure_d*)malloc(sizeof(FilterStructure_d));
	FiltStruct_f* angfilter    = (FilterStructure_f*)malloc(sizeof(FilterStructure_f));
	FiltStruct_f* remotefilter = (FilterStructure_f*)malloc(sizeof(FilterStructure_f));

	LowState state = {0};
	LowCmd cmd = {0};
	xRockerBtnDataStruct remote;

	int stop = 0;
	float vel[3] = {0};
	float pose[6] = {0};
	float ang[2] = {0};
	int actCon[4] = {0};

	float dt = 0.001f;
};

void ExternalComm::Calc()
{
//	timer tset;
//	tic(&tset);
	
	udpComp.Recv();
	udpComp.GetRecv(state);
	motiontime += 1;
	int standtime = 2000;
	int starttime = 1000;

	// ===================================================== //
	// ============== Wireless Remote Stuff  =============== //
	// ===================================================== //
	memcpy(&remote, state.wirelessRemote, 40);
	if((int)remote.btn.components.L2!=0){
		if((int)remote.btn.components.B!=0){
			stop = 1;
		}
		if((int)remote.btn.components.A!=0){
			softFall = true;
		}
	}
	if( ((int)remote.btn.components.A!=0) && ((int)remote.btn.components.L2==0) ){
		beginCommand = 1;
	}
	vel[0] = 0.75f*remote.ly; // x vel
	vel[1] = -0.4f*remote.rx; // y vel
	ang[0] = 20.0f*3.14f/180.0f*remote.ry; // yaw vel
	ang[1] = -2.0f*remote.lx; // pitch pos
	discrete_butter_f(remotefilter,vel);
	discrete_butter_f(angfilter,ang);
	pose[4] = ang[0]; // set filtered pose
	vel[2] = ang[1];  // set filtered ang vel

	// ===================================================== //
	// =========== Filter the joint pos and vel ============ //
	// ===================================================== //
	for (int i=0; i<12; ++i){
		q[i+6] = state.motorState[i].q;
		dq[i+6] = state.motorState[i].dq;
	}
	discrete_butter_d(jointfilter,&dq[6]);

	q[3] = state.imu.rpy[0]; q[4] = state.imu.rpy[1]; q[5] = state.imu.rpy[2];
	dq[3] = state.imu.gyroscope[0]; dq[4] = state.imu.gyroscope[1]; dq[5] = state.imu.gyroscope[2];
	
	
	quat_to_XYZ(state.imu.quaternion[0],state.imu.quaternion[1],state.imu.quaternion[2],state.imu.quaternion[3],
	            q[3],q[4],q[5]);
	q[4] += 0.0*MY_PI/180.0;

	double *rotMat;
	Eigen::Matrix3d R;
	quat_to_R(state.imu.quaternion,R);
	rotMat = R.data();

	// ================================== //
	// ========= Con Estimator ========== //
	// ================================== //
	int footForce[4];
	footForce[0] = state.footForce[0]; footForce[1] = state.footForce[1];
	footForce[2] = state.footForce[2]; footForce[3] = state.footForce[3];
	
	static int contactIndex[4] = {1,1,1,1};
	int thresh = 20;
	actCon[0] = (footForce[0]>thresh) ? 1 : 0;
	actCon[1] = (footForce[1]>thresh) ? 1 : 0;
	actCon[2] = (footForce[2]>thresh) ? 1 : 0;
	actCon[3] = (footForce[3]>thresh) ? 1 : 0;
	
	float weightedCon[4];
	weightedCon[0] = actCon[0]+contactIndex[0];
	weightedCon[1] = actCon[1]+contactIndex[1];
	weightedCon[2] = actCon[2]+contactIndex[2];
	weightedCon[3] = actCon[3]+contactIndex[3];
	float numContact = (weightedCon[0]+weightedCon[1]+weightedCon[2]+weightedCon[3]);

	// ================================== //
	// ========= Kin Estimator ========== //
	// ================================== //

	// toe pos
	double fr_toe[3], fl_toe[3], rl_toe[3], rr_toe[3];
	static double COM[3]= {0,0,0};
	q[0] = 0; q[1] = 0; q[2] = 0;
	FK_FR_toe(fr_toe, q); FK_FL_toe(fl_toe, q);
	FK_RR_toe(rr_toe, q); FK_RL_toe(rl_toe, q);
	
	// update change in com pos
	static double fr_prev[3] = {fr_toe[0],fr_toe[1],fr_toe[2]};
	static double fl_prev[3] = {fl_toe[0],fl_toe[1],fl_toe[2]};
	static double rr_prev[3] = {rr_toe[0],rr_toe[1],rr_toe[2]};
	static double rl_prev[3] = {rl_toe[0],rl_toe[1],rl_toe[2]};
	double deltaPos[2] = {0.0};
	for(int i=0; i<2; ++i){
		deltaPos[i] -= (fr_toe[i]-fr_prev[i])*weightedCon[0];
		deltaPos[i] -= (fl_toe[i]-fl_prev[i])*weightedCon[1];
		deltaPos[i] -= (rr_toe[i]-rr_prev[i])*weightedCon[2];
		deltaPos[i] -= (rl_toe[i]-rl_prev[i])*weightedCon[3];
		deltaPos[i] /= numContact;
	}
	COM[0] += deltaPos[0];
	COM[1] += deltaPos[1];
	COM[2]  = -1.0*(fr_toe[2]*weightedCon[0]+fl_toe[2]*weightedCon[1]+rr_toe[2]*weightedCon[2]+rl_toe[2]*weightedCon[3])/numContact;
	
	for(int i=0; i<3; ++i){
		fr_prev[i] = fr_toe[i]; fl_prev[i] = fl_toe[i];
		rr_prev[i] = rr_toe[i]; rl_prev[i] = rl_toe[i];		
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
	q[0] = COM[0]; q[1] = COM[1]; q[2] = COM[2];
	dq[0] = COM_vel[0]; dq[1] = COM_vel[1]; dq[2] = COM_vel[2];

	// ===================================================== //
	// ============= Quad Initialization time ============== //
	// ===================================================== //
	if ( (motiontime < starttime) ){
		for(int i=0; i<12; ++i){
			cmd.motorCmd[i].dq = 0.0;
			cmd.motorCmd[i].Kp = 0.0f;
			cmd.motorCmd[i].Kd = 0.0f;
			cmd.motorCmd[i].tau = 0.0f;
		}
	}

	// ===================================================== //
	// ================== LL Controller ==================== //
	// ===================================================== //
	static long int endStand = standtime;
	if ( motiontime >= starttime ){
		if (beginCommand & !softFall){
			double phaseVar;
			if (setup){
				Eigen::Matrix<double, 3, 1> com;
				com(0) = q[0], com(1) = q[1], com(2) = q[2];
				endStand = motiontime+standtime;
				loco_obj->initStandVars(com,q[5],standtime);
				setup = false;
			}
			
			beginPose = (motiontime<endStand) ? 0 : 1;
			if (beginPose == 0){
				loco_obj->calcTau(q, dq, rotMat, footForce, STAND, motiontime);
				if (motiontime >= endStand){
					static bool printedX = false;
					if (!printedX){
						printf("\nPress X to continue\n");
						printedX = true;
					}
				}
			}else if (beginPose == 1 && motiontime>=endStand){
				loco_obj->updateVel(vel);
				loco_obj->updatePose(pose);
				loco_obj->calcTau(q, dq, rotMat, footForce, WALK, motiontime);
				const int* contactMat = loco_obj->getConDes();
				contactIndex[0] = contactMat[0]; contactIndex[1] = contactMat[1];
				contactIndex[2] = contactMat[2]; contactIndex[3] = contactMat[3];
			}

			// Set the command
			for (int i = 0; i < 12; ++i){
				cmd.motorCmd[i].tau = loco_obj->ll->tau[i + 6];
				cmd.motorCmd[i].q  = loco_obj->ll->q(i + 6);
				cmd.motorCmd[i].dq = loco_obj->ll->dq(i + 6);

				cmd.motorCmd[i].Kp = 10;
				cmd.motorCmd[i].Kd = 8;
				if(beginPose==true){
					for(int i=0; i<4; ++i){
						if(contactIndex[i]==0){
//							cmd.motorCmd[i].tau = 0;
							cmd.motorCmd[3*i].Kp = 20;
							cmd.motorCmd[3*i+1].Kp = 20;
							cmd.motorCmd[3*i+2].Kp = 20;

							cmd.motorCmd[3*i].Kd = 1;
							cmd.motorCmd[3*i+1].Kd = 1;
							cmd.motorCmd[3*i+2].Kd = 1;  
						}
					}	
				}
			}

			// Saturate the command
			float hr_max = 8.0f,  hr_min = -8.0f;
			float hp_max = 30.0f, hp_min = -30.0f;
			float kn_max = 33.0f, kn_min = -33.0f;
			for (int i = 0; i < 4; i++){
				cmd.motorCmd[3 * i + 0].tau = (cmd.motorCmd[3 * i + 0].tau > hr_max) ? hr_max : cmd.motorCmd[3 * i + 0].tau;
				cmd.motorCmd[3 * i + 0].tau = (cmd.motorCmd[3 * i + 0].tau < hr_min) ? hr_min : cmd.motorCmd[3 * i + 0].tau;
				cmd.motorCmd[3 * i + 1].tau = (cmd.motorCmd[3 * i + 1].tau > hp_max) ? hp_max : cmd.motorCmd[3 * i + 1].tau;
				cmd.motorCmd[3 * i + 1].tau = (cmd.motorCmd[3 * i + 1].tau < hp_min) ? hp_min : cmd.motorCmd[3 * i + 1].tau;
				cmd.motorCmd[3 * i + 2].tau = (cmd.motorCmd[3 * i + 2].tau > kn_max) ? kn_max : cmd.motorCmd[3 * i + 2].tau;
				cmd.motorCmd[3 * i + 2].tau = (cmd.motorCmd[3 * i + 2].tau < kn_min) ? kn_min : cmd.motorCmd[3 * i + 2].tau;
			}

		}else if ( (!beginCommand) && (!softFall) ){
			static bool printedA = false;
			if(!printedA){
				printf("\nPress A to continue\n");
				printedA = true;
			}
		}else if ( softFall ) {
			for (int i = 0; i < 12; ++i){
				cmd.motorCmd[i].tau = 0.0;
				cmd.motorCmd[i].q   = 0.0;
				cmd.motorCmd[i].dq  = 0.0;

				cmd.motorCmd[i].Kp = 0;
				cmd.motorCmd[i].Kd = ( ( (i+1)%3 ) == 0 ) ? 6 : 3;
			}
		}
	}
	
	
	// for(int i=0; i<12; ++i){
	// 	fprintf(fid,"%0.6f,",state.motorState[i].q);
	// }
	// for(int i=0; i<12; ++i){
	// 	fprintf(fid,"%0.6f,",state.motorState[i].dq);
	// }
	// for(int i=0; i<3; ++i){
	// 	fprintf(fid,"%0.6f,",state.imu.accelerometer[i]);
	// }
	// for(int i=0; i<4; ++i){
	// 	fprintf(fid,"%i,",state.footForce[i]);
	// }
	// for(int i=0; i<12; ++i){
	// 	fprintf(fid,"%0.6f,",loco_obj->ll->ddq(i + 6));
	// }
	// fprintf(fid,"\n");

//	printf("%f\n",1000*toc(&tset));

	udpComp.SetSend(cmd);
	udpComp.Send();
}


int main(int argc, char *argv[])
{
	ExternalComm extComm;

	extComm.loco_obj = std::unique_ptr<LocoWrapper>(new LocoWrapper(argc, argv));

	LoopFunc loop_calc("calc_loop", extComm.dt, boost::bind(&ExternalComm::Calc, &extComm));
	loop_calc.start();

	InitEnvironment();
	extComm.udpComp.InitCmdData(extComm.cmd);
	
	while ( (extComm.stop==0) ){
		sleep(0.1);
	};

	std::cout<<"closing  file"<<std::endl;
	fclose(extComm.fid);
	return 0;
}




