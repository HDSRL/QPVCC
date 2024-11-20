#ifndef SHARED_DATA
#define SHARED_DATA

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Sparse"
#include "global_loco_opts.h"

#include "mutex"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#define SET_DATA 1
#define GET_DATA 0
#define HL_DATA 1
#define LL_DATA 0
boost::mutex mtx;

struct sharedData
{
	// Provided by LL
	size_t gait = STAND;
	double domLen = 200;
	double phaseVar = 0;
	int ind[4] = {1};
	double q[18] = {0};
	double dq[18] = {0};
	Eigen::Matrix<double, 400, 1> redDes = Eigen::Matrix<double, 400, 1>::Zero(); 
	Eigen::Matrix<double, 12, 1> QP_force = Eigen::Matrix<double, 12, 1>::Zero();
	Eigen::Matrix<double, 12, 25> comHist = Eigen::Matrix<double, 12, 25>::Zero(); 

	// Provided by HL
	Eigen::Matrix<double,  12, 25> comDes = Eigen::Matrix<double, 12, 25>::Zero();
	Eigen::Matrix<double,  12, 25> fDes   = Eigen::Matrix<double, 12, 25>::Zero();
	int MPC_cnt = 0;
};

sharedData data;

void updateData(int setget, int highlow, sharedData *newData){
	// set=1,  get=0
	// high=1, low=0
	// boost::lock_guard<boost::mutex> guard(mtx);
	if (setget==SET_DATA){
		if (highlow==HL_DATA){ // set high level data
			data.fDes = newData->fDes;
			data.comDes = newData->comDes;
			data.MPC_cnt = newData->MPC_cnt;
		}else{ // set low level data
			Eigen::Matrix<double, 12, 25> fDes_temp = data.fDes;
			Eigen::Matrix<double, 12, 25> comDes_temp = data.comDes;
			memcpy(&data,newData,sizeof(sharedData));
			data.fDes = fDes_temp;
			data.comDes = comDes_temp;
		}
	}else {
		if (highlow==HL_DATA){ // get data for high level
			Eigen::Matrix<double, 12, 25> fDes_temp = newData->fDes;
			Eigen::Matrix<double, 12, 25> comDes_temp = newData->comDes;
			memcpy(newData,&data,sizeof(sharedData));
			newData->fDes = fDes_temp;
			newData->comDes = comDes_temp;
		}else{ // get data for low level
			newData->fDes = data.fDes;
			newData->comDes = data.comDes;
			newData->MPC_cnt = data.MPC_cnt;
		}
	}
};


#endif
