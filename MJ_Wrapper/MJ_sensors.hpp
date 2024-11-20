#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Sparse"
#include "mujoco.h"

struct A1_MotorState
{
    double q;
    double dq;
    double ddq;
    double tauEst;
};

struct A1_IMU
{
    Eigen::Matrix<double, 4, 1> quaternion;
    Eigen::Matrix<double, 3, 1> gyroscope;
    Eigen::Matrix<double, 3, 1> accelerometer;
    Eigen::Matrix<double, 3, 1> rpy;
};

struct A1_LowState
{
    A1_IMU imu;
    A1_MotorState motorState[12];
};


inline void A1_ParseSensors(const mjModel* m, const mjData* d, A1_LowState *sensorData){
    for (int i=0; i<12; ++i){
        sensorData->motorState[i].q  = d->sensordata[i];
        sensorData->motorState[i].dq = d->sensordata[i+12];
    }
    sensorData->imu.accelerometer << d->sensordata[24],d->sensordata[25],d->sensordata[26];
    sensorData->imu.gyroscope << d->sensordata[27],d->sensordata[28],d->sensordata[29];
    sensorData->imu.quaternion << d->qpos[3],d->qpos[4],d->qpos[5],d->qpos[6];
}
