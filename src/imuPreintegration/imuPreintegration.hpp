#pragma once
#include <Eigen/Core>
#include "utils/input/input.hpp"
#include <sophus/se3.hpp>

namespace imu_preinter{

using V3d = Eigen::Vector3d;
using Quad = Eigen::Quaterniond;
using M3d = Eigen::Matrix3d;

struct StatusVar{
    V3d p;
    V3d v;
    V3d theta;
    V3d ba;
    V3d bw;
};

class ParamServer
{

public:
    V3d P_0inW;
    Quad q_0inW;
    V3d bias_a;

    ParamServer(){}
    ~ParamServer(){}
};





class ImuPreintegration: public ParamServer
{
private:
    slam_utils::ROSParamInput& param = slam_utils::ROSParamInput::getInstance();

public:
    ImuPreintegration();
    void calculateVariance(){
        double dt;
        V3d a_i, a_j, w_i, w_j, b_ai, b_wi;
        M3d R_i, R_j; 

        V3d w_hat = 0.5 * (w_i + w_j) - b_wi;

        Eigen::Matrix<double, 15, 15> F_i;
        Eigen::Matrix<double, 15, 18> B_i;
        F_i.setIdentity(); B_i.setZero();
        // f_12
        F_i.block<3,3>(0,3) = - 0.25 * dt * dt * (R_i * Sophus::SO3d::hat(a_i - b_ai) + R_j* Sophus::SO3d::hat(a_j - b_ai) * (M3d::Identity() - Sophus::SO3d::hat(w_hat)));
        // f_15
        F_i.block<3,3>(0,12) = 0.25 * dt * dt * dt * R_j * Sophus::SO3d::hat(a_j - b_ai);
        // f_32
        F_i.block<3,3>(6,3) = - 0.5 * dt * (R_i * Sophus::SO3d::hat(a_i - b_ai) + R_j * Sophus::SO3d::hat(a_j - b_ai) * (M3d::Identity() - Sophus::SO3d::hat(w_hat)));
        // f_35
        F_i.block<3,3>(6,12) = 0.5 * dt * dt * R_j * Sophus::SO3d::hat(a_j - b_ai);

        F_i.block<3,3>(0,6) = M3d::Identity() * dt;
        F_i.block<3,3>(0,9) = - 0.25 * (R_i + R_j) * dt * dt;

        F_i.block<3,3>(3,3) = M3d::Identity() - Sophus::SO3d::hat(w_hat) * dt;
        F_i.block<3,3>(3,12) = - M3d::Identity() * dt;

        F_i.block<3,3>(6,9) = - 0.5 * (R_i + R_j) * dt;

        // g_12
        M3d g12 = - 0.125 * dt * dt * dt * R_j * Sophus::SO3d::hat(a_j - b_ai);
        B_i.block<3,3>(0,3) = g12;
        // g_14
        B_i.block<3,3>(0,9) = g12;
        // g_32
        B_i.block<3,3>(6,3) = 2 * g12;
        // g_34
        B_i.block<3,3>(6,9) = 2 * g12;

        B_i.block<3,3>(0,0) = 0.25 * R_i * dt * dt;
        B_i.block<3,3>(0,6) = 0.25 * R_j * dt * dt;

        B_i.block<3,3>(3,3) = 0.5 * M3d::Identity() * dt;
        B_i.block<3,3>(3,9) = 0.5 * M3d::Identity() * dt;

        B_i.block<3,3>(6,0) = 0.5 * R_i * dt;
        B_i.block<3,3>(6,6) = 0.5 * R_j * dt;

        B_i.block<3,3>(9,12) = M3d::Identity() * dt;
        
        B_i.block<3,3>(12,15) = M3d::Identity() * dt;
    }
    void update();
    ~ImuPreintegration();
};

}
