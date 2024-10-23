#pragma once
#include <Eigen/Core>
#include "utils/input/input.hpp"
#include <sophus/se3.hpp>

namespace imu_preinter{



// struct StatusVar{
//     V3d p;
//     V3d v;
//     V3d theta;
//     V3d ba;
//     V3d bw;
// };

// class ParamServer
// {

// public:
//     V3d P_0inW;
//     Quad q_0inW;
//     V3d bias_a;

//     ParamServer(){}
//     ~ParamServer(){}
// };

// template<typename Scalar>
struct PreInterVar{
    typedef double Scalar;
    using V3T = Eigen::Matrix<Scalar, 3, 1>;
    using QuaT = Eigen::Quaternion<Scalar>;
    using M3T = Eigen::Matrix<Scalar, 3, 3>;

    V3T w = V3T::Zero();
    V3T a = V3T::Zero();
    Scalar t = 0.00;
};


// template<typename Scalar>
class ImuPreintegration
{
public:
    typedef double Scalar;
    using V3T = Eigen::Matrix<Scalar, 3, 1>;
    using QuaT = Eigen::Quaternion<Scalar>;
    using M3T = Eigen::Matrix<Scalar, 3, 3>;

    ImuPreintegration();
    void init(V3T& bg_, 
                V3T& ba_,
                Eigen::Matrix<Scalar, 15, 15>& cov_,
                Eigen::Matrix<Scalar, 15, 15>& jac_,
                Eigen::Matrix<Scalar, 18, 18>& noise_);
    ~ImuPreintegration();

    void propagate(PreInterVar& v);
    void repropagate(const V3T& ba, const V3T& bg);
    void update(const V3T& ba, const V3T& bg);
    std::vector<PreInterVar>& getImuData(){return imu_src;};


    Eigen::VectorXd getStateVar()
    {
        Eigen::VectorXd ret;
        ret.conservativeResize(15);
        ret.block<3,1>(0,0) = p_itok;
        ret.block<3,1>(3,0) = v_itok;
        ret.block<3,1>(6,0) = Sophus::SO3d(q_itok).log();
        ret.block<3,1>(9,0) = bg;
        ret.block<3,1>(12,0) = ba;
        return ret;
    }

    V3T getBiasGyroscope(){return bg;}
    V3T getBiasAccel(){return ba;}

    // 积分变量
    V3T p_itok = V3T::Zero(), v_itok = V3T::Zero();
    QuaT q_itok = QuaT::Identity();
    Scalar total_t = 0;

    // 积分量对bias的雅克比
    M3T jac_pg = M3T::Identity();
    M3T jac_pa = M3T::Identity();
    M3T jac_vg = M3T::Identity();
    M3T jac_va = M3T::Identity();
    M3T jac_qg = M3T::Identity();

    Eigen::Matrix<Scalar, 15, 15> cov = Eigen::Matrix<Scalar, 15, 15>::Identity();
    Eigen::Matrix<Scalar, 18, 18> noise = Eigen::Matrix<Scalar, 18, 18>::Identity();
    Eigen::Matrix<Scalar, 15, 15> jac = Eigen::Matrix<Scalar, 15, 15>::Identity();
    std::vector<PreInterVar> imu_src;
    V3T bg = V3T::Zero();
    V3T ba = V3T::Zero();
};

}
