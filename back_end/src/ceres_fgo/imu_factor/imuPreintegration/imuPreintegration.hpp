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
    Scalar dt = 0.001;
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
    ~ImuPreintegration();

    void propagate(PreInterVar& v);
    void repropagate(const V3T& ba, const V3T& bg);
    void update(const V3T& ba, const V3T& bg);

private:
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
