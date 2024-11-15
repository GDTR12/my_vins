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

    enum InterDeltaVar{
        IDX_P = 0,
        IDX_R = 3,
        IDX_V = 6,
        IDX_BA = 9,
        IDX_BG = 12
    };
    enum InterNoiseVar{
        IDX_N_AK = 0,
        IDX_N_WK = 3,
        IDX_N_AK_1 = 6,
        IDX_N_WK_1 = 9,
        IDX_N_BA = 12,
        IDX_N_BW = 15
    };

    struct PoseVar
    {
    public:
        PoseVar()
        {
            delete_mark = true;
            data_pq = new Scalar[7];
            data_vag = new Scalar[9];
        }
        PoseVar(Scalar* var_pq, Scalar* var_vag)
        {
            data_pq = var_pq;
            data_vag = var_pq;
            delete_mark = false;
        }
        ~PoseVar()
        {
            if (delete_mark)
            {
                delete[] data_pq;
                delete[] data_vag;
            }
        }
        V3T& p() {return *reinterpret_cast<V3T*>(data_pq);}
        QuaT& q() {return *reinterpret_cast<QuaT*>(data_pq + 3);}
        V3T& v() {return *reinterpret_cast<V3T*>(data_vag);}
        V3T& ba() {return *reinterpret_cast<V3T*>(data_vag + 3);}
        V3T& bg() {return *reinterpret_cast<V3T*>(data_vag + 6);}

    private:
        bool delete_mark = true;
        Scalar* data_pq;
        Scalar* data_vag;
    };





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
    Eigen::Matrix<Scalar, 15,1> evaluate(QuaT& q_i, V3T& p_i, V3T& v_i, V3T& bg_i,  V3T& ba_i,
                                         QuaT& q_j, V3T& p_j, V3T& v_j, V3T& bg_j,  V3T& ba_j,
                                         QuaT* q_ij_corrected = nullptr, V3T* p_ij_corrected = nullptr, V3T* v_ij_corrected = nullptr);

    void computePrevPoseJacobian(InterDeltaVar P_DX, Eigen::Matrix<Scalar, 15, 3>& jac,
                            PoseVar& Xi, PoseVar& Xj,
                            QuaT* q_ij_corrected = nullptr);
    void computeBackPoseJacobian(InterDeltaVar P_DX, Eigen::Matrix<Scalar, 15, 3>& jac,
                            PoseVar& Xi, PoseVar& Xj,
                            QuaT* q_ij_corrected = nullptr);

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


    Eigen::Matrix<Scalar, 15, 15> cov = Eigen::Matrix<Scalar, 15, 15>::Identity();
    Eigen::Matrix<Scalar, 18, 18> noise = Eigen::Matrix<Scalar, 18, 18>::Identity();
    Eigen::Matrix<Scalar, 15, 15> jac = Eigen::Matrix<Scalar, 15, 15>::Identity();
    std::vector<PreInterVar> imu_src;
    V3T bg = V3T::Zero();
    V3T ba = V3T::Zero();
};

}
