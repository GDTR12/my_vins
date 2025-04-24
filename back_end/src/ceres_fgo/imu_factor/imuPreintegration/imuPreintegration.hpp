#pragma once
#include <Eigen/Core>
#include "utils/input/input.hpp"
#include <sophus/se3.hpp>

namespace imu_preintegrate{



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

    struct ConstPoseVar
    {
    public:

        ConstPoseVar(const Scalar* var_pq, const Scalar* var_vag) 
        {
            data_pq = var_pq;
            data_vag = var_pq;
        }

        ~ConstPoseVar(){}

        inline Eigen::Map<const V3T> p() const {return Eigen::Map<const V3T>(data_pq);}
        inline Eigen::Map<const QuaT> q() const {return Eigen::Map<const QuaT>(data_pq + 3);}
        inline Eigen::Map<const V3T> v() const {return Eigen::Map<const V3T>(data_vag);}
        inline Eigen::Map<const V3T> ba() const {return Eigen::Map<const V3T>(data_vag + 3);}
        inline Eigen::Map<const V3T> bg() const {return Eigen::Map<const V3T>(data_vag + 6);}



    private:
        const Scalar* data_pq;
        const Scalar* data_vag;
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
    Eigen::Matrix<Scalar, 15,1> evaluate(const QuaT& q_i, const V3T& p_i, const V3T& v_i, const V3T& bg_i, const V3T& ba_i,
                                         const QuaT& q_j, const V3T& p_j, const V3T& v_j, const V3T& bg_j, const V3T& ba_j,
                                         QuaT* q_ij_corrected = nullptr, V3T* p_ij_corrected = nullptr, V3T* v_ij_corrected = nullptr);
    
    Eigen::Matrix<Scalar, 15,1> evaluate(const ConstPoseVar& Xi, const ConstPoseVar& Xj,
                                         QuaT* q_ij_corrected = nullptr, V3T* p_ij_corrected = nullptr, V3T* v_ij_corrected = nullptr);

    Eigen::Matrix<Scalar, 15, 3> computePrevPoseJacobian(InterDeltaVar P_DX,
                                                        ConstPoseVar& Xi, ConstPoseVar& Xj,
                                                        QuaT* q_ij_corrected = nullptr);
    Eigen::Matrix<Scalar, 15, 3> computeBackPoseJacobian(InterDeltaVar P_DX, 
                                                        ConstPoseVar& Xi, ConstPoseVar& Xj,
                                                        QuaT* q_ij_corrected = nullptr);
    
    void complete();

    std::vector<PreInterVar>& getImuData(){return imu_src;};
    
    Eigen::Matrix<Scalar, 15, 15>& getCov(){return cov;}
    Eigen::Matrix<Scalar, 18, 18>& getNoise(){return noise;}
    Eigen::Matrix<Scalar, 15, 15>& getJac(){return jac;}
    Eigen::Matrix<Scalar, 15, 15>& getInfo(){return info_mat;}
    Scalar getTotalTime(){return total_t;}

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

    Eigen::Map<V3T> p() {return Eigen::Map<V3T>(p_itok.data());}
    Eigen::Map<QuaT> q() {return Eigen::Map<QuaT>(q_itok.coeffs().data());}
    Eigen::Map<V3T> v() {return Eigen::Map<V3T>(v_itok.data());}
    Eigen::Map<V3T> ba_() {return Eigen::Map<V3T>(ba.data());}
    Eigen::Map<V3T> bg_() {return Eigen::Map<V3T>(bg.data());}


    V3T getBiasGyroscope(){return bg;}
    V3T getBiasAccel(){return ba;}
    static void initializeNoise(double acc_n, double acc_w, double gyr_n, double gyr_w)
    {
        ACC_N = acc_n;
        ACC_W = acc_w;
        GYR_N = gyr_n;
        GYR_W = gyr_w;
        noise_initialized = true;
    }

private:
    // 积分变量
    static double ACC_N, ACC_W, GYR_N, GYR_W;
    static bool noise_initialized;

    V3T p_itok = V3T::Zero(), v_itok = V3T::Zero();
    QuaT q_itok = QuaT::Identity();
    Scalar total_t = 0;
    bool complete_mark = false;
    Eigen::Matrix<Scalar, 15, 15> info_mat = Eigen::Matrix<Scalar, 15, 15>::Identity();

    Eigen::Matrix<Scalar, 15, 15> cov = Eigen::Matrix<Scalar, 15, 15>::Zero();
    Eigen::Matrix<Scalar, 18, 18> noise = Eigen::Matrix<Scalar, 18, 18>::Identity();
    Eigen::Matrix<Scalar, 15, 15> jac = Eigen::Matrix<Scalar, 15, 15>::Identity();
    std::vector<PreInterVar> imu_src;
    V3T bg = V3T::Zero();
    V3T ba = V3T::Zero();
};

}
