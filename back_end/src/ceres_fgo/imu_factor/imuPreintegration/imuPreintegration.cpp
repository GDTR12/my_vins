#include "imuPreintegration.hpp"

namespace imu_preinter{

ImuPreintegration::ImuPreintegration(/* args */)
{
}

ImuPreintegration::~ImuPreintegration()
{
}


void ImuPreintegration::repropagate(const V3T& new_ba, const V3T& new_bg)
{
    ba = new_ba; bg = new_bg;  
    p_itok.setZero();
    v_itok.setZero();
    q_itok.setIdentity();
    cov.setZero();
    jac.setIdentity();

    for(auto& data : imu_src){
        propagate(data);
    }
}


void ImuPreintegration::update(const V3T& new_ba, const V3T& new_bg)
{
    V3T delta_ba = new_ba - ba; V3T delta_bg = bg - new_bg;
    p_itok = p_itok + jac_pa * delta_ba + jac_pg * delta_bg;
    v_itok = v_itok + jac_va * delta_ba + jac_vg * delta_bg;

    V3T delta_q = 0.5 * jac_qg * delta_bg;
    q_itok = q_itok * QuaT(1, delta_q.x(), delta_q.y(), delta_q.z());
    q_itok.normalize();
    ba = new_ba; bg = new_bg;
}

void ImuPreintegration::propagate(PreInterVar& v)
{
    PreInterVar lv;
    if (imu_src.empty()){
        lv = v;
    }else{
        lv = imu_src.back();
    }
    imu_src.push_back(v);
    Scalar dt = v.dt;
    Scalar dtt = dt * dt;
    Scalar dttt = dtt * dt;

    M3T R_i = q_itok.toRotationMatrix();

    // 中值积分
    V3T a = 0.5 * (q_itok * (lv.a - ba));
    q_itok = q_itok * QuaT(1, v.w.x(), v.w.y(), v.w.z());
    q_itok.normalize();
    M3T R_j = q_itok.toRotationMatrix();

    a = a + 0.5 * (q_itok * (v.a - ba));
    V3T w = 0.5 * (lv.w - bg + v.w - bg);

    p_itok = p_itok + v_itok * dt + 0.5 * a * dtt;
    v_itok = v_itok + a * dt;
    total_t += dt;

    // 更新误差协方差矩阵
    Eigen::Matrix<Scalar, 15, 15> F;
    Eigen::Matrix<Scalar, 15, 18> G;

    M3T R_hat_ai = R_i * Sophus::SO3<Scalar>::hat(lv.a - ba);
    M3T R_hat_aj = Sophus::SO3<Scalar>::hat(v.a - ba);
    M3T hat_w = Sophus::SO3<Scalar>::hat(w);
    

    F.setIdentity(); G.setZero();
    // f_12
    F.block<3,3>(0,3) = - 0.25 * dtt * (R_hat_ai + R_hat_aj * (M3T::Identity() - hat_w));
    // f_15
    F.block<3,3>(0,12) = 0.25 * dttt * R_hat_aj;
    // f_32
    F.block<3,3>(6,3) = - 0.5 * dt * (R_hat_ai + R_hat_aj * (M3T::Identity() - hat_w));
    // f_35
    F.block<3,3>(6,12) = 0.5 * dtt * R_hat_aj;

    F.block<3,3>(0,6) = M3T::Identity() * dt;
    F.block<3,3>(0,9) = - 0.25 * (R_i + R_j) * dtt;

    F.block<3,3>(3,3) = M3T::Identity() - hat_w * dt;
    F.block<3,3>(3,12) = - M3T::Identity() * dt;

    F.block<3,3>(6,9) = - 0.5 * (R_i + R_j) * dt;

    // g_12
    M3T g12 = - 0.125 * dttt * R_hat_aj;
    G.block<3,3>(0,3) = g12;
    // g_14
    G.block<3,3>(0,9) = g12;
    // g_32
    G.block<3,3>(6,3) = 2 * g12;
    // g_34
    G.block<3,3>(6,9) = 2 * g12;

    G.block<3,3>(0,0) = 0.25 * R_i * dtt;
    G.block<3,3>(0,6) = 0.25 * R_j * dtt;

    G.block<3,3>(3,3) = 0.5 * M3T::Identity() * dt;
    G.block<3,3>(3,9) = 0.5 * M3T::Identity() * dt;

    G.block<3,3>(6,0) = 0.5 * R_i * dt;
    G.block<3,3>(6,6) = 0.5 * R_j * dt;

    G.block<3,3>(9,12) = M3T::Identity() * dt;
    
    G.block<3,3>(12,15) = M3T::Identity() * dt;

    cov = F * cov * F.transpose() + G * noise * G.transpose();
    jac = F * jac;
}

}