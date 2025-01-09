#include "imuPreintegration.hpp"
#include "utils/common/math_utils.hpp"

namespace imu_preintegrate{


const Eigen::Vector3d gravity(0,0,9.8);

ImuPreintegration::ImuPreintegration()
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
    V3T delta_ba = new_ba - ba; V3T delta_bg = new_bg - bg;
    M3T dq_dbg = jac.block<3,3>(IDX_R, IDX_BG);

    M3T dp_dba = jac.block<3,3>(IDX_P, IDX_BA);
    M3T dp_dbg = jac.block<3,3>(IDX_P, IDX_BG);

    M3T dv_dba = jac.block<3,3>(IDX_V, IDX_BA);
    M3T dv_dbg = jac.block<3,3>(IDX_V, IDX_BG);

    p_itok = p_itok + dp_dba * delta_ba + dp_dbg * delta_bg;
    v_itok = v_itok + dv_dba * delta_ba + dv_dbg * delta_bg;

    V3T delta_q = 0.5 * dq_dbg * delta_bg;
    q_itok = q_itok * QuaT(1, delta_q.x(), delta_q.y(), delta_q.z());
    q_itok.normalize();
    ba = new_ba; bg = new_bg;
}


void ImuPreintegration::init(V3T& bg_, 
                V3T& ba_,
                Eigen::Matrix<Scalar, 15, 15>& cov_,
                Eigen::Matrix<Scalar, 15, 15>& jac_,
                Eigen::Matrix<Scalar, 18, 18>& noise_)
{
    bg = bg;
    ba = ba;
    cov = cov_;
    jac = jac_;
    noise = noise_;
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
    Scalar dt = v.t - lv.t;
    Scalar dtt = dt * dt;
    Scalar dttt = dtt * dt;

    M3T R_i = q_itok.toRotationMatrix();

    // 中值积分
    V3T a = 0.5 * (q_itok * (lv.a - ba));
    V3T w = 0.5 * (lv.w - bg + v.w - bg);
    V3T w_dt = 0.5 * dt * w;
    q_itok = q_itok * QuaT(1, w_dt.x(), w_dt.y(), w_dt.z());
    q_itok.normalize();
    M3T R_j = q_itok.toRotationMatrix();

    a = a + 0.5 * (q_itok * (v.a - ba));

    p_itok = p_itok + v_itok * dt + 0.5 * a * dtt;
    v_itok = v_itok + a * dt;
    total_t += dt;

    // 更新误差协方差矩阵
    Eigen::Matrix<Scalar, 15, 15> F;
    Eigen::Matrix<Scalar, 15, 18> G;

    M3T R_hat_ai = R_i * Sophus::SO3<Scalar>::hat(lv.a - ba);
    M3T R_hat_aj = R_j * Sophus::SO3<Scalar>::hat(v.a - ba);
    M3T hat_w = Sophus::SO3<Scalar>::hat(w);
    

    F.setIdentity(); G.setZero();
    // f_12
    F.block<3,3>(IDX_P,IDX_R) = - 0.25 * dtt * (R_hat_ai + R_hat_aj * (M3T::Identity() - hat_w));
    // f_15
    F.block<3,3>(IDX_P,IDX_BG) = 0.25 * dttt * R_hat_aj;
    // f_32
    F.block<3,3>(IDX_V,IDX_R) = - 0.5 * dt * (R_hat_ai + R_hat_aj * (M3T::Identity() - hat_w));
    // f_35
    F.block<3,3>(IDX_V,IDX_BG) = 0.5 * dtt * R_hat_aj;

    F.block<3,3>(IDX_P,IDX_V) = M3T::Identity() * dt;
    F.block<3,3>(IDX_P,IDX_BA) = - 0.25 * (R_i + R_j) * dtt;

    F.block<3,3>(IDX_R,IDX_R) = M3T::Identity() - hat_w * dt;
    F.block<3,3>(IDX_R,IDX_BG) = - M3T::Identity() * dt;

    F.block<3,3>(IDX_V,IDX_BA) = - 0.5 * (R_i + R_j) * dt;

    // g_12
    M3T g12 = - 0.125 * dttt * R_hat_aj;
    G.block<3,3>(IDX_N_AK,IDX_N_WK) = g12;
    // g_14
    G.block<3,3>(IDX_N_AK,IDX_N_WK_1) = g12;
    // g_32
    G.block<3,3>(IDX_N_AK_1,IDX_N_WK) = 2 * g12;
    // g_34
    G.block<3,3>(IDX_N_AK_1,IDX_N_WK_1) = 2 * g12;

    G.block<3,3>(IDX_N_AK,IDX_N_AK) = 0.25 * R_i * dtt;
    G.block<3,3>(IDX_N_AK,IDX_N_AK_1) = 0.25 * R_j * dtt;

    G.block<3,3>(IDX_N_WK,IDX_N_WK) = 0.5 * M3T::Identity() * dt;
    G.block<3,3>(IDX_N_WK,IDX_N_WK_1) = 0.5 * M3T::Identity() * dt;

    G.block<3,3>(IDX_N_AK_1,IDX_N_AK) = 0.5 * R_i * dt;
    G.block<3,3>(IDX_N_AK_1,IDX_N_AK_1) = 0.5 * R_j * dt;

    G.block<3,3>(IDX_N_WK_1,IDX_N_BA) = M3T::Identity() * dt;
    
    G.block<3,3>(IDX_N_BA,IDX_N_BW) = M3T::Identity() * dt;

    cov = F * cov * F.transpose() + G * noise * G.transpose();
    jac = F * jac;
}


Eigen::Matrix<ImuPreintegration::Scalar, 15,1> ImuPreintegration::evaluate(const ConstPoseVar& Xi, const ConstPoseVar& Xj,
                                                                           QuaT* q_ij_corrected, V3T* p_ij_corrected, V3T* v_ij_corrected)
{
    return evaluate(Xi.q(), Xi.p(), Xi.v(), Xi.bg(), Xi.ba(),
             Xj.q(), Xj.p(), Xj.v(), Xj.bg(), Xj.ba(),
             q_ij_corrected, p_ij_corrected, v_ij_corrected);
}                                                                           

Eigen::Matrix<ImuPreintegration::Scalar, 15,1> ImuPreintegration::evaluate(const QuaT& qi, const V3T& p_i, const V3T& v_i, const V3T& bg_i, const V3T& ba_i,
                                                                           const QuaT& qj, const V3T& p_j, const V3T& v_j, const V3T& bg_j,  const V3T& ba_j,
                                                                           QuaT* q_ij_corrected, V3T* p_ij_corrected, V3T* v_ij_corrected)
{
    Eigen::Matrix<Scalar, 15, 1> res;
    QuaT q_i = qi.normalized();
    QuaT q_j = qj.normalized();

    M3T dq_dbg = jac.block<3,3>(IDX_R, IDX_BG);

    M3T dp_dba = jac.block<3,3>(IDX_P, IDX_BA);
    M3T dp_dbg = jac.block<3,3>(IDX_P, IDX_BG);

    M3T dv_dba = jac.block<3,3>(IDX_V, IDX_BA);
    M3T dv_dbg = jac.block<3,3>(IDX_V, IDX_BG);

    V3T delta_q = 0.5 * dq_dbg * (bg_i - bg);
    QuaT q_ij = (q_ij * QuaT(1, delta_q.x(), delta_q.y(), delta_q.z())).normalized();
    V3T p_ij = p_itok + dp_dba * (ba_i - ba) + dp_dbg * (bg_i - bg);
    V3T v_ij = v_itok + dv_dba * (ba_i - ba) + dv_dbg * (bg_i - bg);

    res.head(IDX_P) = q_i.conjugate().toRotationMatrix() * (p_j - p_i - v_i * total_t + 0.5 * gravity * total_t * total_t) - p_ij;
    res.head(IDX_R) = 2 * (q_ij.conjugate() * (q_i.conjugate() * q_j)).vec();
    res.head(IDX_V) = q_i.conjugate() * (v_j - v_i + gravity * total_t) - v_ij;
    res.head(IDX_BA) = ba_j - ba_i;
    res.head(IDX_BG) = bg_j - bg_i;

    if (q_ij_corrected != nullptr){
        *q_ij_corrected = q_ij;
    }
    if (p_ij_corrected != nullptr){
        *p_ij_corrected = p_ij;
    }
    if (v_ij_corrected != nullptr){
        *v_ij_corrected = v_ij;
    }
    return res;
}                                     

 
Eigen::Matrix<ImuPreintegration::Scalar, 15, 3> ImuPreintegration::computePrevPoseJacobian(InterDeltaVar P_DX, 
                                                                                            ConstPoseVar& Xi, ConstPoseVar& Xj,
                                                                                            QuaT* q_ij_corrected)
{
    Eigen::Matrix<Scalar, 15, 3> jacobian = Eigen::Matrix<Scalar, 15, 3>::Zero();

    QuaT qij;
    if (q_ij_corrected != nullptr){
        qij = q_ij_corrected->normalized();
    }else {
        qij = q_itok;
    }

    QuaT qi = Xi.q().normalized();
    QuaT qj = Xj.q().normalized();
    M3T Ri = qi.toRotationMatrix();
    M3T Rj = qj.toRotationMatrix();
    switch (P_DX)
    {
    case IDX_P:
        jacobian.block<3, 3>(0,0) = -Ri.transpose();
        break;
    case IDX_R:
        jacobian.block<3,3>(IDX_P, 0) = Sophus::SO3<Scalar>::hat(Ri.transpose() * (Xj.p() - Xi.p() - Xi.v() * total_t + 0.5 * gravity * total_t * total_t));
        jacobian.block<3,3>(IDX_R, 0) = -(MathUtils::quaLeftMultiMat(qj.conjugate() * qi) * MathUtils::quaRightMultiMat(qij)).bottomRightCorner<3,3>();
        jacobian.block<3,3>(IDX_V, 0) = Sophus::SO3<Scalar>::hat(Ri.transpose()* (Xj.v() - Xi.v() + gravity * total_t));
        break;
    case IDX_V:
        jacobian.block<3,3>(IDX_P, 0) = - Ri.transpose() * total_t;
        jacobian.block<3,3>(IDX_V, 0) = - Ri.transpose();
        break;
    case IDX_BA:
        jacobian.block<3,3>(IDX_P, 0) = - jac.block<3,3>(IDX_P, IDX_BA);
        jacobian.block<3,3>(IDX_V, 0) = - jac.block<3,3>(IDX_V, IDX_BA);
        jacobian.block<3,3>(IDX_BA, 0) = - M3T::Identity();
        break;
    case IDX_BG:
        jacobian.block<3,3>(IDX_P, 0) = - jac.block<3,3>(IDX_P, IDX_BG);
        jacobian.block<3,3>(IDX_R, 0) = - MathUtils::quaLeftMultiMat(qj.conjugate() * qi * qij).bottomRightCorner<3,3>() * jac.block<3,3>(IDX_R, IDX_BG);
        jacobian.block<3,3>(IDX_V, 0) = - jac.block<3,3>(IDX_V, IDX_BG);
        jacobian.block<3,3>(IDX_BG, 0) = - M3T::Identity();
        break;
    default:
        break;
    }
    return jacobian;
}

Eigen::Matrix<ImuPreintegration::Scalar, 15, 3> ImuPreintegration::computeBackPoseJacobian(InterDeltaVar P_DX,
                                                ConstPoseVar& Xi, ConstPoseVar& Xj,
                                                QuaT* q_ij_corrected)
{
    Eigen::Matrix<Scalar, 15, 3> jacobian = Eigen::Matrix<Scalar, 15, 3>::Zero();

    QuaT qij;
    if (q_ij_corrected != nullptr){
        qij = q_ij_corrected->normalized();
    }else {
        qij = q_itok;
    }

    QuaT qi = Xi.q().normalized();
    QuaT qj = Xj.q().normalized();
    M3T Ri = qi.toRotationMatrix();
    M3T Rj = qj.toRotationMatrix();
    switch (P_DX)
    {
    case IDX_P:
        jacobian.block<3,3>(IDX_P, 0) = Ri.transpose();
        break;
    case IDX_R:
        jacobian.block<3,3>(IDX_R, 0) = MathUtils::quaLeftMultiMat(qij.conjugate() * qi.conjugate() * qj).bottomRightCorner<3,3>();
        break;
    case IDX_V:
        jacobian.block<3,3>(IDX_V, 0) = Ri.transpose();
        break;
    case IDX_BA:
        jacobian.block<3,3>(IDX_BA, 0) = M3T::Identity();
        break;
    case IDX_BG:
        jacobian.block<3,3>(IDX_BG, 0) = M3T::Identity();
        break;
    default:
        break;
    }
    return jacobian;
}

void ImuPreintegration::complete()
{
    info_mat = Eigen::LLT<Eigen::Matrix<Scalar, 15, 15>>(cov.inverse()).matrixL().transpose();
    complete_mark = true;
}


}