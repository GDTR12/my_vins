#include "vis_meas.hpp"
#include "sophus/se3.hpp"


namespace vis_meas
{

VisMeas::VisMeas(const V3T& p0, const V3T& p1)
{
    pi = p0;
    pj = p1;
    auto b = getSphereTangentOrthonormalBasis(pj);
    b12.block<3, 1>(0, 0) = b.first;
    b12.block<3, 1>(0, 1) = b.second;
}

VisMeas::~VisMeas()
{}


std::pair<V3T, V3T> VisMeas::getSphereTangentOrthonormalBasis(const V3T& pose) const
{
    std::pair<V3T, V3T> tangent_base;
    Eigen::Vector3d a = pj.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;
    tangent_base.first = (tmp - a * (a.transpose() * tmp)).normalized();
    tangent_base.second = a.cross(tangent_base.first);
    return tangent_base;
}


bool VisMeas::Evaluate(double const* const* params, double *residuals, double **jacobians) const
{
    M3T Ri = Eigen::Map<const Eigen::Quaterniond>(params[0] + 3).toRotationMatrix();
    M3T Rj = Eigen::Map<const Eigen::Quaterniond>(params[1] + 3).toRotationMatrix();
    M3T R_ItoC = Eigen::Map<const Eigen::Quaterniond>(params[2] + 3).toRotationMatrix();
    Eigen::Map<const Eigen::Matrix<double, 3, 1>> ti(params[0]);
    Eigen::Map<const Eigen::Matrix<double, 3, 1>> tj(params[1]);
    Eigen::Map<const Eigen::Matrix<double, 3, 1>> t_ItoC(params[2]);
    double inv_depth = *params[3];
    Eigen::Map<Eigen::Matrix<double, 2, 1>> res(residuals);

    M3T R1, R2, R3, R2T, R3T;
    V3T p1, p2, p3;
    R1 = Ri;
    R2 = Rj;
    R3 = R_ItoC;
    R2T = R2.transpose();
    R3T = R3.transpose();
    p1 = ti;
    p2 = tj;
    p3 = t_ItoC;
    double depth = 1. / inv_depth;
    using namespace Sophus;

    /* 计算残差 */ 
    V3T p_inCi = R_ItoC.transpose() * (Rj.transpose() * (Ri * (R_ItoC * (1.0f / inv_depth) * pi + t_ItoC) + ti - tj) - t_ItoC);
    res = b12.transpose() * (pj - p_inCi / p_inCi.norm());
    Eigen::Matrix<double, 2, 3> reduce;
    double pnorm = p_inCi.norm(); 
    reduce =  - (1.0f / (pnorm * pnorm)) * b12.transpose() * ( pnorm * M3T::Identity() -  p_inCi * p_inCi.transpose() / pnorm);

    if (jacobians != nullptr){
        if(jacobians[0] != nullptr){
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jac(jacobians[0]);
            jac.block<2,3>(0,0) = reduce * ( -R3T * R2T );
            jac.block<2,3>(0,3) = reduce *  ( - R3T * R2T * R1 * \
                            SO3d::hat( depth * R3 * pi + p3) );
            jac.block<2,1>(0,6).setZero();
        }
        if (jacobians[1] != nullptr){
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jac(jacobians[1]);
            jac.block<2,3>(0,0) = reduce * R3T * R2T;
            jac.block<2,3>(0,3) = reduce * R3T * SO3d::hat(R2T * (R1 * (depth * R2 * pi + p3) + p2 - p1));
            jac.block<2,1>(0,6).setZero();
        }
        if (jacobians[2] != nullptr){
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jac(jacobians[2]);
            jac.block<2,3>(0,0) = reduce * (R3T * R2T * R1 - R3T);
            jac.block<2,3>(0,3) = reduce * (- depth * SO3d::exp(R3T * SO3d(R2T * R1).log()).matrix()\
                 * SO3d::hat(pi) * SO3d::leftJacobianInverse(R3T * SO3d(R2T * R1).log()) * SO3d::hat(R3T * SO3d(R2T * R1).log()) \
                 + SO3d::hat(R3T * (R2T * R1 * p3 + R2T * p2 - R2T * p1 - p3)));
            jac.block<2,1>(0,6).setZero();
        }
        if (jacobians[3] != nullptr){
            Eigen::Map<V2T> jac(jacobians[3]);
            jac = reduce * R3T * R2T * R1 * R3 * pi;
        }
    }
    return true;
}


} // namespace vis_meas



