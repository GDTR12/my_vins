#pragma once
#include "imuPreintegration/imuPreintegration.hpp"
#include "ceres/ceres.h"


namespace imu_preintegrate
{

class IMUCostFunction : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
{
public:
    typedef double Scalar;
    using V3T = Eigen::Matrix<Scalar, 3, 1>;
    using QuaT = Eigen::Quaternion<Scalar>;
    using M3T = Eigen::Matrix<Scalar, 3, 3>;
    IMUCostFunction() = delete;
    IMUCostFunction(ImuPreintegration* integrate):in(integrate){}
    ~IMUCostFunction(){}

    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const override final
    {
        ImuPreintegration::ConstPoseVar Xi(parameters[0], parameters[1]);
        ImuPreintegration::ConstPoseVar Xj(parameters[2], parameters[3]);
        Eigen::Map<Eigen::Matrix<Scalar, 15, 1>> res(residuals);
        
        QuaT qij_;
        V3T pij_, vij_;
        res = in->getInfo() * in->evaluate(Xi, Xj, &qij_, &pij_, &vij_);
        // std::cout << in-> getInfo() << std::endl;

        if (jacobians != nullptr){
            if (jacobians[0] != nullptr){
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jac(jacobians[0]);
                jac.block<15,3>(0,0) = in->computePrevPoseJacobian(ImuPreintegration::IDX_P, Xi, Xj, &qij_);
                jac.block<15,3>(0,3) = in->computePrevPoseJacobian(ImuPreintegration::IDX_R, Xi, Xj, &qij_);
                jac.rightCols<1>().setZero();
                jac = in->getInfo() * jac;
                // std::cout << std::scientific << std::endl;
                // std::cout << 0 << std::endl << jac << std::endl;
            }
            if (jacobians[1] != nullptr){
                Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jac(jacobians[1]);
                jac.block<15,3>(0,0) = in->computePrevPoseJacobian(ImuPreintegration::IDX_V, Xi, Xj, &qij_);
                jac.block<15,3>(0,3) = in->computePrevPoseJacobian(ImuPreintegration::IDX_BA, Xi, Xj, &qij_);
                jac.block<15,3>(0,6) = in->computePrevPoseJacobian(ImuPreintegration::IDX_BG, Xi, Xj, &qij_);
                jac = in->getInfo() * jac;

                // std::cout << std::scientific << std::endl;
                // std::cout << 1 << std::endl << jac << std::endl;
            }
            if (jacobians[2] != nullptr){
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jac(jacobians[2]);
                jac.block<15,3>(0,0) = in->computeBackPoseJacobian(ImuPreintegration::IDX_P, Xi, Xj, &qij_);
                jac.block<15,3>(0,3) = in->computeBackPoseJacobian(ImuPreintegration::IDX_R, Xi, Xj, &qij_);
                jac.rightCols<1>().setZero();
                jac = in->getInfo() * jac;

                // std::cout << std::scientific << std::endl;
                // std::cout << 2 << std::endl << jac << std::endl;
            }
            if (jacobians[3] != nullptr){
                Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jac(jacobians[3]);
                jac.block<15,3>(0,0) = in->computeBackPoseJacobian(ImuPreintegration::IDX_V, Xi, Xj, &qij_);
                jac.block<15,3>(0,3) = in->computeBackPoseJacobian(ImuPreintegration::IDX_BA, Xi, Xj, &qij_);
                jac.block<15,3>(0,6) = in->computeBackPoseJacobian(ImuPreintegration::IDX_BG, Xi, Xj, &qij_);
                jac = in->getInfo() * jac;

                // std::cout << std::scientific << std::endl;
                // std::cout << 3 << std::endl << jac << std::endl;
            }
        }
        return true;
    }

private:
    ImuPreintegration* in;
};


    
} // namespace imu_pre

