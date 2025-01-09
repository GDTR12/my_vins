#pragma once
#include "sophus/se3.hpp"
#include "Eigen/Core"
#include "ceres/sized_cost_function.h"

namespace vis_meas
{

using V2T = Eigen::Vector2d;
using V3T = Eigen::Vector3d;
using V4T = Eigen::Vector4d;
using M3T = Eigen::Matrix3d;

class VisMeas: public ceres::SizedCostFunction<2, 7, 7, 7, 1>
{
public:
    VisMeas(const V3T& p0, const V3T& p1, const V4T& camera_mat);
    ~VisMeas();

    std::pair<V3T, V3T> getSphereTangentOrthonormalBasis(const V3T& pose) const;
    virtual bool Evaluate(double const* const* params, double *residuals, double **jacobians) const override;
private:
    V3T pi, pj;
    V4T c_mat;
    Eigen::Matrix<double, 3, 2> b12;
};

} // namespace vis_meas

