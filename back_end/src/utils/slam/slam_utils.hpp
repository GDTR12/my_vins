#pragma once

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace slam_utils
{

using V3T = Eigen::Vector3d;
using QuaT = Eigen::Quaterniond;

struct ImuInterpData
{
    V3T w,a;
    V3T ba, bw;
    double time;
};

ImuInterpData ImuLinearInterp(ImuInterpData data0, ImuInterpData data1, double t)
{
    ImuInterpData ret;
    double dt = (t - data0.time);
    double ratio_t = (t - data0.time) / (data1.time - data0.time);
    ret.w = data0.w + ratio_t * (data1.w - data1.bw - data0.w + data0.bw); // 中值积分的位置
    ret.a = data0.a + ratio_t * (data1.a - data1.ba - data0.a + data0.ba); // 
    ret.ba = data0.ba + ratio_t * (data1.ba - data0.ba);
    ret.bw = data0.bw + ratio_t * (data1.bw - data0.bw);
    return ret;
}

} // namespace slam_utils


