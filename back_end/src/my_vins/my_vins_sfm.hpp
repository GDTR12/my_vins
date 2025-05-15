#pragma once
#include "my_vins.hpp"
#include "my_vins_vis.hpp"
#include <ceres/problem.h>


namespace my_vins
{
    
class MyVins;
class MyVinsVis;
class PointFeature;
class PointObservation;

class MyVinsSFM{
public:
    MyVinsSFM()=delete;
    MyVinsSFM(MyVins& vins, MyVinsVis& vis_);
    MyVinsSFM(MyVins& vins, MyVinsVis& vis_, M3T& camera_mat, V4T& distort_mat);

    V3T triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1, V2T pi, V2T pj);
    void triangulate(Eigen::Matrix<double, 3, 4> &Pose0, 
                    Eigen::Matrix<double, 3, 4> &Pose1,
                    std::vector<std::reference_wrapper<PointFeature>>& feas,
                    std::vector<std::reference_wrapper<PointObservation>>& observe_prev,
                    std::vector<std::reference_wrapper<PointObservation>>& observe_back);

    bool solvePnP(std::vector<std::reference_wrapper<PointObservation>>& Observation, std::vector<std::reference_wrapper<PointFeature>>& feas, M4T& T_0toi);

    bool solvePnPNew(std::vector<PointObservation*>& observe, std::vector<PointFeature*>& feas, M4T& T_ito0);

    bool initStructure();

    bool initStructureNew();

    void getMatches(uint32_t idx_prev,
                    uint32_t idx_back,
                    std::vector<std::reference_wrapper<PointFeature>>& feas,
                    std::vector<std::reference_wrapper<PointObservation>>& observe_prev,
                    std::vector<std::reference_wrapper<PointObservation>>& observe_back,
                    int mode);

    Scalar computeParllax(
                    std::vector<std::reference_wrapper<PointObservation>>& observe_prev,
                    std::vector<std::reference_wrapper<PointObservation>>& observe_back);

    bool computeRotateAndUnScaledTranslate(
                    std::vector<std::reference_wrapper<PointObservation>>& observe_prev,
                    std::vector<std::reference_wrapper<PointObservation>>& observe_back,
                    M3T& R_itoj,
                    V3T& t_itoj);
    bool computeRotateAndUnScaledTranslate(
                std::vector<std::reference_wrapper<PointObservation>>& observe_prev,
                std::vector<std::reference_wrapper<PointObservation>>& observe_back,
                M4T& T_itoj);

    void buildBA(std::vector<int>& indices_node);
    void globalBAAuto(int begin_idx, int end_idx);

    void globalBA(int idx_begin, int idx_end);

    void globalBAGTSAM(int idx_begin, int idx_end);

    void transformAllFramesToC0();

    bool solveNewFrameAt(int idx_node, const Sophus::SE3d& T_ItoC);

private:
    M3T camera_mat;
    V4T distort_vec;
    MyVins& vins;
    MyVinsVis& vis;
};


} // namespace my_vins
