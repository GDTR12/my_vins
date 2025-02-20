#pragma once
#include <ceres/problem.h>
#include <ceres/manifold.h>
#include <utils/common/math_utils.hpp>
#include "my_vins_feature.hpp"
#include "ceres_fgo/base/vertex.hpp"
#include "ceres_fgo/base/edge.hpp"
#include "ceres_fgo/base/marginalization_problem.hpp"
#include "my_vins_param.hpp"

namespace my_vins
{


class PoseVertex : public ceres::BaseVertex
{
public:
    MyVinsFeatureManager* fm_ = nullptr;

    int idx_node_ = -1;

    CameraObserver* node_ = nullptr;

    static ceres::Manifold* pose_mainfold;

    PoseVertex(const std::string& id, int idx_node, MyVinsFeatureManager* fm)
        :ceres::BaseVertex(id, 7) 
    {
        idx_node_ = idx_node;
        fm_ = fm;
        node_ = &(fm_->getNodeAt<CameraObserver>(idx_node_));
        setMainfold(pose_mainfold);
        fetchData();
    }

    void fetchData()
    {
        Sophus::SE3d pose = node_->getSE3Position();
        q() = pose.unit_quaternion();
        p() = pose.translation();
    }

    void updateData() {node_->setPosition(q(), p());}

    Eigen::Map<QuaT> q(){return Eigen::Map<QuaT>(param_.data() + 3);}

    Eigen::Map<V3T> p(){return Eigen::Map<V3T>(param_.data());}

};

class VelBiasVertex : public ceres::BaseVertex
{
public:

    MyVinsFeatureManager* fm_ = nullptr;

    int idx_node_ = -1;

    CameraObserver* node_ = nullptr;

    VelBiasVertex(int idx_node, MyVinsFeatureManager* fm)
        :ceres::BaseVertex(9)
    {
        fm_ = fm;
        idx_node_ = idx_node;
        node_ = &(fm->getNodeAt<CameraObserver>(idx_node_));
        fetchData();
    }

    void fetchData()
    {
        v() = node_->vel();
        ba() = node_->ba();
        bg() = node_->bg();
    }

    void updateData()
    {
        node_->vel() = v();
        node_->ba() = ba();
        node_->bg() = bg();
    }

    Eigen::Map<V3T> v(){return Eigen::Map<V3T>(param_.data());}

    Eigen::Map<V3T> ba(){return Eigen::Map<V3T>(param_.data() + 3);}

    Eigen::Map<V3T> bg(){return Eigen::Map<V3T>(param_.data() + 6);}
};

class FeaturePointVertex: public ceres::BaseVertex
{
public:

    MyVinsFeatureManager* fm_ = nullptr;

    int idx_fea_ = -1;

    PointFeature* fea_;

    int idx_node_ = -1;

    int idx_ob_ = -1;

    FeaturePointVertex(const std::string id, MyVinsFeatureManager* fm, int idx_fea, int idx_node, int idx_ob, const Sophus::SE3d& T_ItoC) 
        :ceres::BaseVertex(id, 1)
    {
        fm_ = fm;
        idx_fea_ = idx_fea;
        fea_ = &(fm->getFeatureAt<PointFeature>(idx_fea_));
        idx_node_ = idx_node;
        idx_ob_ = idx_ob;
    }

    double& inv_d(){return param().x();}

    void fetchData(const Sophus::SE3d& T_ItoC)
    {
        V3T p_inW = fea_->getData();
        CameraObserver& node = fm_->getNodeAt<CameraObserver>(idx_node_);
        Sophus::SE3d T_WtoCi = node.getSE3Position() * T_ItoC;
        Sophus::SE3d T_CitoW = T_WtoCi.inverse();
        V4T p_inCi = T_CitoW.matrix() * (V4T() << p_inW, 1.0).finished();
        param().x() = 1.0 / p_inCi.head<3>().norm();
    }

    void updateData(const Sophus::SE3d& T_ItoC, const V4T& camera_mat, Sophus::SE3d T_WtoCi)
    {
        double depth = 1.0 / param().x();
        CameraObserver& node = fm_->getNodeAt<CameraObserver>(idx_node_);
        PointObservation& ob = node.getObservationAt(idx_ob_);

        V3T p_inCi = ob.toSphereObservation(camera_mat) / inv_d();
        V4T p_inW = T_WtoCi.matrix() * (V4T() << p_inCi, 1.0).finished();
        PointFeature& fea = fm_->getFeatureAt<PointFeature>(idx_fea_);
        fea.setData(p_inW.head<3>());
        fea.setOptimizationStatus(true);
    }
};

class MyVinsSlideWindow
{
public:
    MyVinsSlideWindow(MyVinsFeatureManager* feature_manager, const QuaT q_ItoC, const V3T t_ItoC);

    ~MyVinsSlideWindow();

    void marginalization();

    void step();

    MyVinsParamServer& param = MyVinsParamServer::getInstance();

    ceres::MarginalizationProblem problem;

    std::deque<std::pair<std::string, std::string>> win_nodes;

private:
    ceres::BaseVertex* vtx_T_ItoC;

    const std::string id_mar_edge = "MarEdge";

    const std::string id_T_ItoC_vtx = "T_ItoC_Vtx";

    MyVinsFeatureManager* fm;

    V4T camera_mat;
};

} // namespace my_vins



