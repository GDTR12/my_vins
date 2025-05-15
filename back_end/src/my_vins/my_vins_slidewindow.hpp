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

    VelBiasVertex(const std::string& id, int idx_node, MyVinsFeatureManager* fm)
        :ceres::BaseVertex(9)
    {
        id_ = id;
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
        std::cout << "vel: " << v().transpose() << std::endl;
        std::cout << "bg: " << bg().transpose() << std::endl;
    }

    void updateData()
    {
        node_->vel() = v();
        node_->setBias(ba(), bg());
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


    static FeaturePointVertex* createInstance(const std::string id, MyVinsFeatureManager* fm, int idx_fea, int idx_node, int idx_ob, const Sophus::SE3d& T_ItoC)
    {
        // if (abnormal(fm, idx_fea, idx_node, idx_ob, T_ItoC)){
        //     return nullptr;
        // }else{
            return new FeaturePointVertex(id, fm, idx_fea, idx_node, idx_ob, T_ItoC);
        // }
    }

    // static bool abnormal(MyVinsFeatureManager* fm, int idx_fea, int idx_node, int idx_ob, const Sophus::SE3d& T_ItoC)
    // {
    //     V3T p_inW = fm->getFeatureAt<PointFeature>(idx_fea).getData();
    //     if (!fm->getFeatureAt<PointFeature>(idx_fea).is_initialized()) return true;
    //     CameraObserver& node = fm->getNodeAt<CameraObserver>(idx_node);       
    //     Sophus::SE3d T_WtoCi = node.getSE3Position() * T_ItoC;
    //     V4T p_inCi = T_WtoCi.inverse().matrix() * (V4T() << p_inW, 1.0).finished();
    //     // if (p_inCi.head<3>().norm() > 1e5) return true;
    //     PointObservation& ob = node.getObservationAt(idx_ob);
    //     std::cout << fm->getCameraMat().transpose() << std::endl;
    //     V3T ob_vec = ob.toSphereObservation(V4T{461.6, 460.3, 363.0, 248.1});
    //     // std::cout << "ob: " << ob.getData().transpose() << std::endl;
    //     // std::cout << "p_inCi: " << p_inCi.head<3>().transpose() << std::endl;
    //     // if (QuaT::FromTwoVectors(p_inCi.head<3>(), ob_vec).angularDistance(QuaT::Identity()) > 0.1){
    //     //     // std::cout << "obnormal at " << idx_fea << ": " << p_inCi.head<3>().transpose() << std::endl;
    //     //     return true;
    //     // }
    //     return false;
    // }

    double& inv_d(){return param().x();}

    void fetchData(const Sophus::SE3d& T_ItoC)
    {
        V3T p_inW = fea_->getData();
        CameraObserver& node = fm_->getNodeAt<CameraObserver>(idx_node_);
        Sophus::SE3d T_WtoCi = node.getSE3Position() * T_ItoC;
        Sophus::SE3d T_CitoW = T_WtoCi.inverse();
        V4T p_inCi = T_CitoW.matrix() * (V4T() << p_inW, 1.0).finished();
        param().x() = 1.0 / p_inCi.head<3>().norm();
        // std::cout << "fetch " << idx_fea_ << ": " << fea_->getData().transpose() << std::endl;
        // std::cout << "fetch " << idx_fea_ << " in " << idx_node_ << ": "  << param().x() << std::endl;
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
        // std::cout << "update " << idx_fea_ << ": " << p_inW.head<3>().transpose() << std::endl;
    }

private:
    FeaturePointVertex(const std::string id, MyVinsFeatureManager* fm, int idx_fea, int idx_node, int idx_ob, const Sophus::SE3d& T_ItoC) 
        :ceres::BaseVertex(id, 1)
    {
        fm_ = fm;
        idx_fea_ = idx_fea;
        fea_ = &(fm->getFeatureAt<PointFeature>(idx_fea_));
        idx_node_ = idx_node;
        idx_ob_ = idx_ob;
        fetchData(T_ItoC);
    }
};

class MyVinsSlideWindow
{
public:
    MyVinsSlideWindow(MyVinsFeatureManager* feature_manager, const QuaT q_ItoC, const V3T t_ItoC);

    ~MyVinsSlideWindow();

    void step();

    MyVinsParamServer& param = MyVinsParamServer::getInstance();

    ceres::MarginalizationProblem problem;

    std::deque<std::pair<std::string, std::string>> win_nodes;

    std::shared_ptr<std::vector<std::string>> new_feas_ = std::make_shared<std::vector<std::string>>();
    std::shared_ptr<std::vector<std::string>> lst_new_feas_ = std::make_shared<std::vector<std::string>>();

private:
    ceres::BaseVertex* vtx_T_ItoC;

    const std::string id_mar_edge = "MarEdge";

    const std::string id_T_ItoC_vtx = "T_ItoC_Vtx";

    MyVinsFeatureManager* fm;

    V4T camera_mat;
};

} // namespace my_vins



