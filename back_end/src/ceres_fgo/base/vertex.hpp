#pragma once

#include <Eigen/Core>
#include <ceres/manifold.h>

namespace ceres
{
class BaseVertex
{
protected:
    Eigen::VectorXd param_;
    std::string id_;
    ceres::Manifold* manifold_;
public:
    BaseVertex(std::string id, int size){
        param_.resize(size);
        id_ = id;
    }

    BaseVertex(int size){param_.resize(size);}

    ~BaseVertex(){}

    void initializeParameter(Eigen::VectorXd param, ceres::Manifold* manifold=nullptr)
    {
        param_ = param;
        manifold_ = manifold;
    }

    void initializeParameter(Eigen::VectorXd param) {param_ = param;}

    void setMainfold(ceres::Manifold* manifold){manifold_ = manifold;}

    std::string& id(){return id_;}

    int globalSize(){return param_.size();}

    int localSize(){return manifold_ == nullptr ? param_.size() : manifold_->TangentSize();}

    Eigen::VectorXd& param(){return param_;}

    ceres::Manifold* mainfold(){return manifold_;}
};


     
} // namespace ceres


