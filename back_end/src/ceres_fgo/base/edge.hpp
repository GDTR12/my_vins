#pragma
#include "vertex.hpp"
#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/problem.h>

namespace ceres
{

class BaseEdge
{
private:
    ceres::CostFunction* costf_;
    ceres::LossFunction* lossf_;
    std::vector<std::string> vtxes_;
    std::string id_;
    ceres::ResidualBlockId res_id_;

public:
    BaseEdge(std::string id, ceres::CostFunction* costf, ceres::LossFunction* lossf, const std::vector<std::string>& vtxes){
        id_ = id;
        costf_ = costf;
        lossf_ = lossf;
        vtxes_ = vtxes;
    }
    ~BaseEdge(){}
    std::string id(){return id_;}
    ceres::ResidualBlockId& resId(){return res_id_;}
    ceres::CostFunction* costFunction(){return costf_;}
    ceres::LossFunction* lossFunction(){return lossf_;}
    const std::vector<std::string>& vertexes(){return vtxes_;}
    bool findVertex(const std::string id)
    {
        if (std::find(vtxes_.begin(), vtxes_.end(), id) != vtxes_.end()){
            return true;
        }
        return false;
    }
};


} // namespace ceres
