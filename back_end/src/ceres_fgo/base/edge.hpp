#pragma once
#include "vertex.hpp"
#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/problem.h>

namespace ceres
{

class BaseEdge
{
protected:
    ceres::CostFunction* costf_ = nullptr;
    ceres::LossFunction* lossf_ = nullptr;
    std::vector<std::string> vtxes_;
    std::string id_ = "";
    ceres::ResidualBlockId res_id_;

public:
    BaseEdge(std::string id, ceres::CostFunction* costf, ceres::LossFunction* lossf, const std::vector<std::string>& vtxes)
    {
        id_ = id;
        costf_ = costf;
        lossf_ = lossf;
        vtxes_ = vtxes;
    }

    BaseEdge(){}

    ~BaseEdge(){}

    void setId(const std::string id){id_ = id;}

    void setCostFunction(ceres::CostFunction* costf) {costf_ = costf;}

    void setLossFunction(ceres::LossFunction* lossf){lossf_ = lossf;}

    void setVertexes(const std::vector<std::string>& vtxes){vtxes_ = vtxes;}

    void set(std::string id, ceres::CostFunction* costf, ceres::LossFunction* lossf, const std::vector<std::string>& vtxes)
    {
        setId(id);
        setCostFunction(costf);
        setLossFunction(lossf);
        setVertexes(vtxes);
    }

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
