#pragma once

#include <ceres/problem.h>
#include "edge.hpp"
#include "vertex.hpp"

namespace ceres
{
class FGOProblem
{
protected:
    std::shared_ptr<Problem> problem;
    std::unordered_map<std::string, BaseVertex*> vtxes_;
    std::unordered_map<std::string, BaseEdge*> edges_;
    
public:
    FGOProblem(){}
    ~FGOProblem()
    {
        for (auto& [idx, edge] : edges_)
        {
            delete edge;
        }

        for (auto& [idx, vtx] : vtxes_)
        {
            delete vtx;
        }
    }

    /* Note: After this function completed, the ownership of vtx will be obtained */
    BaseVertex* addVertex(BaseVertex* vtx)
    {
        if (vtx != nullptr){
            if (vtx->globalSize() > 0 && vtxes_.find(vtx->id()) == vtxes_.end()){
                vtxes_[vtx->id()] = vtx;
                problem->AddParameterBlock(vtx->param().data(), vtx->globalSize(), vtx->mainfold());
                return vtx;
            }
            delete vtx;
        }
        return nullptr;
    }

    /* Note: After this function completed, the ownership of edge will be obtained */
    BaseEdge* addEdge(BaseEdge* edge)
    {
        if (edge != nullptr){
            if (edges_.find(edge->id()) == edges_.end()){
                if (edge->costFunction() != nullptr && edge->vertexes().size() > 0){
                    edges_[edge->id()] = edge;
                    std::vector<double*> param_block;
                    for (const auto& id_vtx : edge->vertexes()){
                        param_block.push_back(vtxes_[id_vtx]->param().data());
                    }
                    edge->resId() = problem->AddResidualBlock(edge->costFunction(), edge->lossFunction(), param_block);
                    return edge;
                }

                if (edge->costFunction() != nullptr){
                    delete edge->costFunction();
                }
                if (edge->lossFunction() != nullptr){
                    delete edge->lossFunction();
                }
            }
            delete edge;
        }
        return nullptr;
    }

    std::vector<std::string> findVertexAssociatedEdges(const std::string id_vtx, const std::string prefix="")
    {
        std::vector<std::string> ret;
        if (vtxes_.find(id_vtx) != vtxes_.end()){
            for (const auto [idx_edge, edge]: edges_){
                if (edge->id().find(prefix) == 0){
                    if (edge->findVertex(id_vtx)){
                        ret.push_back(idx_edge);
                    }
                }
            }
        }
        return ret;
    }

    /* TODO: implement this function */
    std::vector<std::string> findVertexAssociatedVertexes(const std::string id_vtx, const std::string prefix="")
    {
        
    }

    void removeEdge(const std::string id)
    {
        if (edges_.find(id) != edges_.end()){
            // problem will take the ownership of cost function and loss function
            problem->RemoveResidualBlock(edges_[id]->resId());
            delete edges_[id];
            edges_.erase(id);
        }
    }

    void removeVertex(const std::string id)
    {
        std::vector<std::string> edges = findVertexAssociatedEdges(id);
        for (const std::string& id_edge : edges)
        {
            removeEdge(id_edge);
        }
        if (vtxes_.find(id) != vtxes_.end()){
            problem->RemoveParameterBlock(vtxes_[id]->param().data());
            delete vtxes_[id];
            vtxes_.erase(id);
        }
    }

    int getVertexesParamGlobalSize(const std::vector<std::string>& vtxes)
    {
        int ret = 0;
        for (auto& vtx : vtxes)
        {
            ret += vtxes_[vtx]->globalSize();
        }
        return ret;
    }

    int getVertexesParamLocalSize(const std::vector<std::string>& vtxes)
    {
        int ret = 0;
        for (auto& vtx : vtxes)
        {
            ret += vtxes_[vtx]->localSize();
        }
        return ret;
    }

    BaseVertex* vertexAt(const std::string& id)
    {
        if (vtxes_.end() != vtxes_.find(id)){
            return vtxes_[id];
        }
        return nullptr;
    }

    BaseEdge* edgeAt(const std::string& id)
    {
        if (edges_.end() != edges_.find(id)){
            return edges_[id];
        }
        return nullptr;
    }

    int vertexSize(){return vtxes_.size();}

    int edgeSize(){return edges_.size();}

    ceres::Solver::Summary solve(const ceres::Solver::Options& opt)
    {
        ceres::Solver::Summary ret;
        ceres::Solve(opt, problem.get(), &ret);
        return ret;
    }
};

} // namespace ceres


