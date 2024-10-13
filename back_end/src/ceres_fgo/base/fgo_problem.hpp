#pragma once

#include "ceres/problem.h"
#include "edge.hpp"
#include "vertex.hpp"

namespace ceres
{
class FactorProblem
{
private:
    std::shared_ptr<Problem> problem;
    void addMeasurement(BaseBinaryEdge& edge){}
    void addMeasurement(BaseUnaryEdge& edge){}
    
public:
    FactorProblem(){}
    ~FactorProblem(){}
};



} // namespace ceres


