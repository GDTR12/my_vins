#pragma
#include "vertex.hpp"

namespace ceres
{

class BaseEdge
{
private:
    virtual void setMeasurement();
public:
    BaseEdge(){}
    ~BaseEdge(){}
};


class BaseUnaryEdge
{
private:
public:
    BaseUnaryEdge(){}
    ~BaseUnaryEdge(){}
};

class BaseBinaryEdge
{
private:
public:
    BaseBinaryEdge(){}
    ~BaseBinaryEdge(){}
};

} // namespace ceres
