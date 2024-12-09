#pragma once
#include <ceres/problem.h>
#include "my_vins_feature.hpp"

namespace my_vins
{

class MyVinsSlideWindow
{
public:
    MyVinsSlideWindow(MyVinsFeatureManager* feature_manager);
    ~MyVinsSlideWindow();
    bool checkKeyFrame(int idx);
    void marginalization();
    void step();

    const int WIN_SIZE = 20;
private:
    MyVinsFeatureManager* fm;
    std::list<CameraObserver*> lst_nodes;
    int idx_slide = 0;
};

} // namespace my_vins



