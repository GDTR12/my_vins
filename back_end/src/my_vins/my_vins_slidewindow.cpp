#include "my_vins_slidewindow.hpp"
#include "my_vins_feature.hpp"
#include "ceres_fgo/vis_factor/vis_meas.hpp"

namespace my_vins
{

MyVinsSlideWindow::MyVinsSlideWindow(MyVinsFeatureManager* fm_)
{
    fm = fm_;
}

MyVinsSlideWindow::~MyVinsSlideWindow(){}


void MyVinsSlideWindow::marginalization()
{
    
}

void MyVinsSlideWindow::step()
{
    if (fm->getNodeSize() == 0) return;
    else if (idx_slide < fm->getNodeSize()){
        if (idx_slide == 0){
            lst_nodes.push_back(&fm->getNodeAt<CameraObserver>(idx_slide));
        }else{
            lst_nodes.push_back(&fm->getNodeAt<CameraObserver>(idx_slide));
            if (lst_nodes.size() > WIN_SIZE){
                marginalization();
            }
        }
        idx_slide++;
    }
}

}
