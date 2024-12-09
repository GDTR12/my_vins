#include "my_vins_slidewindow.hpp"
#include "my_vins_feature.hpp"

namespace my_vins
{

MyVinsSlideWindow::MyVinsSlideWindow(MyVinsFeatureManager* fm_)
{
    fm = fm_;
}

MyVinsSlideWindow::~MyVinsSlideWindow(){}

bool MyVinsSlideWindow::checkKeyFrame(int idx)
{
    auto& pnode = fm->getNodeAt<CameraObserver>(idx - 1);
    auto& node = fm->getNodeAt<CameraObserver>(idx);
    // big motion 
    auto se3_prev = pnode.getSE3Position();
    auto se3_now = node.getSE3Position();
    // if (){
    //
    // }
    //
    // small quantity 

    // big interval

    // small shared features

    // big imu resdiual

    return true;
}

void MyVinsSlideWindow::marginalization()
{

}

void MyVinsSlideWindow::step()
{
    if (fm->getNodeSize() == 0) return;
    else if (idx_slide < fm->getNodeSize()){
        if (idx_slide == 0){
            lst_nodes.push_back(&fm->getNodeAt<CameraObserver>(idx_slide));
        }else if (checkKeyFrame(idx_slide)){
            lst_nodes.push_back(&fm->getNodeAt<CameraObserver>(idx_slide));
            if (lst_nodes.size() > WIN_SIZE){
                marginalization();
            }
        }
        idx_slide++;
    }
}

}
