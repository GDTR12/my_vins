#pragma once 
#include "feature_tracker.hpp"
#include "opencv2/features2d.hpp"

namespace feature_tracker
{
struct ORBFeature{
    cv::Mat img;
    cv::Mat des;
    std::vector<cv::KeyPoint> keys;
    ORBFeature(const cv::Mat& image){
        image.copyTo(img);
    }
};
    
class ORBTracker //: public FeatureTracker
{
private:
    int nfeatures = 2000, nlevels = 0.8, scaleFactor = 1.2, iniThFAST = 7, minThFSAT;
    std::vector<ORBFeature> feas;
public:
    void feature_extractor();
    int feedImg(const cv::Mat& img);
    ORBTracker();
    ~ORBTracker();
};


} // namespace my_vins
