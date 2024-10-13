#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "my_vins/msg/feature_msg.hpp"
#include "utils/camera_rviz/camera_rviz.hpp"
#include "feature_tracker.hpp"

namespace feature_tracker
{


class KLTParamServer: public FeatureTracker
{
public:
    int MAX_CNT = 150;
    int MIN_DIST = 20;
    int F_THRESHOLD = 1.0;
    bool SHOW_MATCH = true;
    bool SHOW_SFM = true;
    const std::string namesp = "/KLT";
    KLTParamServer(): FeatureTracker(){
        this->declare_parameter(namesp + "/" + "max_cnt", MAX_CNT);
        this->declare_parameter(namesp + "/" + "min_dist", MIN_DIST);
        this->declare_parameter(namesp + "/" + "f_threshold", F_THRESHOLD);
        this->declare_parameter(namesp + "/" + "show_math", SHOW_MATCH);
        this->declare_parameter(namesp + "/" + "show_sfm", SHOW_MATCH);
    }
};


class KLT : public KLTParamServer
{
    typedef Feature KLTFeature;
private:

public:
    KLT();
    ~KLT();

    void featureExtractor(const cv::Mat& img, std::vector<cv::Point3f>& pts, cv::Mat& mask);
    int geneMatchedFeature(Feature& prev, Feature& cur, cv::Mat& f_mat);
    int featureMatch(Feature& prev, Feature& curr, std::map<int, int>& feas_map);
    int feedImg(const cv::Mat& img, rclcpp::Time t) override;
};



    
} // namespace feature_tracker


