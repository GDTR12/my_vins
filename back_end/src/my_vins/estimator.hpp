#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <my_vins/feature/feature_manager.hpp>
#include <my_vins/msg/feature_msg.hpp>
#include <thread>

namespace my_vins
{
class Estimator // : public rclcpp::Node
{
public:
    Estimator();
    ~Estimator();
    
    // void feaExtractCallback();
    void process();
    void localBA();
private:
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_fea;
    // rclcpp::Subscription<my_vins::msg::FeatureMsg>::SharedPtr sub_custfea;
    // rclcpp::Client<my_vins_msg::srv::PointFeatureExtract>::SharedPtr client_fea;
    my_vins::FeatureManager fea_manager;
    Eigen::Vector4d inner;
    Eigen::Matrix<double, 5, 1> distort;
    std::shared_ptr<std::thread> t_process;
};

 
} // namespace my_vins


