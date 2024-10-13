#pragma once

#include "my_vins.hpp"
#include "utils/camera_rviz/camera_rviz.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>

namespace my_vins
{

class MyVins;

class MyVinsVis: public rclcpp::Node
{
public: 
    MyVinsVis()=delete;
    MyVinsVis(MyVins& vins);
    ~MyVinsVis();

    void visNodesWithFeas(std::vector<int>& indices);

    void visAllNodesWithFeas();

    void visCamearaNodesBetween(int prev_idx, int back_idx);

    void visAllNodesTracjectory();
    
    void visAllFeatures();

    void showTwoNodeMatches(int prev_idx, int back_idx);

    void publishCamera(std::vector<std::pair<M4T, int>>& poses);

    void publishOneCamera(M4T& poses, int id);

    void publishPointFeature(std::vector<V3T>& feas);
private:
    MyVins& vins;

    rclcpp::Publisher<slam_utils::CameraRvizMsg>::SharedPtr pub_camermsg;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_feasmsg;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_pathmsg;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_matchmsg;
    nav_msgs::msg::Path path;
};



} // namespace my_vins
