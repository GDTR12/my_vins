#pragma once
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace slam_utils
{
typedef visualization_msgs::msg::Marker CameraRvizMsg;

class CameraRvizMsgGenerator
{
private: 
    Eigen::Quaterniond qua;
    Eigen::Vector3d trans;
public:
    void msgMaker(visualization_msgs::msg::Marker& msg){
        msg.type = visualization_msgs::msg::Marker::LINE_LIST;
        msg.color.r = msg.color.g = msg.color.b = msg.color.a = 1.0 ;
        msg.action = visualization_msgs::msg::Marker::ADD;
        msg.scale.x = 0.005;
        msg.scale.y = 0.005;
        msg.scale.z = 0.005;
        msg.ns = "line_list";
        std::vector<Eigen::Vector3d> point_list;
        const float scale_ = 0.1;
        point_list.push_back({-scale_, -scale_, scale_});
        point_list.push_back({scale_, -scale_, scale_});
        point_list.push_back({scale_, scale_, scale_});
        point_list.push_back({-scale_, scale_, scale_});
        // Eigen::Vector3d& camera_center = trans;
        qua.normalize();
        for (size_t i = 0; i < point_list.size(); i++){
            point_list[i] = qua.toRotationMatrix() * point_list[i] + trans;
        }
        // 绘制相机可视化方框
        geometry_msgs::msg::Point lp1, lp2;
        lp1.x = point_list[0].x(); lp1.y = point_list[0].y(); lp1.z = point_list[0].z();
        lp2.x = point_list[1].x(); lp2.y = point_list[1].y(); lp2.z = point_list[1].z();
        msg.points.push_back(lp1); msg.points.push_back(lp2);
        lp1.x = point_list[2].x(); lp1.y = point_list[2].y(); lp1.z = point_list[2].z();
        msg.points.push_back(lp2); msg.points.push_back(lp1);
        lp2.x = point_list[3].x(); lp2.y = point_list[3].y(); lp2.z = point_list[3].z();
        msg.points.push_back(lp1); msg.points.push_back(lp2);
        lp1.x = point_list[0].x(); lp1.y = point_list[0].y(); lp1.z = point_list[0].z();
        msg.points.push_back(lp2); msg.points.push_back(lp1);

        lp1.x = trans.x();  lp1.y = trans.y(); lp1.z = trans.z();
        lp2.x = point_list[0].x(); lp2.y = point_list[0].y(); lp2.z = point_list[0].z();
        msg.points.push_back(lp1); msg.points.push_back(lp2);
        lp2.x = point_list[1].x(); lp2.y = point_list[1].y(); lp2.z = point_list[1].z();
        msg.points.push_back(lp1); msg.points.push_back(lp2);
        lp2.x = point_list[2].x(); lp2.y = point_list[2].y(); lp2.z = point_list[2].z();
        msg.points.push_back(lp1); msg.points.push_back(lp2);
        lp2.x = point_list[3].x(); lp2.y = point_list[3].y(); lp2.z = point_list[3].z();
        msg.points.push_back(lp1); msg.points.push_back(lp2);
        msg.lifetime = rclcpp::Duration::from_seconds(0);  // 0 表示永久
    }
    CameraRvizMsgGenerator(Eigen::Quaterniond&& qua_, Eigen::Vector3d&& trans_)
     : CameraRvizMsgGenerator(qua_, trans_){}

    CameraRvizMsgGenerator(Eigen::Quaterniond& qua_, Eigen::Vector3d& trans_)
     :qua(qua_), trans(trans_){}


    ~CameraRvizMsgGenerator(){}
};


} // namespace slam_utils
