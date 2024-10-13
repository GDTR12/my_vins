#include "my_vins.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge/cv_bridge.h>

namespace my_vins
{
    

MyVinsVis::MyVinsVis(MyVins& myvins):
    rclcpp::Node("MyVinsVis"), vins(myvins)
{
    pub_feasmsg = this->create_publisher<sensor_msgs::msg::PointCloud2>("vis/features", 20000);
    pub_camermsg = this->create_publisher<slam_utils::CameraRvizMsg>("vis/camera", 5000);
    pub_pathmsg = this->create_publisher<nav_msgs::msg::Path>("vis/path", 5000);
    pub_matchmsg = this->create_publisher<sensor_msgs::msg::Image>("vis/match_msg", 500);
    path.header.frame_id = "world";
}

MyVinsVis::~MyVinsVis()
{}

void MyVinsVis::publishCamera(std::vector<std::pair<M4T, int>>& poses)
{
    for (auto& pose : poses)
    {
        publishOneCamera(pose.first, pose.second);
    }
}

void MyVinsVis::publishOneCamera(M4T& pose, int id)
{
    slam_utils::CameraRvizMsg msg_camera;
    msg_camera.header.frame_id = "world";
    msg_camera.header.stamp = this->get_clock()->now();
    msg_camera.id = id;
    slam_utils::CameraRvizMsgGenerator msg_gene(QuaT(pose.block<3,3>(0,0)), pose.block<3,1>(0,3));
    msg_gene.msgMaker(msg_camera);
    pub_camermsg->publish(msg_camera);
}

void MyVinsVis::publishPointFeature(std::vector<V3T>& feas)
{
    sensor_msgs::msg::PointCloud2 msg_fea;
    msg_fea.header.frame_id = "world";
    msg_fea.header.stamp = this->get_clock()->now();
    msg_fea.height = 1;
    msg_fea.width = feas.size();
    sensor_msgs::PointCloud2Modifier modifier(msg_fea);
    modifier.setPointCloud2Fields(3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32
        );
    sensor_msgs::PointCloud2Iterator<float> iter_x(msg_fea, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg_fea, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg_fea, "z");

    modifier.resize(feas.size());
    for (size_t i = 0; i < feas.size(); ++i, ++iter_x, ++iter_y, ++iter_z) 
    {
        *iter_x = feas[i].x();
        *iter_y = feas[i].y();
        *iter_z = feas[i].z();
    }
    pub_feasmsg->publish(msg_fea);
}

void MyVinsVis::visNodesWithFeas(std::vector<int>& indices_nodes)
{
    std::vector<std::pair<M4T, int>> camera_data;
    std::vector<V3T> fea_data;
    auto& fea_manager = vins.frame_manager;
    for (auto& idx : indices_nodes){
        CameraObserver& node = fea_manager.getNodeAt<CameraObserver>(idx);
        if (!node.is_initialized()){
            continue;
        }
        camera_data.push_back({node.getPosition(), idx});
        
        for (auto& observe : node.observes){
            PointFeature& fea = fea_manager.getFeatureAt<PointFeature>(observe->idx);
            if (!fea.is_initialized()){
                continue;
            }
            fea_data.push_back(fea.getData());
        }
    }
    publishCamera(camera_data);
    publishPointFeature(fea_data);
}

void MyVinsVis::visAllNodesWithFeas()
{
    std::vector<int> list;
    for (size_t i = 0; i < vins.frame_manager.getNodeSize(); i++)
    {
        list.push_back(i);
    }
    visNodesWithFeas(list);
}

void MyVinsVis::visAllNodesTracjectory()
{
    auto& fea_manager = vins.frame_manager;
    path.poses.clear();
    path.header.stamp = this->get_clock()->now();
    for (int i = 0; i < fea_manager.getNodeSize(); i++){
        CameraObserver& node = fea_manager.getNodeAt<CameraObserver>(i);
        if (!node.is_initialized()){
            continue;
        }
        M4T pose = node.getPosition();
        auto& p = path.poses.emplace_back();
        QuaT q(pose.block<3,3>(0,0));
        V3T t(pose.block<3,1>(0,3));
        p.pose.orientation.w = q.w();
        p.pose.orientation.x = q.x();
        p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z();
        p.pose.position.x = t.x();
        p.pose.position.y = t.y();
        p.pose.position.z = t.z();
        p.header.frame_id = "world";
        p.header.stamp = this->get_clock()->now();
    }
    std::cout << "path: " << path.poses.size() << std::endl;
    pub_pathmsg->publish(path);
}

void MyVinsVis::visAllFeatures()
{
    auto& fea_manager = vins.frame_manager;
    std::vector<V3T> feature_list;
    for (size_t i = 0; i < fea_manager.getFeatures().size(); i++)
    {
        PointFeature& fea = fea_manager.getFeatureAt<PointFeature>(i);
        feature_list.push_back(fea.getData());
    }
    publishPointFeature(feature_list);
}

void MyVinsVis::showTwoNodeMatches(int prev_idx, int back_idx)
{
    auto& fea_manager = vins.frame_manager;
    auto& node0 = fea_manager.getNodeAt<CameraObserver>(prev_idx);
    auto& node1 = fea_manager.getNodeAt<CameraObserver>(back_idx);
    cv::Mat mat_prev = node0.getImage();
    cv::Mat mat_back = node1.getImage();

    cv::Mat gray_mat;
    gray_mat.create(mat_prev.rows, mat_prev.cols * 2, CV_8UC1);
    cv::hconcat(mat_prev, mat_back, gray_mat);
    cv::Mat mat;
    mat.create(mat_prev.rows, mat_prev.cols * 2, CV_8UC3);
    cv::cvtColor(gray_mat, mat, cv::COLOR_GRAY2BGR);

    std::vector<uint32_t> indices_fea;
    std::vector<uint32_t> indices_prev;
    std::vector<uint32_t> indices_back;
    fea_manager.getMatches(prev_idx, back_idx, indices_fea, indices_prev, indices_back);
    for (size_t i = 0; i < indices_prev.size(); i++)
    {
        PointObservation& observe0 = *dynamic_cast<PointObservation*>(node0.observes[indices_prev[i]].get());
        PointObservation& observe1 = *dynamic_cast<PointObservation*>(node1.observes[indices_back[i]].get());
        cv::circle(mat, cv::Point2f(observe0.getData().x(), observe0.getData().y()), 2, cv::Scalar(0,0,255), 2);
        cv::circle(mat, cv::Point2f(observe1.getData().x() + mat_prev.cols, observe1.getData().y()), 2, cv::Scalar(0,0,255), 2);
        cv::line(mat, cv::Point2f(observe0.getData().x(), observe0.getData().y()), 
            cv::Point2f(observe1.getData().x() + mat_prev.cols, observe1.getData().y()), cv::Scalar(0,255,0));
    }
    cv_bridge::CvImage bridge;
    bridge.encoding = "bgr8";
    bridge.image = mat;

    sensor_msgs::msg::Image msg_img;
    msg_img.header.stamp = this->get_clock()->now();
    bridge.toImageMsg(msg_img);
    pub_matchmsg->publish(msg_img);
    
}

void MyVinsVis::visCamearaNodesBetween(int idx_prev, int idx_back)
{
    std::vector<std::pair<M4T, int>> poses;
    auto& fea_manager = vins.frame_manager;
    for (size_t i = idx_prev; i <= idx_back; i++){
        CameraObserver& node = fea_manager.getNodeAt<CameraObserver>(i);
        if (!node.is_initialized()){
            continue;
        }
        poses.push_back({node.getPosition(), i});
    }
    
    for (auto& pose : poses)
    {
        publishOneCamera(pose.first, pose.second);
    }
}

} // namespace my_vins
