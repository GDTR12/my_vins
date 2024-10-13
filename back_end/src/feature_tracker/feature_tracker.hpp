#pragma once 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "my_vins/msg/feature_msg.hpp"
#include <utils/input/input.hpp>
#include "utils/camera_rviz/camera_rviz.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace feature_tracker
{
struct Feature{
    cv::Mat img;
    rclcpp::Time time;
    std::vector<cv::Point3f> pts; // u, v, depth
    std::map<int, int> map_to_prev;
    Eigen::Quaterniond q_itoj = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t_itoj = Eigen::Vector3d::Zero();

    Feature(Feature&) = delete;
    Feature(const cv::Mat& img_, rclcpp::Time& t){img = img_; time = t;}
};


class FeatureTracker : public rclcpp::Node
{
private:

public:
    cv::Mat camera_mat, distort_coeffs;
    std::deque<Feature> feas;
    std::deque<sensor_msgs::msg::Image::SharedPtr> img_src;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_show;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_feas;
    rclcpp::Publisher<my_vins::msg::FeatureMsg>::SharedPtr pub_custfeas;
    rclcpp::Publisher<slam_utils::CameraRvizMsg>::SharedPtr pub_camermsg;

    FeatureTracker():rclcpp::Node("FeatureTracker"){
        auto& input = slam_utils::ROSParamInput::getInstance();
        std::string topic_img = input.getConfigParam<std::string>("camera_topic");
        std::string topic_feas = input.getConfigParam<std::string>("feas_topic");
        std::string topic_cust_feas = input.getConfigParam<std::string>("feas_cust_topic");
        std::string topic_camera_pos = input.getConfigParam<std::string>("front_end_camera_pos");

        std::vector<double> param(5, 0);
        input.getConfigParam<std::vector<double>>("projection_parameters", param);
        camera_mat = (cv::Mat_<double>(3,3) << 
            param[0], 0, param[2],
            0, param[1], param[3],
            0,  0,  0
        );
        input.getConfigParam<std::vector<double>>("distortion_parameters", param);
        distort_coeffs = (cv::Mat_<double>(4,1) << param[0], param[1], param[2], param[3]);

        pub_show = this->create_publisher<sensor_msgs::msg::Image>("matched_show", 10);
        pub_feas = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_feas, 60);
        pub_custfeas = this->create_publisher<my_vins::msg::FeatureMsg>(topic_cust_feas, 60);
        pub_camermsg = this->create_publisher<slam_utils::CameraRvizMsg>(topic_camera_pos, 60);
    }
    ~FeatureTracker(){}

    virtual int feedImg(const cv::Mat& img, rclcpp::Time t) = 0;

    void pubCustomFeatures(rclcpp::Time& t, Feature& curr_fea, Feature& prev_fea)
    {
        my_vins::msg::FeatureMsg feas_msg;
        feas_msg.header.frame_id = "camera";
        feas_msg.header.stamp = t;
        feas_msg.idx = feas.size() - 1;
        Eigen::Quaterniond q = curr_fea.q_itoj;
        Eigen::Vector3d p =  curr_fea.t_itoj;
        feas_msg.q_itoj = {q.x(), q.y(), q.z(), q.w()};
        feas_msg.t_itoj = {p.x(), p.y(), p.z()};

        feas_msg.feature.height = 1;
        feas_msg.feature.width = curr_fea.pts.size();

        sensor_msgs::PointCloud2Modifier modifier(feas_msg.feature);
        modifier.setPointCloud2Fields(5,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "vx", 1, sensor_msgs::msg::PointField::FLOAT32,
            "vy", 1, sensor_msgs::msg::PointField::FLOAT32,
            "idx", 1, sensor_msgs::msg::PointField::INT32);
        sensor_msgs::PointCloud2Iterator<float> iter_x(feas_msg.feature, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(feas_msg.feature, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_vx(feas_msg.feature, "vx");
        sensor_msgs::PointCloud2Iterator<float> iter_vy(feas_msg.feature, "vy");
        sensor_msgs::PointCloud2Iterator<int> iter_idx(feas_msg.feature, "idx");
        modifier.resize(curr_fea.pts.size());

        for (size_t i = 0; i < curr_fea.pts.size(); ++i, ++iter_x, ++iter_vx, ++iter_y, ++iter_vy, ++iter_idx) {
            auto& cur_pts = curr_fea.pts;
            auto& prev_pts = prev_fea.pts;
            auto& mapp = curr_fea.map_to_prev;
            int idx = -1;
            if (mapp.find(i) != mapp.end()){
                idx = mapp[i];
            }
            
            *iter_x = cur_pts[i].x;
            *iter_y = cur_pts[i].y;
            *iter_vx = cur_pts[i].x - prev_pts[idx].x;
            *iter_vy = cur_pts[i].y - prev_pts[idx].y;
            *iter_idx = idx;
        }

        pub_custfeas->publish(feas_msg);
        
    }

    void pubFeatures(rclcpp::Time& t, Feature& curr_fea, Feature& prev_fea){
        sensor_msgs::msg::PointCloud2 feas_msg;
        feas_msg.header.frame_id = feas.size() - 1;
        feas_msg.header.stamp = t;
        feas_msg.height = 1;
        feas_msg.width = curr_fea.pts.size();

        sensor_msgs::PointCloud2Modifier modifier(feas_msg);
        modifier.setPointCloud2Fields(5,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "vx", 1, sensor_msgs::msg::PointField::FLOAT32,
            "vy", 1, sensor_msgs::msg::PointField::FLOAT32,
            "idx", 1, sensor_msgs::msg::PointField::INT32);
        sensor_msgs::PointCloud2Iterator<float> iter_x(feas_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(feas_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_vx(feas_msg, "vx");
        sensor_msgs::PointCloud2Iterator<float> iter_vy(feas_msg, "vy");
        sensor_msgs::PointCloud2Iterator<int> iter_idx(feas_msg, "idx");
        modifier.resize(curr_fea.pts.size());

        for (size_t i = 0; i < curr_fea.pts.size(); ++i, ++iter_x, ++iter_vx, ++iter_y, ++iter_vy, ++iter_idx) {
            auto& cur_pts = curr_fea.pts;
            auto& prev_pts = prev_fea.pts;
            auto& mapp = curr_fea.map_to_prev;
            int idx = -1;
            if (mapp.find(i) != mapp.end()){
                idx = mapp[i];
            }
            
            *iter_x = cur_pts[i].x;
            *iter_y = cur_pts[i].y;
            *iter_vx = cur_pts[i].x - prev_pts[idx].x;
            *iter_vy = cur_pts[i].y - prev_pts[idx].y;
            *iter_idx = idx;
        }
        pub_feas->publish(feas_msg);
    }

    static void  ptsToPoint(cv::Point3f& pts, Eigen::Vector3f& p, cv::Mat& camera_mat)
    {
        double fx, fy, cx, cy;
        if (camera_mat.type() == CV_64F){
            fx = camera_mat.at<double>(0,0);
            fy = camera_mat.at<double>(1,1);
            cx = camera_mat.at<double>(0,2);
            cy = camera_mat.at<double>(1,2);
        }else{
            fx = camera_mat.at<float>(0,0);
            fy = camera_mat.at<float>(1,1);
            cx = camera_mat.at<float>(0,2);
            cy = camera_mat.at<float>(1,2);
        }
        p.z() = pts.z;
        p.x() = (pts.z * pts.x - pts.z * cx) / fx;
        p.y() = (pts.z * pts.y - pts.z * cy) / fy;
    }

};

} // namespace my_vins


