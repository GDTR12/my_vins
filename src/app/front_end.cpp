#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <thread>
#include <queue>
#include <utils/input/input.hpp>
#include "cv_bridge/cv_bridge.h"
#include "feature_tracker/klt.hpp"

// #include <cv_bridge/cv_bridge/cv_bridge.h>

/*
For each new image, existing features are tracked by the
KLT sparse optical flow algorithm [31]. Meanwhile, new corner
features are detected [32] to maintain a minimum number (100–
300) of features in each image. The detector enforces a uniform
feature distribution by setting a minimum separation of pixels
between two neighboring features. Two-dimensional (2-D) fea-
tures are first undistorted, and then, projected to a unit sphere
after passing outlier rejection. Outlier rejection is performed
using RANSAC with a fundamental matrix model [33].
Keyframes are also selected in this step. We have two criteria
for the keyframe selection. The first one is the average parallax
apart from the previous keyframe. If the average parallax of
tracked features is between the current frame and the latest
keyframe is beyond a certain threshold, we treat frame as a new
keyframe. Note that not only translation but also rotation can
cause parallax. However, features cannot be triangulated in the
rotation-only motion. To avoid this situation, we use short-term
integration of gyroscope measurements to compensate rotation
when calculating parallax. Note that this rotation compensation
is only used for the keyframe selection, and is not involved
in rotation calculation in the VINS formulation. To this end,
even if the gyroscope contains large noise or is biased, it will
only result in suboptimal keyframe selection results, and will
not directly affect the estimation quality. Another criterion is
tracking quality. If the number of tracked features goes below a
certain threshold, we treat this frame as a new keyframe. This
criterion is to avoid complete loss of feature tracks.
 
 */
// class FrontEnd : public rclcpp::Node
// {
// public:
//     FrontEnd(std::shared_ptr<feature_tracker::KLT> klt_)
//      :Node("FrontEnd")
//     {
//         // auto& input = slam_utils::ROSParamInput::getInstance();
//         // std::string topic_img = input.getPrivateParam<std::string>("camera_topic");
//         // std::string topic_imu = input.getPrivateParam<std::string>("imu_topic");
//         klt = klt_;
//         std::string topic_img = "/cam0/image_raw";
//         std::string topic_imu = "/imu0";
//         img_sub = this->create_subscription<sensor_msgs::msg::Image>(topic_img, 60, std::bind(&FrontEnd::img_callback,this,std::placeholders::_1));
//         imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(topic_imu, 2000, std::bind(&FrontEnd::imu_callback,this,std::placeholders::_1));
//         thread_imu = std::make_shared<std::thread>(&FrontEnd::imu_process, this);
//         thread_img = std::make_shared<std::thread>(&FrontEnd::img_process, this);
//     }
//     void img_callback(const sensor_msgs::msg::Image::SharedPtr msg)
//     {
//         img_src.push_back(msg);
//     }
//     void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
//     {
//         imu_src.push_back(msg);
//     }
//     /*
//         imu_preintergration
//      */
//     void imu_process(){
//         for (;;)
//         {
//             if (imu_src.empty())continue;
//         }
//     }

//     void img_process(){
//         for (;;)
//         {
//             if (img_src.empty())continue;
            
//             // klt->feedImg(img_src.back());
//         }
//     }

//     ~FrontEnd(){}

// private:
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
//     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
//     // std::string topic_img, topic_imu;
//     std::deque<sensor_msgs::msg::Image::SharedPtr> img_src;
//     std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_src;
//     std::shared_ptr<std::thread> thread_imu;
//     std::shared_ptr<std::thread> thread_img;

//     // extract features and keep the num of features: 100 - 300 
//     std::shared_ptr<feature_tracker::KLT> klt;

// };



int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor muti_exector;
    auto klt_node = std::make_shared<feature_tracker::KLT>();
    muti_exector.add_node(klt_node);
    muti_exector.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
