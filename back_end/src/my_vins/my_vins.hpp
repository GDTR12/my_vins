#pragma once
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <deque>
#include <mutex>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "feature/feature_manager.hpp"
#include "my_vins_msg/msg/feature_match_prev_request.hpp"
#include "my_vins_msg/msg/feature_match_prev_response.hpp"
#include "my_vins_sfm.hpp"
#include "my_vins_vis.hpp"
#include "ceres_fgo/imu_factor/imuPreintegration/imuPreintegration.hpp"
#include "my_vins_feature.hpp"
#include "my_vins_slidewindow.hpp"
#include "my_vins_param.hpp"

namespace my_vins
{

using namespace std::chrono_literals;
using namespace my_vins_msg::msg;




class MyVinsSFM;

class MyVins : public rclcpp::Node
{
public:
    struct State{
        bool initialized = false;
        bool extrinsic_calibed = false;
        bool marginalized = false;
        bool motion_ba_completed = false;
    };

    MyVins();
    ~MyVins();
    void run();
    void imuInit();
    void initStructure();
    void init();
    void imageTopicCallback(const sensor_msgs::msg::Image::SharedPtr);
    void imuTopicCallback(const sensor_msgs::msg::Imu::SharedPtr);
    void matchReponseCallback(const FeatureMatchPrevResponse::SharedPtr);
    bool visualInertialAlign();

private:

    typedef FeatureMatchPrevResponse::SharedPtr MatchBuf;
    struct RequestBuf{
        int id;
        rclcpp::Time t;
        cv::Mat img;
    };
    bool popMatchBuffer();
    bool popImuData();

    rclcpp::Publisher<FeatureMatchPrevRequest>::SharedPtr pub_match;
    rclcpp::Subscription<FeatureMatchPrevResponse>::SharedPtr sub_match;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    std::shared_ptr<std::thread> thread_main;

    std::deque<MatchBuf> match_buf;
    std::deque<RequestBuf> req_buf;
    std::mutex mtx_mb;
    std::condition_variable cv_mb;


    std::deque<CameraObserver::ImuData> imu_buf;
    std::mutex mtx_ib;
    std::condition_variable cv_ib;
    int idx_nodePutImu = 0;

    MyVinsFeatureManager frame_manager;
    int handled_id_fmanager = -1;

    std::shared_ptr<MyVinsSFM> sfm;
    std::shared_ptr<MyVinsVis> vis;
    std::shared_ptr<MyVinsSlideWindow> sldwin;
    MyVinsParamServer& param = MyVinsParamServer::getInstance();
    QuaT q_WtoI0 = QuaT::Identity();

    State state;
    QuaT q_ItoC = QuaT::Identity();
    V3T t_ItoC = V3T::Zero();

    friend class MyVinsSFM;
    friend class MyVinsVis;
};

    
} // namespace my_vins


