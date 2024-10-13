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

namespace my_vins
{

using namespace std::chrono_literals;
using namespace my_vins_msg::msg;


struct ImuData{
    rclcpp::Time t;
    float wx{0}, wy{0}, wz{0};
    float ax{0}, ay{0}, az{0};
};

// 特征点类型
struct PointFeature: public Feature{
public:
    PointFeature():Feature(){}
    PointFeature(Eigen::Matrix<Scalar, -1, 1>& data){
        setExtendData(data);
    }

    void setExtendData(Eigen::Matrix<Scalar, -1, 1>& data) override{
        pose(0) = data(0,0);
        pose(1) = data(1,0);
        pose(2) = data(2,0);
    }
    void getExtendData(Eigen::Matrix<Scalar, -1, 1>& data) override{
        data(0) = pose(0,0);
        data(1) = pose(1,0);
        data(2) = pose(2,0);
    }
    void setData(V3T data){
        pose = data;
        initialized = true;
    }

    V3T getData(){
        return pose;
    }

private:
    V3T pose = V3T::Zero(); 
};

class PointObservation : public Observation{
public:
    typedef PointFeature FeatureType;
    PointObservation(int idx): Observation(idx){}
    PointObservation(int idx, Eigen::Matrix<Scalar, -1, 1>& data)
    : Observation(idx){
        setExtendData(data);
    }

    virtual void setExtendData(Eigen::Matrix<Scalar, -1, 1>& data) override{
        assert(data.rows() == 3);
        x = data(0,0);
        y = data(1,0);
        depth = data(2,0);
    }
    virtual void getExtendData(Eigen::Matrix<Scalar, -1, 1>& data) override{
        data.resize(3,1);
        data(0,0) = x;
        data(1,0) = y;
        data(2,0) = depth;
    }

    void getData(Scalar& x_, Scalar& y_, Scalar& depth_){
        x_ = x;
        y_ = y;
        depth_ = depth;
    }
    V3T getData(){
        return V3T(x, y, depth);
    }

    void setData(Scalar x_, Scalar y_, Scalar depth_){
        x = x_;
        y = y_;
        depth = depth_;
    }

private:

    Scalar x = 0, y = 0, depth = 0; std::vector<float> dsp{256, 0};
};

class CameraObserver: public ObserverNode{
public:
    typedef PointObservation ObservationType;

    CameraObserver(rclcpp::Time t):ObserverNode(t){
    }

    CameraObserver(rclcpp::Time t, cv::Mat& img_):ObserverNode(t){
        img = img_;
    }

    cv::Mat getImage(){
        return img;
    }
    void setImage(cv::Mat& mat){
        img = mat;
    }
private:
    cv::Mat img;
    int idx_map = -1;
    std::deque<ImuData> imu_list; // 存储现在到上一个帧之间的imu信息
};


class MyVinsParamServer: public rclcpp::Node{
private:
public:
    const std::string namesp = "MyVins";
    int WINDOW_SIZE = 10;
    float parallax_threashold = 40;
    MyVinsParamServer():rclcpp::Node("MyVins"){
        this->declare_parameter(namesp + "/" + "max_cnt", WINDOW_SIZE);
        this->declare_parameter(namesp + "/" + "pallax_threashold", parallax_threashold);
    }
};

class MyVinsSFM;

class MyVins : public MyVinsParamServer
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
    void matchReponseCallback(const FeatureMatchPrevResponse::SharedPtr);

private:

    typedef FeatureMatchPrevResponse::SharedPtr MatchBuf;
    bool popMatchBuffer();

    rclcpp::Publisher<FeatureMatchPrevRequest>::SharedPtr pub_match;
    rclcpp::Subscription<FeatureMatchPrevResponse>::SharedPtr sub_match;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    std::shared_ptr<std::thread> thread_main;

    std::deque<MatchBuf> match_buf;
    std::deque<std::pair<int, cv::Mat>> req_buf;
    std::mutex mtx_mb;
    std::condition_variable cv_mb;

    FeatureManager frame_manager;
    int handled_id_fmanager = -1;

    std::shared_ptr<MyVinsSFM> sfm;
    std::shared_ptr<MyVinsVis> vis;

    // std::atomic<bool> fm_bigger_windowsize;
    

    State state;
    QuaT q_ItoC = QuaT::Identity();
    V3T t_ItoC = V3T::Zero();

    friend class MyVinsSFM;
    friend class MyVinsVis;
};

    
} // namespace my_vins


