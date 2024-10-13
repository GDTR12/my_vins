#include "my_vins.hpp"
#include "utils/input/input.hpp"
#include "cv_bridge/cv_bridge.h"

namespace my_vins
{

using namespace std::chrono_literals;

MyVins::MyVins()
{
    auto& input = slam_utils::ROSParamInput::getInstance();
    pub_match = this->create_publisher<FeatureMatchPrevRequest>("front/match_prev_request", 1000);
    sub_match = this->create_subscription<FeatureMatchPrevResponse>(
        "front/match_prev_response", 
        1000, 
        std::bind(&MyVins::matchReponseCallback, this, std::placeholders::_1));
    sub_image = this->create_subscription<sensor_msgs::msg::Image>(
        input.getConfigParam<std::string>("camera_topic"), 
        1000, 
        std::bind(&MyVins::imageTopicCallback, this, std::placeholders::_1));



    std::vector<double> c_mat_vec, d_mat_vec;
    input.getConfigParam<std::vector<double>>("projection_parameters", c_mat_vec);
    input.getConfigParam<std::vector<double>>("projection_parameters", d_mat_vec);
    M3T camera_mat;
    camera_mat << c_mat_vec[0], 0, c_mat_vec[2],
                      0, c_mat_vec[1], c_mat_vec[3],
                      0, 0, 1;

    V4T distort_mat = Eigen::Map<V4T>(d_mat_vec.data());
    vis = std::make_shared<MyVinsVis>(*this);
    sfm = std::make_shared<MyVinsSFM>(*this, *vis.get(), camera_mat, distort_mat);



    thread_main = std::make_shared<std::thread>(&MyVins::run, this);
    pthread_setname_np(thread_main->native_handle(), "back_end_run");
    thread_main->detach();
}

MyVins::~MyVins(){}


void MyVins::imageTopicCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    static size_t req_id = 0;
    FeatureMatchPrevRequest req;
    req.id = req_id++;
    req.img = *msg;
    auto cvImg = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::Mat grayImg;
    if (cvImg->image.channels() != 1){
        cv::cvtColor(cvImg->image, grayImg, cv::COLOR_BGR2GRAY);
    }else{
        grayImg = cvImg->image;
    }
    req_buf.push_back({req.id, grayImg});
    pub_match->publish(req);

    // auto bridge = cv_bridge::toCvCopy(msg);

    // std::unique_lock lock(mtx_fm);
    // CameraObserver& observe = frame_manager.append(msg->header.stamp);
    // observe.img = bridge->image;
    // lock.unlock();
    // std::this_thread::sleep_for(2ms);
}


void MyVins::matchReponseCallback(const FeatureMatchPrevResponse::SharedPtr result)
{
    int id = result->id;
    if (id < 0){
        return;
    }

    std::unique_lock lock(mtx_mb);
    MatchBuf match = result;
    match_buf.push_back(match);
    lock.unlock();
    cv_mb.notify_one();
}

bool MyVins::popMatchBuffer()
{
    std::unique_lock lock(mtx_mb);
    cv_mb.wait(lock, [this](){return !match_buf.empty();});
    MatchBuf result = match_buf.front();
    match_buf.pop_front();
    lock.unlock();


    assert(result->kpts.size() == result->match_prev_id.size() * 2 &&
        result->kpts.size() == result->match_prev_score.size() * 2 && 
        result->kpts.size() == result->scores.size() * 2 &&
        result->kpts.size() * result->dim == result->descriptors.size() * 2);



    std::vector<Eigen::Matrix<Scalar, -1, 1>> observe_data;
    std::vector<Eigen::Matrix<Scalar, -1, 1>> feas_data;
    std::vector<int> map_prev;

    for (size_t i = 0; i < result->kpts.size() - 1; i += 2){
        feas_data.emplace_back(Eigen::Matrix<Scalar, -1, 1>::Zero(3));
        auto& observe = observe_data.emplace_back(3);
        map_prev.push_back(result->match_prev_id[i / 2]);
        Scalar x = result->kpts[i];
        Scalar y = result->kpts[i+1];
        Scalar depth = 1;
        observe(0) = x;
        observe(1) = y;
        observe(2) = depth;
    }
    int id = result->id;
    if (id == frame_manager.getNodeSize()){
        ObserverNode* prev_node;
        if (id == 0){
            prev_node = nullptr;
        }else{
            prev_node = frame_manager.getNodeAt(id - 1);
        }
        rclcpp::Time t(result->header.stamp);
        auto& node = frame_manager.appendAccordingPrev<PointFeature, CameraObserver>(t, observe_data, feas_data, map_prev, prev_node);
        node.setImage(req_buf[id].second);
    }else if(id < frame_manager.getNodeSize()){
        //创建空间点类型,点的观测数据类型,以及观测者
        ObserverNode* prev_node = frame_manager.getNodeAt(id - 1);
        frame_manager.setAccordingPrev<PointFeature, CameraObserver>(id, observe_data, feas_data, map_prev, prev_node);
    }
    return true;
}

void MyVins::imuInit()
{

}



void MyVins::init()
{

}

void MyVins::run()
{
    if (!state.initialized){
        RCLCPP_INFO(this->get_logger(), "[Step]: initialize");
        while (true)
        {
            popMatchBuffer();
            if (frame_manager.getNodeSize() < 20){
                continue;
            }else{
                RCLCPP_INFO(this->get_logger(), "init structer");
                if (sfm->initStructure()){
                    break;
                }
            }
        }
        state.initialized = true;
    }

}

} // namespace my_vins




