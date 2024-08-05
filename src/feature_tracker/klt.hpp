#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

namespace feature_tracker
{


struct Feature{
    cv::Mat img;
    rclcpp::Time time;
    std::vector<cv::Point2f> pts;
    // std::vector<int> pts_cnt;
    std::map<int, int> map_to_prev;
    // std::vector<int> ids_pre;
    Feature(Feature&) = delete;
    Feature(const cv::Mat& img_, rclcpp::Time& t){img = img_; time = t;}
};

class FeaturesManager
{
private:
    std::deque<Feature> feas;
public:
    FeaturesManager(){
        
    }
    // void addFeature(cv::Mat& img, ){
    //     auto& back = feas.emplace_back();
    //     back.img = img;
        
    // }

    Feature& operator[](int idx){
        if (idx > feas.size() - 1 || idx < 0){
            throw std::runtime_error("An error occurred");
        }
        else return feas[idx];
    }
    Feature& at(int idx){
        return (*this)[idx];
    }
    ~FeaturesManager(){}
};




class KLTParamServer: public rclcpp::Node
{
public:
    int MAX_CNT = 150;
    int MIN_DIST = 20;
    int F_THRESHOLD = 1.0;
    bool SHOW_MATCH = true;
    const std::string namesp = "/KLT";
    KLTParamServer(): rclcpp::Node("KLT"){
        this->declare_parameter(namesp + "/" + "max_cnt", MAX_CNT);
        this->declare_parameter(namesp + "/" + "min_dist", MIN_DIST);
        this->declare_parameter(namesp + "/" + "f_threshold", F_THRESHOLD);
        this->declare_parameter(namesp + "/" + "show_math", SHOW_MATCH);
    }
};


class KLT : public KLTParamServer
{
    typedef Feature KLTFeature;
private:
    std::deque<sensor_msgs::msg::Image::SharedPtr> img_src;
    std::shared_ptr<std::thread> t_process;
    std::deque<KLTFeature> feas;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match;

public:
    KLT();
    ~KLT();
    void feature_extractor(const cv::Mat& img, std::vector<cv::Point2f>& pts);
    int feature_match(Feature& prev, Feature& curr, std::map<int, int>& feas_map);
    void img_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void process();
    int feedImg(const cv::Mat& img, rclcpp::Time t);
};



    
} // namespace feature_tracker


