#include "klt.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <utils/input/input.hpp>

namespace feature_tracker
{

KLT::KLT()
{
    auto& input = slam_utils::ROSParamInput::getInstance();
    std::string topic_img = input.getConfigParam<std::string>("camera_topic");
    sub_img = this->create_subscription<sensor_msgs::msg::Image>(topic_img, 60, std::bind(&KLT::img_callback,this,std::placeholders::_1));
    pub_match = this->create_publisher<sensor_msgs::msg::Image>("matched_show", 10);
    t_process =  std::make_shared<std::thread>(&KLT::process, this);
}

KLT::~KLT()
{
}

void KLT::feature_extractor(const cv::Mat& img, std::vector<cv::Point2f>& pts){
    // TODO: 尝试使用 orb-slam2 的 四叉树法
    cv::goodFeaturesToTrack(img, pts, MAX_CNT, 0.01, MIN_DIST);

}

void KLT::img_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    img_src.push_back(msg);
}

int KLT::feature_match(Feature& prev, Feature& cur, std::map<int, int>& feas_map)
{
    std::vector<uchar> status;
    std::vector<float> err;
    std::vector<cv::Point2f> cur_pts;
    cv::calcOpticalFlowPyrLK(prev.img, cur.img, prev.pts, cur_pts, status, err, cv::Size(21, 21), 3);

    std::vector<cv::Point2f> prev_pts, cur_pts_;
    int cnt = 0;
    std::vector<int> prev_indices;
    for (size_t i = 0; i < status.size(); i++){
        if (status[i]){
            prev_indices.push_back(i);
            prev_pts.push_back(prev.pts[i]);
            cur_pts_.push_back(cur_pts[i]);
        }
    }

    // 去除外点:
    status.clear();
    cv::findFundamentalMat(prev_pts, cur_pts_, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
    for (size_t i = 0; i < status.size(); i++){
        if (status[i]){
            cur.pts.push_back(cur_pts_[i]);
        }
        else{
            prev_indices[i] = -1;
        }
    }

    cnt = 0;
    for (auto& i : prev_indices){
        if (i == -1) continue;
        feas_map[i] = cnt;
        cnt++;
    }
    
    if (feas_map.size() != cur.pts.size()){
        RCLCPP_ERROR(this->get_logger(), "Wrong with match, outliers: %ld, %ld", feas_map.size(), cur.pts.size());
    }

    int ret = cur.pts.size();
    std::vector<cv::Point2f> new_pts;
    if (cur.pts.size() == MAX_CNT){
        return MAX_CNT;
    }

    // mask
    cv::Mat mask;
    mask.create(cur.img.rows, cur.img.cols, CV_8UC1);
    mask.setTo(255);
    for (size_t i = 0; i < cur.pts.size(); i++){
        cv::circle(mask, cur.pts[i], MIN_DIST, 0, -1);
    }
    cv::goodFeaturesToTrack(cur.img, new_pts, MAX_CNT - cur.pts.size(), 0.01, MIN_DIST, mask);
    for (size_t i = 0; i < new_pts.size(); i++){
        cur.pts.push_back(new_pts[i]);
    }
    return ret;
}

int KLT::feedImg(const cv::Mat& img, rclcpp::Time t){
    // TODO: 尝试使用 MEI 模型 去畸变
    if (feas.size() == 0){
        Feature& fea = feas.emplace_back(img, t);
        feature_extractor(img, fea.pts);
        return 0;
    }

    Feature& curr_fea = feas.emplace_back(img, t);
    Feature& prev_fea = feas.at(feas.size() - 2);
    int ret = feature_match(prev_fea, curr_fea, curr_fea.map_to_prev);
    RCLCPP_DEBUG(this->get_logger(), "new_points: %d", MAX_CNT - ret);

    if (SHOW_MATCH && feas.size() >= 2){
        int col = prev_fea.img.cols;
        cv::Mat grayImg = cv::Mat::zeros(prev_fea.img.rows, col * 2, CV_8UC1);
        cv::hconcat(prev_fea.img, curr_fea.img, grayImg);
        cv::Mat show_img;
        cv::cvtColor(grayImg, show_img, cv::COLOR_GRAY2BGR);
        for (auto& pair : curr_fea.map_to_prev){
            cv::line(show_img, prev_fea.pts[pair.first], curr_fea.pts[pair.second] + cv::Point2f(col, 0), cv::Scalar(0, 255, 0));
        }
        for (auto& pt : prev_fea.pts){
            cv::circle(show_img, pt, 2, cv::Scalar(0, 0, 255), 2);
        }
        for (auto& pt: curr_fea.pts){
            cv::circle(show_img, pt + cv::Point2f(col, 0), 2, cv::Scalar(0, 0, 255), 2);
        }
        
        sensor_msgs::msg::Image img_msg;
        cv_bridge::CvImage bridge;
        bridge.encoding = "bgr8";
        bridge.image = show_img;
        bridge.toImageMsg(img_msg);
        pub_match->publish(img_msg);
    }
    return 0;
}
    
void KLT::process(){
    rclcpp::Rate rate(100);
    for (;;)
    {
        rate.sleep();
        if (img_src.size() == 0)continue;
        
        auto cvImg = cv_bridge::toCvCopy(img_src.front(), "mono8");
        cv::Mat grayImg;
        if (cvImg->image.channels() != 1){
            cv::cvtColor(cvImg->image, grayImg, cv::COLOR_BGR2GRAY);
        }else{
            grayImg = cvImg->image;
        }
        feedImg(grayImg, rclcpp::Time(cvImg->header.stamp.sec, cvImg->header.stamp.nanosec, RCL_ROS_TIME));
        img_src.pop_front();
    }
}
} // namespace 

