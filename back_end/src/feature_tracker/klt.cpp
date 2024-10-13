#include "klt.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <utils/input/input.hpp>
#include "utils/camera_rviz/camera_rviz.hpp"
#include <ceres/problem.h>
namespace feature_tracker
{

KLT::KLT(){}

KLT::~KLT(){}

void KLT::featureExtractor(const cv::Mat& img, std::vector<cv::Point3f>& pts, cv::Mat& mask){
    // TODO: 尝试使用 orb-slam2 的 四叉树法
    std::vector<cv::Point2f> pts_, undist_pts; 
    if (pts.size() >= MAX_CNT){
        return;
    }
    
    if (mask.empty()){
        cv::goodFeaturesToTrack(img, pts_, MAX_CNT - pts.size(), 0.01, MIN_DIST);
    }else{
        cv::goodFeaturesToTrack(img, pts_, MAX_CNT - pts.size(), 0.01, MIN_DIST, mask);
    }


    cv::undistortPoints(pts_, undist_pts, camera_mat, distort_coeffs, cv::Mat(), camera_mat);
    // cv::Mat show;
    // cv::undistort(img, show, camera_mat, distort_coeffs, camera_mat);
    // cv::imshow("bb", show);
    // cv::imshow("aa", img);
    // cv::waitKey(0);
    // std::cout << "==================" << std::endl;
    // std::cout << pts_.size() << std::endl;
    // std::cout << undist_pts.size() << std::endl;
    for (size_t i = 0; i < undist_pts.size(); i++){
        std::cout << "(" <<  pts_[i].x << ", " << pts_[i].y << ")" << "(" << img.cols / 2 + img.cols * undist_pts[i].x << ", " << img.rows / 2 + img.rows *  undist_pts[i].y << ")" << std::endl;
    }
    for (size_t i = 0; i < pts_.size(); i++){
        pts.push_back(cv::Point3f(pts_[i].x, pts_[i].y, 0));
    }
}


int KLT::geneMatchedFeature(Feature& prev, Feature& cur, cv::Mat& f_mat){
    std::vector<uchar> status;
    std::vector<float> err;
    std::vector<cv::Point2f> prev_pts, cur_pts_lk;
    for (size_t i = 0; i < prev.pts.size(); i++){
        prev_pts.push_back(cv::Point2f(prev.pts[i].x, prev.pts[i].y));
    }
    cv::calcOpticalFlowPyrLK(prev.img, cur.img, prev_pts, cur_pts_lk, status, err, cv::Size(21, 21), 3);
    std::vector<int> lk_prev_idx;
    std::vector<cv::Point2f> match_0, match_1;
    for (size_t i = 0; i < prev.pts.size(); i++){
        if (status.at(i)){
            lk_prev_idx.push_back(i);
            match_0.push_back(cv::Point2f(prev.pts[i].x, prev.pts[i].y));
            match_1.push_back(cur_pts_lk[i]);
        }
    }
    // f_mat = cv::findFundamentalMat(match_0, match_1, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
    f_mat = cv::findEssentialMat(match_0, match_1, camera_mat, cv::RANSAC, 0.999, F_THRESHOLD, status);
    for (size_t i = 0; i < status.size(); i++){
        if (status[i]){
            cur.pts.push_back(cv::Point3f(match_1[i].x, match_1[i].y, 0));
            cur.map_to_prev[cur.pts.size() - 1] = lk_prev_idx[i];
        }
    }
    return cur.pts.size();
}

int KLT::featureMatch(Feature& prev, Feature& cur, std::map<int, int>& feas_map)
{
    cv::Mat e_mat;
    int ret = geneMatchedFeature(prev, cur, e_mat);

    std::vector<cv::Point2f> match_0, match_1;

    std::cout << cur.map_to_prev.size() << std::endl;
    for (auto& map : cur.map_to_prev){
        match_0.push_back(cv::Point2f(prev.pts[map.second].x, prev.pts[map.second].y));
        match_1.push_back(cv::Point2f(prev.pts[map.first].x, prev.pts[map.first].y));
    }
    

    // if (feas.size() == 2){
    //     // 2d-2d 解算 R 和尺度不确定的 t
    //     // cv::Mat e_mat = camera_mat.t() * f_mat * camera_mat;
    //     cv::Mat R, t; // R_0to1 t_0to1
    //     cv::recoverPose(e_mat, match_0, match_1, camera_mat, R, t);

    //     Eigen::Matrix3d R_0to1;
    //     Eigen::Vector3d t_0to1;
    //     cv::cv2eigen(R, R_0to1);
    //     cv::cv2eigen(t, t_0to1);
    //     cur.t_itoj = t_0to1;
    //     cur.q_itoj = Eigen::Quaterniond(R_0to1);

    //     // 2d-3d 计算像素点的深度
    //     cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
    //         1, 0, 0, 0,
    //         0, 1, 0, 0,
    //         0, 0, 1, 0);
    //     cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
    //         R_0to1(0, 0), R_0to1(0, 1), R_0to1(0, 2), t_0to1[0],
    //         R_0to1(1, 0), R_0to1(1, 1), R_0to1(1, 2), t_0to1[1],
    //         R_0to1(2, 0), R_0to1(2, 1), R_0to1(2, 2), t_0to1[2]
    //     );
    //     cv::Mat points4D;
    //     cv::triangulatePoints(T1, T2, match_0, match_1, points4D);
    //     Eigen::MatrixXd p4ds;
    //     cv::cv2eigen(points4D, p4ds);
    //     for (size_t i = 0; i < p4ds.cols(); i++){
    //         // 估计深度
    //         Eigen::Vector3d p_inC0 = p4ds.col(i).head(3) / p4ds.col(i)(3);
    //         Eigen::Vector3d p_inC1 = R_0to1.transpose() * p_inC0 + (- R_0to1.transpose() * t_0to1.head(3));
    //         cur.pts[i].z = p_inC1.z();
    //         prev.pts[cur.map_to_prev[i]].z = p_inC0.z();
    //     }
    // }
    // else{ 
    //     // 3d-2d PnP 解算位姿
    //     cv::Mat R, t; // R_0to1 t_0to1
    //     cv::recoverPose(e_mat, match_0, match_1, camera_mat, R, t);

    //     Eigen::Matrix3d R_itoj;
    //     Eigen::Vector3d t_itoj;
    //     cv::cv2eigen(R, R_itoj);
    //     cv::cv2eigen(t, t_itoj);
    //     cur.t_itoj = t_itoj;
    //     cur.q_itoj = Eigen::Quaterniond(R_itoj);

    //     // // 2d-3d 计算像素点的深度
    //     // cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
    //     //     1, 0, 0, 0,
    //     //     0, 1, 0, 0,
    //     //     0, 0, 1, 0);
    //     // cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
    //     //     R_itoj(0, 0), R_itoj(0, 1), R_itoj(0, 2), t_itoj[0],
    //     //     R_itoj(1, 0), R_itoj(1, 1), R_itoj(1, 2), t_itoj[1],
    //     //     R_itoj(2, 0), R_itoj(2, 1), R_itoj(2, 2), t_itoj[2]
    //     // );
    //     // cv::Mat points4D;
    //     // cv::triangulatePoints(T1, T2, match_0, match_1, points4D);
    //     // Eigen::MatrixXd p4ds;
    //     // cv::cv2eigen(points4D, p4ds);
    //     // for (size_t i = 0; i < p4ds.cols(); i++){
    //     //     // 估计深度
    //     //     Eigen::Vector3d p_inC0 = p4ds.col(i).head(3) / p4ds.col(i)(3);
    //     //     Eigen::Vector3d p_inC1 = R_itoj.transpose() * p_inC0 + (- R_itoj.transpose() * t_itoj.head(3));
    //     //     cur.pts[i].z = p_inC1.z();
    //     //     prev.pts[cur.map_to_prev[i]].z = p_inC0.z();
    //     // }
    //     std::cout << "fund" << std::endl;
    //     std::cout << R_itoj << std::endl;
    //     std::cout << t_itoj.transpose() << std::endl;

    //     std::vector<cv::Point3f> points_inCi;
    //     std::vector<cv::Point2f> match_11;

    //     for (auto& map : cur.map_to_prev){
    //         if (prev.pts[map.second].z == 0) continue;
    //         Eigen::Vector3f point;
    //         ptsToPoint(prev.pts[map.second], point, camera_mat);
    //         points_inCi.push_back(cv::Point3f(point.x(), point.y(), point.z()));
    //         match_11.push_back(cv::Point2f(cur.pts[map.first].x , cur.pts[map.first].y));
    //         // std::cout << points_inCi.back() << std::endl;
    //     }
        

    //     cv::Mat r, tt;
    //     cv::solvePnPRansac(points_inCi, match_11, camera_mat, distort_coeffs, r, tt);
    //     cv::Mat RR;
    //     cv::Rodrigues(r, RR);
    //     Eigen::Matrix3d RR_;
    //     Eigen::Vector3d tt_;
    //     cv::cv2eigen(RR, RR_);
    //     cv::cv2eigen(tt, tt_);
    //     // std::cout << "pnp" << std::endl;
    //     // std::cout << RR_ << std::endl;
    //     std::cout << tt_.transpose() << std::endl;
    //     cur.q_itoj = Eigen::Quaterniond(RR_);
    //     cur.t_itoj = tt_;
    //     cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
    //         1, 0, 0, 0,
    //         0, 1, 0, 0,
    //         0, 0, 1, 0);
    //     cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
    //         RR_(0, 0), RR_(0, 1), RR_(0, 2), tt_[0],
    //         RR_(1, 0), RR_(1, 1), RR_(1, 2), tt_[1],
    //         RR_(2, 0), RR_(2, 1), RR_(2, 2), tt_[2]
    //     );
    //     cv::Mat points4D;
    //     cv::triangulatePoints(T1, T2, match_0, match_1, points4D);
    //     Eigen::MatrixXd p4ds;
    //     cv::cv2eigen(points4D, p4ds);
    //     for (size_t i = 0; i < p4ds.cols(); i++){
    //         // 估计深度
    //         Eigen::Vector3d p_inC0 = p4ds.col(i).head(3) / p4ds.col(i)(3);
    //         std::cout << p_inC0.transpose() << " " <<  p4ds.col(i).transpose() << std::endl;
    //         Eigen::Vector3d p_inC1 = RR_.transpose() * p_inC0 + (- RR_.transpose() * tt_.head(3));
    //         cur.pts[i].z = p_inC1.z();
    //         prev.pts[cur.map_to_prev[i]].z = p_inC0.z();
    //     }
    // }

    // mask 提取新的特征点
    cv::Mat mask;
    mask.create(cur.img.rows, cur.img.cols, CV_8UC1);
    mask.setTo(255);
    for (size_t i = 0; i < cur.pts.size(); i++){
        cv::circle(mask, cv::Point2f(cur.pts[i].x, cur.pts[i].y), MIN_DIST, 0, -1);
    }
    featureExtractor(cur.img, cur.pts, mask);
    return ret;
}

int KLT::feedImg(const cv::Mat& img, rclcpp::Time t){
    // TODO: 尝试使用 MEI 模型 去畸变
    if (feas.size() == 0){
        Feature& fea = feas.emplace_back(img, t);
        fea.q_itoj = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0,1,0)));
        fea.t_itoj = Eigen::Vector3d(0,0,1);
        cv::Mat empty_mask;
        featureExtractor(img, fea.pts, empty_mask);
        // pubFeatures(t, fea, fea);
        pubCustomFeatures(t, fea, fea);
        return 0;
    }

    Feature& curr_fea = feas.emplace_back(img, t);
    Feature& prev_fea = feas.at(feas.size() - 2);
    int ret = featureMatch(prev_fea, curr_fea, curr_fea.map_to_prev);
    RCLCPP_DEBUG(this->get_logger(), "new_points: %d", MAX_CNT - ret);

    // pubFeatures(t, curr_fea, prev_fea);
    pubCustomFeatures(t, curr_fea, prev_fea);

    if (SHOW_MATCH && feas.size() >= 2){
        int col = prev_fea.img.cols;
        cv::Mat grayImg = cv::Mat::zeros(prev_fea.img.rows, col * 2, CV_8UC1);
        cv::hconcat(prev_fea.img, curr_fea.img, grayImg);
        cv::Mat show_img;
        cv::cvtColor(grayImg, show_img, cv::COLOR_GRAY2BGR);
        for (auto& pair : curr_fea.map_to_prev){
            cv::line(show_img, cv::Point2f(prev_fea.pts[pair.second].x, prev_fea.pts[pair.second].y),
                cv::Point2f(curr_fea.pts[pair.first].x, curr_fea.pts[pair.first].y) + cv::Point2f(col, 0), cv::Scalar(0, 255, 0));
        }
        for (auto& pt : prev_fea.pts){
            cv::circle(show_img, cv::Point2f(pt.x, pt.y), 2, cv::Scalar(0, 0, 255), 2);
        }
        for (auto& pt: curr_fea.pts){
            cv::circle(show_img, cv::Point2f(pt.x, pt.y) + cv::Point2f(col, 0), 2, cv::Scalar(0, 0, 255), 2);
        }
        
        sensor_msgs::msg::Image img_msg;
        cv_bridge::CvImage bridge;
        bridge.encoding = "bgr8";
        bridge.image = show_img;
        bridge.toImageMsg(img_msg);
        pub_show->publish(img_msg);
    }
    if (SHOW_SFM && feas.size() >= 2 && feas.size() % 5 == 0){
        Eigen::Matrix4d T_0toj = Eigen::Matrix4d::Identity();

        for (size_t i = 0; i < feas.size(); i++){
            Eigen::Matrix4d T_itoj;
            T_itoj.block<3,3>(0,0) = feas[i].q_itoj.toRotationMatrix();
            T_itoj.block<3,1>(0,3) = feas[i].t_itoj;
            T_0toj = T_0toj * T_itoj;
        }
        // std::cout << T_0toj << std::endl;
        slam_utils::CameraRvizMsgGenerator msg_gene(Eigen::Quaterniond(T_0toj.block<3,3>(0,0)), Eigen::Vector3d(T_0toj.block<3,1>(0,3)));
        std::cout << "pos: " << T_0toj.block<3,1>(0,3).transpose() << std::endl;
        // slam_utils::CameraRvizMsgGenerator msg_gene(Eigen::Quaterniond::Identity(), Eigen::Vector3d(feas.size() * 0.01, 0,0));
        slam_utils::CameraRvizMsg msg_camera;
        msg_camera.header.frame_id = "camera";
        msg_camera.header.stamp = t;
        msg_camera.id = feas.size() - 1;
        msg_gene.msgMaker(msg_camera);
        pub_camermsg->publish(msg_camera);
    }

    return 0;
}
    
} // namespace 

