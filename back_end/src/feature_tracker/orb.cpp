#include "orb.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "ORBextractor.h"
#include "opencv2/core/eigen.hpp"

namespace feature_tracker
{

ORBTracker::ORBTracker(){
    // this->declare_parameter("nfeatures", nfeatures);
    // this->declare_parameter("scaleFactor", scaleFactor);
    // this->declare_parameter("nlevels", nlevels);
    // this->declare_parameter("iniThFAST", iniThFAST);
    // this->declare_parameter("minThFAST", minThFSAT);
}
ORBTracker::~ORBTracker(){}

void feature_extract(const cv::Mat& img, std::vector<cv::KeyPoint>& key_points, cv::Mat& descriptors)
{
    // auto orb = cv::ORB::create(500);
    // orb->detectAndCompute(img, cv::noArray(), key_points, descriptors);

    ORB_SLAM2::ORBextractor orb(1000, 1.2, 8, 20, 7);
    orb(img, cv::noArray(), key_points, descriptors);
}

int ORBTracker::feedImg(const cv::Mat& img)
{
    if (feas.size() == 0){
        auto& cur = feas.emplace_back(img);
        feature_extract(img, cur.keys, cur.des);
    }else{
        auto& cur = feas.emplace_back(img);
        auto& prev = feas.at(feas.size() - 2);

        auto start_t = std::chrono::system_clock::now();
        feature_extract(img, cur.keys, cur.des);

        auto extrac_t = std::chrono::system_clock::now();
        
        // std::cout << "extract: " << std::chrono::duration_cast<std::chrono::nanoseconds>(extrac_t - start_t).count() / 1000000.0f << std::endl;
        auto matcher = cv::BFMatcher::create();
        std::vector<cv::DMatch> matches;

        matcher->match(prev.des, cur.des, matches);
        auto match_t = std::chrono::system_clock::now();
        // std::cout << "match: " << std::chrono::duration_cast<std::chrono::nanoseconds>(match_t - extrac_t).count() / 1000000.0f << std::endl;
        // std::cout << prev.keys.size() << " " << cur.keys.size() << std::endl;
        // std::cout << matches.size() << " " << std::endl;
        // 根据距离排序
        std::sort(matches.begin(), matches.end());
        // 选择前50个最佳匹配
        int numBestMatches = std::min(200, static_cast<int>(matches.size()));
        matches.erase(matches.begin() + numBestMatches, matches.end());

        std::vector<cv::Point2f> match_ps0, match_ps1;
        for (size_t i = 0; i < matches.size(); i++){
            match_ps0.push_back(prev.keys[matches[i].queryIdx].pt);
            match_ps1.push_back(cur.keys[matches[i].trainIdx].pt);
        }
        cv::Mat camera_mat = (cv::Mat_<double>(3,3) << 878.20415, 0, 633.26849,
            0, 875.56807, 530.29927,
            0, 0, 1);
        cv::Mat mask;
        cv::Mat e_mat = cv::findEssentialMat(match_ps0, match_ps1, camera_mat, 8, 0.99, 1.0, mask);
        cv::Mat R, t;
        cv::recoverPose(e_mat, match_ps0, match_ps1, camera_mat, R, t, mask);
        Eigen::Matrix3d R_e;
        cv::cv2eigen(R, R_e);
        Eigen::AngleAxisd r(R_e);

        // std::cout << R << "\n" << t.t() << std::endl;
        // cv::Mat r;
        // cv::Rodrigues(R, r);
        // std::cout << r.axis().transpose() << std::endl;
        // cv::Mat show;
        // cv::drawMatches(prev.img, prev.keys, cur.img, cur.keys, matches, show);
        // cv::imshow("aa", show);
        // cv::waitKey(0);
    }


    return 0;
}

} // namespace feature_tracker
