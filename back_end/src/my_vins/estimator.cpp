#include <ceres/ceres.h>
#include "estimator.hpp"
#include "utils/input/input.hpp"

namespace my_vins
{

Estimator::Estimator()//: rclcpp::Node("Estimator")
{

    auto& param = slam_utils::ROSParamInput::getInstance();
    std::string topic_feas = param.getConfigParam<std::string>("feas_topic");
    std::string topic_custfeas = param.getConfigParam<std::string>("feas_cust_topic");
    std::vector<Scalar> tmp_contain(5, 0);
    param.getConfigParam<std::vector<Scalar>>("projection_parameters", tmp_contain);
    inner = Eigen::Vector4d(tmp_contain[0], tmp_contain[1], tmp_contain[2], tmp_contain[3]);
    param.getConfigParam<std::vector<Scalar>>("distortion_parameters", tmp_contain);
    distort = Eigen::Map<Eigen::Matrix<Scalar, 5, 1>>(tmp_contain.data());
    t_process = std::make_shared<std::thread>(&Estimator::process, this);

    // sub_fea = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     topic_feas, 600, std::bind(&Estimator::feas_callback, this, std::placeholders::_1));
    // sub_custfea = this->create_subscription<my_vins::msg::FeatureMsg>(
    //     topic_custfeas, 600, std::bind(&Estimator::cust_feas_callback, this, std::placeholders::_1));
    // client_fea = this->create_client<my_vins_msg::srv::PointFeatureExtract>("front/feature_extractor");
}

Estimator::~Estimator(){}

// void Estimator::feaExtractCallback(){

// }

// void Estimator::feas_callback(const sensor_msgs::msg::PointCloud2::SharedPtr fea_msg)
// {
//     static int idx_msg = 0;
//     int width = fea_msg->width;
//     int height = fea_msg->height;
//     int point_step = fea_msg->point_step;
//     int row_step = fea_msg->row_step;

//     if (idx_msg == 0){
//         idx_msg = std::atoi(fea_msg->header.frame_id.c_str());
//     }else{
//         if (std::atoi(fea_msg->header.frame_id.c_str()) != idx_msg + 1){ 
//             throw std::runtime_error("Loss some package in front end");
//             return;
//         }
//         idx_msg++;
//     }
    
//     std::vector<my_vins::V4T> feas;
//     std::vector<int> map_prev;
//     const uint8_t* fea_data = reinterpret_cast<const uint8_t*>(fea_msg->data.data());
//     for (int i = 0; i < height; i++) {
//         for (int j = 0; j < width; j++) {
//             float x = *reinterpret_cast<const float*>(fea_data + i * row_step + j * point_step + 0);
//             float y = *reinterpret_cast<const float*>(fea_data + i * row_step + j * point_step + 4);
//             float vx = *reinterpret_cast<const float*>(fea_data + i * row_step + j * point_step + 8);
//             float vy = *reinterpret_cast<const float*>(fea_data + i * row_step + j * point_step + 12);
//             int idx = *reinterpret_cast<const int*>(fea_data + i * row_step + j * point_step + 16);
//             feas.push_back(my_vins::V4T(x, y, vx, vy));
//             map_prev.push_back(idx);
//         }
//     }
//     cv::Mat mat; 
//     fea_manager.addNode(fea_msg->header.stamp, mat, feas, map_prev);
// }


// void Estimator::cust_feas_callback(const my_vins::msg::FeatureMsg::SharedPtr fea_msg)
// {
//     static int idx_msg = 0;

//     int width = fea_msg->feature.width;
//     int height = fea_msg->feature.height;
//     int point_step = fea_msg->feature.point_step;
//     int row_step = fea_msg->feature.row_step;

//     if (idx_msg == 0){
//         idx_msg = fea_msg->idx;
//     }else{
//         if (fea_msg->idx != idx_msg + 1){ 
//             throw std::runtime_error("Loss some package in front end");
//             return;
//         }
//         idx_msg++;
//     }
    
//     std::vector<my_vins::V4T> feas;
//     std::vector<int> map_prev;
//     const uint8_t* fea_data = reinterpret_cast<const uint8_t*>(fea_msg->feature.data.data());
//     for (int i = 0; i < height; i++) {
//         for (int j = 0; j < width; j++) {
//             float x = *reinterpret_cast<const float*>(fea_data + i * row_step + j * point_step + 0);
//             float y = *reinterpret_cast<const float*>(fea_data + i * row_step + j * point_step + 4);
//             float vx = *reinterpret_cast<const float*>(fea_data + i * row_step + j * point_step + 8);
//             float vy = *reinterpret_cast<const float*>(fea_data + i * row_step + j * point_step + 12);
//             int idx = *reinterpret_cast<const int*>(fea_data + i * row_step + j * point_step + 16);
//             feas.push_back(my_vins::V4T(x, y, vx, vy));
//             map_prev.push_back(idx);
//         }
//     }
//     cv::Mat mat; 
//     QuaT q = Eigen::Map<Eigen::Quaterniond>(fea_msg->q_itoj.data()).cast<Scalar>();
//     V3T trans = Eigen::Map<Eigen::Vector3d>(fea_msg->t_itoj.data()).cast<Scalar>();
//     fea_manager.addNode(fea_msg->header.stamp, mat, feas, map_prev, q, trans);
// }


template<typename Scalar>
Eigen::Matrix<Scalar, 2, 1> ceresCameraInnerTransform(const Eigen::Matrix<Scalar, 4, 1>& inner_param, const Eigen::Matrix<Scalar, 5, 1>& distort_param,  const Eigen::Matrix<Scalar, 3, 1>& p)
{
    const Scalar fx = inner_param[0];
    const Scalar fy = inner_param[1];
    const Scalar cx = inner_param[2];
    const Scalar cy = inner_param[3];
    const Scalar k1 = distort_param[0];
    const Scalar k2 = distort_param[1];
    const Scalar k3 = distort_param[2];
    const Scalar p1 = distort_param[3];
    const Scalar p2 = distort_param[4];

    Scalar x_ = p[0] / p[2];
    Scalar y_ = p[1] / p[2];
    Scalar r_square = x_ * x_ + y_ * y_;
    Eigen::Matrix<Scalar, 2, 1> uv;
    uv[0] = fx * ( x_ * (Scalar(1) + k1 * r_square + k2 * r_square * r_square + k3 * r_square * r_square * r_square)
             + Scalar(2) * p1 * x_ * y_ + p2 * (r_square * Scalar(2) * x_ * x_) ) + cx;
             
    uv[1] = fy * ( y_ * (Scalar(1) + k1 * r_square + k2 * r_square * r_square + k3 * r_square * r_square * r_square) 
            + Scalar(2) * p2 * x_ * y_ + p1 * (r_square + Scalar(2) * y_ * y_)) + cy;
    return uv;
}

struct VisionFactor
{
    VisionFactor(V4T& inner_param, Eigen::Matrix<Scalar, 5, 1>& distort_param, Scalar u_, Scalar v_)
     :inner(inner_param), distort(distort_param), u(u_), v(v_){}
    // typedef double Type; 
    template<typename Type>
    bool operator()(const Type *q_, const Type *t_, const Type *p, Type* r) const{
        Eigen::Map<const Eigen::Matrix<Type, 3, 1>> p_inC0(p);
        Eigen::Map<const Eigen::Quaternion<Type>> q_C0toCi(q_);
        Eigen::Map<const Eigen::Matrix<Type,3,1>> t_C0toCi(t_);
        Eigen::Matrix<Type, 3, 3> q_CitoC0 = q_C0toCi.toRotationMatrix().transpose();
        Eigen::Matrix<Type, 3, 1> t_CitoC0 = -q_C0toCi.toRotationMatrix().transpose() * t_C0toCi;
        Eigen::Matrix<Type, 3, 1> p_inCi = q_CitoC0 * p_inC0 + t_CitoC0;

        // Eigen::Matrix<Type, 2, 1> uv = ceresCameraInnerTransform<Type>(inner.cast<Type>(), distort.cast<Type>(), p_inCi);
        // TODO: project the points;
        Eigen::Matrix<Type, 2, 1> uv;

        r[0] = u - uv[0];
        r[1] = u - uv[1];
        return true;
    }
    V4T inner;
    Eigen::Matrix<Scalar, 5, 1> distort;
    Scalar u,v;
};



void Estimator::localBA(){
    // ceres::Problem problem;
    // auto manifold = new ceres::QuaternionManifold;
    // std::vector<my_vins::FeatureNode>& nodes = fea_manager.getNodes();
    // std::vector<my_vins::Feature>& feas = fea_manager.getFeatures();
    // for (auto& node :nodes){
    //     problem.AddParameterBlock(const_cast<Scalar*>(node.t_C0toCi.data()), 3);
    //     problem.AddParameterBlock(const_cast<Scalar*>(node.q_C0toCi.coeffs().data()), 4);
    //     problem.SetManifold(const_cast<Scalar*>(node.q_C0toCi.coeffs().data()), manifold);
    //     for (auto& fea_id : node.map){
    //         auto& fea = feas[fea_id.idx];
    //         problem.AddParameterBlock(const_cast<Scalar*>(fea.p.data()), 3);

    //         auto factor = new VisionFactor(inner, distort, fea_id.x, fea_id.y);
    //         auto* cost_func = new ceres::AutoDiffCostFunction<VisionFactor, 2, 4, 3, 3>(factor);
    //         problem.AddResidualBlock(cost_func, nullptr, node.q_C0toCi.coeffs().data(), node.t_C0toCi.data(), fea.p.data());
    //     }
    // }
    // problem.SetParameterBlockConstant(nodes.front().q_C0toCi.coeffs().data());
    // problem.SetParameterBlockConstant(nodes.front().t_C0toCi.data());

    // ceres::Solver::Options options;
    // ceres::Solver::Summary summary;
    // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.minimizer_progress_to_stdout = true;
    // options.num_threads = std::thread::hardware_concurrency();
    // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << std::endl;
}

void Estimator::process()
{
    // for (;;)
    // {
    //     if (fea_manager.getNodes().size() >= 5 && fea_manager.getNodes().size() <= 6){
    //         localBA();
    //         break;
    //     }
    // }
    // RCLCPP_DEBUG(this->get_logger(), "process completed");
}

} // namespace my_vins