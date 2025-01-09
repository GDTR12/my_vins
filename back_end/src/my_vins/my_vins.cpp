#include "my_vins.hpp"
#include "utils/input/input.hpp"
#include "cv_bridge/cv_bridge.h"
#include "utils/slam/slam_utils.hpp"
#include "utils/common/math_utils.hpp"
#include "utils/common/common_utils.hpp"
#include "my_vins_slidewindow.hpp"

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

    sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
        input.getConfigParam<std::string>("imu_topic"), 
        10000,
        std::bind(&MyVins::imuTopicCallback, this, std::placeholders::_1));


    std::vector<double> c_mat_vec, d_mat_vec;
    input.getConfigParam<std::vector<double>>("projection_parameters", c_mat_vec);
    input.getConfigParam<std::vector<double>>("distortion_parameters", d_mat_vec);
    M3T camera_mat;
    camera_mat << c_mat_vec[0], 0, c_mat_vec[2],
                      0, c_mat_vec[1], c_mat_vec[3],
                      0, 0, 1;

    V4T distort_mat = Eigen::Map<V4T>(d_mat_vec.data());
    vis = std::make_shared<MyVinsVis>(*this);
    sfm = std::make_shared<MyVinsSFM>(*this, *vis.get(), camera_mat, distort_mat);
    sldwin = std::make_shared<MyVinsSlideWindow>(&frame_manager);

    std::vector<double> q_ItoC_vec, t_ItoC_vec;
    input.getConfigParam<std::vector<double>>("extrinsic_param_q", q_ItoC_vec);
    input.getConfigParam<std::vector<double>>("extrinsic_param_t", t_ItoC_vec);
    q_ItoC = Eigen::Map<Eigen::Quaterniond>(q_ItoC_vec.data());
    q_ItoC.normalize();
    t_ItoC = Eigen::Map<V3T>(t_ItoC_vec.data());

    thread_main = std::make_shared<std::thread>(&MyVins::run, this);
    pthread_setname_np(thread_main->native_handle(), "back_end_run");
    thread_main->detach();

    std::cout.flags(std::ios::fixed); 
    std::cout.precision(5); 
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
    auto& req_data = req_buf.emplace_back();
    req_data.id = req.id;
    req_data.img = grayImg;
    req_data.t = msg->header.stamp;
    pub_match->publish(req);
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

void MyVins::imuTopicCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::unique_lock lock(mtx_ib);
    auto& imu_data = imu_buf.emplace_back();
    imu_data.t = rclcpp::Time(msg->header.stamp);
    imu_data.w = V3T(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z
    );
    imu_data.a = V3T(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );
    lock.unlock();
    if (imu_buf.size() > 10)cv_ib.notify_one();
}

bool MyVins::popImuData()
{
    // if (idx_nodePutImu >= frame_manager.getNodeSize() - 1) return false;
    // rclcpp::Time t_begin, t_end;
    // auto& node = frame_manager.getNodeAt<CameraObserver>(idx_nodePutImu);
    // auto& node_nxt = frame_manager.getNodeAt<CameraObserver>(idx_nodePutImu + 1);
    // t_end = node.getTime();
    // if (idx_nodePutImu == 0){
    //     t_begin = rclcpp::Time(0, 0, RCL_ROS_TIME);
    // }else{
    //     t_begin = frame_manager.getNodeAt(idx_nodePutImu - 1)->getTime();
    // }
    int imu_full_num = frame_manager.getValidNodeSize();
    int total_num = frame_manager.getNodeSize();
    if (total_num < 3 || imu_full_num > total_num - 2) return false;
    if (imu_full_num == 0){
        frame_manager.getNodeAt<CameraObserver>(imu_full_num).is_imufull = true;
        return false;
    }
    
    rclcpp::Time t_begin, t_end;
    auto& node_prev = frame_manager.getNodeAt<CameraObserver>(imu_full_num - 1);
    auto& node = frame_manager.getNodeAt<CameraObserver>(imu_full_num);
    auto& node_nxt = frame_manager.getNodeAt<CameraObserver>(imu_full_num + 1);
    
    t_begin = node_prev.getTime();
    t_end = node.getTime();

    std::unique_lock lock(mtx_ib);
    cv_ib.wait(lock, [this](){
        return imu_buf.size() > 10;
    });
    // 有足够的数据
    if (imu_buf.back().t > t_end){
        for (size_t i = 0; i < imu_buf.size() - 1; i++)
        {
            auto data = imu_buf.front();
            auto data_nxt = imu_buf.at(1);
            if (data.t < t_begin){
                imu_buf.pop_front();
                continue;
            } 
            else if (data.t >= t_begin && data.t <= t_end){
                imu_preintegrate::PreInterVar inte_var;
                inte_var.a = data.a;
                inte_var.w = data.w;
                inte_var.t = data.t.seconds();
                node.preintegrator.propagate(inte_var);
                imu_buf.pop_front();
                if (data_nxt.t > t_end){
                    slam_utils::ImuInterpData data0, data1;
                    data0.time = data.t.seconds();
                    data1.time = data_nxt.t.seconds();
                    data0.w = data.w; data0.a = data.a;
                    data1.w = data_nxt.w; data1.a = data_nxt.a;
                    data0.ba = node.imu_ba; data0.bw = node.imu_bw;
                    data1.ba = node_nxt.imu_ba; data1.bw = node_nxt.imu_bw;
                    
                    auto inter_d = slam_utils::ImuLinearInterp(data0, data1, t_end.seconds());
                    imu_preintegrate::PreInterVar inter_imu;
                    inter_imu.a = inter_d.a;
                    inter_imu.w = inter_d.w;
                    inter_imu.t = t_end.seconds();
                    node.is_imufull = true;
                    // 开始插值
                    if (t_end - data.t > rclcpp::Duration::from_seconds(2e-5)){
                        node.preintegrator.propagate(inter_imu);
                    }
                    // if (data_nxt.t - t_end > rclcpp::Duration::from_seconds(2e-5)){
                    //     node_nxt.preintegrator.init(node_nxt.imu_bw, 
                    //                                 node_nxt.imu_ba,
                    //                                 node.preintegrator.getCov(),
                    //                                 node.preintegrator.getJac(),
                    //                                 node.preintegrator.getNoise());
                    if (data_nxt.t - t_end > rclcpp::Duration::from_seconds(2e-5)){
                        node_nxt.preintegrator.init((V3T() << V3T::Zero()).finished(), 
                                                    (V3T() << V3T::Zero()).finished(),
                                                    (Eigen::Matrix<Scalar, 15, 15>() << Eigen::Matrix<Scalar, 15, 15>::Identity()).finished(),
                                                    (Eigen::Matrix<Scalar, 15, 15>() << Eigen::Matrix<Scalar, 15, 15>::Identity()).finished(),
                                                    (Eigen::Matrix<Scalar, 18, 18>() << Eigen::Matrix<Scalar, 18, 18>::Identity()).finished());
                        node_nxt.preintegrator.propagate(inter_imu);
                    }
                    break;
                }
            }
           
        }
    }
    lock.unlock();
    return true;
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
    if (id >= frame_manager.getNodeSize()){
        ObserverNode* prev_node = (id == 0 || frame_manager.getNodeSize() == 0) ? nullptr : frame_manager.getNodeAt(-1);
        auto buf_data = std::find_if(req_buf.begin(), req_buf.end(), [id](RequestBuf& buf){
            return id == buf.id;
        });
        rclcpp::Time t(buf_data->t);
        CameraObserver* node = frame_manager.appendIfKeyFrame(t, observe_data, feas_data, map_prev, prev_node);
        if (nullptr != node){
            node->setImage(buf_data->img);
        }
        if (frame_manager.getNodeSize() > 1)vis->showTwoNodeMatches(frame_manager.getNodeSize() - 2, frame_manager.getNodeSize() - 1);
        // auto& node = frame_manager.appendAccordingPrev<PointFeature, CameraObserver>(t, observe_data, feas_data, map_prev, prev_node);
    }else if(id < frame_manager.getNodeSize()){
        //创建空间点类型,点的观测数据类型,以及观测者
        ObserverNode* prev_node = frame_manager.getNodeAt(id - 1);
        frame_manager.setAccordingPrev<PointFeature, CameraObserver>(id, observe_data, feas_data, map_prev, prev_node);
    }
    return true;
}

void MyVins::imuInit()
{
    common_utils::TimerHelper timer;
    auto start = timer.start();
    RCLCPP_DEBUG(this->get_logger(), "[Step]: Imu init");
    M3T A = M3T::Zero();
    V3T B = V3T::Zero();
    V3T res = V3T::Zero();
    for (size_t i = 1; i < frame_manager.getNodeSize(); i++)
    {
        auto& node = frame_manager.getNodeAt<CameraObserver>(i);
        auto& node_prev = frame_manager.getNodeAt<CameraObserver>(i - 1);
        if (node.is_imufull == false || node_prev.is_imufull == false) continue;
        Sophus::SE3d T_CitoCj(node_prev.getSE3Position().inverse() * node.getSE3Position());
        // V3T t_CitoCj = T_CitoCj.translation();
        QuaT q_CitoCj = T_CitoCj.unit_quaternion();
        V3T r_IitoIj = node.preintegrator.getStateVar().block<3,1>(6,0);
        QuaT q_IitoIj = Sophus::SO3d::exp(r_IitoIj).unit_quaternion();
        M3T dq_dbg = node.preintegrator.getJac().block<3,3>(imu_preintegrate::ImuPreintegration::IDX_R, imu_preintegrate::ImuPreintegration::IDX_BG);
        M4T qmat = MathUtils::quaLeftMultiMat((q_ItoC * q_CitoCj.conjugate() * q_ItoC.inverse() * q_IitoIj).normalized());
        M3T tmp = 0.5 * qmat.bottomRightCorner<3,3>() * dq_dbg;
        V3T b = -qmat.block<3,1>(1,0);
        A = A + tmp;
        B = B + b;
        res = res + (q_ItoC.conjugate() * q_IitoIj * q_ItoC * q_CitoCj.conjugate()).vec();
    }
     
    V3T result  = A.inverse() * B;
    for (size_t i = 0; i < frame_manager.getNodeSize(); i++)
    {
        auto& node = frame_manager.getNodeAt<CameraObserver>(i);
        node.preintegrator.update(V3T::Zero(), result);
    }
    RCLCPP_INFO(this->get_logger(), "Imu bias: %s", EFMT(result.transpose()));
    RCLCPP_INFO(this->get_logger(), "Cost time: %.6f ms", timer.end(start));
#ifdef _DEBUG
    res.setZero();
    for (size_t i = 1; i < frame_manager.getNodeSize(); i++)
    {
        auto& node = frame_manager.getNodeAt<CameraObserver>(i);
        auto& node_prev = frame_manager.getNodeAt<CameraObserver>(i - 1);
        Sophus::SE3d T_CitoCj(node_prev.getSE3Position().inverse() * node.getSE3Position());
        QuaT q_CitoCj = T_CitoCj.unit_quaternion();
        V3T r_IitoIj = node.preintegrator.getStateVar().block<3,1>(6,0);
        QuaT q_IitoIj = Sophus::SO3d::exp(r_IitoIj).unit_quaternion();
        res = res + (q_ItoC.conjugate() * q_IitoIj * q_ItoC * q_CitoCj.conjugate()).vec();
    }
    RCLCPP_INFO(this->get_logger(), "res: %s", EFMT(res.transpose())); 
#endif
}

bool MyVins::visualInertialAlign()
{
    std::vector<V3T> vi;
    V3T g_InC0; Scalar s;
    Eigen::Matrix<Scalar, -1, -1> A;
    Eigen::Matrix<Scalar, -1, 1> B;
    M3T R_ItoC = q_ItoC.toRotationMatrix();
    
    for (size_t i = 1; i < frame_manager.getNodeSize(); i++)
    {
        auto& ni = frame_manager.getNodeAt<CameraObserver>(i - 1);
        auto& nj = frame_manager.getNodeAt<CameraObserver>(i);
        if (nj.is_imufull == false || ni.is_imufull == false) continue;
        Sophus::SE3d T_C0toCi = ni.getSE3Position();
        Sophus::SE3d T_C0toCj = nj.getSE3Position();
        Scalar dt = nj.preintegrator.getTotalTime();
        M3T R_IitoC0 = R_ItoC * T_C0toCi.rotationMatrix().transpose();
        M3T R_C0toIj = T_C0toCj.rotationMatrix() * R_ItoC.transpose();
        if (i == 1) {
            // A.conservativeResize(10, 10);
            // B.conservativeResize(10, Eigen::NoChange);
            A.conservativeResize(6, 10);
            B.conservativeResize(6, Eigen::NoChange);
            A.setZero();
            B.setZero();
        }else{
            A.conservativeResize(A.rows() + 6, A.cols() + 3);
            B.conservativeResize(B.rows() + 6, Eigen::NoChange);
            A.bottomRows<6>().setZero();
            A.rightCols<3>().setZero();
            B.tail<6>().setZero();
        }
        A.block<3,3>(A.rows() - 6, 0) = 0.5 * R_IitoC0 * dt * dt;
        A.block<3,3>(A.rows() - 3, 0) = R_IitoC0 * dt;
        A.block<3,1>(A.rows() - 6, 3) = R_IitoC0 * (T_C0toCj.translation() - T_C0toCi.translation());
        // std::cout << (T_C0toCi.translation()).transpose() << std::endl;
        A.block<3,3>(A.rows() - 6, A.cols() - 6) = -M3T::Identity() * dt;
        A.block<3,3>(A.rows() - 3, A.cols() - 6) = -M3T::Identity();
        A.block<3,3>(A.rows() - 3, A.cols() - 3) = R_IitoC0 * R_C0toIj;

        B.block<3,1>(A.rows() - 6, 0) = nj.preintegrator.getStateVar().head(3) - t_ItoC + R_IitoC0 * R_C0toIj * t_ItoC;
        B.block<3,1>(A.rows() - 3, 0) = nj.preintegrator.getStateVar().block<3,1>(3,0);
    }
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, -1, -1>> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd result = svd.solve(B);
    RCLCPP_INFO(this->get_logger(), "g: %s norm: %.3f", EFMT(result.head<3>().transpose()), result.head<3>().norm());
    if (fabs(result.head<3>().norm() - 9.8) > 0.8){
        RCLCPP_ERROR(this->get_logger(), "failed to align camera and imu!");
        return false;
    }else{
        return true;
    }
}

void MyVins::init()
{

}

void MyVins::run()
{
RCLCPP_INFO(this->get_logger(), "Run:");
while (true)
{
    popMatchBuffer();
    popImuData();
    if (!state.initialized){
        if (frame_manager.getNodeSize() < 20 || frame_manager.getNodeSize() % 5 != 0){
            continue;
        }else{
            RCLCPP_INFO(this->get_logger(), "init structer");
            if (sfm->initStructure()){
                sfm->transformAllFramesToWorld(QuaT::Identity(), V3T::Zero());
                imuInit();
                if (!visualInertialAlign()){
                    exit(-1);
                    continue;
                }
                state.initialized = true;
            }
        }
    }else{
        sldwin->step();
    }
    // std::this_thread::sleep_for(1ms);
}
}

} // namespace my_vins




