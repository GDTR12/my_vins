#include "my_vins_sfm.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/opencv.hpp"
#include "ceres/ceres.h"
#include "sophus/se3.hpp"
#include "utils/common/math_utils.hpp"
#include "utils/common/common_utils.hpp"
// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Projection factors to model the camera's landmark observations.
// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/ProjectionFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use a
// trust-region method known as Powell's Degleg
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>
#include <gtsam_unstable/slam/PartialPriorFactor.h>

namespace my_vins
{

struct SizedVisFactor: public ceres::SizedCostFunction<2, 3, 3, 3>
{
public:
    SizedVisFactor() = delete;

    static ceres::SizedCostFunction<2, 3, 3, 3>* create(M3T& camera_mat, V2T uv){
        return (new SizedVisFactor(camera_mat, uv));
    }

    V2T calculateResidual(const Eigen::Matrix<Scalar, 3, 4>& T_ito0, const V3T& p_in0) const {
        V3T KTP = (K * T_ito0 * V4T(p_in0.x(), p_in0.y(), p_in0.z(), 1));
        Scalar si = KTP[2];
        V3T err = observe - (Scalar(1) / si) * KTP;
        return err.head(2);
    }

    virtual bool Evaluate(
        Scalar const* const* params,
        Scalar *residuals,
        Scalar **jacobians
    ) const
    {
        Eigen::Map<const Eigen::Matrix<Scalar, 3, 1>> r_ito0(params[0]);
        Eigen::Map<const Eigen::Matrix<Scalar, 3, 1>> t_ito0(params[1]);
        Eigen::Map<const V3T> p_in0(params[2]);
        Eigen::Map<V2T> res(residuals);
        Eigen::Matrix<Scalar, 3, 4> T_ito0;
        T_ito0.block<3,3>(0,0) = Sophus::SO3<Scalar>::exp(r_ito0).matrix();
        T_ito0.block<3,1>(0,3) = t_ito0;
        res = calculateResidual(T_ito0, p_in0);

        if (jacobians != nullptr)
        {
            V3T p_ini = T_ito0 * V4T(p_in0.x(), p_in0.y(), p_in0.z(), 1);
            Scalar X = p_ini[0], Y = p_ini[1], Z = p_ini[2];
            Scalar _Z = 1 / Z, X_Z = X / Z, Y_Z = Y / Z; 
            Eigen::Matrix<Scalar, 2, 3,  Eigen::RowMajor> de_dp;
            de_dp << fx * _Z, 0, -fx * X_Z * _Z,
                     0, fy * _Z, -fy * Y_Z * _Z;
            de_dp = -de_dp;
        
            if (jacobians[0] != nullptr)
            {
                Eigen::Matrix<Scalar, 3, 3> dp_dx;
                dp_dx = -Sophus::SO3<Scalar>::hat(T_ito0.block<3,3>(0,0) * p_in0);// * Sophus::SO3<Scalar>::leftJacobian(r_ito0);
                Eigen::Map<Eigen::Matrix<Scalar, 2, 3, Eigen::RowMajor>> jac(jacobians[0]);
                jac = de_dp * dp_dx;
            }

            if (jacobians[1] != nullptr)
            {
                Eigen::Map<Eigen::Matrix<Scalar, 2, 3, Eigen::RowMajor>> jac(jacobians[1]);
                jac = de_dp;
            }
            
            if (jacobians[2] != nullptr)
            {
                Eigen::Map<Eigen::Matrix<Scalar, 2, 3, Eigen::RowMajor>> jac(jacobians[2]);
                jac = de_dp * T_ito0.block<3,3>(0,0);
            }
        }
        return true;
    }

private:

    SizedVisFactor(M3T& camera_mat, V2T uv)
    {
        K = camera_mat;
        fx = camera_mat(0,0);
        fy = camera_mat(1,1);
        cx = camera_mat(0,2);
        cy = camera_mat(1,2);
        observe.head<2>() = uv;
    }

    V3T observe = V3T(0,0,1);
    M3T K = M3T::Identity();  // camera_mat
    Scalar fx, fy, cx, cy;
};




// struct SizedVisFactor: public ceres::SizedCostFunction<2, 3, 3, 3>
// {
// public:
//     SizedVisFactor() = delete;

//     static ceres::SizedCostFunction<2, 3, 3, 3>* create(M3T& camera_mat, V2T uv){
//         return (new SizedVisFactor(camera_mat, uv));
//     }

//     V2T calculateResidual(const Eigen::Matrix<Scalar, 3, 4>& T_ito0, const V3T& p_in0) const {
//         V2T res;
//         V3T KTP = (K * T_ito0 * V4T(p_in0.x(), p_in0.y(), p_in0.z(), 1));
//         Scalar si = KTP[2];
//         V3T err = observe - (Scalar(1) / si) * KTP;
//         res = err.head(2);
//         return res;
//     }

//     virtual bool Evaluate(
//         Scalar const* const* params,
//         Scalar *residuals,
//         Scalar **jacobians
//     ) const
//     {
//         Eigen::Map<const Eigen::Matrix<Scalar, 3, 1>> r_ito0(params[0]);
//         Eigen::Map<const Eigen::Matrix<Scalar, 3, 1>> t_ito0(params[1]);
//         Eigen::Map<const V3T> p_in0(params[2]);
//         Eigen::Map<V2T> res(residuals);
//         Eigen::Matrix<Scalar, 3, 4> T_ito0;
//         T_ito0.block<3,3>(0,0) = Sophus::SO3<Scalar>::exp(r_ito0).matrix();
//         T_ito0.block<3,1>(0,3) = t_ito0;
//         // std::cout << T_ito0 << std::endl;
//         res = calculateResidual(T_ito0, p_in0);

//         if (jacobians != nullptr)
//         {
//             V3T p_ini = T_ito0 * V4T(p_in0.x(), p_in0.y(), p_in0.z(), 1);
//             Scalar X = p_ini[0], Y = p_ini[1], Z = p_ini[2];
//             Scalar _Z = 1 / Z, X_Z = X / Z, Y_Z = Y / Z; 
//             Eigen::Matrix<Scalar, 2, 3,  Eigen::RowMajor> de_dp;
//             de_dp << fx * _Z, 0, -fx * X_Z * _Z,
//                      0, fy * _Z, -fy * Y_Z * _Z;
//             de_dp = -de_dp;
//             // std::cout << de_dp << std::endl;
//             if (jacobians[0] != nullptr)
//             {
//                 Eigen::Matrix<Scalar, 3, 3> dp_dx;
//                 dp_dx = -Sophus::SO3<Scalar>::hat(T_ito0.block<3,3>(0,0) * p_in0);
//                 // dp_dx = -Sophus::SO3<Scalar>::hat(p_ini);
//                 Eigen::Map<Eigen::Matrix<Scalar, 2, 3>, Eigen::RowMajor> jac(jacobians[0]);
//                 jac = de_dp * dp_dx;
//                 // std::cout << jac << std::endl;
//             }

//             if (jacobians[1] != nullptr)
//             {
//                 Eigen::Map<Eigen::Matrix<Scalar, 2, 3>, Eigen::RowMajor> jac(jacobians[1]);
//                 jac = de_dp;
//             }
            
//             if (jacobians[2] != nullptr)
//             {
//                 Eigen::Map<Eigen::Matrix<Scalar, 2, 3>, Eigen::RowMajor> jac(jacobians[2]);
//                 jac = de_dp * T_ito0.block<3,3>(0,0);
//             }
//         }
//         return true;
//     }

// private:

//     SizedVisFactor(M3T& camera_mat, V2T uv)
//     {
//         K = camera_mat;
//         fx = camera_mat(0,0);
//         fy = camera_mat(1,1);
//         cx = camera_mat(0,2);
//         cy = camera_mat(1,2);
//         observe.head<2>() = uv;
//     }

//     V3T observe = V3T(0,0,1);
//     M3T K = M3T::Identity();  // camera_mat
//     Scalar fx, fy, cx, cy;
// };


struct AutoVisFactor{
public:
    AutoVisFactor() = delete;
    static ceres::CostFunction* create(M3T& camera_mat, V2T uv){
        return new ceres::AutoDiffCostFunction<AutoVisFactor, 2, 4, 3, 3>(new AutoVisFactor(camera_mat, uv));
    }
    template<typename Type>
    bool operator()(const Type *q_ito0_, const Type *t_ito0_, const Type *p_, Type* r) const{
        Eigen::Map<const Eigen::Quaternion<Type>> q_ito0(q_ito0_);
        Eigen::Map<const Eigen::Matrix<Type, 3, 1>> t_ito0(t_ito0_);
        Eigen::Map<const Eigen::Matrix<Type, 3, 1>> p_in0(p_);
        Eigen::Map<Eigen::Matrix<Type, 2, 1>> res(r);

        Eigen::Matrix<Type, 3, 4> T_ito0;
        T_ito0.template block<3,3>(0,0) = q_ito0.matrix();
        T_ito0.template block<3,1>(0,3) = t_ito0;

        Eigen::Matrix<Type, 3, 1> KTP = (K.cast<Type>() * T_ito0 * Eigen::Matrix<Type, 4, 1>(p_in0.x(), p_in0.y(), p_in0.z(), Type(1)));
        Eigen::Matrix<Type, 3, 1> err = observe.cast<Type>() - (Type(1) / KTP[2]) * KTP;
        res = err.head(2);
        return true;
    }

private:
    AutoVisFactor(M3T& camera_mat, V2T uv)
    {
        K = camera_mat;
        fx = camera_mat(0,0);
        fy = camera_mat(1,1);
        cx = camera_mat(0,2);
        cy = camera_mat(1,2);
        observe.head<2>() = uv;
    }

    V3T observe = V3T(0,0,1);
    M3T K = M3T::Identity();  // camera_mat
    Scalar fx, fy, cx, cy;
};

    
MyVinsSFM::MyVinsSFM(MyVins& vins, MyVinsVis& vis_)
    :vins(vins), vis(vis_){}

MyVinsSFM::MyVinsSFM(MyVins& vins, MyVinsVis& vis_, M3T& c_mat, V4T& d_mat)
    :vins(vins), vis(vis_), camera_mat(c_mat), distort_vec(d_mat){}




V3T MyVinsSFM::triangulatePoint(Eigen::Matrix<Scalar, 3, 4> &Pose0, Eigen::Matrix<Scalar, 3, 4> &Pose1, V2T pi, V2T pj){

    M4T design_matrix;
    V3T point_3d;
    design_matrix.row(0) = pi[0] * Pose0.row(2) - Pose0.row(0);
	design_matrix.row(1) = pi[1] * Pose0.row(2) - Pose0.row(1);
	design_matrix.row(2) = pj[0] * Pose1.row(2) - Pose1.row(0);
	design_matrix.row(3) = pj[1] * Pose1.row(2) - Pose1.row(1);
    V4T triangulated_point;
	triangulated_point =
		      design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
	point_3d(0) = triangulated_point(0) / triangulated_point(3);
	point_3d(1) = triangulated_point(1) / triangulated_point(3);
	point_3d(2) = triangulated_point(2) / triangulated_point(3);
    return point_3d;
}

void MyVinsSFM::triangulate(Eigen::Matrix<Scalar, 3, 4> &Pose0, 
                            Eigen::Matrix<Scalar, 3, 4> &Pose1,
                            std::vector<std::reference_wrapper<PointFeature>>& feas,
                            std::vector<std::reference_wrapper<PointObservation>>& observe_prev,
                            std::vector<std::reference_wrapper<PointObservation>>& observe_back)
{
    Eigen::Matrix<Scalar, 3, 4> R0 = camera_mat * Pose0;
    Eigen::Matrix<Scalar, 3, 4> R1 = camera_mat * Pose1;

    for (size_t i = 0; i < observe_prev.size(); i++)
    {
        Scalar x0, y0, depth0, x1, y1, depth1;
        observe_prev[i].get().getData(x0, y0, depth0);
        observe_back[i].get().getData(x1, y1, depth1);
        V3T p_in0 = triangulatePoint(R0, R1, V2T(x0, y0), V2T(x1, y1));
        feas[i].get().setData(p_in0);
    }
}



void MyVinsSFM::getMatches(uint32_t idx_prev,
                uint32_t idx_back,
                std::vector<std::reference_wrapper<PointFeature>>& feas,
                std::vector<std::reference_wrapper<PointObservation>>& observe_prev,
                std::vector<std::reference_wrapper<PointObservation>>& observe_back, int mode)
{
    auto& fea_manager = vins.frame_manager;
    std::vector<uint32_t> indices_feas, indices_prev, indices_back;
    fea_manager.getMatches(idx_prev, idx_back, indices_feas, indices_prev, indices_back, mode);

    for (size_t i = 0; i < indices_feas.size(); i++){
        feas.push_back(std::ref(*dynamic_cast<PointFeature*>(fea_manager.getFeatureAt(indices_feas[i]))));
    }

    CameraObserver& node_prev = *dynamic_cast<CameraObserver*>(fea_manager.getNodeAt(idx_prev));
    CameraObserver& node_back = *dynamic_cast<CameraObserver*>(fea_manager.getNodeAt(idx_back));

    for (size_t i = 0; i < indices_prev.size(); i++){
        observe_prev.push_back(std::ref(*dynamic_cast<PointObservation*>(node_prev.observes.at(indices_prev[i]).get())));
    }
    for (size_t i = 0; i < indices_back.size(); i++){
        observe_back.push_back(std::ref(*dynamic_cast<PointObservation*>(node_back.observes.at(indices_back[i]).get())));
    }
}


Scalar MyVinsSFM::computeParllax(std::vector<std::reference_wrapper<PointObservation>>& observe_prev,
                            std::vector<std::reference_wrapper<PointObservation>>& observe_back)
{
    auto& fea_manager = vins.frame_manager;
    Scalar parllax = 0.0f; 
    for (size_t i = 0; i < observe_prev.size(); i++){
        Scalar x0, y0, depth0, x1, y1, depth1;
        observe_prev[i].get().getData(x0, y0, depth0);
        observe_back[i].get().getData(x1, y1, depth1);
        parllax += sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    }
    return parllax / observe_prev.size();
}

bool MyVinsSFM::computeRotateAndUnScaledTranslate(
                std::vector<std::reference_wrapper<PointObservation>>& observe_prev,
                std::vector<std::reference_wrapper<PointObservation>>& observe_back,
                M3T& R_itoj,
                V3T& t_itoj
                )
{
    std::vector<cv::Point2f> match_pts0, match_pts1;

    for (size_t i = 0; i < observe_prev.size(); i++){
        Scalar x0, y0, depth0, x1, y1, depth1;
        observe_prev[i].get().getData(x0, y0, depth0);
        observe_back[i].get().getData(x1, y1, depth1);
        match_pts0.push_back(cv::Point2f(x0, y0));
        match_pts1.push_back(cv::Point2f(x1, y1));
    }

    cv::Mat c_mat, R_jtoi, t_jtoi;
    cv::eigen2cv(camera_mat, c_mat);
    cv::Mat mask;
    cv::Mat e_mat = cv::findEssentialMat(match_pts0, match_pts1, c_mat, 8, 0.99, 1.0, mask);
    int inner = cv::recoverPose(e_mat, match_pts0, match_pts1, c_mat, R_jtoi, t_jtoi, mask);
    if (inner < observe_prev.size() * 0.5f){
        return false;
    }
    cv::cv2eigen(R_jtoi, R_itoj);
    cv::cv2eigen(t_jtoi, t_itoj);
    R_itoj.transposeInPlace();
    t_itoj = -R_itoj * t_itoj;
    return true;
}


bool MyVinsSFM::computeRotateAndUnScaledTranslate(
                std::vector<std::reference_wrapper<PointObservation>>& observe_prev,
                std::vector<std::reference_wrapper<PointObservation>>& observe_back,
                M4T& T_itoj)
{
    M3T R_itoj; V3T t_itoj;
    bool ret = computeRotateAndUnScaledTranslate(observe_prev, observe_back, R_itoj, t_itoj);
    T_itoj.block<3,3>(0,0) = R_itoj;
    T_itoj.block<3,1>(0,3) = t_itoj;
    return ret;
}

bool MyVinsSFM::solvePnP(std::vector<std::reference_wrapper<PointObservation>>& observe, std::vector<std::reference_wrapper<PointFeature>>& feas, M4T& T_ito0)
{
    std::vector<cv::Point2f> cv_observes;
    std::vector<cv::Point3f> cv_feas;
    for (size_t i = 0; i < observe.size(); i++)
    {
        V3T observe_data = observe[i].get().getData();
        cv_observes.push_back(cv::Point2f(observe_data.x(), observe_data.y()));
        V3T fea_data = feas[i].get().getData();
        cv_feas.push_back(cv::Point3f(fea_data.x(), fea_data.y(), fea_data.z()));
    }
    cv::Mat c_mat, d_mat, r_vec, t_vec, r;
    r_vec.create(cv::Size(3,1), CV_32F);
    t_vec.create(cv::Size(3,1), CV_32F);

    Eigen::AngleAxis<Scalar> rotate(T_ito0.block<3,3>(0,0));
    V3T trans = T_ito0.block<3,1>(0,3);
    cv::eigen2cv(rotate.axis(), r_vec);
    cv::eigen2cv(trans, t_vec);

    cv::eigen2cv(camera_mat, c_mat);
    cv::eigen2cv(distort_vec, d_mat);
    bool ret = cv::solvePnPRansac(cv_feas, cv_observes, c_mat, cv::noArray(), r_vec, t_vec);
    M3T R_0toi;
    V3T t_ito0;
    if (ret){
        cv::Rodrigues(r_vec, r);
        cv::cv2eigen(r, R_0toi);
        cv::cv2eigen(t_vec, t_ito0);
    }
    T_ito0.block<3,3>(0,0) = R_0toi;
    T_ito0.block<3,1>(0,3) = t_ito0;
    return ret;
}

void MyVinsSFM::buildBA(std::vector<int>& indices_node)
{

}

void MyVinsSFM::globalBAAuto(int begin_idx, int end_idx)
{
    auto& fea_manager = vins.frame_manager;
    std::unique_ptr<ceres::Problem> problem = std::make_unique<ceres::Problem>();
    ceres::Manifold* manifold = new ceres::QuaternionManifold();
    // ceres::LocalParameterization* manifold = new MathUtils::QuaternionLocalParameter();

    std::unordered_map<int, QuaT> q_ito0_l;
    std::unordered_map<int, V3T> t_ito0_l;
    std::unordered_map<int, V3T> p_param_l;

    for (size_t i = 0; i < fea_manager.getFeatureSize(); i++)
    {
        auto& fea = fea_manager.getFeatureAt<PointFeature>(i);
        if (fea.map_node.size() < 2)
            continue;

        auto& p = p_param_l[i] = fea.getData();
        problem->AddParameterBlock(p.data(), 3);

        for (auto idx_node : fea.map_node)
        {
            auto& node = fea_manager.getNodeAt<CameraObserver>(idx_node);
            auto& observes = node.observes;
            V2T ob_data;
            auto iter = std::find_if(observes.begin(), observes.end(), [i, &ob_data](std::unique_ptr<Observation>& observation){
                if (observation->idx == i){
                    PointObservation& p_observe = *dynamic_cast<PointObservation*>(observation.get());
                    ob_data = p_observe.getData().head(2);
                    return true;
                }
                return false;
            });
            if (iter == observes.end()){
                std::cerr << "Wrong mapping" << std::endl;
            }

            ceres::CostFunction* cost_func = AutoVisFactor::create(camera_mat, ob_data);
            QuaT q_0toi; V3T t_0toi;
            node.getPosition(q_0toi, t_0toi);

            auto& q_ito0 = q_ito0_l[idx_node];
            auto& t_ito0 = t_ito0_l[idx_node];

            
            q_ito0 = q_0toi.conjugate();
            t_ito0 = -q_0toi.toRotationMatrix().transpose() * t_0toi;

            problem->AddParameterBlock(q_ito0.coeffs().data(), 4, manifold);
            problem->AddParameterBlock(t_ito0.data(), 3);

            if (idx_node == begin_idx){
                problem->SetParameterBlockConstant(q_ito0.coeffs().data());
            }
            if (idx_node == begin_idx || idx_node == end_idx)
            {
                problem->SetParameterBlockConstant(t_ito0.data());
            }
            
            
            // problem->SetManifold(q_ito0.coeffs().data(), manifold);

            std::vector<double*> res_associated_param;
            res_associated_param.push_back(q_ito0.coeffs().data());
            res_associated_param.push_back(t_ito0.data());
            res_associated_param.push_back(p.data());

            problem->AddResidualBlock(cost_func, new ceres::CauchyLoss(50), res_associated_param);
        }

    }

    // problem->SetParameterBlockConstant(q_ito0_l[0].coeffs().data());
    // problem->SetParameterBlockConstant(t_ito0_l[0].data());
    // std::cout << "======================" << std::endl;
    // for (size_t i = 0; i < p_param_l.size(); i++)
    // {
    //     std::cout << p_param_l[i].transpose() << std::endl;
    // }
    

    std::cout << "begin solve" << std::endl;

    ceres::Solver::Options options;
    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.num_threads = std::thread::hardware_concurrency(); 
    // options.minimizer_type = ceres::TRUST_REGION;
    // options.line_search_direction_type = ceres::LBFGS;
    // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    // 检查问题的初始状态
    // ceres::Problem::EvaluateOptions eval_options;
    // eval_options.apply_loss_function = true;
    // double total_cost = 0.0;
    // std::vector<double> residuals;
    // ceres::CRSMatrix jacobians;
    // if ( problem->Evaluate(eval_options, &total_cost, &residuals, nullptr, &jacobians)){
    //     std::cout << "Initial cost = " << total_cost << std::endl;
    //     for (size_t i = 0; i < 18; i++)
    //     {
    //         std::cout << jacobians.values[i] << " ";
    //     }
    //     std::cout << std::endl;
         
    // }else{
    //     std::cout << "Initial check failed" << std::endl;
    // }


    // std::cout << "Number of residuals: " << problem->NumResiduals() << std::endl;
    // std::cout << "Number of parameters: " << problem->NumParameters() << std::endl;
    // std::cout << "Number of parameter blocks: " << problem->NumParameterBlocks() << std::endl;


    ceres::Solver::Summary summary;
    ceres::Solve(options, problem.get(), &summary);
    std::cout << summary.BriefReport() << std::endl;

    for (auto& data_p : p_param_l)
    {
        auto& fea = fea_manager.getFeatureAt<PointFeature>(data_p.first);
        fea.setData(data_p.second);
    }
    for (auto& data_n : q_ito0_l)
    {
        auto& node = fea_manager.getNodeAt<CameraObserver>(data_n.first);
        // std::cout << data_n.second.coeffs().transpose() << std::endl; 
        QuaT qua = data_n.second.normalized().conjugate();
        V3T trans = -qua.matrix() * t_ito0_l[data_n.first];
        std::cout << trans.transpose() << std::endl;
        node.setPosition(qua, trans);
    }

}

void MyVinsSFM::globalBA(int idx_begin, int idx_end)
{
    auto& fea_manager = vins.frame_manager;
    std::unique_ptr<ceres::Problem> problem = std::make_unique<ceres::Problem>();
    // ceres::LocalParameterization* so3_manifold = new MathUtils::SO3Parameterization();
    // ceres::Manifold* so3_mainfold = new ceres::QuaternionManifold();
    // ceres::LocalParameterization* so3_mainfold = new MathUtils::QuaternionLocalParameter();
    std::unordered_map<int, Eigen::Matrix<Scalar, 3, 1>> r_ito0_l;
    // std::unordered_map<int, Eigen::Quaternion<Scalar>> q_ito0_l;
    std::unordered_map<int, Eigen::Matrix<Scalar, 3, 1>> t_ito0_l;
    std::unordered_map<int, V3T> p_param_l;

    for (size_t i = 0; i < fea_manager.getFeatureSize(); i++)
    {
        auto& fea = fea_manager.getFeatureAt<PointFeature>(i);
        if (fea.map_node.size() < 2)
            continue;

        auto& p = p_param_l[i];
        p = fea.getData();
        problem->AddParameterBlock(p.data(), 3);

        for (auto idx_node : fea.map_node)
        {
            auto& node = fea_manager.getNodeAt<CameraObserver>(idx_node);
            auto& observes = node.observes;
            V2T ob_data;
            auto iter = std::find_if(observes.begin(), observes.end(), [i, &ob_data](std::unique_ptr<Observation>& observation){
                if (observation->idx == i){
                    PointObservation& p_observe = *dynamic_cast<PointObservation*>(observation.get());
                    ob_data = p_observe.getData().head(2);
                    return true;
                }
                return false;
            });
            if (iter == observes.end()){
                std::cerr << "Wrong mapping" << std::endl;
            }

            ceres::SizedCostFunction<2, 3, 3, 3>* cost_func = SizedVisFactor::create(camera_mat, ob_data);
            QuaT q_0toi; V3T t_0toi;
            node.getPosition(q_0toi, t_0toi);

            auto& r_ito0 = r_ito0_l[idx_node];
            auto& t_ito0 = t_ito0_l[idx_node];

            r_ito0 = Sophus::SO3<Scalar>(q_0toi.toRotationMatrix().transpose()).log();
            // r_ito0 = Sophus::SO3<Scalar>(q_0toi.normalized().conjugate()).log();
            t_ito0 = -q_0toi.toRotationMatrix().transpose() * t_0toi;

            // problem->AddParameterBlock(q_ito0.coeffs().data(), 4, so3_mainfold);
            problem->AddParameterBlock(r_ito0.data(), 3);
            problem->AddParameterBlock(t_ito0.data(), 3);

            if (idx_node == idx_begin){
                problem->SetParameterBlockConstant(r_ito0.data());
            }
            if (idx_node == idx_begin || idx_node == idx_end){
                problem->SetParameterBlockConstant(t_ito0.data());
            }

            std::vector<double*> res_associated_param;
            res_associated_param.push_back(r_ito0.data());
            res_associated_param.push_back(t_ito0.data());
            res_associated_param.push_back(p.data());

            problem->AddResidualBlock(cost_func, new ceres::CauchyLoss(50), res_associated_param);
        }
    }
    std::cout << "begin solve" << std::endl;

    ceres::Solver::Options options;
    // options.minimizer_type = ceres::TRUST_REGION;
    // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.num_threads = 1; 
    options.minimizer_progress_to_stdout = true;
    options.num_threads = std::thread::hardware_concurrency();

    // 检查问题的初始状态
    ceres::Problem::EvaluateOptions eval_options;
    eval_options.apply_loss_function = true;
    double total_cost = 0.0;
    std::vector<double> residuals;
    ceres::CRSMatrix jacobians;
    if ( problem->Evaluate(eval_options, &total_cost, &residuals, nullptr, &jacobians)){
        std::cout << "Initial cost = " << total_cost << std::endl;
        for (size_t i = 0; i < 18; i++)
        {
            std::cout << jacobians.values[i] << " ";
        }
        std::cout << std::endl;
         
    }else{
        std::cout << "Initial check failed" << std::endl;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem.get(), &summary);
    std::cout << summary.BriefReport() << std::endl;

    for (auto& data_p : p_param_l)
    {
        auto& fea = fea_manager.getFeatureAt<PointFeature>(data_p.first);
        fea.setData(data_p.second);
    }
    for (auto& data_n : r_ito0_l)
    {
        auto& node = fea_manager.getNodeAt<CameraObserver>(data_n.first);
        QuaT qua = Sophus::SO3<Scalar>::exp(data_n.second).unit_quaternion().conjugate();
        V3T trans = -qua.matrix() * t_ito0_l[data_n.first];
        node.setPosition(qua, trans);
    }
}


void MyVinsSFM::globalBAGTSAM(int idx_begin, int idx_end)
{
    auto& fea_manager = vins.frame_manager;
    gtsam::NonlinearFactorGraph graph;
    auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.3)).finished()
    );
    auto measurement_noise =
      gtsam::noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v
    std::unordered_map<int, gtsam::Symbol> x_0toi_l;
    std::unordered_map<int, gtsam::Symbol> p_param_l;
    gtsam::Values initialEstimate;
    gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(camera_mat(0,0), camera_mat(1,1), 0, camera_mat(0,2), camera_mat(1,2)));
    auto huberModel = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(50),
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.0, 0.0))  // 观测噪声
    );

    auto pointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

    for (size_t i = 0; i < fea_manager.getFeatureSize(); i++)
    {
        auto& fea = fea_manager.getFeatureAt<PointFeature>(i);
        if (fea.map_node.size() < 2)
            continue;

        p_param_l[i] = gtsam::Symbol('l', i);
        gtsam::Point3 p = fea.getData();
        if (!initialEstimate.exists(gtsam::Symbol('l', i))){
            initialEstimate.insert<gtsam::Point3>(gtsam::Symbol('l', i), p);
        }

        for (auto idx_node : fea.map_node)
        {
            auto& node = fea_manager.getNodeAt<CameraObserver>(idx_node);
            auto& observes = node.observes;
            V2T ob_data;
            auto iter = std::find_if(observes.begin(), observes.end(), [i, &ob_data](std::unique_ptr<Observation>& observation){
                if (observation->idx == i){
                    PointObservation& p_observe = *dynamic_cast<PointObservation*>(observation.get());
                    ob_data = p_observe.getData().head(2);
                    return true;
                }
                return false;
            });
            if (iter == observes.end()){
                std::cerr << "Wrong mapping" << std::endl;
            }

            QuaT q_0toi; V3T t_0toi;
            node.getPosition(q_0toi, t_0toi);

            x_0toi_l[idx_node] = gtsam::Symbol('x', idx_node);

            auto x_0toi = gtsam::Pose3(Sophus::SE3<Scalar>(q_0toi.toRotationMatrix(), t_0toi).matrix());
            if (!initialEstimate.exists(gtsam::Symbol('x', idx_node))){
                initialEstimate.insert<gtsam::Pose3>(gtsam::Symbol('x', idx_node), x_0toi);
            }

            if (idx_node == idx_begin){
                gtsam::noiseModel::Diagonal::shared_ptr rotation_oise = 
                    gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0., 0., 0.));
                graph.add(gtsam::PartialPriorFactor<gtsam::Pose3>(gtsam::Symbol('x', idx_node), std::vector<size_t>({0,1,2}), gtsam::Rot3::Logmap(x_0toi.rotation()), rotation_oise));
            }

            if (idx_node == idx_end || idx_node == idx_begin){
                gtsam::noiseModel::Diagonal::shared_ptr trans_noise = 
                    gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.0, 0.0, 0.));
                graph.add(gtsam::PartialPriorFactor<gtsam::Pose3>(gtsam::Symbol('x', idx_node), std::vector<size_t>({3,4,5}), x_0toi.translation(), trans_noise));
            }

            graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
                ob_data, huberModel, gtsam::Symbol('x', idx_node), gtsam::Symbol('l', i), K
            );
        }
    }
    std::cout << "begin solve" << std::endl;


    auto start_t = common_utils::TimerHelper::start();
    auto optimizer = gtsam::LevenbergMarquardtOptimizer(graph, initialEstimate);
    gtsam::Values result = optimizer.optimize();
    double elapsed_ms = common_utils::TimerHelper::end(start_t);

    // result.print("Final results: \n");
    std::cout << "initial error = " << graph.error(initialEstimate) << std::endl;
    std::cout << "final error = " << graph.error(result) << std::endl;
    std::cout << "time = " << elapsed_ms << std::endl;

    for (auto& data_p : p_param_l)
    {
        auto& fea = fea_manager.getFeatureAt<PointFeature>(data_p.first);
        auto data = result.at<gtsam::Point3>(data_p.second.key());
        fea.setData(data);
    }
    for (auto& data_n : x_0toi_l)
    {
        auto& node = fea_manager.getNodeAt<CameraObserver>(data_n.first);
        // Sophus::SE3<Scalar> data = Sophus::SE3<Scalar>(result.at<gtsam::Pose3>(data_n.second.key()).rotation().toQuaternion().conjugate(), 
        //    -result.at<gtsam::Pose3>(data_n.second.key()).rotation().toQuaternion().conjugate().toRotationMatrix() * result.at<gtsam::Pose3>(data_n.second.key()).translation());

        Sophus::SE3<Scalar> data = Sophus::SE3<Scalar>(result.at<gtsam::Pose3>(data_n.second.key()).rotation().toQuaternion(), 
           result.at<gtsam::Pose3>(data_n.second.key()).translation());

        node.setPosition(data.unit_quaternion(), data.translation());
    }

}





bool MyVinsSFM::initStructure()
{
    auto& fea_manager = vins.frame_manager;
    int idx_node_begin, idx_node_end;
    for (int i = fea_manager.getNodeSize() - 2; i >= 0; i--)
    {
        std::vector<std::reference_wrapper<PointFeature>> feas;
        std::vector<std::reference_wrapper<PointObservation>> observe_prev;
        std::vector<std::reference_wrapper<PointObservation>> observe_back;

        CameraObserver& node_prev = *dynamic_cast<CameraObserver*>(fea_manager.getNodeAt(i));
        CameraObserver& node_back = *dynamic_cast<CameraObserver*>(fea_manager.getNodeAt(fea_manager.getNodeSize() - 1));

        getMatches(i, fea_manager.getNodeSize() - 1, feas, observe_prev, observe_back, 0);
        
        Scalar pallax = computeParllax(observe_prev, observe_back);
        if (pallax < vins.parallax_threashold || feas.size() < 20){
            continue;
        }

        M3T R_itoj;
        V3T t_itoj;
        if(!computeRotateAndUnScaledTranslate(observe_prev, observe_back, R_itoj, t_itoj)){
            continue;
        }
        QuaT q_0toi; V3T t_0toi;
        // node_prev.setPosition(QuaT::Identity(), V3T::Zero());
        node_prev.getPosition(q_0toi, t_0toi);

        M3T R_0toj; V3T t_0toj;
        R_0toj = q_0toi.toRotationMatrix() * R_itoj;
        t_0toj = q_0toi.toRotationMatrix() * t_itoj + t_0toi;

        Eigen::Matrix<Scalar, 3, 4> T_ito0, T_jto0;
        // T_ito0.block<3,3>(0,0) = q_0toi.toRotationMatrix();// .transpose();
        // T_ito0.block<3,1>(0,3) = t_0toi;//-q_0toi.toRotationMatrix().transpose() * t_0toi;
        // T_jto0.block<3,3>(0,0) = R_0toj;//.transpose();
        // T_jto0.block<3,1>(0,3) = t_0toj;//-R_0toj.transpose() * t_0toj;
        T_ito0.block<3,3>(0,0) = q_0toi.toRotationMatrix().transpose();
        T_ito0.block<3,1>(0,3) = -q_0toi.toRotationMatrix().transpose() * t_0toi;
        T_jto0.block<3,3>(0,0) = R_0toj.transpose();
        T_jto0.block<3,1>(0,3) = -R_0toj.transpose() * t_0toj;

        node_prev.setPosition(q_0toi, t_0toi);
        node_back.setPosition(QuaT(R_0toj), t_0toj);

        triangulate(T_ito0, T_jto0, feas, observe_prev, observe_back);

        std::vector<V3T> fea_list;
        for (auto& fea : feas){
            fea_list.push_back(fea.get().getData());
        }
        // std::cout << t_0toj.transpose() << std::endl;
        idx_node_begin = i;
        idx_node_end = fea_manager.getNodeSize() - 1;
        break;
    }
    
    std::cout << "idx begin:" << idx_node_begin << std::endl; 

    // 重建其他的帧
    // 求解 begin + 1 到 end - 2 的位姿
    for (size_t i = idx_node_begin + 1; i < idx_node_end; i++)
    {
        auto& node = fea_manager.getNodeAt<CameraObserver>(i);
        auto& node_back = fea_manager.getNodeAt<CameraObserver>(idx_node_end);
        std::vector<std::reference_wrapper<PointFeature>> feas_node;
        std::vector<std::reference_wrapper<PointObservation>> observations;
        fea_manager.getNodeFeatures(i, observations,feas_node, 1);
        if (feas_node.size() < 5) continue;

        std::vector<std::reference_wrapper<my_vins::PointFeature>> feas;
        std::vector<std::reference_wrapper<my_vins::PointObservation>> observe_prev;
        std::vector<std::reference_wrapper<my_vins::PointObservation>> observe_back;
        getMatches(i, idx_node_end, feas, observe_prev, observe_back, 0);
        if (feas.size() < 10) continue;
        M4T T_itoj, T_ito0;
        T_ito0 = node.getPosition().inverse();

        if (!solvePnP(observations, feas_node, T_ito0)) 
            continue;
        
        node.setPosition(QuaT(T_ito0.inverse().block<3,3>(0,0)).normalized(), T_ito0.inverse().block<3,1>(0,3));
    }
    // 求解 begin + 1 到 end - 2 的 feas 

    if (idx_node_end - idx_node_begin >= 2){
        for (size_t i = idx_node_begin + 1; i < idx_node_begin + (idx_node_end - idx_node_begin) / 2; i++)
        {
            std::vector<std::reference_wrapper<PointFeature>> feas;
            std::vector<std::reference_wrapper<PointObservation>> observe_prev;
            std::vector<std::reference_wrapper<PointObservation>> observe_back;

            CameraObserver& node_prev = *dynamic_cast<CameraObserver*>(fea_manager.getNodeAt(i));
            CameraObserver& node_back = *dynamic_cast<CameraObserver*>(fea_manager.getNodeAt(idx_node_end));

            getMatches(i, idx_node_end, feas, observe_prev, observe_back, 2);
            Eigen::Matrix<Scalar, 3, 4> T_ito0 = node_prev.getPosition().inverse().block<3,4>(0,0);
            Eigen::Matrix<Scalar, 3, 4> T_jto0 = node_back.getPosition().inverse().block<3,4>(0,0);
            triangulate(T_ito0, T_jto0, feas, observe_prev, observe_back);
        }
        for (size_t i = idx_node_end - 1; i >= idx_node_begin + (idx_node_end - idx_node_begin) / 2; i--)
        {
            std::vector<std::reference_wrapper<PointFeature>> feas;
            std::vector<std::reference_wrapper<PointObservation>> observe_prev;
            std::vector<std::reference_wrapper<PointObservation>> observe_back;

            CameraObserver& node_prev = *dynamic_cast<CameraObserver*>(fea_manager.getNodeAt(idx_node_begin));
            CameraObserver& node_back = *dynamic_cast<CameraObserver*>(fea_manager.getNodeAt(i));
            getMatches(idx_node_begin, i, feas, observe_prev, observe_back, 2);
            Eigen::Matrix<Scalar, 3, 4> T_ito0 = node_prev.getPosition().inverse().block<3,4>(0,0);
            Eigen::Matrix<Scalar, 3, 4> T_jto0 = node_back.getPosition().inverse().block<3,4>(0,0);
            triangulate(T_ito0, T_jto0, feas, observe_prev, observe_back);
        }
    }
    // // // 求解 0 到 begin - 1 的位姿和feas
    for (int i = idx_node_begin - 1; i >= 0; i--)
    {
        std::vector<std::reference_wrapper<PointFeature>> feas_node;
        std::vector<std::reference_wrapper<PointObservation>> observations;
        fea_manager.getNodeFeatures(i, observations, feas_node, 1);
        M4T T_0toi;
        if (!solvePnP(observations, feas_node, T_0toi)) 
            continue;
        
        auto& node = fea_manager.getNodeAt<CameraObserver>(i);
        node.setPosition(QuaT(T_0toi.block<3,3>(0,0)).normalized(), T_0toi.block<3,1>(0,3));

        std::vector<std::reference_wrapper<PointFeature>> feas;
        std::vector<std::reference_wrapper<PointObservation>> observe_prev;
        std::vector<std::reference_wrapper<PointObservation>> observe_back;
        CameraObserver& node_prev = *dynamic_cast<CameraObserver*>(fea_manager.getNodeAt(i));
        CameraObserver& node_back = *dynamic_cast<CameraObserver*>(fea_manager.getNodeAt(idx_node_begin));
        getMatches(i, idx_node_begin, feas, observe_prev, observe_back, 2);
        Eigen::Matrix<Scalar, 3, 4> T_ito0 = node_prev.getPosition().inverse().block<3,4>(0,0);
        Eigen::Matrix<Scalar, 3, 4> T_jto0 = node_back.getPosition().inverse().block<3,4>(0,0);
        triangulate(T_ito0, T_jto0, feas, observe_prev, observe_back);
    }
    std::vector<std::unique_ptr<my_vins::Feature>>& feas = fea_manager.getFeatures(); 
    for (int i = 0; i < feas.size(); i++){
        auto& fea = *dynamic_cast<PointFeature*>(feas[i].get());
        if (fea.is_initialized() || fea.map_node.size() < 2){
            continue;
        }
        auto& node0 = fea_manager.getNodeAt<CameraObserver>(fea.map_node[0]);
        auto& node1 = fea_manager.getNodeAt<CameraObserver>(fea.map_node[1]);
        
        auto iter0 = std::find_if(node0.observes.begin(), node0.observes.end(), [i](std::unique_ptr<my_vins::Observation>& observe){
            return observe.get()->idx == i;
        });

        PointObservation& observe0 = *dynamic_cast<PointObservation*>((*iter0).get());
        V3T observ_data0 = observe0.getData();

        auto iter1 = std::find_if(node1.observes.begin(), node1.observes.end(), [i](std::unique_ptr<my_vins::Observation>& observe){
            return observe.get()->idx == i;
        });
        PointObservation& observe1 = *dynamic_cast<PointObservation*>((*iter1).get());
        V3T observ_data1 = observe1.getData();

        Eigen::Matrix<Scalar, 3, 4> R0 = camera_mat * node0.getPosition().block<3,4>(0,0);
        Eigen::Matrix<Scalar, 3, 4> R1 = camera_mat * node1.getPosition().block<3,4>(0,0);
        V3T p_in0 = triangulatePoint(R0, R1, observ_data0.head(2), observ_data1.head(2));
        fea.setData(p_in0);
    }
    // vis.visAllNodesWithFeas();
    // vis.showTwoNodeMatches(idx_node_begin, idx_node_end);
    // // vis.visAllFeatures();
    // // vis.visCamearaNodesBetween(idx_node_begin, idx_node_end);
    // vis.visAllNodesTracjectory();
    globalBA(idx_node_begin, idx_node_end);
    // globalBAAuto(idx_node_begin, idx_node_end);
    // globalBAGTSAM(idx_node_begin, idx_node_end);
    // vis.visAllNodesWithFeas();
    // // vis.visAllFeatures();
    // // vis.visCamearaNodesBetween(idx_node_begin, idx_node_end);
    // vis.visAllNodesTracjectory();

    return true;
}

void MyVinsSFM::transformAllFramesToWorld(QuaT q_ItoC, V3T t_ItoC)
{
    auto& fea_manager = vins.frame_manager;
    if (fea_manager.getNodeSize() == 0) return;
    auto& front = fea_manager.getNodeAt<CameraObserver>(0);
    Sophus::SE3<Scalar> T_C0_hat(front.getPosition());
    Sophus::SE3<Scalar> T_W0toC0(q_ItoC, t_ItoC);
    front.setPosition(T_W0toC0.unit_quaternion(), T_W0toC0.translation());
    Sophus::SE3<Scalar> T_W0toC0_hat = T_W0toC0 * T_C0_hat.inverse(); 
    for(int i = 1; i < fea_manager.getNodeSize(); i++)
    {
        auto& node = fea_manager.getNodeAt<CameraObserver>(i);
        Sophus::SE3<Scalar> T_C0toCi_hat(node.getPosition());
        Sophus::SE3<Scalar> T_W0toCi = T_W0toC0_hat * T_C0toCi_hat;
        node.setPosition(T_W0toCi.unit_quaternion(), T_W0toCi.translation());
    }
    for (size_t i = 0; i < fea_manager.getFeatureSize(); i++)
    {
        auto& fea = fea_manager.getFeatureAt<PointFeature>(i);
        V3T data_hat = fea.getData();
        V3T data = T_W0toC0_hat * data_hat;
        fea.setData(data);
    }
    vis.visAllNodesWithFeas();
    // vis.visAllFeatures();
    // vis.visCamearaNodesBetween(idx_node_begin, idx_node_end);
    vis.visAllNodesTracjectory();
}

} // namespace my_vins
