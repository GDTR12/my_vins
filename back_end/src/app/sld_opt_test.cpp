#include "ceres_fgo/base/marginalization_problem.hpp"
#include "ceres_fgo/vis_factor/vis_meas.hpp"
#include <sophus/se3.hpp>


using V3T = Eigen::Vector3d;
using V2T = Eigen::Vector2d;
using V4T = Eigen::Vector4d;
using M3T = Eigen::Matrix3d;
using M4T = Eigen::Matrix4d;
using QuaT = Eigen::Quaterniond;
using SO3T = Sophus::SO3d;
using SE3T = Sophus::SE3d;

class PoseVertex : public ceres::BaseVertex
{
public:
    int idx_node_ = -1;

    static ceres::Manifold* pose_mainfold;

    PoseVertex(const std::string& id, int idx_node)
        :ceres::BaseVertex(id, 7) 
    {
        idx_node_ = idx_node;
        setMainfold(pose_mainfold);
    }

    Eigen::Map<QuaT> q(){return Eigen::Map<QuaT>(param_.data() + 3);}

    Eigen::Map<V3T> p(){return Eigen::Map<V3T>(param_.data());}

};


class FeaVertex : public ceres::BaseVertex
{
public: 
    int idx_fea_ = -1;
    int id_node_ = -1;
    FeaVertex(const std::string& id, int id_fea)
        : ceres::BaseVertex(id, 1)
    {
        idx_fea_ = id_fea;
    }
    ~FeaVertex(){}
    double& inv_d(){return this->param_.x();}
};

ceres::Manifold* PoseVertex::pose_mainfold = new MathUtils::PoseEigenQuaRightPerturbManifold;

bool isPointInCamera(const M3T& K, const V2T& res, const V3T& p)
{
    V3T pp = K * p;
    return pp.x() / pp.z() > 0 && pp.x() / pp.z() < res.x() \
        && pp.y() / pp.z() > 0 && pp.y() / pp.z() < res.y() \
        && pp.z() > 0;
}

Eigen::VectorXd generate_gaussian_noise_vector(int size, double mu = 0.0, double sigma = 1.0) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(mu, sigma);
    
    Eigen::VectorXd noise(size);
    for (int i = 0; i < size; ++i) {
        noise(i) = dist(gen);
    }
    return noise;
}

// 生成旋转噪声（以旋转向量形式表示）
Eigen::AngleAxisd generate_rotation_noise(double sigma_rad) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0.0, sigma_rad);

    // 生成三维高斯噪声向量（表示李代数中的扰动）
    Eigen::Vector3d delta_axis(dist(gen), dist(gen), dist(gen));

    // 将扰动转换为旋转向量（角度为噪声向量的模长）
    return Eigen::AngleAxisd(delta_axis.norm(), delta_axis.normalized());
}


V3T toSphereObservation(const V2T& p, const V4T& K)
{
    double fx = K(0), fy = K(1), cx = K(2), cy = K(3);
    double A = std::pow((p.x() - cx) / fx, 2);
    double B = std::pow((p.y() - cy) / fy, 2);
    double sign_x = (p.x() - cx) > 0 ? 1.0 : -1.0;
    double sign_y = (p.y() - cy) > 0 ? 1.0 : -1.0;
    
    return V3T(
        sign_x / std::sqrt(1.0 + B / A + 1.0 / A),
        sign_y / std::sqrt(1.0 + A / B + 1.0 / B),
        1.0 / std::sqrt(1.0 + A + B)
    );
}

int main(int argc, char* argv[])
{
    typedef std::pair<V3T, QuaT> Pose;

    /* 在 10 m * 10 m 的体素内随机生成 1000 个 features */
    int size_feas = 1000;
    std::vector<V3T> feas;
    feas.reserve(size_feas);

    for (size_t i = 0; i < size_feas; i++)
    {
        feas.push_back(5 * (V3T::Random() + V3T::Ones()));
    }
    

    /* 在 (0, 1, 2) ~ (8, 10, 9) 的这个随即轨迹生成 20 个 观测的相机帧 */ 
    /* 相机大小为 */
    int size_pose = 20;
    Pose begin_pose, end_pose;
    begin_pose.first = V3T(0, 1, 2);
    begin_pose.second = QuaT(0.934, 0.215, -0.192, 0.212).normalized();

    end_pose.first = V3T(9, 7 ,10);
    end_pose.second = QuaT(0.295, -0.371,  0.731, 0.492).normalized();

    V3T step_p = (end_pose.first - begin_pose.first) / double(size_pose - 1);
    double step_q = 1.0 / double(size_pose - 1);
    double slerp_value = 0.0;

    std::vector<std::pair<V3T, QuaT>> trajectory;
    trajectory.push_back(begin_pose);

    for (size_t i = 0; i < size_pose - 2; i++)
    {
        V3T p = trajectory.back().first + step_p;
        slerp_value += step_q;
        QuaT q = begin_pose.second.slerp(slerp_value, end_pose.second);
        trajectory.push_back(std::make_pair(p, q));
    }
    trajectory.push_back(end_pose);

    // for (auto pose : trajectory)
    // {
    //     std::cout << "t: " << pose.first.transpose()  << " q: " << pose.second.coeffs().transpose() << std::endl;
    // }

    // 设置相机内参
    M3T K;
    K << 400,  0, 200,
         0,  300, 150,
         0,    0,   1;
    V2T resolution(400, 300);

    /* 根据相机内参判断点是否被相机观测到 */
    std::vector<std::unordered_map<int, V2T>> ob(trajectory.size());
    for (size_t i = 0; i < trajectory.size(); i++)
    {
        for (size_t j = 0; j < feas.size(); j++)
        {
            Eigen::Affine3d T_CitoC0 =  (Eigen::Translation3d(trajectory[i].first) * Eigen::Isometry3d(trajectory[i].second)).inverse();
            V3T p_inCi = T_CitoC0.rotation() * feas[j] + T_CitoC0.translation();
            if (isPointInCamera(K, resolution, p_inCi)){
                ob[i][j] = (K * p_inCi).head(2) / (K * p_inCi).z();
            }
        }
    }
    for(const auto& obb : ob)
    {
        std::cout << obb.size() << " ";
    }
    std::cout << std::endl;
    
    ceres::MarginalizationProblem problem;
    double sigma_position = 1.0;
    double sigma_rotation = 0.1;
    std::vector<V3T> est_feas(feas.size());
    std::vector<Pose> est_pose(trajectory.size());

    ceres::BaseVertex* vtx_T_ItoC = new ceres::BaseVertex("T_ItoC", 7);
    vtx_T_ItoC->initializeParameter((Eigen::Matrix<double,7,1>() << 0,0,0,0,0,0,1).finished(), 
        PoseVertex::pose_mainfold);
    vtx_T_ItoC = problem.addVertex(vtx_T_ItoC);
    problem.setVertexConstant("T_ItoC");

    for (size_t i = 0; i < trajectory.size(); i++)
    {
        PoseVertex* vtx = new PoseVertex("p" + std::to_string(i), i);
        vtx->p() = trajectory[i].first + generate_gaussian_noise_vector(3, 0, sigma_position);
        vtx->q() = trajectory[i].second * generate_rotation_noise(sigma_rotation);
        est_pose[i].first = vtx->p();
        est_pose[i].second = vtx->q();

        Eigen::Affine3d T_CitoC0 = (Eigen::Translation3d(vtx->p()) * Eigen::Isometry3d(vtx->q())).inverse();

        problem.addVertex(vtx);
        for (const auto& [idx, data]: ob[i])
        {
            FeaVertex* vtx_fea = new FeaVertex("f" + std::to_string(idx), idx);
            est_feas[idx] = feas[idx] + generate_gaussian_noise_vector(3, 0, sigma_position);
            vtx_fea->inv_d() = 1.0f / (T_CitoC0.matrix() * (V4T() << est_feas[idx], 1).finished()).norm();
            vtx_fea->id_node_ = i;
            if (vtx_fea == problem.addVertex(vtx_fea)){

            }else{
                FeaVertex* pfea_vtx =  static_cast<FeaVertex*>(problem.vertexAt("f" + std::to_string(idx)));
                
                ceres::CostFunction* costf = new vis_meas::VisMeas(
                    toSphereObservation(ob[pfea_vtx->id_node_][idx], (V4T() << 400,300,200,150).finished()), 
                    toSphereObservation(ob[i][idx], (V4T() << 400,300,200,150).finished()));
                std::vector<std::string> vtxes_of_edge;
                vtxes_of_edge.push_back("p" + std::to_string(pfea_vtx->id_node_));
                vtxes_of_edge.push_back(vtx->id());
                vtxes_of_edge.push_back("T_ItoC");
                vtxes_of_edge.push_back(pfea_vtx->id());
                std::string id_edge = "p" + std::to_string(pfea_vtx->id_node_) + "_" + pfea_vtx->id() + "_" + vtx->id();
                ceres::BaseEdge* edge = new ceres::BaseEdge(id_edge, costf, nullptr, vtxes_of_edge);
                problem.addEdge(edge);
                std::cout << id_edge << " ";
            }
        }
    }
    std::cout << std::endl;

    ceres::Problem::EvaluateOptions eval_options;
    eval_options.apply_loss_function = true;
    double total_cost = 0.0;
    std::vector<double> residuals;
    ceres::CRSMatrix jacobians;
    if ( problem.problem->Evaluate(eval_options, &total_cost, &residuals, nullptr, &jacobians)){
        std::cout << "Initial cost = " << total_cost << std::endl;
        for (size_t i = 0; i < 18; i++)
        {
            std::cout << jacobians.values[i] << " ";
        }
        std::cout << std::endl;
        
    }else{
        std::cout << "Initial check failed" << std::endl;
    }

    ceres::Solver::Options opt;
    opt.linear_solver_type = ceres::SPARSE_SCHUR;
    opt.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    opt.num_threads = 16;
    ceres::Solver::Summary summary = problem.solve(opt);
    std::cout << summary.BriefReport() << std::endl;
    std::vector<std::string> id_poses;
    problem.findVertexesWithPrefix(id_poses, {"p"});
    for (const auto& id: id_poses)
    {
        int idx_pose = atoi(id.data() + 1);
        std::cout << "pose at " << idx_pose << ": ";
        // std::cout << problem.vertexAt(id)->param().transpose() << std::endl;
        // std::cout << trajectory.at(idx_pose).first.transpose() << trajectory.at(idx_pose).second.coeffs().transpose() << std::endl;
        std::cout << (problem.vertexAt(id)->param().head(3) - trajectory.at(idx_pose).first).norm() << " " 
                << (est_pose.at(idx_pose).first - trajectory.at(idx_pose).first).norm() << " ";
        std::cout << (QuaT(problem.vertexAt(id)->param().tail(4).data()) * trajectory.at(idx_pose).second.inverse()).vec().norm() << " " 
                << (est_pose.at(idx_pose).second * trajectory.at(idx_pose).second.inverse()).vec().norm() << std::endl;
    }
    

}

