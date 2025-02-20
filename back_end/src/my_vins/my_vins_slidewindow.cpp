#include "my_vins_slidewindow.hpp"
#include "my_vins_feature.hpp"
#include "ceres_fgo/vis_factor/vis_meas.hpp"
#include "ceres_fgo/imu_factor/ceres_imu_preitg_factor.hpp"

namespace my_vins
{

ceres::Manifold* PoseVertex::pose_mainfold = new MathUtils::PoseEigenQuaRightPerturbManifold();

MyVinsSlideWindow::MyVinsSlideWindow(MyVinsFeatureManager* fm_, const QuaT q_ItoC, const V3T t_ItoC)
{
    fm = fm_;
    vtx_T_ItoC = new ceres::BaseVertex(id_T_ItoC_vtx, 7);
    Eigen::VectorXd data = (Eigen::Matrix<double, 7, 1>() << t_ItoC, q_ItoC.coeffs()).finished();
    vtx_T_ItoC->initializeParameter(data, PoseVertex::pose_mainfold);
    vtx_T_ItoC = problem.addVertex(vtx_T_ItoC);
    std::vector<double> c_mat_vec;
    param.getConfigParam<std::vector<double>>("projection_parameters", c_mat_vec);
    camera_mat = Eigen::Map<V4T>(c_mat_vec.data());
}

MyVinsSlideWindow::~MyVinsSlideWindow(){}


void MyVinsSlideWindow::marginalization()
{
    
}

void MyVinsSlideWindow::step()
{
    int opt_size = fm->getOptimizedNodeSize();
    int init_size = fm->getInitializedNodeSize();

    ceres::Solver::Options opt;
    opt.linear_solver_type = ceres::DENSE_SCHUR;
    opt.trust_region_strategy_type = ceres::DOGLEG;
    opt.max_num_iterations = param.NUM_ITERATIONS;

    if (init_size < opt_size){
        CameraObserver node = fm->getNodeAt<CameraObserver>(init_size);
        /* Add node to the problem */
        std::string id_vtx_pose = "n" + std::to_string(init_size);
        ceres::BaseVertex* pose_vtx = new PoseVertex(id_vtx_pose, init_size, fm);

        std::string id_vtx_vbag = "v" + std::to_string(init_size);
        ceres::BaseVertex* vbag_vtx = new PoseVertex(id_vtx_vbag, init_size, fm);
        if (nullptr == problem.addVertex(pose_vtx) || nullptr == problem.addVertex(vbag_vtx)){
            problem.removeVertex(id_vtx_pose);
            problem.removeVertex(id_vtx_vbag);
            std::cout << "Please do not add the same vertex repeatedly! Id of vertex: " << id_vtx_pose << std::endl;
            return;
        }
        
        win_nodes.push_back(std::make_pair(id_vtx_vbag, id_vtx_pose));

        /* Add node IMU measurement */
        if (win_nodes.size() >= 2){
            std::string id_vtx_ppose = win_nodes.at(win_nodes.size() - 2).first;
            PoseVertex* vtx_ppose = static_cast<PoseVertex*>(problem.vertexAt(id_vtx_ppose));
            std::string id_vtx_pvbag = win_nodes.at(win_nodes.size() - 2).second;            
            VelBiasVertex* vtx_pvbag = static_cast<VelBiasVertex*>(problem.vertexAt(id_vtx_pvbag));

            std::string id_imu_edge = id_vtx_ppose + "_" + id_vtx_pose;
            CameraObserver pnode = fm->getNodeAt<CameraObserver>(vtx_ppose->idx_node_);
            ceres::CostFunction* costf_imu = new imu_preintegrate::IMUCostFunction(&pnode.getPreintegration());
            std::vector<std::string> vtxes_of_edge;
            vtxes_of_edge.push_back(id_vtx_ppose);
            vtxes_of_edge.push_back(id_vtx_pvbag);
            vtxes_of_edge.push_back(id_vtx_pose);
            vtxes_of_edge.push_back(id_vtx_vbag);
            ceres::BaseEdge* edge_imu = new ceres::BaseEdge(id_imu_edge, costf_imu, nullptr, vtxes_of_edge);
            problem.addEdge(edge_imu);
        }

        /* Add feature and feature measurement */
        for (int i = 0; i < node.observes.size(); i++)
        {
            PointObservation& ob = node.getObservationAt(i);
            std::string id_vtx_fea = "f" + std::to_string(ob.idx);
            Eigen::Map<const V3T> t_ItoC(vtx_T_ItoC->param().data());
            Eigen::Map<const QuaT> q_ItoC(vtx_T_ItoC->param().data() + 3);
            Sophus::SE3d T_ItoC(q_ItoC, t_ItoC);
            FeaturePointVertex* fea_vtx = new FeaturePointVertex(id_vtx_fea, fm, ob.idx, init_size, i, T_ItoC);
            if (nullptr == problem.addVertex(fea_vtx)){
                /* Feature point exists in problem */
                if (win_nodes.size() < 2) continue;
                /* Add feature measurement */
                FeaturePointVertex* pfea_vtx = static_cast<FeaturePointVertex*>(problem.vertexAt(id_vtx_fea));
                PointObservation& pob = fm->getNodeAt<CameraObserver>(pfea_vtx->idx_node_).getObservationAt(pfea_vtx->idx_ob_);
                std::string id_edge_fea = pfea_vtx->id() + "_" + id_vtx_fea + "_" + id_vtx_pose;
                ceres::CostFunction* cost_vis = new vis_meas::VisMeas(pob.toSphereObservation(camera_mat), ob.toSphereObservation(camera_mat));
                std::vector<std::string> vtxes_of_edge;
                vtxes_of_edge.push_back("n" + std::to_string(pfea_vtx->idx_node_));
                vtxes_of_edge.push_back(id_vtx_pose);
                vtxes_of_edge.push_back(id_T_ItoC_vtx);
                vtxes_of_edge.push_back(id_vtx_fea);
                ceres::BaseEdge* edge_vis = new ceres::BaseEdge(id_edge_fea, cost_vis, nullptr, vtxes_of_edge);
                problem.addEdge(edge_vis);
            }
        }

        if (win_nodes.size() < param.WINDOW_SIZE){
            node.setEstimateStatus(true);
        }else if(win_nodes.size() == param.WINDOW_SIZE){
            ceres::Solver::Summary summary = problem.solve(opt);
            std::cout << summary.BriefReport() << std::endl;
        }else if (win_nodes.size() > param.WINDOW_SIZE){
            std::string id_vtx_pose_front = win_nodes.front().first;
            std::string id_vtx_vbag_front = win_nodes.front().second;
            std::vector<std::string> indices_vtxes = problem.findVertexAssociatedVertexes(id_vtx_pose_front, "f");
            indices_vtxes.push_back(id_vtx_vbag_front);
            indices_vtxes.push_back(id_vtx_pose_front);
            if (nullptr == problem.marginalization(indices_vtxes, id_mar_edge)){
                problem.removeEdge(id_mar_edge);
                problem.marginalization(indices_vtxes, id_mar_edge);
            }
            ceres::Solver::Summary summary = problem.solve(opt);
            std::cout << summary.BriefReport() << std::endl;
        }

    }
    return;
}

}
