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
    problem.setVertexConstant(id_T_ItoC_vtx);
    std::vector<double> c_mat_vec;
    param.getConfigParam<std::vector<double>>("projection_parameters", c_mat_vec);
    camera_mat = Eigen::Map<V4T>(c_mat_vec.data());
}

MyVinsSlideWindow::~MyVinsSlideWindow(){}

void MyVinsSlideWindow::step()
{
    int opt_size = fm->getOptimizedNodeSize();
    int init_size = fm->getInitializedNodeSize();

    ceres::Solver::Options opt;
    opt.linear_solver_type = ceres::DENSE_SCHUR;
    opt.trust_region_strategy_type = ceres::DOGLEG;
    opt.max_num_iterations = 5;
    // opt.num_threads = std::thread::hardware_concurrency();
    opt.num_threads = 1;
    ceres::LossFunction* loss_f = new ceres::CauchyLoss(1.0);
    // ceres::LossFunction* loss_f = nullptr;
    lst_new_feas_ = new_feas_;
    new_feas_ = std::make_shared<std::vector<std::string>>();

    

    if (init_size > opt_size){

        // if (opt_size >= 1){
        //     CameraObserver& node_  = fm->getNodeAt<CameraObserver>(opt_size);
        //     CameraObserver& nbb  = fm->getNodeAt<CameraObserver>(opt_size - 1);
        //     auto prep = nbb.getSE3Position();
        //     node_.setPosition(prep.unit_quaternion() * nbb.getPreintegration().q(), 
        //      nbb.getSE3Position().unit_quaternion().toRotationMatrix() * nbb.getPreintegration().p() + nbb.getSE3Position().translation() + nbb.vel() * nbb.getPreintegration().getTotalTime() - 0.5 * V3T(0,0,9.8) * nbb.getPreintegration().getTotalTime() * nbb.getPreintegration().getTotalTime());
        //     node_.vel() = nbb.getSE3Position().unit_quaternion().toRotationMatrix() * nbb.getPreintegration().v()  - V3T(0,0,9.8) * nbb.getPreintegration().getTotalTime() + nbb.vel();
        // }


        CameraObserver& node = fm->getNodeAt<CameraObserver>(opt_size);
        /* Add node to the problem */
        std::string id_vtx_pose = "n" + std::to_string(opt_size);
        ceres::BaseVertex* pose_vtx = new PoseVertex(id_vtx_pose, opt_size, fm);

        std::string id_vtx_vbag = "v" + std::to_string(opt_size);
        ceres::BaseVertex* vbag_vtx = new VelBiasVertex(id_vtx_vbag, opt_size, fm);

        if (nullptr == problem.addVertex(pose_vtx) || nullptr == problem.addVertex(vbag_vtx)){
            problem.removeVertex(id_vtx_pose);
            problem.removeVertex(id_vtx_vbag);
            std::cout << "Please do not add the same vertex repeatedly! Id of vertex: " << id_vtx_pose << std::endl;
            return;
        }
        // if (win_nodes.size() == 0){
        //     problem.setVertexConstant(id_vtx_pose);
        //     problem.setVertexConstant(id_vtx_vbag);
        // }
        
        win_nodes.push_back(std::make_pair(id_vtx_pose, id_vtx_vbag));

        /* Add node IMU measurement */
        if (win_nodes.size() >= 2){
            std::string id_vtx_ppose = win_nodes.at(win_nodes.size() - 2).first;
            PoseVertex* vtx_ppose = static_cast<PoseVertex*>(problem.vertexAt(id_vtx_ppose));
            std::string id_vtx_pvbag = win_nodes.at(win_nodes.size() - 2).second;            
            VelBiasVertex* vtx_pvbag = static_cast<VelBiasVertex*>(problem.vertexAt(id_vtx_pvbag));

            std::string id_imu_edge = id_vtx_ppose + "_" + id_vtx_pose;
            CameraObserver& pnode = fm->getNodeAt<CameraObserver>(vtx_ppose->idx_node_);
            ceres::CostFunction* costf_imu = new imu_preintegrate::IMUCostFunction(&pnode.getPreintegration());
            std::vector<std::string> vtxes_of_edge;
            vtxes_of_edge.push_back(id_vtx_ppose);
            vtxes_of_edge.push_back(id_vtx_pvbag);
            vtxes_of_edge.push_back(id_vtx_pose);
            vtxes_of_edge.push_back(id_vtx_vbag);
            ceres::BaseEdge* edge_imu = new ceres::BaseEdge(id_imu_edge, costf_imu, nullptr, vtxes_of_edge);
            std::cout << id_vtx_ppose << " " << id_vtx_pvbag << " " << id_vtx_pose << " " << id_vtx_vbag << std::endl;
            std::cout << problem.vertexAt(id_vtx_ppose)->globalSize() << " " << problem.vertexAt(id_vtx_pvbag)->globalSize() << " " << problem.vertexAt(id_vtx_pose)->globalSize() << " " << problem.vertexAt(id_vtx_vbag)->globalSize() << std::endl; 
            std::cout << id_imu_edge << std::endl;
            problem.addEdge(edge_imu);
        }

        /* Add feature and feature measurement */
        std::vector<std::string> old_feas;
        for (int i = 0; i < node.observes.size(); i++)
        {
            PointObservation& ob = node.getObservationAt(i);
            std::string id_vtx_fea = "f" + std::to_string(ob.idx);
            Eigen::Map<const V3T> t_ItoC(vtx_T_ItoC->param().data());
            Eigen::Map<const QuaT> q_ItoC(vtx_T_ItoC->param().data() + 3);
            Sophus::SE3d T_ItoC(q_ItoC, t_ItoC);
            std::cout << T_ItoC.matrix3x4() << std::endl;
            FeaturePointVertex* fea_vtx = new FeaturePointVertex(id_vtx_fea, fm, ob.idx, opt_size, i, T_ItoC);
            if (nullptr == problem.addVertex(fea_vtx)){
                /* Feature point exists in the problem */
                if (win_nodes.size() < 2) continue;
                /* Add feature measurement */
                FeaturePointVertex* pfea_vtx = static_cast<FeaturePointVertex*>(problem.vertexAt(id_vtx_fea));
                PointObservation& pob = fm->getNodeAt<CameraObserver>(pfea_vtx->idx_node_).getObservationAt(pfea_vtx->idx_ob_);
                std::string id_edge_fea = pfea_vtx->id() + "_" + id_vtx_fea + "_" + id_vtx_pose;
                // std::cout << "camera mat: " << camera_mat.transpose() << std::endl;
                // std::cout << pob.getData().transpose() << std::endl;
                // std::cout << pob.toSphereObservation(camera_mat).transpose() << std::endl;
                ceres::CostFunction* cost_vis = new vis_meas::VisMeas(pob.toSphereObservation(camera_mat), ob.toSphereObservation(camera_mat));
                std::vector<std::string> vtxes_of_edge;
                vtxes_of_edge.push_back("n" + std::to_string(pfea_vtx->idx_node_));
                vtxes_of_edge.push_back(id_vtx_pose);
                vtxes_of_edge.push_back(id_T_ItoC_vtx);
                vtxes_of_edge.push_back(id_vtx_fea);
                ceres::BaseEdge* edge_vis = new ceres::BaseEdge(id_edge_fea, cost_vis, loss_f, vtxes_of_edge);
                problem.addEdge(edge_vis);
                old_feas.push_back(id_vtx_fea);
            }else{
                new_feas_->push_back(id_vtx_fea);
            }
        }

        if (win_nodes.size() >= 2){
            int size_removed = 0;
            for (const auto& lst_fea : *lst_new_feas_)
            {
                if (std::find(old_feas.begin(), old_feas.end(), lst_fea) == old_feas.end()){
                    problem.removeVertex(lst_fea);
                    size_removed ++;
                }
            }
            std::cout << "removed fea: " << size_removed << std::endl;
        }

        if (win_nodes.size() > 0){
            problem.setVertexConstant(win_nodes.front().first);
            problem.setVertexConstant(win_nodes.front().second);
        }

        if(win_nodes.size() == param.WINDOW_SIZE){
            std::vector<std::string> indices_opt_list, indices_feas_list;
            problem.findVertexesWithPrefix(indices_opt_list, {"n", "v"});
            problem.findVertexesWithPrefix(indices_feas_list, {"f"});
            std::cout << "opt size: " << indices_opt_list.size() << std::endl;
            std::copy(indices_opt_list.begin(), indices_opt_list.end(), std::ostream_iterator<std::string>(std::cout, " "));
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

            
            std::cout << std::endl << "size fea: " << indices_feas_list.size() << std::endl;
            ceres::Solver::Summary summary = problem.solve(opt);
            std::cout << summary.BriefReport() << std::endl;

            // exit(0);
        }else if (win_nodes.size() > param.WINDOW_SIZE){

            std::cout << "Marginalization" << std::endl;

            std::string id_vtx_pose_front = win_nodes.front().first;
            std::string id_vtx_vbag_front = win_nodes.front().second;
            std::vector<std::string> indices_vtxes = problem.findVertexAssociatedVertexes(id_vtx_pose_front, "f");
            std::cout << "mar feas: " << indices_vtxes.size() << std::endl;
            std::copy(indices_vtxes.begin(), indices_vtxes.end(), std::ostream_iterator<std::string>(std::cout, " "));
            std::cout << std::endl;
            
            indices_vtxes.push_back(id_vtx_vbag_front);
            indices_vtxes.push_back(id_vtx_pose_front);
            std::cout << "size of mar vtxes:" << indices_vtxes.size() << std::endl;
            if (nullptr == problem.marginalization(indices_vtxes, id_mar_edge)){
                problem.removeEdge(id_mar_edge);
                problem.marginalization(indices_vtxes, id_mar_edge);
            }
            win_nodes.pop_front();

            std::vector<std::string> indices_opt_list, indices_feas_list;
            problem.findVertexesWithPrefix(indices_opt_list, {"n", "v"});
            problem.findVertexesWithPrefix(indices_feas_list, {"f"});
            std::copy(indices_opt_list.begin(), indices_opt_list.end(), std::ostream_iterator<std::string>(std::cout, " "));
            std::cout << std::endl << "size fea: " << indices_feas_list.size() << std::endl;
            ceres::Solver::Summary summary = problem.solve(opt);
            std::cout << summary.BriefReport() << std::endl;
        }

        // PoseVertex* opted_vtx_pose =  static_cast<PoseVertex*>(problem.vertexAt(win_nodes.back().first));
        // VelBiasVertex* opted_vtx_vbag =  static_cast<VelBiasVertex*>(problem.vertexAt(win_nodes.back().second));
        // std::cout << "opt at " << opted_vtx_pose->idx_node_ << ":" << std::endl;
        // std::cout << opted_vtx_pose->p().transpose() << std::endl;
        // std::cout << opted_vtx_pose->q().coeffs().transpose() << std::endl;
        // opted_vtx_pose->updateData();
        // opted_vtx_vbag->updateData();

        // for (const std::string& id_new_fea: *new_feas_)
        // {
        //     FeaturePointVertex* opted_vtx_fea =  static_cast<FeaturePointVertex*>(problem.vertexAt(id_new_fea));
        //     Eigen::Map<const V3T> t_ItoC(vtx_T_ItoC->param().data());
        //     Eigen::Map<const QuaT> q_ItoC(vtx_T_ItoC->param().data() + 3);
        //     Sophus::SE3d T_ItoC(q_ItoC, t_ItoC);
        //     CameraObserver& node = fm->getNodeAt<CameraObserver>(opted_vtx_fea->idx_node_);
        //     Sophus::SE3d T_WtoCi = node.getSE3Position() * T_ItoC;
        //     opted_vtx_fea->updateData(T_ItoC, camera_mat, T_WtoCi);
        // }

        std::vector<std::string> indices_opt_list, indices_feas_list;
        problem.findVertexesWithPrefix(indices_opt_list, {"n"});
        problem.findVertexesWithPrefix(indices_feas_list, {"f"});
        for (const std::string& id_pose: indices_opt_list)
        {
            PoseVertex* pose_vtx = static_cast<PoseVertex*>(problem.vertexAt(id_pose));
            // std::cout << id_pose << std::endl;
            // std::cout << pose_vtx->q().coeffs().transpose() << std::endl;
            // std::cout << pose_vtx->p().transpose() << std::endl;
            pose_vtx->updateData();
        }
        
        for (const std::string& id_fea: indices_feas_list)
        {
            FeaturePointVertex* opted_vtx_fea = static_cast<FeaturePointVertex*>(problem.vertexAt(id_fea));
            Eigen::Map<const V3T> t_ItoC(vtx_T_ItoC->param().data());
            Eigen::Map<const QuaT> q_ItoC(vtx_T_ItoC->param().data() + 3);
            Sophus::SE3d T_ItoC(q_ItoC, t_ItoC);

            CameraObserver& node = fm->getNodeAt<CameraObserver>(opted_vtx_fea->idx_node_);

            Sophus::SE3d T_WtoCi = node.getSE3Position() * T_ItoC;
            opted_vtx_fea->updateData(T_ItoC, camera_mat, T_WtoCi);
        }
        

        node.setEstimateStatus(true);
    }
    return;
}

}
