#pragma once
#include "fgo_problem.hpp"
#include "utils/common/math_utils.hpp"
#include "utils/common/common_utils.hpp"

namespace ceres
{

struct MarginalizationInfo : public BaseEdge
{
    MarginalizationInfo() = delete;

    MarginalizationInfo(MarginalizationInfo&) = delete;

    MarginalizationInfo(MarginalizationInfo&&) = delete;

    MarginalizationInfo(std::unordered_map<std::string, BaseVertex*>* vertex)
    {
        vtxes_map_ = vertex;
    }

    Eigen::MatrixXd fej_;

    Eigen::MatrixXd feb_;

    int local_size = 0;

    int global_size = 0;

    Eigen::VectorXd x0;

    std::vector<std::string> mar_vtxes_;

    std::unordered_map<std::string, BaseVertex*>* vtxes_map_;
};

class MarginlizationFactor: public ceres::CostFunction
{
private:
    MarginalizationInfo* mar_info_;

    MarginlizationFactor(MarginalizationInfo* mar_info)
    {
        mar_info_ = mar_info;
        for (const std::string& vtx : mar_info_->vertexes())
        {
            mutable_parameter_block_sizes()->push_back(mar_info_->vtxes_map_->at(vtx)->globalSize());
        }
        set_num_residuals(mar_info_->local_size);
    }

public:

    static ceres::CostFunction* create(MarginalizationInfo* mar_info)
    {
        if (mar_info == nullptr){
            return nullptr;
        }
        if (mar_info->vertexes().size() == 0){
            return nullptr;
        }
        return new MarginlizationFactor(mar_info);
    }


    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::VectorXd dx;
        dx.resize(mar_info_->local_size);
        int idx_x0 = 0;
        int idx_dx = 0;
        auto& vtxes = *mar_info_->vtxes_map_;
        for (int i = 0; i < mar_info_->vertexes().size(); i++)
        {
            auto vtx = vtxes[mar_info_->vertexes()[i]];
            int global_size = vtx->globalSize();
            int local_size = vtx->localSize();
            Eigen::Map<const Eigen::VectorXd> x0(mar_info_->x0.segment(idx_x0, global_size).data(), global_size);
            Eigen::Map<const Eigen::VectorXd> x(parameters[i], global_size);
            if (nullptr != vtx->mainfold()){
                vtx->mainfold()->Minus(x.data(), 
                                       x0.data(), 
                                       dx.segment(idx_dx, local_size).data());
            }else{
                dx.segment(idx_dx, local_size) = x - x0;
            }
            idx_x0 += global_size;
            idx_dx += local_size;
        }
        Eigen::Map<Eigen::VectorXd> res(residuals, mar_info_->local_size);
        res = mar_info_->feb_ + mar_info_->fej_ * dx;
        if (jacobians){
            int idx_x0 = 0;
            for (size_t i = 0; i < mar_info_->vertexes().size(); i++)
            {
                auto vtx = vtxes[mar_info_->vertexes()[i]];
                int global_size = vtx->globalSize();
                int local_size = vtx->localSize();
                if (jacobians[i]){
                    Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>> jac(jacobians[i], mar_info_->fej_.rows(), global_size);
                    jac.setZero();
                    jac.leftCols(local_size) = mar_info_->fej_.middleCols(idx_x0, local_size);
                }
                idx_x0 += local_size;
            }
        }
        return true;
    }
};


class MarginalizationProblem : public FGOProblem
{
private:
public:
    MarginalizationInfo* marginalization(std::vector<std::string>& indices, std::string id, int mar_threads=4)
    {

        MarginalizationInfo* mar_info_ = new MarginalizationInfo(&vtxes_);
        std::vector<std::string> keeped_vtxes;
        mar_info_->mar_vtxes_.clear();
        mar_info_->setId(id);
        mar_info_->setLossFunction(nullptr);
        
        for (const auto [idx, vtx] : vtxes_)
        {
            if (std::find(indices.begin(), indices.end(), idx) != indices.end()){
                mar_info_->mar_vtxes_.push_back(idx);
            }else{
                int idx_x0 = mar_info_->global_size;
                int size_x0 = vtx->globalSize();
                mar_info_->x0.conservativeResize(idx_x0 + size_x0);
                mar_info_->x0.segment(idx_x0, size_x0) = vtx->param();
                mar_info_->global_size += size_x0;
                mar_info_->local_size += vtx->localSize();
                keeped_vtxes.push_back(idx);
            }
        }
        std::cout << "true mar: " << mar_info_->mar_vtxes_.size() << std::endl;

        // mar_info_->setVertexes(keeped_vtxes);
        // ceres::CostFunction* costf = MarginlizationFactor::create(mar_info_);
        // mar_info_->setCostFunction(costf);

        // ceres::Problem::EvaluateOptions eva_opts;
        // eva_opts.num_threads = mar_threads;
        // eva_opts.apply_loss_function = true;
        // std::vector<double*>& block =  eva_opts.parameter_blocks;
        // for (std::string id : mar_info_->vertexes())
        // {
        //     block.push_back(vtxes_[id]->param().data());
        // }

        // for (std::string id : mar_info_->mar_vtxes_)
        // {
        //     block.push_back(vtxes_[id]->param().data());
        // }


        // double cost = 0.0;
        // std::vector<double> residual(problem->NumResiduals());
        // ceres::CRSMatrix J;
        // problem->Evaluate(eva_opts, &cost, &residual, nullptr, &J);
        // Eigen::SparseMatrix<double> crs_J = MathUtils::ceresCRS2EigenSparse(J);
        // Eigen::Map<Eigen::VectorXd> e(residual.data(), residual.size());
        // Eigen::MatrixXd H = crs_J.transpose() * crs_J;
        // Eigen::VectorXd b = crs_J.transpose() * e;
        // std::cout << "Init H: " << H.rows() << " " << H.cols() << std::endl;
        // std::cout << "Init b: " << b.cols() << std::endl;
        
        // int k = getVertexesParamLocalSize(mar_info_->vertexes());
        // int m = getVertexesParamLocalSize(mar_info_->mar_vtxes_);

        // std::cout << "k: " << k << ", m: " << m << std::endl;

        // Eigen::MatrixXd Hkk = H.block(0, 0, k, k);
        // Eigen::MatrixXd Hkm = H.block(0, k, k, m);
        // Eigen::MatrixXd Hmm = H.block(k, k, m, m);
        // Eigen::VectorXd bk = b.head(k);
        // Eigen::VectorXd bm = b.tail(m);
        // Hmm = 0.5 * (Hmm.transpose() + Hmm);

        // Eigen::MatrixXd Hmm_inv = Hmm.inverse();
        
        // Eigen::MatrixXd H_ = Hkk - Hkm * Hmm_inv * Hkm.transpose();
        // Eigen::VectorXd b_ = bk - Hkm * Hmm_inv * bm;
        
        // double eps = 1e-8;
        // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(H_);
        // Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
        // Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

        // Eigen::VectorXd S_sqrt = S.cwiseSqrt();
        // Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

        // mar_info_->fej_ = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
        // mar_info_->feb_ = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b_;
        // std::cout << "FEJ: " << mar_info_->fej_.rows() << " " << mar_info_->fej_.cols() << std::endl;
        // std::cout << "FEB: " << mar_info_->feb_.rows() << " " << mar_info_->feb_.cols() << std::endl;


        // BaseEdge* ret = addEdge(mar_info_);
        // if (ret == nullptr){
        //     std::cout << "Add marginalization edge failed! id: " << id << std::endl;
        //     return nullptr;
        // }else{
        //     std::cout << "Add marginalization edge successful!" << std::endl;
        // }

        /* Remove the vertex and edge in the problem */
        for (const std::string id_vtx: mar_info_->mar_vtxes_)
        // for (const std::string id_vtx: indices)
        {
            removeVertex(id_vtx);
        }
        return mar_info_;
    }
    MarginalizationProblem(){};
    ~MarginalizationProblem(){};
};

    
} // namespace ceres


