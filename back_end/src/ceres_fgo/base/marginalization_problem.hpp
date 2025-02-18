#pragma once
#include "fgo_problem.hpp"
#include "utils/common/math_utils.hpp"

namespace ceres
{

struct MarginalizationInfo
{
    MarginalizationInfo() = delete;

    MarginalizationInfo(MarginalizationInfo&) = delete;

    MarginalizationInfo(MarginalizationInfo&&) = delete;

    MarginalizationInfo(std::unordered_map<std::string, BaseVertex*>* vertex)
    {
        vtxes_ = vertex;
    }
    
    Eigen::MatrixXd fej_;

    Eigen::MatrixXd feb_;

    int local_size = 0;

    int global_size = 0;

    Eigen::VectorXd x0;

    std::vector<std::string> keeped_vtxes_;

    std::vector<std::string> mar_vtxes_;

    std::unordered_map<std::string, BaseVertex*>* vtxes_;
};

class MarginlizationFactor: public ceres::CostFunction
{
private:
    std::shared_ptr<MarginalizationInfo> mar_info_;
    MarginlizationFactor(std::shared_ptr<MarginalizationInfo> mar_info)
    {
        mar_info_ = mar_info;
    }
public:
    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::VectorXd x;
        x.resize(mar_info_->global_size);
        for (const std::string& idx_vtx: mar_info_->mar_vtxes_)
        {
            
        }
        
    }
};


class MarginalizationProblem : public FGOProblem
{
private:
public:
    std::shared_ptr<MarginalizationInfo> marginalization(std::vector<std::string>& indices, int mar_threads=4)
    {

        std::shared_ptr<MarginalizationInfo> mar_info_ = std::make_shared<MarginalizationInfo>(&vtxes_);
        mar_info_->keeped_vtxes_.clear();
        mar_info_->mar_vtxes_.clear();
        
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
                mar_info_->keeped_vtxes_.push_back(idx);
            }
        }

        ceres::Problem::EvaluateOptions eva_opts;
        eva_opts.num_threads = mar_threads;
        eva_opts.apply_loss_function = true;
        std::vector<double*>& block =  eva_opts.parameter_blocks;
        for (std::string id : mar_info_->keeped_vtxes_)
        {
            block.push_back(vtxes_[id]->param().data());
        }

        for (std::string id : mar_info_->mar_vtxes_)
        {
            block.push_back(vtxes_[id]->param().data());
        }


        double cost = 0.0;
        std::vector<double> residual(problem->NumResiduals());
        ceres::CRSMatrix J;
        problem->Evaluate(eva_opts, &cost, &residual, nullptr, &J);
        Eigen::SparseMatrix<double> crs_J = MathUtils::ceresCRS2EigenSparse(J);
        Eigen::Map<Eigen::VectorXd> e(residual.data());
        Eigen::MatrixXd H = crs_J.transpose() * crs_J;
        Eigen::VectorXd b = crs_J * e;
        std::cout << "Init H: " << H.rows() << " " << H.cols() << std::endl;
        std::cout << "Init b: " << b.cols() << std::endl;
        
        int k = getVertexesParamLocalSize(mar_info_->keeped_vtxes_);
        int m = getVertexesParamLocalSize(mar_info_->mar_vtxes_);

        std::cout << "k: " << k << ", m: " << m << std::endl;

        Eigen::MatrixXd Hkk = H.block(0, 0, k, k);
        Eigen::MatrixXd Hkm = H.block(0, k, k, m);
        Eigen::MatrixXd Hmm = H.block(k, k, m, m);
        Eigen::VectorXd bk = b.head(k);
        Eigen::VectorXd bm = b.tail(m);
        Hmm = 0.5 * (Hmm.transpose() + Hmm);
        
        Eigen::MatrixXd H_ = Hkk - Hkm * Hmm.inverse() * Hkm.transpose();
        Eigen::VectorXd b_ = bk - Hkm * Hmm.inverse() * bm;
        
        double eps = 1e-8;
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(H_);
        Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
        Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

        Eigen::VectorXd S_sqrt = S.cwiseSqrt();
        Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

        mar_info_->fej_ = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
        mar_info_->feb_ = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b_;
        std::cout << "FEJ: " << mar_info_->fej_.rows() << " " << mar_info_->fej_.cols() << std::endl;
        std::cout << "FEB: " << mar_info_->feb_.rows() << " " << mar_info_->feb_.cols() << std::endl;

        /* Remove the vertex and edge in the problem */
        for (const std::string id_vtx: mar_info_->mar_vtxes_)
        {
            removeVertex(id_vtx);
        }
        
        return mar_info_;
    }
    MarginalizationProblem(){};
    ~MarginalizationProblem(){};
};

    
} // namespace ceres


