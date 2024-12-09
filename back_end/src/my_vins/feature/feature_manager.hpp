#pragma once 
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace my_vins
{
typedef double Scalar;
using V3T = Eigen::Matrix<Scalar, 3, 1>;
using V4T = Eigen::Matrix<Scalar, 4, 1>;
using QuaT = Eigen::Quaternion<Scalar>;
using M3T = Eigen::Matrix<Scalar, 3, 3>;
using M4T = Eigen::Matrix<Scalar, 4, 4>;
using V2T = Eigen::Matrix<Scalar, 2, 1>;
// 空间中的特征基类 可以衍生出 点, 线, 面
struct Feature{
public:
    Feature(){}
    ~Feature(){}
    std::vector<int> map_node;
    virtual void setExtendData(Eigen::Matrix<Scalar, -1, 1>&) = 0;
    virtual void getExtendData(Eigen::Matrix<Scalar, -1, 1>& data) = 0;
    bool is_initialized(){
        return initialized;
    }
protected:
    bool initialized = false;
};





class Observation{
public:
    int idx;  
    Observation() = delete;
    Observation(int idx_): idx(idx_){}
    virtual ~Observation(){}
    virtual void setExtendData(Eigen::Matrix<Scalar, -1, 1>& data) = 0;
    virtual void getExtendData(Eigen::Matrix<Scalar, -1, 1>& data) = 0;
};


class ObserverNode
{
public:
    ObserverNode() = delete;
    ObserverNode(rclcpp::Time t){
        time = t;
    }
    ObserverNode(rclcpp::Time t, QuaT& q_WtoB_, V3T& t_WtoB_){
        time = t;
        q_WtoB = q_WtoB_;
        t_WtoB = t_WtoB_;
        initialized = true;
    }
    virtual ~ObserverNode(){}

    void setPosition(QuaT q_WtoB_, V3T t_WtoB_){
        q_WtoB = q_WtoB_;
        q_WtoB.normalize();
        t_WtoB = t_WtoB_;
        initialized = true;
    }

    bool is_initialized(){
        return initialized;
    }

    void getPosition(QuaT& q_0toi, V3T& t_0toi){
        // if (is_initialized()){
            q_0toi = q_WtoB;
            t_0toi = t_WtoB;
        // }
    }


    M4T getPosition(){
        M4T ret = M4T::Identity();
        if (is_initialized()){
            ret.block<3,3>(0,0) = q_WtoB.toRotationMatrix();
            ret.block<3,1>(0,3) = t_WtoB;
        }
        return ret;
    }

    Sophus::SE3d getSE3Position(){
        Sophus::SE3d ret;
        if (is_initialized()){
            ret.setQuaternion(q_WtoB);
            ret.translation() = t_WtoB;
        }
        return ret;
    }

    rclcpp::Time getTime(){return time;}

    std::vector<std::unique_ptr<Observation>> observes;

private:
    bool initialized = false;
    rclcpp::Time time;
    QuaT q_WtoB = QuaT::Identity();
    V3T t_WtoB = V3T::Zero();
};





class FeatureManager
{
public:
    FeatureManager(){}
    ~FeatureManager(){}

    std::vector<std::unique_ptr<ObserverNode>>& getNodes() {return nodes;}
    std::vector<std::unique_ptr<Feature>>& getFeatures() {return feas;}

    template<typename ObserverNodeType>
    ObserverNodeType& append(rclcpp::Time t){
        auto& node = nodes.emplace_back(std::make_unique<ObserverNodeType>(t));
        return *(dynamic_cast<ObserverNodeType*>(node.get()));
    }

    template<typename ObserverNodeType>
    ObserverNodeType& append(rclcpp::Time t, QuaT& q_WtoB, V3T& t_WtoB){
        auto& node = nodes.emplace_back(std::make_unique<ObserverNodeType>(t));
        node->setPosition(q_WtoB, t_WtoB);
        return *(dynamic_cast<ObserverNodeType*>(node.get()));
    }

    template<typename FeatureType, typename ObserverNodeType>
    ObserverNodeType& setAccordingPrev(int idx, 
                          std::vector<Eigen::Matrix<Scalar, -1, 1>>& observation_data, 
                          std::vector<Eigen::Matrix<Scalar, -1, 1>>& feas_data,
                          std::vector<int>& map_prev,
                          ObserverNode* prev)
    {
        assert(idx < nodes.size());
        // auto& node = nodes.emplace_back(std::make_unique<ObserverNodeType>(t));
        auto& node = nodes[idx];

        if (feas.empty()){
            for (size_t i = 0; i < observation_data.size(); i++){
                auto& fea = feas.emplace_back(std::make_unique<FeatureType>(feas_data[i]));
                fea->map_node.push_back(0);
                auto& observation = node->observes.emplace_back(std::make_unique<typename ObserverNodeType::ObservationType>(i));
                observation->setExtendData(observation_data[i]);
            }
            return *(dynamic_cast<ObserverNodeType*>(node.get()));
        }else{
            assert(prev != nullptr);
            assert(observation_data.size() == feas_data.size());
            assert(observation_data.size() == map_prev.size());
            for (size_t i = 0; i < observation_data.size(); i++){
                if (map_prev[i] == -1){ // 新的Feature
                    auto& fea = feas.emplace_back(std::make_unique<FeatureType>(feas_data[i]));
                    fea->map_node.push_back(idx);
                    auto& observation = node->observes.emplace_back(std::make_unique<typename ObserverNodeType::ObservationType>(feas.size() - 1));
                    observation->setExtendData(observation_data[i]);
                }else{
                    int fea_idx = prev->observes[map_prev[i]]->idx;
                    feas[fea_idx]->map_node.push_back(idx);
                    auto& observation = node->observes.emplace_back(std::make_unique<typename ObserverNodeType::ObservationType>(fea_idx));
                    observation->setExtendData(observation_data[i]);
                }
            }
            return *(dynamic_cast<ObserverNodeType*>(node.get()));
        }
    }

    // 添加新的 Node 
    template<typename FeatureType, typename ObserverNodeType>
    void appendAccordingPrev(rclcpp::Time t, 
                 std::vector<Eigen::Matrix<Scalar, -1, 1>>& observation_data, 
                 std::vector<Eigen::Matrix<Scalar, -1, 1>>& feas_data,
                 std::vector<int>& map_prev,
                 ObserverNode* prev_node,
                 QuaT& qua,
                 V3T& trans)
    {
        appendAccordingPrev<FeatureType, ObserverNodeType>(t, observation_data, feas_data, map_prev, prev_node);
        nodes.back()->setPosition(qua, trans);
    }

    template<typename FeatureType, typename ObserverNodeType>
    ObserverNodeType& appendAccordingPrev(rclcpp::Time t, 
                 std::vector<Eigen::Matrix<Scalar, -1, 1>>& observation_data, 
                 std::vector<Eigen::Matrix<Scalar, -1, 1>>& feas_data,
                 std::vector<int>& map_prev,
                 ObserverNode* prev) // map_prev存的是 -1: 新点, idx: 上一个node.observes 的索引
    {
        append<ObserverNodeType>(t);
        return setAccordingPrev<FeatureType, ObserverNodeType>(nodes.size() - 1, observation_data, feas_data, map_prev, prev);
    }

    int getNodeSize(){return nodes.size();}
    int getFeatureSize(){return feas.size();}

    ObserverNode* getNodeAt(int id){
        // std::cout << nodes.size() << " " << id << std::endl;
        assert(nodes.size() > 0 && id < int(nodes.size()) && id >= -int(nodes.size()));
        if (id >= 0) return nodes[id].get();
        else return nodes[nodes.size() + id].get();
    }

    template<typename NodeType>
    NodeType& getNodeAt(int id){
        // std::cout << ": " << nodes.size() << " :" << id << " : " << int(-nodes.size()) << std::endl;
        // std::cout << bool(nodes.size() > 0) << " "  << bool(id < nodes.size()) << " " << bool(id >= int(-nodes.size())) << std::endl;
        assert(nodes.size() > 0 && id < int(nodes.size()) && id >= -int(nodes.size()));
        if (id >= 0) return *dynamic_cast<NodeType*>(nodes[id].get());
        else return *dynamic_cast<NodeType*>(nodes[nodes.size() + id].get());
    }


    Feature* getFeatureAt(int id){
        assert(feas.size() > 0 && id < int(feas.size()) && id >= -int(feas.size()));
        if (id >= 0) return feas[id].get();
        else return feas[feas.size() + id].get();
    }
    template<typename FeatureType>
    FeatureType& getFeatureAt(int id){
        assert(feas.size() > 0 && id < int(feas.size()) && id >= -int(feas.size()));
        if (id >= 0) return *dynamic_cast<FeatureType*>(feas[id].get());
        else return *dynamic_cast<FeatureType*>(feas[feas.size() + id].get());
    }
    
    void getFeasIndicesHasNodesOver(size_t min_nods, std::vector<int>& indices){
        assert(min_nods > 0);
        indices.clear();
        for (size_t i = 0; i < feas.size(); i++){
            if (feas[i]->map_node.size() >= min_nods){
                indices.push_back(i);
            }
        }
    }




    /* 
        mode = 0: all
        mode = 1: initialized
        mode = 2: uninitialized
     */
    template<typename FeatureType, typename ObservationType>
    void getNodeFeatures(uint32_t idx_node, std::vector<std::reference_wrapper<ObservationType>>& observes_ref,std::vector<std::reference_wrapper<FeatureType>>& feas_ref, int mode = 0){
        ObserverNode* node = getNodeAt(idx_node);
        if (mode == 0){
            for (auto& observe: node->observes){
                observes_ref.push_back(*dynamic_cast<ObservationType*>(observe.get()));
                feas_ref.push_back(*dynamic_cast<FeatureType*>(feas[observe->idx].get()));
            }
        }else if(mode == 1){
            for (auto& observe: node->observes){
                if (feas[observe->idx]->is_initialized()){
                    observes_ref.push_back(*dynamic_cast<ObservationType*>(observe.get()));
                    feas_ref.push_back(*dynamic_cast<FeatureType*>(feas[observe->idx].get()));
                }
            }
        }else if(mode == 2){
            for (auto& observe: node->observes){
                if (!feas[observe->idx]->is_initialized()){
                    observes_ref.push_back(*dynamic_cast<ObservationType*>(observe.get()));
                    feas_ref.push_back(*dynamic_cast<FeatureType*>(feas[observe->idx].get()));
                }
            }
        }
    }
    /* 
    mode = 0: all feas
    mode = 1: initialized feas
    mode = 2: uninitialized feas
     */
    void getMatches(uint32_t idx_nprev, 
                    uint32_t idx_nback, 
                    std::vector<uint32_t>& idx_feas, 
                    std::vector<uint32_t>& idx_prev, 
                    std::vector<uint32_t>& idx_back,
                    int mode = 0)
    {
        assert(idx_nprev < nodes.size() && idx_nback < nodes.size());
        idx_feas.clear(); 
        idx_prev.clear();
        idx_back.clear();
        ObserverNode* n_prev = nodes[idx_nprev].get();
        ObserverNode* n_back = nodes[idx_nback].get();
        for (size_t i = 0; i < n_prev->observes.size(); i++){
            Feature* fea = feas[n_prev->observes[i]->idx].get();
            std::vector<int>::iterator it = std::find_if(fea->map_node.begin(), fea->map_node.end(), [idx_nback](int idx_n){
                return idx_n == idx_nback;
            });
            if (fea->map_node.end() == it){
                continue;
            }else{
                int fea_idx = n_prev->observes[i]->idx;
                auto push_into = [&,this](){
                    idx_feas.push_back(fea_idx);
                    idx_prev.push_back(i);
                    std::vector<std::unique_ptr<my_vins::Observation>>::iterator it_back = std::find_if(n_back->observes.begin(), n_back->observes.end(), [fea_idx](std::unique_ptr<my_vins::Observation>& observe){
                        return fea_idx == observe->idx;
                    });
                    if (it_back == n_back->observes.end()){
                        std::cerr << __LINE__ << ": bug in this feature manager!" << std::endl;
                    }
                    idx_back.push_back(std::distance(n_back->observes.begin(), it_back));
                };
                if (mode == 0){
                    push_into();
                }else if (feas[fea_idx]->is_initialized() && mode == 1){
                    push_into();
                }else if (!feas[fea_idx]->is_initialized() && mode == 2){
                    push_into();
                }
            }
        }
    }


protected:
    // 维护一个双向查找的 搜索结构
    std::vector<std::unique_ptr<ObserverNode>> nodes;
    std::vector<std::unique_ptr<Feature>> feas;
    std::vector<int> idx_keyframe;
};

} // namespace my_vins

  