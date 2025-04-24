#pragma once
#include "feature/feature_manager.hpp"
#include "ceres_fgo/imu_factor/imuPreintegration/imuPreintegration.hpp"
#include "my_vins_param.hpp"

namespace my_vins
{
    
struct ImuData_6DOF{
    rclcpp::Time t;
    V3T w = V3T::Zero();
    V3T a = V3T::Zero();
    // float wx{0}, wy{0}, wz{0};
    // float ax{0}, ay{0}, az{0};
};

// 特征点类型
struct PointFeature: public Feature{
public:
    PointFeature():Feature(){}
    PointFeature(Eigen::Matrix<Scalar, -1, 1>& data){
        setExtendData(data);
    }

    void setExtendData(Eigen::Matrix<Scalar, -1, 1>& data) override{
        pose(0) = data(0,0);
        pose(1) = data(1,0);
        pose(2) = data(2,0);
    }
    void getExtendData(Eigen::Matrix<Scalar, -1, 1>& data) override{
        data(0) = pose(0,0);
        data(1) = pose(1,0);
        data(2) = pose(2,0);
    }
    void setData(V3T data){
        pose = data;
        initialized = true;
    }

    V3T getData(){
        return pose;
    }

    bool optimized(){return is_optimized;}
    void setOptimizationStatus(bool status){is_optimized = status;}

private:
    V3T pose = V3T::Zero(); 
    bool is_optimized = false;
};

class PointObservation : public Observation{
public:
    typedef PointFeature FeatureType;
    PointObservation(int idx): Observation(idx){}
    PointObservation(int idx, Eigen::Matrix<Scalar, -1, 1>& data)
    : Observation(idx){
        setExtendData(data);
    }

    virtual void setExtendData(Eigen::Matrix<Scalar, -1, 1>& data) override{
        assert(data.rows() == 3);
        x = data(0,0);
        y = data(1,0);
        depth = data(2,0);
    }
    virtual void getExtendData(Eigen::Matrix<Scalar, -1, 1>& data) override{
        data.resize(3,1);
        data(0,0) = x;
        data(1,0) = y;
        data(2,0) = depth;
    }

    void getData(Scalar& x_, Scalar& y_, Scalar& depth_){
        x_ = x;
        y_ = y;
        depth_ = depth;
    }
    V3T getData(){
        return V3T(x, y, depth);
    }

    void setData(Scalar x_, Scalar y_, Scalar depth_){
        x = x_;
        y = y_;
        depth = depth_;
    }

    std::vector<float>& desp(){return dsp;}

    V3T toSphereObservation(const V4T& K)
    {
        Scalar fx = K(0), fy = K(1), cx = K(2), cy = K(3);
        Scalar A = std::pow((x - cx) / fx, 2);
        Scalar B = std::pow((y - cy) / fy, 2);
        double sign_x = (x - cx) > 0 ? 1.0 : -1.0;
        double sign_y = (y - cy) > 0 ? 1.0 : -1.0;
        
        return V3T(
            sign_x / std::sqrt(1.0 + B / A + 1.0 / A),
            sign_y / std::sqrt(1.0 + A / B + 1.0 / B),
            1.0 / std::sqrt(1.0 + A + B)
        );
    }

private:

    Scalar x = 0, y = 0, depth = 0; std::vector<float> dsp;
};

class CameraObserver: public ObserverNode{
public:

    typedef ImuData_6DOF ImuData;
    typedef PointObservation ObservationType;

    CameraObserver(rclcpp::Time t):ObserverNode(t){
    }

    CameraObserver(rclcpp::Time t, cv::Mat& img_):ObserverNode(t){
        img = img_;
    }

    PointObservation& getObservationAt(int id)
    {
        assert(observes.size() > 0 && id < int(observes.size()) && id >= -int(observes.size()));
        if (id >= 0) return *dynamic_cast<PointObservation*>(observes[id].get());
        else return *dynamic_cast<PointObservation*>(observes[observes.size() + id].get());
    }

    cv::Mat getImage(){
        return img;
    }
    void setImage(cv::Mat& mat){
        img = mat;
    }

    V3T& vel(){return vel_;}

    V3T& ba(){return imu_ba;}
    
    V3T& bg(){return imu_bw;}

    void setEstimateStatus(bool status){is_estimated = status;}

    imu_preintegrate::ImuPreintegration& getPreintegration(){return preintegrator;}

private:
    cv::Mat img;
    int idx_map = -1;

    imu_preintegrate::ImuPreintegration preintegrator;
    bool is_imufull = false;
    bool is_estimated = false;
    V3T imu_bw = V3T::Zero();
    V3T imu_ba = V3T::Zero();
    V3T vel_ = V3T::Zero();

    friend class MyVins;
    friend class MyVinsFeatureManager;

};

class MyVinsFeatureManager : public FeatureManager
{
public:
    MyVinsFeatureManager(){dyn_map.reserve(500);}
    ~MyVinsFeatureManager(){}

    int getIMUFullNodeSize()
    {
        int size = FeatureManager::getNodeSize();
        std::find_if(nodes.rbegin(), nodes.rend(), [&size](std::unique_ptr<my_vins::ObserverNode>& node){
            CameraObserver& cam_node = *dynamic_cast<CameraObserver*>(node.get());
            if (cam_node.is_imufull == false){
                size--;
                return false;
            }else{
                return true;
            }
        });
        return size;
    }

    int getInitializedNodeSize()
    {
        int size = FeatureManager::getNodeSize();
        std::find_if(nodes.rbegin(), nodes.rend(), [&size](std::unique_ptr<my_vins::ObserverNode>& node){
            CameraObserver& cam_node = *dynamic_cast<CameraObserver*>(node.get());
            if (cam_node.is_initialized() == false){
                size--;
                return false;
            }else{
                return true;
            }
        });
        return size;
    }


    int getOptimizedNodeSize()
    {
        int size = FeatureManager::getNodeSize();
        std::find_if(nodes.rbegin(), nodes.rend(), [&size](std::unique_ptr<my_vins::ObserverNode>& node){
            CameraObserver& cam_node = *dynamic_cast<CameraObserver*>(node.get());
            if (cam_node.is_estimated == false){
                size--;
                return false;
            }else{
                return true;
            }
        });
        return size;
    }

    void getOptimizedFeaturesOfNode(int idx_node, std::vector<std::reference_wrapper<PointObservation>>& observes_ref,std::vector<std::reference_wrapper<PointFeature>>& feas_ref)
    {
        ObserverNode* node = getNodeAt(idx_node);
        for (auto& observe: node->observes){
            if (getFeatureAt<PointFeature>(observe->idx).optimized()){
                observes_ref.push_back(*dynamic_cast<PointObservation*>(observe.get()));
                feas_ref.push_back(*dynamic_cast<PointFeature*>(feas[observe->idx].get()));
            }
        }
    }


    CameraObserver* appendIfKeyFrame(  rclcpp::Time t, 
                            std::vector<Eigen::Matrix<Scalar, -1, 1>>& observation_data, 
                            std::vector<Eigen::Matrix<Scalar, -1, 1>>& feas_data,
                            std::vector<float>& desp, int dim,
                            std::vector<int>& map_prev,
                            ObserverNode* prev)
    {
        CameraObserver* node = nullptr;
        if (prev == nullptr){
            keyf_updated = true;
            lst_obd = observation_data;
            node = &appendAccordingPrev<PointFeature, CameraObserver>(t, observation_data, feas_data, map_prev, prev);
            return node;
        }

        if (keyf_updated){
            dyn_map.clear();
            dyn_map = map_prev;
        }else{
            std::vector<int> last_map = dyn_map;
            dyn_map.clear();
            dyn_map.resize(map_prev.size(), -1);
            for (int i = 0; i < map_prev.size(); i++)
            {
                if (map_prev[i] != -1){
                    if (last_map[map_prev[i]] != -1){
                        dyn_map[i] = last_map[map_prev[i]];
                    }
                }
            }
        }
        if (isKeyFrame(lst_obd, observation_data, dyn_map)){
            keyf_updated = true;
            lst_obd = observation_data;
            node = &appendAccordingPrev<PointFeature, CameraObserver>(t, observation_data, feas_data, dyn_map, prev);
        }else {
            keyf_updated = false;
        }
        return node;
    }

    bool isKeyFrame(std::vector<Eigen::Matrix<Scalar, -1, 1>>& lst_obd,
                    std::vector<Eigen::Matrix<Scalar, -1, 1>>& observation_data, 
                    std::vector<int>& map_prev)
    {
        static int count_not_keyframe = 0; 
        assert(lst_obd.size() == observation_data.size());
        double parallax = 0;
        int count = 0;
        for (size_t i = 0; i < lst_obd.size(); i++)
        {
            if (map_prev[i] >= 0){
                V2T pob = lst_obd.at(map_prev[i]).head<2>();
                V2T ob = observation_data.at(i).head<2>();
                parallax += std::abs((pob - ob).norm());
                count += 1;
            }
        }
        double avg_plx = parallax / count;
        // std::cout << avg_plx << std::endl;
        if (avg_plx < param.PARALLAX_THREASHOLD && count_not_keyframe < param.NOKEYFRAME_THREASHOLD){
            count_not_keyframe ++;
            return false;
        }else{
            count_not_keyframe = 0;
            return true;
        }
    }

private:
    std::vector<int> dyn_map;
    bool keyf_updated = true;
    std::vector<Eigen::Matrix<Scalar, -1, 1>> lst_obd;
    MyVinsParamServer& param = MyVinsParamServer::getInstance();
    V4T camera_mat;
};


} // namespace my_vins

