#pragma once
#include "iostream"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/converter.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace slam_utils
{


class SensorInput{
public:
    SensorInput(){}
    // using TopicT = sensor_msgs::msg::Imu;
    // using DataT = int;
    template<typename TopicT, typename DataT>
    void getDataFromRosBag(std::string bag_path, 
            std::string topic_name, 
            std::vector<DataT>& data, 
            std::function<bool(std::shared_ptr<TopicT>&, DataT&)> dataTransform,
            double start_time = 0, 
            double end_time = INFINITY)
    {
        rosbag2_cpp::Reader reader;
        std::vector<std::shared_ptr<TopicT>> vec_raw_data;
        reader.open(bag_path);
        rclcpp::Time bag_start(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                reader.get_metadata().starting_time.time_since_epoch()).count(), RCL_ROS_TIME);
        while (reader.has_next()){
            rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            if (msg->topic_name == topic_name){
                std::shared_ptr<TopicT> t_data = std::make_shared<TopicT>();
                rclcpp::Serialization<TopicT> slz_data;
                slz_data.deserialize_message(&serialized_msg, t_data.get());
                auto raw_time = rclcpp::Time(t_data->header.stamp);
                // std::cout << raw_time.get_clock_type() << std::endl;
                if (raw_time >  bag_start + rclcpp::Duration::from_seconds(start_time) + rclcpp::Duration::from_seconds(end_time - start_time)){
                    break;
                }
                data.emplace_back(DataT{});
                DataT& d_data = data.back();
                if(!dataTransform(t_data, d_data)){
                    data.pop_back();
                    continue;
                }
            }
        }
        reader.close();
        // std::cout << "=========================" << bag_start << std::endl;
    }
private:
    
};




class ROSParamInput : rclcpp::Node
{
private:
    ROSParamInput(): rclcpp::Node("RosParam"){}
public:
    static ROSParamInput& getInstance() {
        static ROSParamInput instance;
        return instance;
    }

    template<typename T>
    T getConfigParam(std::string name){
        T ret;
        try{
            this->declare_parameter(name, rclcpp::ParameterValue(ret).get_type());
        }catch(const std::exception& e){
            // std::cerr << e.what() << '\n';
        }
        this->get_parameter(name, ret);
        return ret;
    }
    template<typename T>
    void getConfigParam(std::string name, T& ret){
        try{
            this->declare_parameter(name, rclcpp::ParameterValue(ret).get_type());
        }catch(const std::exception& e){
            // std::cerr << e.what() << '\n';
        }
        this->get_parameter(name, ret);
    }
    template<typename T> 
    T getPrivateParam(std::string name){
        std::string file = getConfigParam<std::string>("this_file_url");
        YAML::Node root = YAML::LoadFile(file);
        return root[name].as<T>();
    }
    ~ROSParamInput(){}
    ROSParamInput(const ROSParamInput&) = delete;
    ROSParamInput& operator=(const ROSParamInput&) = delete;
};


} // namespace slam_utils
