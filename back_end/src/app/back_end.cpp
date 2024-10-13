#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <utils/input/input.hpp>
#include <thread>
#include <queue>

#include "my_vins/estimator.hpp"
#include "my_vins/my_vins.hpp"


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<my_vins::MyVins>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
