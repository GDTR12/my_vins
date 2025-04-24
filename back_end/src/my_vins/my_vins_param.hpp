#pragma once
#include "utils/input/input.hpp"

namespace my_vins{


class MyVinsParamServer: public slam_utils::ROSParamInput{
public:
    const std::string namesp = "MyVinsParam";
    int WINDOW_SIZE = 20;
    float PARALLAX_THREASHOLD = 10.1;
    int NOKEYFRAME_THREASHOLD = 10;
    int NUM_ITERATIONS = 5;

    static MyVinsParamServer& getInstance()
    {
        static MyVinsParamServer instance;
        return instance;
    }
    MyVinsParamServer(const MyVinsParamServer&) = delete;
    MyVinsParamServer& operator=(const MyVinsParamServer&) = delete;

private:
    MyVinsParamServer(){
        this->declare_parameter(namesp + "/" + "max_cnt", WINDOW_SIZE);
        this->declare_parameter(namesp + "/" + "pallax_threashold", PARALLAX_THREASHOLD);
        this->declare_parameter(namesp + "/" + "nokeyframe_threashold", NOKEYFRAME_THREASHOLD);
    }
};

}