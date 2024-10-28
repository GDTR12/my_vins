#pragma once
#include <chrono>


namespace utils_common
{
    

class TimerHelper {
public:
    // 开始计时
    static std::chrono::high_resolution_clock::time_point start() {
        return std::chrono::high_resolution_clock::now();
    }
    
    // 结束计时并返回耗时(毫秒)
    static double end(const std::chrono::high_resolution_clock::time_point& start_time) {
        auto end_time = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    }
};

} // namespace utils_common
