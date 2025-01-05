#pragma once
#include <chrono>
#include <Eigen/Core>

#define EFMT(...) common_utils::eigenFmt(__VA_ARGS__).c_str()

namespace common_utils
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
        return double(std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count()) / 1000000.0f;
    }
};

enum class PrintFormat {
    ONELINE,    // 默认格式 [1,2,3; 4,5,6]
    CLEAN,      // 清晰格式 [1, 2, 3]\n[4, 5, 6]
    COMMA,      // 逗号分隔 1, 2, 3, 4, 5, 6
    SPACE,      // 空格分隔 1 2 3\n4 5 6
    CSV         // CSV格式
};
template<typename Derived>
std::string eigenFmt(const Eigen::MatrixBase<Derived>& mat,
                         PrintFormat format = PrintFormat::SPACE,
                         int precision = 4)
{
    Eigen::IOFormat fmt(precision, 0); // 默认格式
    
    switch(format) {
        case PrintFormat::CLEAN:
            fmt = Eigen::IOFormat(precision, 0, ", ", "\n", "[", "]");
            break;
            
        case PrintFormat::COMMA:
            fmt = Eigen::IOFormat(precision, 0, ", ", ", ", "", "");
            break;
            
        case PrintFormat::SPACE:
            fmt = Eigen::IOFormat(precision, 0, " ", "\n", "", "");
            break;
            
        case PrintFormat::CSV:
            fmt = Eigen::IOFormat(precision, 0, ", ", "\n", "", "");
            break;
            
        case PrintFormat::ONELINE:
        default:
            fmt = Eigen::IOFormat(precision, 0, ", ", "; ", "[", "]");
            break;
    }
    
    std::stringstream ss;
    ss << mat.format(fmt);
    return ss.str();
}

} // namespace common_utils
