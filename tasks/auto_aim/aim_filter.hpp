#ifndef TASKS_AUTO_AIM_AIM_FILTER_HPP
#define TASKS_AUTO_AIM_AIM_FILTER_HPP

#include <deque>
#include "io/command.hpp"
#include "tasks/auto_aim/target.hpp"

namespace auto_aim {

class AimFilter {
public:
    AimFilter();
    
    // 传入原始的pitch, yaw(度), 目标速度(m/s), 和上一次的控制指令，修改并返回滤波后的控制指令
    void filter(double raw_pitch_deg, double raw_yaw_deg, double target_speed, const io::Command& last_command, io::Command& current_command, int frame_count);

private:
    double filtered_pitch_;
    double filtered_yaw_;
    static constexpr double FILTER_ALPHA = 0.3;
    static constexpr double DEADZONE_DEG = 0.05;
    
    static constexpr int MA_WINDOW_SIZE = 5;
    std::deque<double> pitch_history_;
    std::deque<double> yaw_history_;
    
    static constexpr double MIN_ALPHA = 0.2;
    static constexpr double MAX_ALPHA = 0.6;
    static constexpr double SPEED_THRESHOLD_LOW = 1.0;
    static constexpr double SPEED_THRESHOLD_HIGH = 3.0;
};

} // namespace auto_aim

#endif // TASKS_AUTO_AIM_AIM_FILTER_HPP