#include "tasks/auto_aim/aim_filter.hpp"
#include "tools/logger.hpp"
#include <cmath>

namespace auto_aim {

AimFilter::AimFilter() : filtered_pitch_(0.0), filtered_yaw_(0.0) {}

void AimFilter::filter(double raw_pitch_deg, double raw_yaw_deg, double target_speed, const io::Command& last_command, io::Command& current_command, int frame_count) {
    if (frame_count == 1) {
        filtered_pitch_ = raw_pitch_deg;
        filtered_yaw_ = raw_yaw_deg;
        pitch_history_.clear();
        yaw_history_.clear();
    }
    
    // === 第一层滤波：自适应低通滤波 ===
    // 根据目标速度动态调整滤波系数
    double adaptive_alpha = FILTER_ALPHA;
    if (target_speed < SPEED_THRESHOLD_LOW) {
        adaptive_alpha = MIN_ALPHA;  // 低速：更平滑
    } else if (target_speed > SPEED_THRESHOLD_HIGH) {
        adaptive_alpha = MAX_ALPHA;  // 高速：更快响应
    } else {
        // 中速：线性插值
        double ratio = (target_speed - SPEED_THRESHOLD_LOW) / (SPEED_THRESHOLD_HIGH - SPEED_THRESHOLD_LOW);
        adaptive_alpha = MIN_ALPHA + ratio * (MAX_ALPHA - MIN_ALPHA);
    }
    
    // 应用自适应低通滤波
    filtered_pitch_ = adaptive_alpha * raw_pitch_deg + (1.0 - adaptive_alpha) * filtered_pitch_;
    filtered_yaw_ = adaptive_alpha * raw_yaw_deg + (1.0 - adaptive_alpha) * filtered_yaw_;
    
    // === 第二层滤波：移动平均滤波 ===
    // 添加当前值到历史队列
    pitch_history_.push_back(filtered_pitch_);
    yaw_history_.push_back(filtered_yaw_);
    
    // 保持队列长度不超过窗口大小
    if (pitch_history_.size() > MA_WINDOW_SIZE) {
        pitch_history_.pop_front();
    }
    if (yaw_history_.size() > MA_WINDOW_SIZE) {
        yaw_history_.pop_front();
    }
    
    // 计算加权移动平均（近期权重更大）
    double ma_pitch = 0.0, ma_yaw = 0.0;
    double weight_sum = 0.0;
    for (size_t i = 0; i < pitch_history_.size(); ++i) {
        double weight = i + 1;  // 线性权重：1, 2, 3, 4, 5
        ma_pitch += pitch_history_[i] * weight;
        ma_yaw += yaw_history_[i] * weight;
        weight_sum += weight;
    }
    if (weight_sum > 0) {
        ma_pitch /= weight_sum;
        ma_yaw /= weight_sum;
    }
    
    // === 第三层：死区处理 ===
    // 如果变化量小于阈值，保持上一次的值
    double delta_pitch = std::abs(ma_pitch - (last_command.pitch * 57.3));
    double delta_yaw = std::abs(ma_yaw - (last_command.yaw * 57.3));
    
    if (delta_pitch < DEADZONE_DEG && delta_yaw < DEADZONE_DEG && last_command.control) {
        // 在死区内，使用上一次的指令
        current_command.pitch = last_command.pitch;
        current_command.yaw = last_command.yaw;
    } else {
        // 超出死区，使用滤波后的新指令
        current_command.pitch = ma_pitch / 57.3;
        current_command.yaw = ma_yaw / 57.3;
    }
    
    // 调试输出滤波信息
    if (frame_count % 30 == 0) {  // 每30帧输出一次
        tools::logger()->debug(
            "[滤波调试] 速度={:.2f}m/s, alpha={:.2f}, 原始P={:.2f}° Y={:.2f}°, "
            "低通P={:.2f}° Y={:.2f}°, 移动平均P={:.2f}° Y={:.2f}°, 最终P={:.2f}° Y={:.2f}°",
            target_speed, adaptive_alpha,
            raw_pitch_deg, raw_yaw_deg,
            filtered_pitch_, filtered_yaw_,
            ma_pitch, ma_yaw,
            current_command.pitch * 57.3, current_command.yaw * 57.3
        );
    }
}

} // namespace auto_aim