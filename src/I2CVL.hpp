#pragma once

#include <Wire.h>
#include <VL53L0X.h>
#include "usefun.hpp"

#define CHANGE_LIMIT 500 // 最大允许的高度突变 (毫米)    

extern TwoWire I2C_Bus;

void beginI2C();
void beginVL53L0X();
int VL53L0X_readDistance();

class AltitudeSystem {
public:
    // 假设传感器更新频率为 50Hz (即 dt = 0.02s)
    AltitudeSystem() 
        : dt_(0.02),
          // 高度滤波器：截止频率 2.0Hz (较平滑，保留动态)
          height_filter_(2.0, 0.02), 
          // 速度滤波器：截止频率 1.0Hz (速度由差分得来，噪声极大，需要更强的滤波)
          velocity_filter_(1.0, 0.02),
          last_raw_height_(0.0),
          is_first_loop_(true)
    {
    }

    void IRAM_ATTR update(float roll_rad, float pitch_rad) {
        // 1. 获取原始数据
        float raw_height = VL53L0X_readDistance();

        float tilt_factor = std::cos(roll_rad*DEG_TO_RAD) * std::cos(pitch_rad*DEG_TO_RAD);

        // 2. 安全保护
        // 如果倾斜超过 60度 (cos(60)=0.5)，传感器可能根本没打到地面，或者数据极不准。
        // 这时候强行补偿会导致数据跳变，不如直接沿用原始值或上次的值。
        if (tilt_factor > 0.5f) {
            raw_height *= tilt_factor;
        }

        // 2. 异常值剔除 (可选但推荐)
        // 如果数据跳变过大（例如 0.02秒内突变 5米），视为传感器故障或超声波反射错误
        if (!is_first_loop_ && std::abs(raw_height - last_raw_height_) > CHANGE_LIMIT) {
            raw_height = last_raw_height_; // 保持上一次的值，或者使用预测值
        }

        // 3. 高度滤波
        float filtered_height = height_filter_.update(raw_height);

        // 4. 计算垂直速度 (微分)
        // velocity = (当前高度 - 上次高度) / dt
        // 注意：这里既可以使用 filtered_height 也可以使用 raw_height 进行差分
        // 推荐：使用 raw_height 差分后再滤波，延迟更低
        float raw_velocity = 0.0;
        if (!is_first_loop_) {
            raw_velocity = -(filtered_height - prev_filtered_height_) / dt_;
        }
        
        // 5. 速度滤波
        float filtered_velocity = velocity_filter_.update(raw_velocity);

        // 更新状态
        last_raw_height_ = raw_height;
        prev_filtered_height_ = filtered_height;
        is_first_loop_ = false;

        // 输出结果给控制环路 PID 使用
        current_height_ = filtered_height;
        current_velocity_ = filtered_velocity;
    }

    float getHeight() const { return current_height_; }
    float getVelocity() const { return current_velocity_; }

private:
    float dt_;
    FirstOrderFilter<float> height_filter_;
    FirstOrderFilter<float> velocity_filter_;

    float last_raw_height_;
    float prev_filtered_height_;
    float current_height_;
    float current_velocity_;
    bool is_first_loop_;
};