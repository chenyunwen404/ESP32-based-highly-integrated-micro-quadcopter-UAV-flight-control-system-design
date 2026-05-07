#pragma once

#include <Arduino.h>
#include <math.h>
#include "IMU.hpp"

class IMUSolver {
private:
    // 互补滤波系数
    // 0.98 代表 98% 信赖陀螺仪积分，2% 信赖加速度计纠正
    // 这个值决定了纠正漂移的快慢：越小纠正越快但也越容易受震动影响
    const float ALPHA = 0.97f; 

    // 上一次更新的时间 (用于计算 dt)
    unsigned long last_update_micros;

    // 陀螺仪零偏 (校准值)
    float offset_gx = 0;
    float offset_gy = 0;
    float offset_gz = 0;

public:
    // --- 输出的姿态角 (单位: 度) ---
    DrIMU imu;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    // 构造函数
    IMUSolver();

    // [关键修改] 初始化函数
    // 现在需要传入初始的加速度值 (ax, ay, az)，用于瞬间计算出当前的初始角度
    // 防止上电时因为不是绝对水平而导致的缓慢收敛或侧翻
    void begin(float ax, float ay, float az);

    // 陀螺仪校准 (建议在起飞前静止时调用)
    void calibrateGyro(int count = 500);

    // 核心解算函数
    // ax, ay, az: 加速度 (单位 g)
    // gx, gy, gz: 角速度 (单位 deg/s)
    void update(float ax, float ay, float az, float gx, float gy, float gz);
    
    // 手动设置零偏
    void setOffsets(float ox, float oy, float oz);
};
