#pragma once

#include <Arduino.h> // 引入Arduino核心库以使用 constrain, abs 等函数

class PID {
public:
    // 构造函数：新增了 alpha 参数 (默认 0.1)
    // alpha: D项低通滤波系数 (0.0 - 1.0)，越小滤波越强
    PID(float p, float i, float d, float max_out, float max_i_term, float alpha = 0.1f);

    // 核心计算函数
    // setpoint: 期望值 (比如 0 deg/s)
    // measurement: 传感器实测值 (比如 Gyro_Z)
    // dt: 循环时间间隔 (单位：秒)
    float compute(float setpoint, float measurement, float dt);

    // 重置 PID 状态 (在解锁前调用，防止积分累积)
    void reset();

    // 动态调整 PID 参数 (用于蓝牙调参)
    void setTunings(float p, float i, float d);

    // [新增] 动态调整滤波系数
    // 调试时很有用：如果你发现电机发烫，可以调用这个把 alpha 调小
    void setDFilter(float alpha);

    // 检查油门并在地面时重置 PID
    // 返回值：如果当前处于地面状态则返回 true
    bool checkAndResetPID(int currentThrottle);

private:
    float kp;
    float ki;
    float kd;

    float max_output;   // 总输出限制
    float max_i_output; // 积分项单独限制

    float integral_error; // 积分累积值
    float prev_measurement;     // 上一次测量值

    // === [新增] D项滤波器专用变量 ===
    float prev_derivative; // 上一次滤波后的微分值 (用于迭代计算)
    float lpf_alpha;       // 当前使用的滤波系数
};
