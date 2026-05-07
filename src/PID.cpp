#include "PID.hpp"

#define MIN_CHECK_THROTTLE 10 // 用于地面检测的油门阈值

// 构造函数：初始化新增的 alpha 和 prev_derivative
PID::PID(float p, float i, float d, float max_out, float max_i_term, float alpha) {
    kp = p;
    ki = i;
    kd = d;
    max_output = max_out;
    max_i_output = max_i_term;
    
    // 滤波系数 (默认 0.1，越小滤波越强，延迟越大)
    lpf_alpha = alpha; 

    integral_error = 0.0f;
    prev_measurement = 0.0f;
    prev_derivative = 0.0f; // 初始化
}

void IRAM_ATTR PID::reset() {
    integral_error = 0.0f;
    prev_measurement = 0.0f;
    prev_derivative = 0.0f; // 复位时也要清零
}

void PID::setTunings(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
}

// 新增：允许你在代码里动态修改滤波系数
void PID::setDFilter(float alpha) {
    // 限制范围 0.0 ~ 1.0
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    lpf_alpha = alpha;
}

bool PID::checkAndResetPID(int currentThrottle) {
    // 如果油门低于阈值（相当于没起飞，或者刚刚降落）
    if (currentThrottle < MIN_CHECK_THROTTLE) {
        
        // 1. 强制重置所有轴的 PID
        this->reset();

        // 2. 调试输出 (可选，不用太频繁)
        // Serial.println("Ground Idling: PID Reset");

        return true; // 告诉主程序：现在是地面状态
    }
    return false; // 正在飞行
}

float IRAM_ATTR PID::compute(float setpoint, float measurement, float dt) {
    // 1. 计算误差
    float error = setpoint - measurement;

    // 2. 比例项 (P)
    float P = kp * error;

    // 3. 积分项 (I)
    if (dt > 0.00001f) {
        float tmp_i = ki * integral_error;
        if( tmp_i > max_i_output && error < 0 )
        {
            integral_error += error * dt;
        }
        else if( tmp_i > -max_i_output && error > 0 )
        {
            integral_error += error * dt;
        }
        else if( tmp_i < max_i_output && tmp_i > -max_i_output )
        {
            integral_error += error * dt;
        }
            
    }
    
    // 积分抗饱和
    float I = ki * integral_error;
    I = constrain(I, -max_i_output, max_i_output);

    // 4. 微分项 (D) + 低通滤波 (LPF)
    float derivative = 0.0f;
    
    if (dt > 0.00001f) {
        // A. 计算原始微分 (包含巨大的噪音)
        float raw_derivative = -(measurement - prev_measurement) / dt;

        // B. === 关键：应用低通滤波器 ===
        // 公式：Out = Last_Out * (1 - alpha) + New_In * alpha
        // alpha 越小，保留的历史数据越多，滤波越强，但延迟越大
        derivative = prev_derivative * (1.0f - lpf_alpha) + raw_derivative * lpf_alpha;
        
        // 更新历史数据供下一次使用
        prev_derivative = derivative;
    } else {
        // 如果 dt 无效，保持上一次的微分值，或者设为0
        derivative = prev_derivative; 
    }

    // C. 计算最终的 D 项输出
    float D = kd * derivative;

    // 保存当前误差
    prev_measurement = measurement;

    // 5. 总输出
    float output = P + I + D;

    // 总输出限幅
    output = constrain(output, -max_output, max_output);

    return output;
}