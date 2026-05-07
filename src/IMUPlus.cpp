#include "IMUPlus.hpp"
#include <math.h>

// 滤波系数：0.98 意味着 98% 信赖陀螺仪积分，2% 信赖加速度计纠正

IMUSolver::IMUSolver() {
    last_update_micros = 0;
}

// 增加 ax, ay, az 用于初始化
void IMUSolver::begin(float ax, float ay, float az) {
    last_update_micros = micros();
    
    // 1. 立即计算初始角度
    // 【修改点 1】Pitch 初始化去掉了 -ax，改成 ax
    // 目的是让抬头动作产生的角度为“负”，适配你的混控
    roll  = atan2(ay, az) * RAD_TO_DEG;
    pitch = atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG; // 去掉了负号
    yaw   = 0; 
    
    Serial.printf("[IMUSolver] Init Angles: R=%.1f, P=%.1f\n", roll, pitch);
}

void IMUSolver::setOffsets(float ox, float oy, float oz) {
    offset_gx = ox;
    offset_gy = oy;
    offset_gz = oz;
}

void IMUSolver::update(float ax, float ay, float az, float gx, float gy, float gz) {
    // 1. 计算 dt
    float dt = 1.0f / 4000.0f; // 转换为秒

    // 异常保护
    if (dt > 0.1f || dt <= 0.0f) {
        return; 
    }

    // 2. 去除陀螺仪零偏
    float real_gx = gx - offset_gx;
    float real_gy = gy - offset_gy;
    float real_gz = gz - offset_gz;

    // ================== 【新增功能：加速度计模长检测】 ==================
    
    // A. 计算当前加速度计的合力模长
    float acc_norm = sqrt(ax * ax + ay * ay + az * az);
    
    // B. 定义你的重力单位参考值 (请根据实际情况修改！)
    // 如果你的 ax, ay, az 是归一化的 g 值，这里填 1.0f
    // 如果是 m/s^2，这里填 9.8f
    // 如果是 LSB (例如 4096 代表 1g)，这里填 4096.0f
    const float GRAVITY_REF = 1.0f; 

    // C. 定义容忍范围 (例如 ±15%)
    // 范围越窄(0.1)抗震越强但修正越慢，范围越宽(0.2)容错越高但易受干扰
    const float ACC_TOLERANCE = 0.15f; 
    
    // D. 决定当前的信任系数
    // 默认使用全局定义的 ALPHA (例如 0.98)
    float current_alpha = ALPHA; 

    // 判断：如果模长偏差超过容忍范围 (震动 或 加速运动)
    if (abs(acc_norm - GRAVITY_REF) > (GRAVITY_REF * ACC_TOLERANCE)) {
        // 模长异常！认为加速度计不可信，完全断开修正
        // alpha = 1.0 表示 100% 信任陀螺仪，0% 信任加速度计
        current_alpha = 1.0f; 
    }
    // =================================================================

    // 3. 加速度计解算 (绝对角度)
    // 只有在加速度计可信时，计算这个才有意义，但为了代码简洁，算一下也无妨
    // 注意：atan2 并不依赖模长，所以这里算出来的角度本身没问题，只是物理意义上可能不对
    float acc_roll  = atan2(ay, az) * RAD_TO_DEG;
    float acc_pitch = atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG; // 保持你的反向逻辑

    // 4. 互补滤波核心 (使用动态调整后的 current_alpha)
    // 【修改点 3】Pitch 和 Yaw 的陀螺仪积分方向取反
    
    // Roll
    roll  = current_alpha * (roll  + real_gx * dt) + (1.0f - current_alpha) * acc_roll;
    
    // Pitch (保持你原本的逻辑：减去 real_gy)
    pitch = current_alpha * (pitch - real_gy * dt) + (1.0f - current_alpha) * acc_pitch;
    
    // Yaw (保持你原本的逻辑：减去 real_gz，且 Yaw 不受加速度计修正)
    yaw   += -real_gz * dt; 
}