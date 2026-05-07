#pragma once

#include <Arduino.h>
#include <math.h>

#include "soc/ledc_struct.h"
#include "soc/ledc_reg.h"

#define MOTOR_FREQ    19500
#define MOTOR_RES     12       // 1. 修改分辨率为 12 位
#define MAX_THRUST    4095     // 2. 最大值变为 2^12 - 1
#define MIN_THRUST    0

class BrushlessMotor {
private:
    uint8_t  _pin;
    uint8_t  _channel;
    uint32_t _freq;
    uint8_t  _resolution;
    int      _maxDuty;
    bool     _isArmed = false;
    int _currentDuty = 0;

public:
    BrushlessMotor(uint8_t pin, uint8_t ch, uint32_t freq, uint8_t res) 
        : _pin(pin), _channel(ch), _freq(freq), _resolution(res) {
        _maxDuty = (1 << _resolution) - 1; // 自动计算为 4095
    }

    void begin() {
        ledcSetup(_channel, _freq, _resolution);
        ledcAttachPin(_pin, _channel);
        ledcWrite(_channel, 0); 
    }

    void arm() { _isArmed = true; }
    void disarm() { _isArmed = false; ledcWrite(_channel, 0); }

    void setPhase(float degrees) {
        degrees = fmodf(degrees, 360.0f);
        if (degrees < 0.0f) degrees += 360.0f;
        
        uint32_t max_count = (1 << _resolution); // 这里会自动变成 4096
        uint32_t hpoint = (uint32_t)((degrees / 360.0f) * (float)max_count);
        if (hpoint >= max_count) hpoint = max_count - 1;

        volatile ledc_dev_t *ledc_dev = &LEDC;
        // 注意：ESP32 的 LEDC 硬件通道组索引可能需要根据芯片型号调整（ESP32-S3 等不同）
        ledc_dev->channel_group[0].channel[_channel].hpoint.val = hpoint;
        ledc_dev->channel_group[0].channel[_channel].conf0.val |= (1U << 4);
    }

    void update(int input) {
        // 3. 修改 constrain 范围
        int duty = constrain(input, 0, _maxDuty);
        ledcWrite(_channel, duty);
    }
};

class DroneManager {
public:
    BrushlessMotor M1, M2, M3, M4;

    DroneManager() : 
        M1(6, 0, MOTOR_FREQ, MOTOR_RES),
        M2(5, 1, MOTOR_FREQ, MOTOR_RES),
        M3(4, 2, MOTOR_FREQ, MOTOR_RES),
        M4(7, 3, MOTOR_FREQ, MOTOR_RES)
    {}

    void begin() {
        M1.begin(); M2.begin(); M3.begin(); M4.begin();
        M1.setPhase(0.0f);
        M2.setPhase(90.0f);
        M3.setPhase(270.0f);
        M4.setPhase(180.0f);
        Serial.println(">>> PWM Phase Interleaving 90° Active (12-bit Mode)");
    }

    void armAll()    { M1.arm(); M2.arm(); M3.arm(); M4.arm(); }
    void disarmAll() { M1.disarm(); M2.disarm(); M3.disarm(); M4.disarm(); }

    void mixTable(int throttle, int roll, int pitch, int yaw, float cmop) {
        float t1 = throttle - roll + pitch - yaw;
        float t2 = throttle - roll - pitch + yaw;
        float t3 = throttle + roll + pitch + yaw;
        float t4 = throttle + roll - pitch - yaw;

        // 4. 修改混控上限检查为 4095
        float max_t = max({t1, t2, t3, t4});
        if (max_t > 4095.0f) {
            float scale = 4095.0f / max_t;
            t1 *= scale; t2 *= scale; t3 *= scale; t4 *= scale;
        }

        auto finalize = [&](float t) {
            if (t <= 10.0f) return 0;
            float scientific_cmop = powf(cmop, 1.28f); 
            // 5. 这里的数学映射需要同步：sqrt(MAX_DUTY * t)
            // 如果 t 本身也是 0-4095 范围，这里的开方补偿逻辑应保持线性一致性
            float pwm = sqrt(4095.0f * t) * scientific_cmop; 
            return (int)constrain(pwm, 0, 4095);
        };

        M1.update(finalize(t1));
        M2.update(finalize(t2));
        M3.update(finalize(t3));
        M4.update(finalize(t4));
    }
};