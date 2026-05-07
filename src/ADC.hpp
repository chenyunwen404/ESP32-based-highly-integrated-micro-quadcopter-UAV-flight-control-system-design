#include <Arduino.h>

#define ADC_OFFSET 0.0f 

class BatteryMonitor {
private:
    const uint8_t _pin;           // 改为构造函数初始化，更灵活
    const float _divider_ratio;   
    float _filtered_mv = 0; 
    float _alpha;
    
    // 时间控制
    unsigned long _last_update = 0;
    const unsigned long _interval_ms = 20; // 50Hz 采样率

public:
    // 允许在构造时修改引脚，默认 GPIO 9
    BatteryMonitor(uint8_t pin = 3, float ratio = 2.0f, float alpha = 1.0f) 
        : _pin(pin), _divider_ratio(ratio), _alpha(alpha) {}

    void begin() {
        // 1. 设置分辨率
        analogReadResolution(12);

        // 2. 【改进】只设置该引脚的衰减，不影响其他传感器
        analogSetPinAttenuation(_pin, ADC_11db);
        
        // 3. 预填充滤波器，避免启动时的数值“爬升”过程
        _filtered_mv = analogReadMilliVolts(_pin);
    }

    void update() {
        // 【改进】非阻塞时间控制，保证滤波系数的物理意义恒定
        if (millis() - _last_update < _interval_ms) return;
        _last_update = millis();

        uint32_t raw_mv = analogReadMilliVolts(_pin);
        
        // 简单的 IIR 低通滤波
        _filtered_mv = (_alpha * raw_mv) + (1.0f - _alpha) * _filtered_mv;
    }

    float getVoltage() {
        float pin_v = _filtered_mv / 1000.0f;
        return (pin_v * _divider_ratio) + ADC_OFFSET;
    }
    
    // 调试用：查看原始 mV 值
    float getMilliVolts() {
        return _filtered_mv;
    }
};