#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// =================================================================================
//  工具类：PT1 一阶低通滤波器 (高效实现)
// =================================================================================
class PT1Filter {
public:
    PT1Filter() : _state(0), _rc(0), _dt(0), _initialized(false) {}

    // 设置截止频率 (Hz)
    void setFrequency(float hz) {
        if (hz > 0.0f) {
            _rc = 1.0f / (2.0f * PI * hz);
        } else {
            _rc = 0.0f; // 0 表示不过滤
        }
    }

    // 应用滤波
    float apply(float input, float dt) {
        if (!_initialized) {
            _state = input;
            _initialized = true;
            return input;
        }
        if (dt <= 0.0f || _rc <= 0.0f) return input;

        // alpha = dt / (RC + dt)
        float alpha = dt / (_rc + dt);
        _state = _state + alpha * (input - _state);
        return _state;
    }

    // 重置状态
    void reset() { _initialized = false; }

private:
    float _state;
    float _rc;
    float _dt;
    bool _initialized;
};

// =================================================================================
//  MPU6000 驱动类 (软件滤波版)
// =================================================================================
class MPU6000_S3 {
public:
    // --- 公共数据 (已滤波、已修正) ---
    float ax = 0, ay = 0, az = 0; 
    float gx = 0, gy = 0, gz = 0; 
    float temp = 0;

    // --- 构造函数 ---
    explicit MPU6000_S3(uint8_t cs, SPIClass* spi) : _cs(cs), _spi(spi) {}

    // --- 初始化 ---
    bool begin() {
        if (!_spi) return false;
        pinMode(_cs, OUTPUT);
        digitalWrite(_cs, HIGH);

        // 1. 复位与时钟
        writeReg(PWR_MGMT_1, 0x80); delay(100);
        writeReg(PWR_MGMT_1, 0x01); delay(10);
        writeReg(USER_CTRL, 0x10);  // 禁用 I2C

        // 2. ID 检查
        if (readReg(WHO_AM_I) != 0x68) return false;

        // 3. 【核心修改】配置滤波与采样
        // CONFIG = 0x00: DLPF_CFG = 0 (带宽 256Hz, 延迟 0.98ms, 内部采样 8kHz)
        // 相比原来的 0x03 (延迟 4.8ms)，这里极大降低了物理延迟
        writeReg(CONFIG, 0x00);
        
        // SMPLRT_DIV = 0: 输出分频为 0，保持 8kHz 输出给 ESP32
        writeReg(SMPLRT_DIV, 0x00);

        // 4. 量程设置
        setGyroRange(GyroRange::DPS2000);
        setAccelRange(AccelRange::G8);

        // 5. 初始化软件滤波器参数 (针对有刷电机优化)
        // 陀螺仪: 60Hz (既能响应快，又能滤掉有刷火花干扰)
        // 加速度: 10Hz (加速度不需要快，越稳越好)
        setFilterFreq(42.0f, 10.0f);

        // 初始化时间戳
        _last_update_us = micros();
        return true;
    }

    // --- 设置软件滤波截止频率 ---
    void setFilterFreq(float gyro_hz, float accel_hz) {
        _filt_gx.setFrequency(gyro_hz);
        _filt_gy.setFrequency(gyro_hz);
        _filt_gz.setFrequency(gyro_hz);
        
        _filt_ax.setFrequency(accel_hz);
        _filt_ay.setFrequency(accel_hz);
        _filt_az.setFrequency(accel_hz);
    }

    // --- 中断设置 ---
    void enableDataReadyISR() {
        writeReg(INT_PIN_CFG, 0x10); // 读清除
        writeReg(INT_ENABLE, 0x01);  // 数据就绪中断
    }

    // --- 核心更新函数 (需 8kHz 频率调用) ---
    void update() {
        // 1. 计算时间间隔 dt (秒)
        unsigned long now_us = micros();
        float dt = (now_us - _last_update_us) * 1e-6f;
        _last_update_us = now_us;
        
        // 容错：防止 dt 过大或过小破坏滤波
        if (dt <= 0.0f) dt = 0.000125f; // 默认 8kHz
        if (dt > 0.02f) dt = 0.02f;     // 限制最大 dt

        // 2. SPI 突发读取
        uint8_t b[15];
        b[0] = ACCEL_XOUT_H | 0x80;
        transfer(b, 15, 20000000); // 20MHz

        auto s16 = [&](int i) -> int16_t {
            return (int16_t)(((uint16_t)b[i] << 8) | b[i + 1]);
        };

        // 3. 解析原始数据 (Raw Data)
        // 注意：这里是有噪点的数据
        float raw_ax = s16(1) / _aScale;
        float raw_ay = s16(3) / _aScale;
        float raw_az = s16(5) / _aScale;

        temp = (s16(7) / 340.0f) + 36.53f;

        float raw_gx = s16(9) / _gScale;
        float raw_gy = s16(11) / _gScale;
        float raw_gz = s16(13) / _gScale;

        // 4. 坐标系旋转 (Sensor -> Drone)
        // 规则: X -> -X, Y -> -Y, Z -> Z
        float r_ax = raw_ay;
        float r_ay = -raw_ax;
        float r_az =  raw_az;

        float r_gx = raw_gy;
        float r_gy = -raw_gx;
        float r_gz =  raw_gz;

        // 5. 【关键】应用软件滤波
        // 这里的 _state 会保存平滑后的值
        float f_ax = _filt_ax.apply(r_ax, dt);
        float f_ay = _filt_ay.apply(r_ay, dt);
        float f_az = _filt_az.apply(r_az, dt);

        float f_gx = _filt_gx.apply(r_gx, dt);
        float f_gy = _filt_gy.apply(r_gy, dt);
        float f_gz = _filt_gz.apply(r_gz, dt);

        // 6. 减去校准零偏 (Offset)
        // 使用滤波后的数据减 Offset，效果更稳
        ax = f_ax - _offset[0];
        ay = f_ay - _offset[1];
        az = f_az - _offset[2];

        gx = f_gx - _offset[3];
        gy = f_gy - _offset[4];
        gz = f_gz - _offset[5];
    }

    // --- 校准功能 (基本保持不变，但会自动享受到软件滤波的好处) ---
    void calibrateGyro(uint16_t n = 2000) {
        float sum[3] = {0};
        _offset[3] = _offset[4] = _offset[5] = 0;
        
        // 重置滤波器状态，防止之前的残余值影响
        _filt_gx.reset(); _filt_gy.reset(); _filt_gz.reset();

        for (uint16_t i = 0; i < n; i++) {
            update();
            sum[0] += gx; sum[1] += gy; sum[2] += gz;
            delay(2); // 这里可以用 delay，因为校准时不关心实时性，只关心均值
        }
        _offset[3] = sum[0] / n;
        _offset[4] = sum[1] / n;
        _offset[5] = sum[2] / n;
    }

    void calibrateAccel(uint16_t n = 2000) {
        float sum[3] = {0};
        _offset[0] = _offset[1] = _offset[2] = 0;
        
        _filt_ax.reset(); _filt_ay.reset(); _filt_az.reset();

        for (uint16_t i = 0; i < n; i++) {
            update();
            sum[0] += ax; sum[1] += ay; sum[2] += az;
            delay(2);
        }
        _offset[0] = sum[0] / n;
        _offset[1] = sum[1] / n;
        _offset[2] = (sum[2] / n) - 1.0f; // 去重力

        // 保存 NVS
        Preferences prefs;
        prefs.begin("mpu6k", false);
        prefs.putUInt("ver", 1);
        prefs.putBool("valid", true);
        prefs.putFloat("ax", _offset[0]);
        prefs.putFloat("ay", _offset[1]);
        prefs.putFloat("az", _offset[2]);
        prefs.end();
    }

    bool loadAccelFromFlash() {
        Preferences prefs;
        prefs.begin("mpu6k", true);
        if (!prefs.getBool("valid", false) || prefs.getUInt("ver", 0) != 1) {
            prefs.end(); return false;
        }
        _offset[0] = prefs.getFloat("ax", 0);
        _offset[1] = prefs.getFloat("ay", 0);
        _offset[2] = prefs.getFloat("az", 0);
        prefs.end();
        return true;
    }

    // --- 量程枚举与设置 ---
    enum class GyroRange : uint8_t { DPS250 = 0, DPS500 = 8, DPS1000 = 16, DPS2000 = 24 };
    enum class AccelRange : uint8_t { G2 = 0, G4 = 8, G8 = 16, G16 = 24 };

    void setGyroRange(GyroRange r) {
        writeReg(GYRO_CONFIG, (uint8_t)r);
        static const float scales[] = {131.0f, 65.5f, 32.8f, 16.4f};
        _gScale = scales[((uint8_t)r >> 3) & 3];
    }
    void setAccelRange(AccelRange r) {
        writeReg(ACCEL_CONFIG, (uint8_t)r);
        static const float scales[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
        _aScale = scales[((uint8_t)r >> 3) & 3];
    }

private:
    // 寄存器地址
    enum {
        SMPLRT_DIV = 0x19, CONFIG = 0x1A, GYRO_CONFIG = 0x1B, ACCEL_CONFIG = 0x1C,
        INT_PIN_CFG = 0x37, INT_ENABLE = 0x38,
        ACCEL_XOUT_H = 0x3B, USER_CTRL = 0x6A, PWR_MGMT_1 = 0x6B, WHO_AM_I = 0x75
    };

    uint8_t _cs;
    SPIClass* _spi;
    float _aScale = 4096.0f, _gScale = 16.4f;
    float _offset[6] = {0};
    unsigned long _last_update_us = 0;

    // 滤波器实例
    PT1Filter _filt_ax, _filt_ay, _filt_az;
    PT1Filter _filt_gx, _filt_gy, _filt_gz;

    void transfer(uint8_t* buf, size_t len, uint32_t freq) {
        if (!_spi) return;
        _spi->beginTransaction(SPISettings(freq, MSBFIRST, SPI_MODE3));
        digitalWrite(_cs, LOW);
        _spi->transfer(buf, len);
        digitalWrite(_cs, HIGH);
        _spi->endTransaction();
    }

    void writeReg(uint8_t reg, uint8_t data) {
        uint8_t b[2] = {reg, data};
        transfer(b, 2, 1000000);
    }

    uint8_t readReg(uint8_t reg) {
        uint8_t b[2] = {(uint8_t)(reg | 0x80), 0};
        transfer(b, 2, 1000000);
        return b[1];
    }
};

// =================================================================================
//  TiltCompensator 类 (保持原样，未修改，为了方便你复制保留在此)
// =================================================================================
class TiltCompensator {
public:
    struct Config {
        int max_throttle = 4095;       
        float max_scale = 2.0f;        
        float slew_step = 0.02f;       
        float min_thr_norm = 0.05f;    
        uint8_t update_div = 5;        
    };

    TiltCompensator() { initHelper(); }
    TiltCompensator(Config cfg) : _cfg(cfg) { initHelper(); }

    int update(int base_throttle, float roll_rad, float pitch_rad) {
        float thr_norm = (float)base_throttle / (float)_cfg.max_throttle;
        if (++_div_counter >= _cfg.update_div) {
            _div_counter = 0;
            float cos_tilt = cosf(roll_rad) * cosf(pitch_rad);
            if (!isfinite(cos_tilt)) cos_tilt = 1.0f;
            if (cos_tilt < 0.0f) cos_tilt = 0.0f; 
            if (cos_tilt < _min_cos_tilt) cos_tilt = _min_cos_tilt;
            float s = 1.0f / cos_tilt;
            if (thr_norm < _cfg.min_thr_norm) s = 1.0f;
            _scale_target = constrain_f(s, 1.0f, _cfg.max_scale);
        }
        _current_scale = slew(_current_scale, _scale_target, _step_per_tick);
        float out_f = (float)base_throttle * _current_scale;
        int out = (int)(out_f + 0.5f);
        if (out > _cfg.max_throttle) out = _cfg.max_throttle;
        if (out < 0) out = 0;
        return out;
    }
    void reset() {
        _current_scale = 1.0f; _scale_target = 1.0f; _div_counter = 0;
    }

private:
    Config _cfg;
    float _min_cos_tilt;
    float _step_per_tick;
    uint8_t _div_counter = 0;
    float _scale_target = 1.0f;
    float _current_scale = 1.0f;

    void initHelper() {
        _min_cos_tilt = 1.0f / _cfg.max_scale;
        _step_per_tick = _cfg.slew_step / (float)_cfg.update_div;
    }
    inline float constrain_f(float x, float min, float max) {
        if (x < min) return min; if (x > max) return max; return x;
    }
    inline float slew(float curr, float target, float step) {
        float diff = target - curr;
        if (diff > step) return curr + step;
        if (diff < -step) return curr - step;
        return target;
    }
};