//BMP581.hpp
/*
 * BMP581 Drone Driver (Adafruit_BMP5xx Wrapper)
 * 功能：处理高度归零、升降速率计算（带低通滤波）
 */

#ifndef BMP581_DRONE_H
#define BMP581_DRONE_H

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_BMP5xx.h"

#define BMP581_ADDR_PRIM    0x47
#define BMP581_ADDR_SEC     0x46

class BMP581_Drone {
public:
    BMP581_Drone(uint8_t addr = BMP581_ADDR_SEC) 
        : _addr(addr), _init(false), 
          _altitude(0.0f), _velocity(0.0f), _lastMicros(0), 
          _groundAltitude(0.0f) {} 

    bool begin(TwoWire &w = Wire) {
        w.setClock(400000); 

        if (!_bmp.begin(_addr, &w)) return false;

        // 配置传感器参数
        _bmp.setTemperatureOversampling((bmp5xx_oversampling_t)BMP5_OVERSAMPLING_1X);
        _bmp.setPressureOversampling((bmp5xx_oversampling_t)BMP5_OVERSAMPLING_8X);
        _bmp.setIIRFilterCoeff((bmp5xx_iir_filter_t)BMP5_IIR_FILTER_COEFF_3);
        _bmp.setOutputDataRate((bmp5xx_odr_t)BMP5_ODR_50_HZ);

        _init = true;
        
        delay(100); 
        tare(); // 获取地面基准

        _lastMicros = micros();
        return true;
    }

    /**
     * @brief 频率建议：50Hz+
     */
    bool update() {
        if (!_init || !_bmp.performReading()) return false;

        // 计算相对高度 
        float currentAbsAlt = _bmp.readAltitude(1013.25f);
        float newAltitude = currentAbsAlt - _groundAltitude;

        uint32_t now = micros();
        float dt = (now - _lastMicros) / 1000000.0f;

        // 仅在合理的时间步长内计算速度
        if (dt > 0.0f && dt < 0.2f) {
            float rawVelocity = (newAltitude - _altitude) / dt;
            
            // 垂直速度低通滤波 (Alpha = 0.15)
            _velocity = (_velocity * 0.85f) + (rawVelocity * 0.15f);
            
            _altitude = newAltitude;
            _lastMicros = now;
            return true;
        } 
        else if (dt >= 0.2f) {
             _lastMicros = now; // 间隔过长则重置，避免速度跳变
        }

        return false;
    }

    // 采样10组数据计算地面静态高度基准
    void tare() {
        if (!_init) return;

        float sumAlt = 0;
        int count = 0;
        for(int i = 0; i < 10; i++) {
            if (_bmp.performReading()) {
                sumAlt += _bmp.readAltitude(1013.25f);
                count++;
                delay(20); // 对应 50Hz ODR
            }
        }
        
        if (count > 0) {
            _groundAltitude = sumAlt / count;
            _altitude = 0.0f;
            _velocity = 0.0f;
        }
    }

    float getAltitude() const { return _altitude; } // (m)
    float getVelocity() const { return _velocity; } // (m/s)

private:
    uint8_t _addr;
    bool _init;
    Adafruit_BMP5xx _bmp; 
    
    float _altitude;       // 相对高度 (m)
    float _velocity;       // 垂直速度 (m/s)
    uint32_t _lastMicros;
    float _groundAltitude; // 地面起始海拔
};

#endif