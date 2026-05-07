#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "ICM42688.h"

#define IMU_SPI_MOSI 10
#define IMU_SPI_MISO 11  // 对应上面的 MPU_MISO
#define IMU_SPI_CLK  12  // 对应上面的 MPU_SCK
#define IMU_CS_PIN   8   // 对应上面的 MPU_CS
#define IMU_INT_PIN  15  // 上面未提及，保持原样

class DrIMU {
private:
    // 关键：声明一个静态指针，指向当前实例

public:
    ICM42688 sensor;
    volatile bool dataReady = false;

    DrIMU() : sensor(SPI, IMU_CS_PIN,2400000) {
    }

    bool init() {
        

        SPI.begin(IMU_SPI_CLK, IMU_SPI_MISO, IMU_SPI_MOSI, IMU_CS_PIN);
        
        if (sensor.begin() < 0) return false;

        return true;
    }

    void IRAM_ATTR upData() {
        // 注意：在中断中调用 getAGT() 可能会有问题
        // 因为 getAGT 内部通常使用 SPI 通信，而 SPI 也是基于中断或阻塞的
        
        int x = sensor.getAGT();
        if (x < 0) {
            return;
        }
        
    }
};

