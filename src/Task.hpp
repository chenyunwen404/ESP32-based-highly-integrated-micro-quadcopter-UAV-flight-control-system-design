#pragma once
#include <Arduino.h>
#include "motor.hpp"
#include "WIFI.hpp"
#include "commonuse.hpp"
#include "I2CVL.hpp"
#include "Pros.hpp"



extern DroneManager droneManager; // 声明动力管理实例






class TaskManager {
public:
    // 只写函数声明，不写大括号里的内容
    void begin();

private:
    // 静态任务函数声明
    static void WIFIControlTask(void *pvParameters);

    static void MotorControlTask(void *pvParameters);
};

