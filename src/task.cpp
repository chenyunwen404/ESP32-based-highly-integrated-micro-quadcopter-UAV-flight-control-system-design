#include "Task.hpp"

#define TEST_HIGH 600.0f // 目标高度 60厘米

// 使用 volatile 确保变量在多核间可见
static volatile int globalThrottle = 0;
static volatile bool globalArmed = false;

unsigned long DRAM_ATTR nextLoopTime = 0;
unsigned long DRAM_ATTR nextLoopTime2 = 0;
unsigned long DRAM_ATTR nextLoopTime3 = 0;


// 共享数据缓存（示例）
Control_Frame safeCmd; // Core 1 使用的安全副本

void TaskManager::WIFIControlTask(void *pvParameters) {
    Serial.printf("WiFi任务启动 Core: %d\n", xPortGetCoreID());

    // 建议：WiFi 业务逻辑不需要太高的实时性，优先级设为 5-10 即可
    // 让出高优先级给 ESP32 内部的 WiFi 协议栈
    
    for (;;) {
        // 1. 接收处理 WiFi 数据
        WIFILoop(IMU, udp, targetIP);

        // // 2. [关键改进] 写入共享数据时进入临界区
        // // 只有当接收到新命令时才锁住总线，极大减小开销
        // if (udp.available()) { // 伪代码，假设收到新包
        //     portENTER_CRITICAL(&sharedDataMux);
        //     // 将 udp 收到的数据写入全局 cmd 变量
        //     // global_cmd = received_cmd; 
        //     portEXIT_CRITICAL(&sharedDataMux);
        // }

        // 适当的延时，防止 WiFi 任务饿死 IDLE 任务
        vTaskDelay(pdMS_TO_TICKS(5)); 
    }
}

// 必须确保 loop 内调用的所有子函数都在 IRAM 中
// void IRAM_ATTR sensorUpdata(); 
// void IRAM_ATTR controlProcess(); 

void IRAM_ATTR TaskManager::MotorControlTask(void *pvParameters) {
    
    // 注册看门狗，防止任务卡死导致系统重启 (可选，视配置而定)
    // esp_task_wdt_add(NULL); 

    unsigned long loopStartTime;

    for (;;) {
        loopStartTime = micros();

        // ----------------------------------------------------
        // 1. 极其精准的循环控制 (死等待模式)
        // ----------------------------------------------------
        // 注意：这种写法占用 Core 1 100% CPU，这是飞控的标准做法。
        // 确保 LOOP_PERIOD_US 足够长，能跑完里面的逻辑，否则会发生"时序滑移"
        while ((micros() - nextLoopTime) > LOOP_PERIOD_US) {
             // 如果代码跑得太慢，甚至赶不上一个周期，就不要死等了，直接立即执行下一次
             nextLoopTime = micros(); 
        }
        while (micros() < nextLoopTime) {
            // 空转等待，什么都不做，只为精度
            // 可以在这里插入 NOP 指令减少总线竞争
            __asm__ __volatile__("nop");
        }
        nextLoopTime += LOOP_PERIOD_US; // 设定下个周期的目标时间

        // ----------------------------------------------------
        // 2. 读取共享数据 (核心改进)
        // ----------------------------------------------------
        // // 进入临界区，快速拷贝一份数据出来，马上退出
        // portENTER_CRITICAL(&sharedDataMux);
        // // safeCmd = global_cmd; // 从全局变量拷贝到局部/安全变量
        // portEXIT_CRITICAL(&sharedDataMux);

        // 后续计算全部使用 safeCmd，避免计算过程中数据突变

        // ----------------------------------------------------
        // 3. 传感器与解算
        // ----------------------------------------------------
        sensorUpdata(); // 必须是 IRAM_ATTR

        // ----------------------------------------------------
        // 4. 低频任务分片 (50Hz)
        // ----------------------------------------------------
        if(micros() > nextLoopTime3) {
            nextLoopTime3 += 20000; // 20ms = 50Hz

            bmp581.update();
            
            // // 高度融合算法 (注意：这部分代码耗时如果波动大，会影响主循环)
            // altitudeSystem.update(IMU_C.roll, IMU_C.pitch);
            
            // 可以在这里喂狗，表明系统还活着
            // esp_task_wdt_reset();
        }

        // ----------------------------------------------------
        // 5. PID 与 电机输出
        // ----------------------------------------------------
        controlProcess(); // 使用 safeCmd 进行计算

        // ----------------------------------------------------
        // 6. 性能监测 (调试用)
        // ----------------------------------------------------
        // 检查这一圈用了多久，如果超过 LOOP_PERIOD_US，说明算力不够了
        // unsigned long executionTime = micros() - loopStartTime;
        // if (executionTime > LOOP_PERIOD_US) {
             // 警告：主循环超时！
        // }
    }
}

void TaskManager::begin() {
    // 创建互斥锁（其实是自旋锁，因为是在多核间）
    // 已在头部定义 sharedDataMux

    xTaskCreatePinnedToCore(
        WIFIControlTask,    
        "WIFIControlTask",  
        4096,              // WiFi 任务不需要太大栈，除非用了很重的库。4096通常够用
        NULL,              
        10,                // 【改进】优先级调低，不要和系统协议栈抢
        NULL,              
        0                  // Core 0
    );

    xTaskCreatePinnedToCore(
        MotorControlTask,   
        "MotorControlTask", 
        8192,              // 飞控栈给大点防止溢出
        NULL,              
        24,                // 【保持】最高优先级，甚至高于 loopTask(1)
        NULL,              
        1                  // Core 1 独占
    );
}