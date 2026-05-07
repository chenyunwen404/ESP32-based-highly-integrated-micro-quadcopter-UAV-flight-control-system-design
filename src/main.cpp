#include <Arduino.h>
#include <SPI.h>
#include "ICM42688.h"
#include "IMU.hpp"
#include "WIFI.hpp"
#include "motor.hpp"
#include "Task.hpp"

// --- 硬件引脚配置 (保持和你之前的一样) ---



//WIFI

TaskManager sysTask; // 全局任务管理器实例



void setup() {
  Serial.begin(115200);

  // 1. 【极大优化】将等待时间缩短为 100ms
  // 实际上如果不插线，根本不需要等，100ms 足够判断状态了
  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 100)) {
    delay(10); 
  }

  // 2. 【并行优化】先启动 WiFi！
  // WiFi 硬件启动需要时间，让它在后台跑，我们利用这段时间去校准传感器
  WIFIInit(targetIP); 

  // 3. 初始化总线
  beginI2C();

  if(bmp581.begin(I2C_Bus))
  {
    Serial.printf("成功\n");
  }
  else
  {
    Serial.printf("失败\n");
  }

  delay(100); // 等待 10ms 再试

  bmp581.tare();

  bmp581.update();

  float x = bmp581.getAltitude();

  Serial.printf("x:%.1f\n",x);

    
  SPI.begin(IMU_SPI_CLK,IMU_SPI_MISO,IMU_SPI_MOSI,IMU_CS_PIN);

  // 4. 初始化传感器
  MPU6000.begin();

  // 5. 【掩盖耗时】在 WiFi 建立连接的过程中，做耗时的校准
  // 此时 CPU 在算校准，RF 射频电路在后台发射信号，互不耽误

  //MPU6000.calibrateAccel();

  MPU6000.calibrateGyro(); 

  MPU6000.loadAccelFromFlash();
  // 6. 飞控系统初始化
  //droneManager.begin();
  
  // ⚠️ 再次提醒：armAll 放在这里真的很危险，一旦上电油门不在低位，
  // 或者电调协议识别错误，电机可能瞬间全速旋转。
  droneManager.begin();
  droneManager.mixTable(0,0,0,0,0);

  sysTask.begin(); 
  bat.begin(); 
  
  Serial.println("System Ready!");
}

void loop() {

    
}