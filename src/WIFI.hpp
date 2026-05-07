#pragma once
#include <WiFi.h>
#include <esp_wifi.h> // 必须包含底层头文件
#include <WiFiUdp.h>
// 包含你原来的 IMU 相关头文件
#include "IMU.hpp"
#include <AsyncUDP.h>  // 1. 替换为异步UDP库
#include "commonuse.hpp"

// --- WiFi 配置 ---
extern  const char* ssid    ;
extern  const char* password;
extern  const char* hostIP   ;
extern  const int   port    ;           // 目标端口

extern Control_Frame lastCtrl;
// 全局异步UDP对象
extern AsyncUDP udp;
extern IPAddress targetIP; // 预解析IP，减少发送时的开销

void WIFIInit(IPAddress &targetIP) ;

void fullSendIMUData(IMUSolver& imu,Send_Frame& frame) ;
// 4. 改良后的发送函数
void sendIMUData(IMUSolver& imu,AsyncUDP& udp, IPAddress& targetIP);

void WIFILoop(IMUSolver& imu,AsyncUDP& udp, IPAddress& targetIP);