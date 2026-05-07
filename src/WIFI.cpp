#include "WIFI.hpp"

// --- WiFi 配置 ---
const char* ssid     = "ESP328";
const char* password = "12345678";
const char* hostIP   = "192.168.4.2"; // 电脑的IP地址
const int   port     = 8888;     

Control_Frame lastCtrl = {0, 0, 0, 0};
AsyncUDP udp;
IPAddress targetIP; // 预解析IP，减少发送时的开销


void RXDone(AsyncUDPPacket packet)
{
     if (packet.length() == sizeof(Control_Frame)) {
                // 将收到的原始字节直接拷贝进结构体
                memcpy(&lastCtrl, packet.data(), sizeof(Control_Frame));
                updateCommand(lastCtrl);

                
                
                // 调试输出（建议只在初期测试时开启，以免影响性能）
                
                // Serial.printf("Thr: %.2f | Roll: %.2f\n", 
                //                  lastCtrl.throttle, lastCtrl.roll);
                
            }
}

void WIFIInit(IPAddress &targetIP) {
    // 1. 配置静态IP (加速连接)
    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_IP, gateway, subnet);

    // 2. 启动 AP
    // 【修改】信道改为 6 (兼容性最好)，最大连接数改为 1 (减少管理开销)
    if (WiFi.softAP(ssid, password, 13, 0, 1)) {
        // 成功启动，不要在这里 print 太多
    }
    
    // 3. 【关键】关闭 Wi-Fi 省电模式 (必须保留)
    esp_wifi_set_ps(WIFI_PS_NONE); 
    
    // 4. 设置目标 IP
    targetIP.fromString(hostIP); 

    // 5. 启动 UDP 监听
    if(udp.listen(port)) { 
        udp.onPacket([](AsyncUDPPacket packet) {
            RXDone(packet);
        });
    }
}


// 建议去掉 IRAM_ATTR，除非你在定时器中断里调用它。如果在 loop 里调用，不需要 IRAM。
void fullSendIMUData(IMUSolver& imu, Send_Frame& frame) {
    // // 【优化】直接赋值比 memcpy 更快，且避免内存越界风险
    // // 假设 MPU6000 是全局对象，且数据已在主循环更新
    // frame.ax = MPU6000.ax;
    // frame.ay = MPU6000.ay;
    // frame.az = MPU6000.az;
    // frame.gx = MPU6000.gx;
    // frame.gy = MPU6000.gy;
    // frame.gz = MPU6000.gz;

    // // 【解决卡顿】不要用 memcpy 拷贝 float，直接赋值。
    // // 如果 imu.roll 是 public 变量直接访问；如果是函数 getRoll() 确保它没有复杂计算
    // frame.roll  = imu.roll;
    // frame.pitch = imu.pitch;
    // frame.yaw   = imu.yaw;

    // 赋值电压
    frame.VBAT = v_comp;
}

void batPro()
{
    bat.update();
    v_comp = bat.getVoltage();
}

void sendIMUData(IMUSolver& imu, AsyncUDP& udp, IPAddress& targetIP) {
    Send_Frame frame; // 在栈上创建，速度快
    
    // 填充数据
    fullSendIMUData(imu, frame);

    // 发送 (AsyncUDP 的 writeTo 是非阻塞的，但也需要时间拷贝到缓冲区)
    udp.writeTo((uint8_t*)&frame, sizeof(frame), targetIP, port);
}

void WIFILoop(IMUSolver& imu, AsyncUDP& udp, IPAddress& targetIP)
{
    static unsigned long lastSendTime = 0;
    static unsigned long lastBatTime = 0; // 单独给电池用一个计时器

    unsigned long now = millis();

    // -------------------------------------------------
    // 任务1: 发送 IMU 数据 (建议 20Hz - 30Hz，太快 UDP 会堵塞)
    // -------------------------------------------------
    if (now - lastSendTime > 2000) { // 40ms = 25Hz
        // 【极大性能优化】如果没有设备连接 AP，就根本不要打包发送！
        // 这能拯救因 WiFi 发送导致的飞行手感卡顿
        if (WiFi.softAPgetStationNum() > 0) {
            sendIMUData(imu, udp, targetIP);
        }
        lastSendTime = now;
    }

    // -------------------------------------------------
    // 任务2: 电池电压读取 (建议 1Hz)
    // -------------------------------------------------
    // ADC 读取非常慢，绝对不能 50Hz 跑，会拖慢姿态解算
    if (now - lastBatTime > 100) { // 1000ms = 1Hz
        batPro();
        lastBatTime = now;
    }
}