#pragma once
#include <Arduino.h>
#include "motor.hpp"
#include "PID.hpp"  
#include "IMUPlus.hpp"
#include "I2CVL.hpp"
#include "ADC.hpp"
#include "MPU6000.hpp"
#include "BMP581.hpp"
// 引入必要的头文件
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LOOP_FREQUENCY_HZ 8000                 // 设定目标频率 4000Hz
#define LOOP_FREQUENCY_HZ2 8000                // 设定目标频率 500Hz
#define LOOP_PERIOD_US (1.0f / LOOP_FREQUENCY_HZ) // 周期 250us

#define PITCH_OFFSET -1.5 
#define ROLL_OFFSET 5.0

#define IMU_SPI_MOSI 10
#define IMU_SPI_MISO 11  // 对应上面的 MPU_MISO
#define IMU_SPI_CLK  12  // 对应上面的 MPU_SCK
#define IMU_CS_PIN   8   // 对应上面的 MPU_CS
#define IMU_INT_PIN  15  // 上面未提及，保持原样


extern portMUX_TYPE flightMux; // 任务间互斥锁

enum FlightMode {
    MODE_WAIT = 0,
    MODE_TAKEOFF = 1,
    MODE_STOP = 2,
    MODE_ACRO = 3
};

enum ContrleMode {
    CMODE_WAIT = 0,
    CMODE_TAKEOFF = 1,
    CMODE_STOP = 2,
    CMODE_ACRO = 3
};

// 飞行指令结构体：包含油门和目标姿态角
struct FlightCommand {
    float throttle;     // 0 ~ 1023
    float targetPitch;  // 目标角度 (度)
    float targetRoll;   // 目标角度 (度)
    float targetYaw;    // 目标偏航 (度或角速度)
    ContrleMode mode;
};

struct sensorData {
    float accX;
    float accY;
    float accZ;
    float gyrX;
    float gyrY;
    float gyrZ;
};

// --- 数据结构定义 (关键) ---
// 使用 packed 确保没有多余的填充字节
struct __attribute__((packed)) Send_Frame {
    float ax, ay, az;
    float gx, gy, gz;
    float VBAT; // 电池电压，单位 V
};

// 增加控制数据结构体，与 Python 中的 struct.pack('<ffff', ...) 对应
struct __attribute__((packed)) Control_Frame {
    float throttle;
    float yaw;
    float pitch;
    float roll;
    int mode;
    // int PIDnum;
    // float P;
    // float I;
    // float D;
};

struct Controlstruct{
    int hight;//高度，mm
    float yaw;//控制Z旋转
    float yaw_sp;//控制Z轴旋转速度
    float roll;//滚转角度
    float pitch;//俯仰角度

};

struct FlagStruct{
    bool flag_first;
    int time;
    int flag_num;

};


extern FlightCommand globalCmd; // 全局飞行指令变量

extern DroneManager droneManager; // 全局动力管理实例

extern PID pidgx; // X轴角速度 PID 控制器
extern PID pidgy; // Y轴角速度 PID 控制器
extern PID pidgz; // Z轴角速度 PID 控制器

extern PID pidPPitchAltitude; // Pitch 角度环 PID 控制器
extern PID pidPRollAltitude;  // Roll 角度环 PID 控制器

extern PID pidPYawAltitude; // Yaw 角度环 PID 控制器
extern PID pidHighSpeed; // 高度环 PID 控制器

extern PID pidHigh; // 高度环 PID 控制器

extern IMUSolver IMU; // 全局 IMU 实例
extern IMUSolver IMU_C; // 备用 IMU 实例
extern FlightCommand cmd; // 当前飞行指令缓存

extern AltitudeSystem altitudeSystem; // 全局高度系统实例

extern float roll_offset;  // 机体安装误差补偿
extern float pitch_offset; // 机体安装误差补偿

extern sensorData sensor_cache; // 传感器数据缓存

extern BatteryMonitor bat; // 电池监测实例

extern volatile float v_comp; // 电压补偿系数

// 确保 FlightMode 的定义在声明之前
extern volatile FlightMode f_mode;

extern volatile Controlstruct ctls; // 控制结构体变量
extern DRAM_ATTR FlagStruct mian_flag;
extern DRAM_ATTR FlagStruct sub_flag;

extern DRAM_ATTR ContrleMode lmode;
extern DRAM_ATTR ContrleMode cmode;

extern DRAM_ATTR MPU6000_S3 MPU6000;

extern DRAM_ATTR BMP581_Drone bmp581;

extern DRAM_ATTR  bool PIDmap[4][2];
extern DRAM_ATTR  float setmap[4][2];

// 在头文件或源文件中定义
template<typename T_Outer, typename T_Inner>
static inline float IRAM_ATTR run_cascade_control(
    int idx, T_Outer& outer, T_Inner& inner, float o_meas, float i_meas,float offset) 
{
    if (!PIDmap[idx][0]) {
        inner.reset();
        outer.reset();
        return 0.0f;
    }

    float target = setmap[idx][0];
    if (PIDmap[idx][1]) {
        float v = outer.compute(setmap[idx][1] + offset, o_meas, LOOP_PERIOD_US);
        if (target == -1) target = v;
    } else {
        outer.reset();
    }

    return inner.compute(target, i_meas, LOOP_PERIOD_US);
}


void updateCommand(Control_Frame& ctrl);

void IRAM_ATTR cleanFlag(FlagStruct& fl);

void IRAM_ATTR PIDMixContorl(float comp,int thr);

extern TiltCompensator DRAM_ATTR TCR;

extern portMUX_TYPE DRAM_ATTR sharedDataMux;

extern volatile Control_Frame globalCtrlData;