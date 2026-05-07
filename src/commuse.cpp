#include "commonuse.hpp"

// --- 核心修改 1: 定义自旋锁和共享数据 ---
portMUX_TYPE DRAM_ATTR sharedDataMux = portMUX_INITIALIZER_UNLOCKED;

portMUX_TYPE DRAM_ATTR flightMux = portMUX_INITIALIZER_UNLOCKED;

FlightCommand DRAM_ATTR globalCmd = {0, 0, 0, 0, CMODE_WAIT};

DroneManager DRAM_ATTR droneManager;

MPU6000_S3 DRAM_ATTR MPU6000(8, &SPI);

// PID DRAM_ATTR pidgx(5e-1f, 20.0e-1f, 4.0e-4f*2, 400.0f, 150.0f, 0.05f);
// PID DRAM_ATTR pidgy(5e-1f, 20.0e-1f, 4.0e-4f*2, 400.0f, 150.0f, 0.05f);
PID DRAM_ATTR pidgx(4.0f*10e-1f, 4.0f*1.2e-1f*8.0, 4.0f*8e-4/8.0, 4.0f*500.0f, 4.0f*300.0f, 0.08f);
PID DRAM_ATTR pidgy(4.0f*20e-1f, 4.0f*1.2e-1f*8.0, 4.0f*14e-4/8.0, 4.0f*500.0f, 4.0f*300.0f, 0.08f);
PID DRAM_ATTR pidgz(4.0f*1.2e1f,4.0f*2.2e-1f*8.0, 0.00f, 4.0f*200.0f, 4.0f*200.0f, 1.0f);
//角度环
// 【修正 1】角度环 P 值建议从 3.5 开始，16 太大了会震荡
PID DRAM_ATTR pidPPitchAltitude(2.6, 0, 0.0, 200.0, 100.0, 0.1f); 
PID DRAM_ATTR pidPRollAltitude (2.6, 0, 0.0, 200.0, 100.0, 0.1f);
PID DRAM_ATTR pidPYawAltitude(2, 0, 0.0, 200.0, 100.0, 0.1f);

PID DRAM_ATTR pidHighSpeed(4.0*0.0f, 0.0f, 0.0f, 4.0f*500.0f, 4.0f*300.0f, 1.0f); // 高度环 PID 控制器

PID DRAM_ATTR pidHigh(0.01f, 0.0f, 0.0f, 200.0f, 0.0f, 1.0f); // 高度环 PID 控制器

float DRAM_ATTR roll_offset = -0.0f; // 机体安装误差补偿
float DRAM_ATTR pitch_offset = 0.0f; // 机体安装误差补偿

IMUSolver DRAM_ATTR IMU;
IMUSolver DRAM_ATTR IMU_C;

FlightCommand DRAM_ATTR cmd;

sensorData DRAM_ATTR sensor_cache;

AltitudeSystem DRAM_ATTR altitudeSystem;

BatteryMonitor DRAM_ATTR bat;

TiltCompensator DRAM_ATTR TCR;

DRAM_ATTR volatile float v_comp = 0.0f; // 电压补偿系数

DRAM_ATTR volatile FlightMode f_mode = MODE_WAIT;
DRAM_ATTR volatile Controlstruct ctls;

DRAM_ATTR FlagStruct mian_flag = {0,0,0};
DRAM_ATTR FlagStruct sub_flag = {0,0,0};


//先内后外，高度->俯仰->滚转->偏航
DRAM_ATTR bool PIDmap[4][2] = {0};

//赋值为-1,表示继承父级，表示否则采用
DRAM_ATTR float setmap[4][2] = {0};

DRAM_ATTR BMP581_Drone bmp581;

DRAM_ATTR ContrleMode lmode;
DRAM_ATTR ContrleMode cmode;

// 全局共享变量（WiFi 写入，电机读取）
DRAM_ATTR volatile Control_Frame globalCtrlData = {0, 0, 0, 0};


// 更新飞行数据接口
void IRAM_ATTR updateCommand(Control_Frame& ctrl) {
    portENTER_CRITICAL(&flightMux);
    globalCmd.throttle    = ctrl.throttle;
    globalCmd.targetYaw   = ctrl.yaw;
    globalCmd.targetPitch = ctrl.pitch;
    globalCmd.targetRoll  = ctrl.roll;
    globalCmd.mode = (ContrleMode)ctrl.mode;

    portEXIT_CRITICAL(&flightMux);
}

//作用：综合控制PID
void IRAM_ATTR PIDMixContorl(float comp,int thr) {
    // 1. 高度控制 (Axis 0)
    float h = run_cascade_control(0, pidHigh, pidHighSpeed, bmp581.getAltitude(), bmp581.getVelocity(),0);

    // 2. 俯仰控制 (Axis 1) - 修复了原代码索引错误
    float y = run_cascade_control(1, pidPPitchAltitude, pidgy, IMU_C.pitch, sensor_cache.gyrY,PITCH_OFFSET);

    // 3. 横滚控制 (Axis 2)
    float x = run_cascade_control(2, pidPRollAltitude, pidgx, IMU_C.roll, sensor_cache.gyrX,ROLL_OFFSET);



    // 4. 航向控制 (Axis 3)
    float z = run_cascade_control(3, pidPYawAltitude, pidgz, IMU_C.yaw, sensor_cache.gyrZ,0);

    droneManager.mixTable(
        (thr + h) ,
        x,
        y,
        z,
        comp
    );

}

void IRAM_ATTR cleanFlag(FlagStruct& fl)
{
    fl.flag_first = 0;
    fl.flag_num = 0;
    fl.time = 0;
}




