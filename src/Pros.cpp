

#include "Pros.hpp"

//作用:模式跳变检测
void IRAM_ATTR modeJumpDetect(ContrleMode& lastMode, ContrleMode currentMode)
{
    if (lastMode != currentMode)
    {
        //模式发生跳变，执行相应操作
        lastMode = currentMode;
        //在这里添加模式跳变时的处理逻辑
        f_mode = (FlightMode)currentMode;

    }
}


//作用：IMU采样
void IRAM_ATTR sensorUpdata()
{
    
      portENTER_CRITICAL(&flightMux);

      MPU6000.update();

      //IMU.imu.upData(); // 更新IMU数据
      portEXIT_CRITICAL(&flightMux);

       // 1. 同步数据
      portENTER_CRITICAL(&flightMux);
      cmd = (FlightCommand)globalCmd;
      sensor_cache.accX = MPU6000.ax;
      sensor_cache.accY = MPU6000.ay;
      sensor_cache.accZ = MPU6000.az;
      sensor_cache.gyrX = MPU6000.gx;
      sensor_cache.gyrY = MPU6000.gy;
      sensor_cache.gyrZ = MPU6000.gz;

      portEXIT_CRITICAL(&flightMux);
    
      IMU_C.update(
          sensor_cache.accX,
          sensor_cache.accY,
          sensor_cache.accZ,
          sensor_cache.gyrX,
          sensor_cache.gyrY,
          sensor_cache.gyrZ
      );
      portENTER_CRITICAL(&flightMux);
      memcpy(&IMU.roll, &IMU_C.roll, sizeof(float)*3);
      portEXIT_CRITICAL(&flightMux);
}


//作用：控制过程进行
void IRAM_ATTR controlProcess()
{

    float cmop = (4.0f / v_comp); // 假设 4V为基准电压

    
    switch (f_mode)
    {
    case MODE_WAIT:


        //Serial.println(cmd.targetYaw);
        
        if (globalCmd.mode != 0)
        {
            f_mode = (FlightMode)globalCmd.mode;
            cleanFlag(mian_flag);
        }
        /* code */
        break;
    case MODE_TAKEOFF:

        

        if(mian_flag.flag_first == false)
        {
            mian_flag.flag_num = 1;
            mian_flag.time = millis();
            mian_flag.flag_first = true;
        }
        if (globalCmd.mode == MODE_STOP)
        {
            cleanFlag(mian_flag);
            f_mode = MODE_STOP;
            /* code */
        }

        //Serial.printf("f:%dn:%d\n", mian_flag.time,millis());

        if(mian_flag.flag_num == 1)
        {
            IMU_C.pitch = 0;
            IMU_C.roll  = 0;
            IMU_C.yaw = 0;
            float sum_ax = 0.0f;
            float sum_ay = 0.0f; 
            float sum_az = 0.0f;

            MPU6000.calibrateGyro(500);

            // for(int i = 0 ;i< 100 ;i++)
            // {
            //     IMU.imu.upData();
            //     sum_ax += IMU.imu.sensor.accX();
            //     sum_ay += IMU.imu.sensor.accY();
            //     sum_az += IMU.imu.sensor.accZ();
            //     delay(5);
            // }

            // IMU.begin(sum_ax/100.0f, sum_ay/100.0f, sum_az/100.0f);
            mian_flag.flag_num = 2;
            mian_flag.time = millis();
        }
        else if(mian_flag.flag_num == 2)
        {
            memset(&PIDmap[0][0],0,sizeof(bool)*4*2);
            memset(&setmap[0][0],0,sizeof(float)*4*2);

            PIDmap[1][0] = 0;
            PIDmap[2][0] = 0;
            PIDmap[3][0] = 0;
            PIDmap[1][1] = 0;
            PIDmap[2][1] = 0;
            PIDmap[3][1] = 0;
            
            PIDMixContorl(cmop,50*4);
            if(millis() - mian_flag.time > 1000)
            {
                mian_flag.flag_num = 3;
                mian_flag.time = millis();
            }
        }
        else if(mian_flag.flag_num == 3)
        {
            memset(&PIDmap[0][0],0,sizeof(bool)*4*2);
            PIDmap[0][0] = 0;
            PIDmap[1][0] = 1;
            PIDmap[2][0] = 1;
            PIDmap[3][0] = 1;
            PIDmap[0][1] = 0;
            PIDmap[1][1] = 0;
            PIDmap[2][1] = 0;
            PIDmap[3][1] = 0;

            memset(&setmap[0][0],0,sizeof(float)*4*2);
            setmap[0][1] = 0;
            setmap[0][0] = 0;
            setmap[1][0] = 0;
            setmap[2][0] = 0;
            setmap[3][0] = 0;
            PIDMixContorl(cmop,TCR.update(300*4,IMU_C.roll* 0.0174533f,IMU_C.pitch* 0.0174533f));
             if(millis() - mian_flag.time > 100)
            {
                mian_flag.flag_num = 4;
                mian_flag.time = millis();
                // f_mode = MODE_STOP;
                // cleanFlag(mian_flag);
            }
        }
        else if(mian_flag.flag_num == 4)
        {
            memset(&PIDmap[0][0],0,sizeof(bool)*4*2);
            PIDmap[0][0] = 0;
            PIDmap[1][0] = 1;
            PIDmap[2][0] = 1;
            PIDmap[3][0] = 1;
            PIDmap[0][1] = 0;
            PIDmap[1][1] = 1;
            PIDmap[2][1] = 1;
            PIDmap[3][1] = 0;

            memset(&setmap[0][0],0,sizeof(float)*4*2);
            setmap[0][1] = 0;
            setmap[0][0] = 0;
            setmap[1][1] = cmd.targetPitch;
            setmap[2][1] = cmd.targetRoll;
            // setmap[1][0] = -1;
            // setmap[2][0] = -1;
            setmap[3][0] = cmd.targetYaw;
            setmap[1][0] = -1;
            setmap[2][0] = -1;
            PIDMixContorl(cmop,TCR.update(191*4 + cmd.throttle,IMU_C.roll* 0.0174533f,IMU_C.pitch* 0.0174533f));
             if(millis() - mian_flag.time > 120000)
            {
                mian_flag.flag_num = 5;
                mian_flag.time = millis();
                f_mode = MODE_STOP;
                cleanFlag(mian_flag);
            }
        }
        // else if(mian_flag.flag_num == 4)
        // {
        //     memset(&PIDmap[0][0],0,sizeof(bool)*4*2);
        //     PIDmap[0][0] = 1;
        //     PIDmap[1][0] = 1;
        //     PIDmap[2][0] = 1;
        //     PIDmap[3][0] = 1;
        //     PIDmap[1][1] = 1;
        //     PIDmap[2][1] = 1;
        //     PIDmap[3][1] = 1;
        //     memset(&setmap[0][0],0,sizeof(float)*4*2);
        //     setmap[0][0] = 0;
        //     setmap[1][0] = -1;
        //     setmap[2][0] = -1;
        //     setmap[3][0] = 0;
        //     PIDMixContorl(cmop,218);

        //     if(millis() - mian_flag.time > 600)
        //     {
        //         mian_flag.flag_num = 5;
        //         mian_flag.time = millis();
        //         setmap[0][1] = altitudeSystem.getHeight();
        //     }
        // }
        // else if(mian_flag.flag_num == 5)
        // {
        //     PIDmap[0][0] = 1;
        //     PIDmap[1][0] = 1;
        //     PIDmap[2][0] = 1;
        //     PIDmap[3][0] = 1;
        //     PIDmap[0][1] = 1;
        //     PIDmap[1][1] = 1;
        //     PIDmap[2][1] = 1;
        //     PIDmap[3][1] = 1;

        //     setmap[0][0] = -1;
        //     setmap[1][0] = -1;
        //     setmap[2][0] = -1;
        //     setmap[3][0] = 0;


        //     PIDMixContorl(cmop,218);

        //     if(millis() - mian_flag.time > 50)
        //     {
        //         f_mode = MODE_ACRO;
        //         cleanFlag(mian_flag);
        //     }

        // }
        
        break;
    case MODE_STOP:

        if (globalCmd.mode == MODE_TAKEOFF)
        {
            cleanFlag(mian_flag);
            f_mode = MODE_TAKEOFF;
            /* code */
        }
        
        memset(&PIDmap[0][0],0,sizeof(bool)*4*2);
        memset(&setmap[0][0],0,sizeof(float)*4*2);
        PIDMixContorl(cmop,0);
        break;
    case MODE_ACRO:
        if(mian_flag.flag_first == false)
        {
            mian_flag.flag_num = 1;
            mian_flag.time = millis();
            mian_flag.flag_first = true;
        }
        if (globalCmd.mode == MODE_STOP)
        {
            cleanFlag(mian_flag);
            f_mode = MODE_STOP;
            /* code */
        }
            PIDmap[0][0] = 1;
            PIDmap[1][0] = 1;
            PIDmap[2][0] = 1;
            PIDmap[3][0] = 1;
            PIDmap[0][1] = 1;
            PIDmap[1][1] = 1;
            PIDmap[2][1] = 1;
            PIDmap[3][1] = 1;

            setmap[0][0] = -1;
            setmap[1][0] = -1;
            setmap[2][0] = -1;
            setmap[3][0] = 0;
            setmap[1][1] = cmd.targetPitch;
            setmap[2][1] = cmd.targetRoll;

            PIDMixContorl(cmop,218);

        break;
    default:
        break;
    }
}