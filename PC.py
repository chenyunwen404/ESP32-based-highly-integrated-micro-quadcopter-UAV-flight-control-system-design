import pygame
import socket
import struct
import time

EPS = 0.5

# 配置与 ESP32 一致
ESP32_IP = "192.168.4.1"  # ESP32 在 AP 模式下的默认 I
UDP_IP = "0.0.0.0"  # 监听所有网卡
UDP_PORT = 8888

TELEPORT_IP = "127.0.0.1"  # Teleport 软件所在 IP
TELEPORT_PORT = 47269  # Teleport 监听的端口

# --- 初始化 Pygame 手柄 ---
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("未检测到手柄！")
    exit()
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"已连接手柄: {joystick.get_name()}")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)  # 设置为非阻塞模式，关键优化

sock_tx_teleport = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 控制指令结构体定义 (假设发送 4 个 float 模拟量: Throttle, Yaw, Pitch, Roll)
# '<ffff' 表示小端序，4 个 float
CTRL_FORMAT = '<ffffI'

print(f"实时控制与监控启动，目标 ESP32: {ESP32_IP}:{UDP_PORT}")

last_send_time = 0
send_interval = 0.005  # 200Hz 发送控制频率

thr, yaw, pit, rol, mode = 0.0, 0.0, 0.0, 0.0, 0


def InputRemap(axis0, axis1, axis2, axis3, but0, but1, but3,
               thr, yaw, pit, rol, mode):
    
    if but3 == 1.:
        mode = 1
    else:
        pass

    if but1 == 1.:
        mode = 2
        thr = 0
    else:
        pass
    
    
    # 核心逻辑：将 axis 使用三次方处理 (axis**3)
    # 这样在中心 (0 附近) 的变化会非常缓慢，适合微调
    # 而在末端 (1.0 附近) 依然能达到最大值

    yaw = -130 * ((1- EPS)*axis0 + EPS *(axis0 ** 3))

    # 油门微调：三次方能让你在悬停点附近的控制精度提升数倍
    thr = -130.0*4.0 * ((1- EPS)*axis1 + EPS *(axis1 ** 3))

    rol = 15.0 * ((1- EPS)*axis2 + EPS *(axis2 ** 3))
    pit = -12.0 *((1- EPS)*axis3 + EPS *(axis3 ** 3))

    return thr, yaw, pit, rol, mode


try:
    while True:                                                                             
        
        mode = 0

        # 1. 获取手柄事件 (极快)
        pygame.event.pump()

        # 2. 定时发送控制指令给 ESP32 (50Hz)
        current_time = time.time()
        
        

        if current_time - last_send_time >= send_interval:

            thr, yaw, pit, rol, mode = InputRemap(
                joystick.get_axis(0),
                joystick.get_axis(1),
                joystick.get_axis(2),
                joystick.get_axis(3),
                joystick.get_button(0),
                joystick.get_button(1),
                joystick.get_button(3),
                thr, yaw, pit, rol, mode
            )

            # print(f"\r发送控制指令: 油门={thr:.1f} 航向={yaw:.1f} "
            #       f"俯仰={pit:.1f} 横滚={rol:.1f}    ")
            # 打包并异步发送
            ctrl_data = struct.pack(CTRL_FORMAT, thr, yaw, pit, rol, mode)
            sock.sendto(ctrl_data, (ESP32_IP, UDP_PORT))
            last_send_time = current_time

        # 3. 非阻塞接收来自 ESP32 的 IMU 数据 (极快)
        try:
            while True:
                # 1. 接收数据
                data, addr = sock.recvfrom(1024)

                # 2. 解析数据 (对应 C++ 的结构体格式: 6个float)
                # 'fffffff' 表示 7个 float (f)

                unpacked_data = struct.unpack('<fffffff', data)
                ax, ay, az, gx, gy, gz, VBAT = unpacked_data

                msg = (f'ax:{ax:.2f}\nay:{ay:.2f}\naz:{az:.2f}\n'
                       f'gx:{gx:.2f}\ngy:{gy:.2f}\ngz:{gz:.2f}\n'
                       f'roll:{rol:.2f}\npitch:{pit:.2f}\n'
                       f'yaw:{yaw:.2f}\nVBAT:{VBAT:.2f}\n').encode()

                sock_tx_teleport.sendto(msg, (TELEPORT_IP, TELEPORT_PORT))
        except BlockingIOError:
            # 没有收到数据包时会跳到这里，直接继续循环
            pass
        
        time.sleep(0.001)  # 短暂休眠，避免 CPU 占用过高
            
except KeyboardInterrupt:
    print("\n停止控制")
    pygame.quit()