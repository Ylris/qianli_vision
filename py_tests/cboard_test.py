import struct, time, serial
import configparser
import os
import math

# ==== 加载配置文件 ====
config_path = os.path.join(os.path.dirname(__file__), "config.ini")
config = configparser.ConfigParser()
if not config.read(config_path, encoding='utf-8'):
    print(f"配置文件 {config_path} 未找到或无法读取，请确保文件存在并使用 UTF-8 编码。")
    exit(1)

# 获取配置 [CboardTest] 部分的参数
section = "CboardTest"
if section not in config:
    print(f"配置文件中缺少 [{section}] 部分。")
    exit(1)

# 串口参数
port = config.get(section, "port", fallback=None)
baudrate = config.getint(section, "baudrate", fallback=115200)

# 初始化串口
ser = None
if port:
    try:
        ser = serial.Serial(port, baudrate)
        print(f"[INFO] 串口已打开: {port} 波特率 {baudrate}")
    except Exception as e:
        print(f"警告: 无法打开串口 {port}: {e}")
        print("将运行在离线模式，仅打印接收到的数据帧而不实际读取串口。")
        ser = None
else:
    print("警告: 配置未提供串口端口，将运行在离线模式。")
    ser = None

# 辅助函数：从四元数计算欧拉角 (Yaw, Pitch, Roll)，返回以度为单位的元组
def quaternion_to_eulers(qw, qx, qy, qz):
    # 基于标准公式，将四元数转为ZYX欧拉角（yaw, pitch, roll）
    # 这里假定四元数已单位化
    # Yaw (Z轴旋转)
    yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    # Pitch (Y轴旋转)
    pitch = math.asin(max(-1.0, min(1.0, 2*(qw*qy - qx*qz))))
    # Roll (X轴旋转)
    roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    # 转换为度
    return (yaw * 180.0 / math.pi, pitch * 180.0 / math.pi, roll * 180.0 / math.pi)

# CRC16 查表法计算（多项式0x1021/0xA001，初始值0xFFFF）
CRC16_TABLE = [
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
]

def calc_crc16(data: bytes) -> int:
    """计算给定数据的 CRC16 校验值（算法: 初始0xFFFF, 多项式0x1021）。"""
    crc = 0xFFFF
    for b in data:
        i = (crc ^ b) & 0xFF
        crc = (crc >> 8) ^ CRC16_TABLE[i]
    return crc

def generate_mock_frame(counter):
    """生成模拟的 NucToVisionFrame 数据帧"""
    # 模拟变化的数据
    t = counter * 0.1  # 时间参数
    
    # 模拟四元数（绕 Z 轴旋转）
    angle = math.sin(t) * 0.5  # 角度在 -0.5 到 0.5 弧度之间变化
    qw = math.cos(angle / 2)
    qx = 0.0
    qy = 0.0
    qz = math.sin(angle / 2)
    
    # 模拟其他数据
    mode_TJ = 1  # 模式
    yaw = angle
    yaw_vel = math.cos(t) * 0.2
    pitch = math.sin(t * 0.5) * 0.3
    pitch_vel = math.cos(t * 0.5) * 0.15
    bullet_speed = 28.0 + math.sin(t) * 2.0  # 子弹速度在 26-30 m/s 之间
    bullet_count = (counter % 500) + 100  # 子弹数量
    game_status = 4  # 游戏状态
    
    # 构造数据帧（不含CRC16）
    frame_without_crc = struct.pack('<2cB4f4f4fHb',
                                    b'S', b'P',
                                    mode_TJ,
                                    qw, qx, qy, qz,
                                    yaw, yaw_vel, pitch, pitch_vel,
                                    bullet_speed, 0.0, 0.0, 0.0,  # 后面3个float填充
                                    bullet_count,
                                    game_status)
    
    # 计算CRC16
    crc_val = calc_crc16(frame_without_crc)
    
    # 完整数据帧
    frame = frame_without_crc + struct.pack('<H', crc_val)
    
    return frame

# 不断读取串口数据
try:
    buffer = b''
    mock_counter = 0  # 离线模式计数器
    print("开始读取数据帧... 按 Ctrl+C 可停止。")
    if not ser:
        print("注意: 离线模式 - 将生成并显示模拟数据帧")
    
    while True:
        if ser:
            # 在线模式：从串口读取实际数据
            buffer += ser.read(64)  # 每次读取最多64字节
            # 查找数据帧头 "SP"
            head_index = buffer.find(b'SP')
            if head_index != -1 and len(buffer) >= head_index + 44:
                # 找到帧头且缓冲区有完整帧长度（NucToVisionFrame 为44字节）
                frame = buffer[head_index:head_index+44]
                buffer = buffer[head_index+44:]  # 移除已处理的部分
                
                # 打印接收到的数据帧 HEX 格式
                hex_str = ' '.join(f'{b:02X}' for b in frame)
                print(f"接收帧 HEX: {hex_str}")
                
                # 解包数据帧（小端格式）
                data = struct.unpack('<2cB4f4f4fHbH', frame)
                # data解析:
                # [0:1] = 'S','P'
                # [2] mode_TJ (uint8)
                # [3:6] 四元数 q0,q1,q2,q3 (floats)
                # [7] yaw (float)
                # [8] yaw_vel (float)
                # [9] pitch (float)
                # [10] pitch_vel (float)
                # [11] bullet_speed (float)
                # [12] bullet_count (uint16)
                # [13] game_status (int8)
                # [14] crc16 (uint16)
                qw, qx, qy, qz = data[3], data[4], data[5], data[6]
                bullet_speed = data[11]
                # 计算欧拉角
                yaw_deg, pitch_deg, roll_deg = quaternion_to_eulers(qw, qx, qy, qz)
                # 打印结果
                print(f"Yaw={yaw_deg:.2f}°, Pitch={pitch_deg:.2f}°, Roll={roll_deg:.2f}°, "
                      f"BulletSpeed={bullet_speed:.2f} m/s")
            # 控制读取频率，避免占用过高CPU
            time.sleep(0.001)
        else:
            # 离线模式：生成模拟数据帧
            frame = generate_mock_frame(mock_counter)
            mock_counter += 1
            
            # 打印模拟数据帧 HEX 格式
            hex_str = ' '.join(f'{b:02X}' for b in frame)
            print(f"\n[模拟] 接收帧 HEX: {hex_str}")
            
            # 解包并显示
            data = struct.unpack('<2cB4f4f4fHbH', frame)
            qw, qx, qy, qz = data[3], data[4], data[5], data[6]
            mode_TJ = data[2]
            yaw = data[7]
            yaw_vel = data[8]
            pitch = data[9]
            pitch_vel = data[10]
            bullet_speed = data[11]
            bullet_count = data[15]
            game_status = data[16]
            crc16 = data[17]
            
            # 计算欧拉角
            yaw_deg, pitch_deg, roll_deg = quaternion_to_eulers(qw, qx, qy, qz)
            
            # 打印解析结果
            print(f"[模拟] 帧字段: mode_TJ={mode_TJ}, "
                  f"q=({qw:.4f}, {qx:.4f}, {qy:.4f}, {qz:.4f})")
            print(f"[模拟] Yaw={yaw_deg:.2f}° (vel={yaw_vel:.3f}), "
                  f"Pitch={pitch_deg:.2f}° (vel={pitch_vel:.3f})")
            print(f"[模拟] BulletSpeed={bullet_speed:.2f} m/s, "
                  f"BulletCount={bullet_count}, GameStatus={game_status}, CRC16=0x{crc16:04X}")
            
            # 离线模式下每秒生成约10帧
            time.sleep(0.1)
            
except KeyboardInterrupt:
    if ser:
        ser.close()
    print("\n[INFO] cboard_test stopped.")
