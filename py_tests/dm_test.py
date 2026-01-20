import struct
import time
import serial
import math
import configparser
import os

# 加载配置文件 config.ini
config_path = os.path.join(os.path.dirname(__file__), "config.ini")
config = configparser.ConfigParser()
if not config.read(config_path, encoding='utf-8'):
    print(f"配置文件 {config_path} 未找到或无法读取，请确保文件存在并使用 UTF-8 编码。")
    exit(1)

# 获取配置 [DMTest] 部分的参数
section = "DMTest"
if section not in config:
    print(f"配置文件中缺少 [{section}] 部分。")
    exit(1)

# 串口参数
port = config.get(section, "port", fallback=None)                  # 串口号，例如 "/dev/ttyACM0"
baudrate = config.getint(section, "baudrate", fallback=921600)     # 波特率，默认 921600

# 尝试打开串口
ser = None
if port:
    try:
        ser = serial.Serial(port, baudrate)
        print(f"[INFO] 串口已打开: {port} 波特率 {baudrate}")
    except Exception as e:
        print(f"[警告] 无法打开串口 {port}: {e}")
        print("[INFO] 将运行在离线模式（dry-run），仅生成并打印测试数据帧。")
        ser = None
else:
    print("[警告] 配置未提供串口端口，将运行在离线模式。")
    ser = None

def generate_dm_imu_frame(roll_deg, pitch_deg, yaw_deg):
    """
    生成一个 DM IMU 数据帧（57字节）用于离线测试。
    帧结构（简化版本，仅包含欧拉角数据）：
    - 帧头: 0x55 0xAA 0xD8 0x01 (4字节)
    - 段1数据: 16字节 (此处填充0)
    - 段2数据: 16字节 (此处填充0)
    - 段3数据: 17字节，其中:
      - 段头: 0x90 (1字节)
      - 欧拉角: roll, pitch, yaw (各4字节小端浮点数)
      - 其余: 4字节填充0
    - 帧尾: 0x00 0x00 0x00 (3字节)
    """
    frame = bytearray(57)
    # 帧头
    frame[0:4] = b'\x55\xAA\xD8\x01'
    # 段1和段2填充0 (偏移4-35)
    # 段3起始偏移36
    frame[36] = 0x90  # 段3标识
    # 欧拉角数据偏移40-51
    struct.pack_into('<f', frame, 40, roll_deg)
    struct.pack_into('<f', frame, 44, pitch_deg)
    struct.pack_into('<f', frame, 48, yaw_deg)
    # 帧尾偏移54-56
    frame[54:57] = b'\x00\x00\x00'
    return bytes(frame)

def parse_dm_imu_frame(frame):
    """解析 DM IMU 数据帧，提取欧拉角"""
    if len(frame) < 57:
        return None
    # 欧拉角数据在偏移40-51字节
    roll_deg = struct.unpack('<f', frame[40:44])[0]
    pitch_deg = struct.unpack('<f', frame[44:48])[0]
    yaw_deg = struct.unpack('<f', frame[48:52])[0]
    return roll_deg, pitch_deg, yaw_deg

print("[INFO] 开始运行 DM IMU 测试... 按 Ctrl+C 可停止。")

try:
    if ser:
        # 在线模式：从串口读取真实数据
        buffer = b''
        while True:
            # 读取数据并累积到 buffer
            buffer += ser.read(100)  # 读入一些字节
            # DM IMU 帧头为固定4字节 0x55 0xAA 0xD8 0x01
            header_index = buffer.find(b'\x55\xAA\xD8\x01')
            if header_index != -1 and len(buffer) >= header_index + 57:
                # 找到帧头且长度足够完整57字节帧
                frame = buffer[header_index:header_index+57]
                buffer = buffer[header_index+57:]  # 从buffer中移除该帧

                # 解析欧拉角数据
                result = parse_dm_imu_frame(frame)
                if result:
                    roll_deg, pitch_deg, yaw_deg = result
                    # 打印 HEX 格式
                    hex_str = ' '.join(f'{b:02X}' for b in frame)
                    print(f"接收帧 HEX: {hex_str}")
                    print(f"欧拉角: Roll={roll_deg:.2f}°, Pitch={pitch_deg:.2f}°, Yaw={yaw_deg:.2f}°")
            time.sleep(0.001)
    else:
        # 离线模式：生成模拟数据帧并打印
        print("[INFO] 离线模式：生成模拟 DM IMU 数据帧")
        t = 0
        while True:
            # 生成模拟的欧拉角数据（正弦波变化）
            roll_deg = 10.0 * math.sin(t * 0.5)
            pitch_deg = 15.0 * math.cos(t * 0.3)
            yaw_deg = 180.0 * math.sin(t * 0.2)
            
            # 生成数据帧
            frame = generate_dm_imu_frame(roll_deg, pitch_deg, yaw_deg)
            
            # 打印 HEX 格式
            hex_str = ' '.join(f'{b:02X}' for b in frame)
            print(f"\n生成帧 HEX: {hex_str}")
            print(f"模拟欧拉角: Roll={roll_deg:.2f}°, Pitch={pitch_deg:.2f}°, Yaw={yaw_deg:.2f}°")
            
            t += 0.1
            time.sleep(0.1)  # 10Hz 更新频率
            
except KeyboardInterrupt:
    if ser:
        ser.close()
    print("\n[INFO] dm_test 已停止。")
