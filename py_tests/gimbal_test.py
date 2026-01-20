import serial
import struct
import time
import configparser
import os

# 加载配置文件 config.ini
config_path = os.path.join(os.path.dirname(__file__), "config.ini")
config = configparser.ConfigParser()
if not config.read(config_path, encoding='utf-8'):
    print(f"配置文件 {config_path} 未找到或无法读取，请确保文件存在并使用 UTF-8 编码。")
    exit(1)

# 获取配置 [GimbalTest] 部分的参数
section = "GimbalTest"
if section not in config:
    print(f"配置文件中缺少 [{section}] 部分。")
    exit(1)

# 串口参数
port = config.get(section, "port", fallback=None)                  # 串口号，例如 "/dev/ttyS3"
baudrate = config.getint(section, "baudrate", fallback=115200)     # 波特率，默认 115200
frequency = config.getfloat(section, "frequency", fallback=10.0)   # 发送频率（Hz），默认 10Hz

# 控制指令参数（未配置则默认使用0或False）
yaw = config.getfloat(section, "yaw", fallback=0.0)                # 目标 Yaw 角度（单位：弧度）
pitch = config.getfloat(section, "pitch", fallback=0.0)            # 目标 Pitch 角度（单位：弧度）
yaw_vel = config.getfloat(section, "yaw_vel", fallback=0.0)        # 目标 Yaw 角速度（弧度/秒）
yaw_acc = config.getfloat(section, "yaw_acc", fallback=0.0)        # 目标 Yaw 角加速度（弧度/秒²）
pitch_vel = config.getfloat(section, "pitch_vel", fallback=0.0)    # 目标 Pitch 角速度（弧度/秒）
pitch_acc = config.getfloat(section, "pitch_acc", fallback=0.0)    # 目标 Pitch 角加速度（弧度/秒²）
# 开火测试标志：True 表示周期性模拟开火，False 表示不发送开火指令
try:
    fire_test = config.getboolean(section, "fire", fallback=False)
except ValueError:
    # 若读取失败，进行手动解析
    fire_val = config.get(section, "fire", fallback="false").strip().lower()
    fire_test = fire_val in ("1", "true", "yes", "y")

# 初始化串口
ser = None
if port:
    try:
        ser = serial.Serial(port, baudrate)
        # 可根据需要设置超时等参数：ser.timeout = 0
        print(f"串口已打开: {port} 波特率 {baudrate}")
    except Exception as e:
        print(f"警告: 无法打开串口 {port}: {e}")
        print("将运行在离线模式，仅打印数据帧而不发送。")
        ser = None
else:
    print("警告: 配置未提供串口端口，将运行在离线模式。")
    ser = None

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

# 周期性开火控制变量初始化
fire_state = False      # 当前是否处于开火状态
toggle_counter = 0      # 计数器，用于跟踪状态持续时间
# 计算开火/停火持续的循环数（约1.0秒不开火，0.2秒开火）
if frequency > 0:
    off_loops = int(frequency * 1.0)   # 停火持续循环数（频率 * 1秒）
    on_loops = int(frequency * 0.2)    # 开火持续循环数（频率 * 0.2秒）
    if on_loops < 1:
        on_loops = 1   # 确保至少一个循环
else:
    off_loops = on_loops = 0

print("开始发送数据帧... 按 Ctrl+C 可停止。")
try:
    # 按照指定频率发送数据帧
    interval = 1.0 / frequency if frequency > 0 else 0
    while True:
        # 设置模式位 (mode): 0=不控制, 1=控制不开火, 2=控制且开火:contentReference[oaicite:1]{index=1}
        # 此测试中，我们始终发送控制命令（mode不会为0）。根据 fire_test 决定是否模拟开火。
        if fire_test:
            # 周期性切换开火状态
            if fire_state:
                # 当前在开火状态，判断是否达到持续时间，应停止开火
                if on_loops and toggle_counter >= on_loops:
                    fire_state = False
                    toggle_counter = 0
            else:
                # 当前未开火，判断是否达到间隔时间，应开始开火
                if off_loops and toggle_counter >= off_loops:
                    fire_state = True
                    toggle_counter = 0
            toggle_counter += 1
            mode = 2 if fire_state else 1
        else:
            mode = 1  # 不进行开火测试时，始终为控制但不开火

        # 构造数据帧（不含CRC16），按照小端序打包:
        # 帧头 'S','P' (2字节), mode (1字节), 后续依次为6个 float (yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc)
        frame_without_crc = struct.pack('<2sBffffff', b'SP', mode, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc)
        # 计算CRC16校验值
        crc_val = calc_crc16(frame_without_crc)
        # 将CRC16附加到帧末尾（小端序2字节）
        frame = frame_without_crc + struct.pack('<H', crc_val)

        # 通过串口发送数据帧
        if ser:
            try:
                ser.write(frame)
            except Exception as e:
                print(f"串口发送错误: {e}")
                # 发送失败则跳过本次循环（仍继续后续发送）

        # 将发送的数据帧以 HEX 格式和各字段值打印输出
        hex_str = ' '.join(f'{b:02X}' for b in frame)
        print(f"发送帧 HEX: {hex_str}")
        mode_str = {0: '0(不控制)', 1: '1(控制, 不开火)', 2: '2(控制, 开火)'}.get(mode, str(mode))
        print(f"帧字段: head='SP', mode={mode_str}, yaw={yaw}, yaw_vel={yaw_vel}, "
              f"yaw_acc={yaw_acc}, pitch={pitch}, pitch_vel={pitch_vel}, "
              f"pitch_acc={pitch_acc}, crc16=0x{crc_val:04X}")

        # 等待下次发送
        if interval > 0:
            time.sleep(interval)
        else:
            # 若频率为0（不应发生）则稍作延时避免死循环
            time.sleep(0.1)
except KeyboardInterrupt:
    # 捕捉 Ctrl+C 退出
    print("\n发送已停止。")
finally:
    if ser:
        ser.close()
        print("串口已关闭。")
