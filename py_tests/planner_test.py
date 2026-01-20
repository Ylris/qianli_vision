import struct, time, math, serial
import configparser
import os

# 加载配置文件 config.ini
config_path = os.path.join(os.path.dirname(__file__), "config.ini")
config = configparser.ConfigParser()
if not config.read(config_path, encoding='utf-8'):
    print(f"配置文件 {config_path} 未找到或无法读取，请确保文件存在并使用 UTF-8 编码。")
    exit(1)

# 获取配置 [PlannerTest] 部分的参数
section = "PlannerTest"
if section not in config:
    print(f"配置文件中缺少 [{section}] 部分。")
    exit(1)

# 串口参数
port = config.get(section, "port", fallback=None)
baudrate = config.getint(section, "baudrate", fallback=115200)

# 规划参数
d = config.getfloat(section, "distance", fallback=3.0)           # 目标距离 (米)
w = config.getfloat(section, "angular_velocity", fallback=5.0)   # 目标角速度 (弧度/秒)
bullet_speed = config.getfloat(section, "bullet_speed", fallback=18.0)  # 子弹初速 (m/s)
frequency = config.getfloat(section, "frequency", fallback=100.0)        # 控制频率 (Hz)

# CRC16 查表法计算（多项式0x1021，初始值0xFFFF）
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

# 初始化串口
ser = None
if port:
    try:
        ser = serial.Serial(port, baudrate)
        print(f"[INFO] 串口已打开: {port} 波特率 {baudrate}")
    except Exception as e:
        print(f"[警告] 无法打开串口 {port}: {e}")
        print("[INFO] 将运行在离线模式，仅打印数据帧而不发送。")
        ser = None
else:
    print("[警告] 配置未提供串口端口，将运行在离线模式。")
    ser = None

# 简化假设：目标在水平面绕机器人旋转，距机器人d米，角速度w绕垂直轴运动

# 模拟目标初始状态
target_yaw = 0.0         # 目标相对于机器人初始方位角 (弧度)
target_pitch = 0.0       # 简化假设目标与云台高度一致，pitch角为0

# 计算发送间隔
interval = 1.0 / frequency if frequency > 0 else 0.01

print("[INFO] 开始发送数据帧... 按 Ctrl+C 可停止。")
start_time = time.time()
try:
    while True:
        # 更新时间
        t = time.time() - start_time
        # 更新目标位置（角度随时间线性增加）
        target_yaw = (w * t) % (2 * math.pi)  # 目标方位角随时间增加，取模2π防止过大

        # 简单瞄准策略：直接将云台yaw对准目标当前方位角，pitch保持0
        plan_yaw = target_yaw
        plan_pitch = target_pitch
        # （无弹道下坠补偿，忽略目标移动预测和开火决策，实际Planner算法更复杂）

        # 控制命令：始终控制云台，不自动开火
        mode = 1  # 控制但不开火

        # 构造发送数据帧（VisionToGimbal）：帧头 'S','P' (2字节), mode (1字节), 
        # 后续依次为6个 float (yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc)
        frame_without_crc = struct.pack('<2sBffffff', b'SP', mode, 
                                        plan_yaw, 0.0, 0.0,
                                        plan_pitch, 0.0, 0.0)
        # 计算CRC16校验值
        crc_val = calc_crc16(frame_without_crc)
        # 将CRC16附加到帧末尾（小端序2字节）
        send_frame = frame_without_crc + struct.pack('<H', crc_val)

        # 通过串口发送数据帧（如果串口可用）
        if ser:
            try:
                ser.write(send_frame)
            except Exception as e:
                print(f"[错误] 串口发送错误: {e}")

        # 打印发送的数据帧（HEX格式）
        hex_str = ' '.join(f'{b:02X}' for b in send_frame)
        deg = 180.0 / math.pi
        print(f"\n[发送] t={t:.2f}s")
        print(f"  HEX: {hex_str}")
        mode_str = {0: '0(不控制)', 1: '1(控制, 不开火)', 2: '2(控制, 开火)'}.get(mode, str(mode))
        print(f"  字段: head='SP', mode={mode_str}, yaw={plan_yaw:.4f}rad({plan_yaw*deg:.1f}°), "
              f"pitch={plan_pitch:.4f}rad({plan_pitch*deg:.1f}°), crc16=0x{crc_val:04X}")

        # 离线模式：模拟接收云台反馈数据（GimbalToVision）
        if not ser:
            # 模拟云台四元数（假设云台跟随目标旋转，简化为绕Z轴旋转）
            # 四元数 q = [w, x, y, z]，绕Z轴旋转角度θ: q = [cos(θ/2), 0, 0, sin(θ/2)]
            half_yaw = plan_yaw / 2.0
            qw = math.cos(half_yaw)
            qx = 0.0
            qy = 0.0
            qz = math.sin(half_yaw)
            
            # 模拟子弹速度（使用配置的子弹初速）
            sim_bullet_speed = bullet_speed
            
            # 构造模拟接收帧（GimbalToVision）：帧头 'G','P' (2字节), 
            # 4个四元数 float (qw, qx, qy, qz), 1个子弹速度 float
            recv_frame_without_crc = struct.pack('<2Bfffff', ord('G'), ord('P'),
                                                  qw, qx, qy, qz, sim_bullet_speed)
            recv_crc = calc_crc16(recv_frame_without_crc)
            recv_frame = recv_frame_without_crc + struct.pack('<H', recv_crc)
            
            # 打印模拟接收的数据帧（HEX格式）
            recv_hex = ' '.join(f'{b:02X}' for b in recv_frame)
            print(f"\n[模拟接收]")
            print(f"  HEX: {recv_hex}")
            print(f"  字段: head='GP', 四元数=[{qw:.4f}, {qx:.4f}, {qy:.4f}, {qz:.4f}], "
                  f"子弹速度={sim_bullet_speed:.2f}m/s, crc16=0x{recv_crc:04X}")
            
            # 从四元数反算云台姿态角（简化：仅计算yaw）
            # yaw = 2 * atan2(qz, qw)
            gimbal_yaw = 2.0 * math.atan2(qz, qw)
            print(f"  解析: 云台yaw={gimbal_yaw:.4f}rad({gimbal_yaw*deg:.1f}°)")

        time.sleep(interval)
except KeyboardInterrupt:
    print("\n[INFO] planner_test 已停止。")
finally:
    if ser:
        ser.close()
        print("[INFO] 串口已关闭。")
