import struct
import time
import sys
import math
import serial
import configparser
import os

# ==== 加载配置文件 ====
config_path = os.path.join(os.path.dirname(__file__), "config.ini")
config = configparser.ConfigParser()
if not config.read(config_path, encoding='utf-8'):
    print(f"配置文件 {config_path} 未找到或无法读取，请确保文件存在并使用 UTF-8 编码。")
    exit(1)

# 获取配置 [GimbalResponseTest] 部分的参数
section = "GimbalResponseTest"
if section not in config:
    print(f"配置文件中缺少 [{section}] 部分。")
    exit(1)

# 串口参数
port = config.get(section, "port", fallback=None)
baudrate = config.getint(section, "baudrate", fallback=115200)

# 控制信号模式
mode = config.get(section, "mode", fallback="triangle")
# 如需更改模式，可以通过命令行参数覆盖配置文件
if len(sys.argv) > 1:
    mode = sys.argv[1]

# 控制参数
delta_angle = config.getfloat(section, "delta_angle", fallback=8.0)
triangle_period = config.getfloat(section, "triangle_period", fallback=0.2)
step_interval = config.getfloat(section, "step_interval", fallback=3.0)

# 圆形轨迹参数
circle_yaw_amp = config.getfloat(section, "circle_yaw_amp", fallback=7.0)
circle_pitch_amp = config.getfloat(section, "circle_pitch_amp", fallback=7.0)
circle_period = config.getfloat(section, "circle_period", fallback=4.0)

# 将度转换为弧度
deg2rad = lambda deg: deg * 3.1415926 / 180.0

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

# 初始化串口
ser = None
offline_mode = False
if port:
    try:
        ser = serial.Serial(port, baudrate, timeout=0.001)
        print(f"[INFO] 串口已打开: {port} 波特率 {baudrate}, 模式={mode}")
    except Exception as e:
        print(f"[WARN] 无法打开串口 {port}: {e}")
        print("[INFO] 将运行在离线模式（dry-run）：组包、打印HEX、模拟接收数据。")
        ser = None
        offline_mode = True
else:
    print("[WARN] 配置未提供串口端口，将运行在离线模式（dry-run）。")
    ser = None
    offline_mode = True

def generate_mock_response(yaw_cmd_rad, pitch_cmd_rad):
    """
    生成模拟的云台响应数据帧
    假设响应帧格式: 'G','R' (2字节帧头) + 4个float四元数(q0,q1,q2,q3) + 1个float子弹速度 + CRC16(2字节)
    总共: 2 + 4*4 + 4 + 2 = 24 字节
    """
    # 模拟四元数：根据命令的yaw和pitch生成简单的四元数
    # 简化：假设roll=0，只有yaw和pitch旋转
    # 四元数计算: q = qz * qy (先绕Y轴pitch，再绕Z轴yaw)
    half_yaw = yaw_cmd_rad / 2.0
    half_pitch = pitch_cmd_rad / 2.0
    
    # 绕Z轴yaw的四元数: [cos(yaw/2), 0, 0, sin(yaw/2)]
    # 绕Y轴pitch的四元数: [cos(pitch/2), 0, sin(pitch/2), 0]
    # 组合四元数 (简化计算)
    q0 = math.cos(half_yaw) * math.cos(half_pitch)
    q1 = math.cos(half_yaw) * math.sin(half_pitch)
    q2 = math.sin(half_yaw) * math.cos(half_pitch)
    q3 = math.sin(half_yaw) * math.sin(half_pitch)
    
    # 模拟子弹速度 (m/s)，这里假设固定值加一点随机波动
    bullet_speed = 18.0 + 0.5 * math.sin(time.time() * 2)
    
    # 构造响应帧（不含CRC16）
    response_without_crc = struct.pack(
        "<2cfffff",
        b"G",
        b"R",
        q0, q1, q2, q3,
        bullet_speed
    )
    
    # 计算CRC16
    crc_val = calc_crc16(response_without_crc)
    response_frame = response_without_crc + struct.pack("<H", crc_val)
    
    return response_frame, q0, q1, q2, q3, bullet_speed

def parse_and_print_response(frame, q0, q1, q2, q3, bullet_speed):
    """解析并打印模拟响应帧"""
    hex_str = " ".join(f"{b:02X}" for b in frame)
    print(f"  [模拟接收] 响应帧 HEX: {hex_str}")
    print(f"  [解析] 四元数: q0={q0:.4f}, q1={q1:.4f}, q2={q2:.4f}, q3={q3:.4f}")
    print(f"  [解析] 子弹速度: {bullet_speed:.2f} m/s")
    
    # 从四元数反推yaw和pitch角度（简化计算）
    # yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2))
    # pitch = asin(2*(q0*q1 - q2*q3))
    yaw_rad = math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
    pitch_rad = math.asin(max(-1.0, min(1.0, 2*(q0*q1 - q2*q3))))  # 限制在[-1,1]
    
    yaw_deg = yaw_rad * 180.0 / 3.1415926
    pitch_deg = pitch_rad * 180.0 / 3.1415926
    
    print(f"  [解析] 姿态角: Yaw={yaw_deg:.2f}°, Pitch={pitch_deg:.2f}°")
    print()

# 初始化命令参数
yaw_cmd = 0.0    # 当前命令 Yaw 角（度）
pitch_cmd = 0.0  # 当前命令 Pitch 角（度）
count = 0        # 帧计数器（用于控制时序）
t = 0.0          # 时间计数（秒，用于圆形轨迹）
last_step_time = time.time()

print("[INFO] 开始发送数据帧... 按 Ctrl+C 可停止。")
try:
    while True:
        # 根据模式更新命令值
        if mode == "triangle":
            # 三角波：从0逐步增至 delta_angle，然后跳回0重复
            # 计算总步数（帧数） = 三角波周期 * 发送频率(约100Hz)
            step_count = int(triangle_period * 100)  # 简化假设发送频率100Hz
            step = delta_angle / max(step_count, 1)
            if count >= step_count:
                yaw_cmd = 0.0
                count = 0
            else:
                yaw_cmd += step
                if yaw_cmd > delta_angle:
                    yaw_cmd = delta_angle
                count += 1
            pitch_cmd = 0.0  # 仅绕Yaw轴测试

        elif mode == "step":
            # 阶跃：每隔 step_interval 切换一次目标角度，累积增加 delta_angle
            if time.time() - last_step_time >= step_interval:
                yaw_cmd += delta_angle
                last_step_time = time.time()
            pitch_cmd = 0.0

        elif mode == "circle":
            # 圆形轨迹：Yaw 和 Pitch 按正弦函数变化，相位相差90度，形成圆周运动
            yaw_deg = circle_yaw_amp * math.sin(2 * 3.1415926 * t / circle_period)
            pitch_deg = circle_pitch_amp * math.sin(2 * 3.1415926 * t / circle_period + 3.1415926/2) + 18.0
            yaw_cmd = yaw_deg
            pitch_cmd = pitch_deg
            # 时间步进
            t += 0.005  # 5ms 步长模拟约200Hz控制频率

        else:
            print(f"[WARN] Unknown mode '{mode}', default to triangle.")
            mode = "triangle"
            continue

        # 准备数据包 (VisionToNucFrame 格式: 'S','P', mode, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc, game_status, crc16)
        # 控制模式: 1=控制不开火; 本测试不开火
        control_mode = 1
        # 将当前命令角度转换为弧度，速度和加速度简单设为0（或根据需要计算）
        yaw_rad = deg2rad(yaw_cmd)
        pitch_rad = deg2rad(pitch_cmd)
        yaw_vel = 0.0
        yaw_acc = 0.0
        pitch_vel = 0.0
        pitch_acc = 0.0
        game_status = 1  # 游戏状态标志(0:未开始,1:进行中)，此处默认设为1

        # 构造数据帧（不含CRC16），按照小端序打包
        # 帧头 'S','P' (2字节), mode (1字节), 后续依次为6个 float (yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc), game_status (1字节)
        frame_without_crc = struct.pack(
            "<2cBffffffB",
            b"S",
            b"P",
            control_mode,
            yaw_rad,
            yaw_vel,
            yaw_acc,
            pitch_rad,
            pitch_vel,
            pitch_acc,
            game_status,
        )
        # 计算CRC16校验值
        crc_val = calc_crc16(frame_without_crc)
        # 将CRC16附加到帧末尾（小端序2字节）
        frame = frame_without_crc + struct.pack("<H", crc_val)

        # 通过串口发送数据帧
        if ser:
            try:
                ser.write(frame)
            except Exception as e:
                print(f"[ERROR] 串口发送错误: {e}")
                # 发送失败则跳过本次循环（仍继续后续发送）

        # 将发送的数据帧以 HEX 格式和各字段值打印输出
        hex_str = " ".join(f"{b:02X}" for b in frame)
        print(f"[发送] 命令帧 HEX: {hex_str}")
        mode_str = {
            0: "0(不控制)",
            1: "1(控制, 不开火)",
            2: "2(控制, 开火)",
        }.get(control_mode, str(control_mode))
        print(
            f"[发送] 帧字段: head='SP', mode={mode_str}, yaw={yaw_rad:.4f}rad({yaw_cmd:.2f}°), "
            f"yaw_vel={yaw_vel:.4f}, yaw_acc={yaw_acc:.4f}, "
            f"pitch={pitch_rad:.4f}rad({pitch_cmd:.2f}°), pitch_vel={pitch_vel:.4f}, "
            f"pitch_acc={pitch_acc:.4f}, game_status={game_status}, crc16=0x{crc_val:04X}"
        )
        
        # 离线模式：生成并打印模拟的响应数据
        if offline_mode:
            response_frame, q0, q1, q2, q3, bullet_speed = generate_mock_response(yaw_rad, pitch_rad)
            parse_and_print_response(response_frame, q0, q1, q2, q3, bullet_speed)

        time.sleep(0.01)  # 控制发送频率 ~100Hz
except KeyboardInterrupt:
    print("\n[INFO] gimbal_response_test 已停止。")
finally:
    if ser:
        ser.close()
        print("[INFO] 串口已关闭。")
