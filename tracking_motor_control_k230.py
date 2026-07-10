# =============================================================================
# CanMV K230 视觉追踪 + ZDT_Y42 闭环步进电机双轴控制
#   v3.1 — Emm 固件自由协议控制电机（不再使用 Modbus）
#
# 功能：
#   1. 通过 UART2 接收外部触发信号，启动/停止系统
#   2. 摄像头实时检测黑色矩形目标，计算矩形中心与画面中心的像素偏差
#   3. 若视野内无矩形 → X 轴电机自动旋转搜索（往复扫描）
#   4. 找到矩形后 → 自由协议串口控制双轴电机 PID 追踪目标
#   5. 矩形中心与目标点重合后 → GPIO 输出信号控制激光笔发射
#
# 工作状态机：
#   IDLE ──(收到触发信号)──▶ SEARCHING ──(找到矩形)──▶ TRACKING
#     ▲                         │                        │
#     │                         │(持续无矩形)              │(矩形丢失)
#     │                         ▼                        ▼
#     ◀────────────(收到停止信号)──────────── SEARCHING ◀──
#
# 硬件连接：
#   K230                  ZDT_Y42 (X轴)     ZDT_Y42 (Y轴)    外部设备
#   ─────────────────────────────────────────────────────────────────
#   UART3_TX (GPIO50) →    RX               RX         (TX→两电机RX并联)
#   UART3_RX (GPIO51) ←    (可不接)           (可不接)    (只发不收)
#   GPIO49             →    ─                 ─                 激光笔控制
#   UART2_RX (GPIO12) ←    ─                 ─                 外部信号源 TX
#   UART2_TX (GPIO11) →    ─                 ─                 外部信号源 RX
#   GND                →    GND              GND              共地
#
#   ZDT_Y42 驱动器需单独 12-32V 供电，K230 与驱动器共地
#   ※ ZDT_Y42 需预先通过驱动器面板设为"速度模式" 和 "串口通讯"
# =============================================================================

import time
import os
import math
from media.sensor import *
from media.display import *
from media.media import *
from machine import FPIOA
from machine import Pin
from machine import UART

# =============================================================================
# 一、全局配置参数（根据实际情况修改）
# =============================================================================

# ---------- 摄像头 & 追踪参数 ----------
sensor = None
black = (2, 126)                     # 黑色阈值 (LAB 色彩空间 L 分量)
TARGET_POINT = (156, 113)            # 目标点坐标（QVGA 画面中心）

# ---------- 检测状态滤波 ----------
detect_counter = 0
lost_counter = 0
min_detect_frames = 2                # 连续检测 2 帧确认"发现"
min_lost_frames = 5                  # 连续丢失 5 帧确认"丢失"
flag_detected = False

# ---------- 激光控制引脚 ----------
LASER_PIN = 49                       # 激光笔控制信号（高电平发射）
LASER_ACTIVE_LEVEL = 1               # 激光发射时引脚电平（1=高电平有效）

# ---------- PID 控制参数 ----------
KP_X = 8.0                           # X 轴比例系数 (RPM/pixel)
KI_X = 0.5                           # X 轴积分系数 (RPM/(pixel*s))
KD_X = 2.0                           # X 轴微分系数 (RPM/(pixel/s))
KP_Y = 8.0                           # Y 轴比例系数 (RPM/pixel)
KI_Y = 0.5                           # Y 轴积分系数 (RPM/(pixel*s))
KD_Y = 2.0                           # Y 轴微分系数 (RPM/(pixel/s))
MAX_INTEGRAL = 200                   # 积分项上限 (RPM)，防止积分饱和
MAX_SPEED_RPM = 300                  # 最大转速 (RPM)，保护电机
MIN_SPEED_RPM = 10                   # 最小转速 (RPM)，低于此值电机可能不转
DEADZONE = 5                         # 死区 (pixel)，偏差在此范围内停止电机

# =============================================================================
# ★★★ ZDT_Y42 串口/Modbus 配置 — 根据实际接线和驱动器设置修改 ★★★
# =============================================================================

# ---------- X 轴电机 (UART3, 地址1) ----------
MOTOR_X_ADDR    = 1                  # Modbus 从站地址（需与驱动器一致）

# ---------- Y 轴电机 (UART3, 地址2) ----------
MOTOR_Y_ADDR    = 2                  # Modbus 从站地址

# ---------- 电机共用 UART3 ----------
MOTOR_UART      = 3                  # UART 端口号（UART0/1 不可用，UART2=信号）
MOTOR_UART_TX   = 50                 # TX 引脚 (UART3_TXD) → 接两个电机的 RX
MOTOR_UART_RX   = 51                 # RX 引脚 (UART3_RXD) → 可不接（只发不收）
MOTOR_UART_BAUD = 115200             # 波特率（需与驱动器一致）

# ╔══════════════════════════════════════════════════════════════╗
# ║  ※ Emm 固件：自由协议 (校验码固定 0x6B)                     ║
# ║    速度模式帧: [Addr][0xF6][Dir][SpdH][SpdL][Accel][Sync][0x6B] ║
# ║    Dir: 00=CW, 01=CCW    Spd: 0000-0BB8 (0-3000 RPM)       ║
# ║    Accel: 00-FF 档位 (0=无加减速, 值越大加速越快)           ║
# ║    Sync: 00=立即执行                                         ║
# ╚══════════════════════════════════════════════════════════════╝
MOTOR_ACCEL_GEAR = 10                # ★ 加速度档位 (0-255)，10=中等加速

# ---------- 搜索模式参数 ----------
SEARCH_SPEED_RPM = 30                # 搜索时 X 轴转速 (RPM)
SEARCH_SWEEP_TIME = 2000             # 单向扫描持续时间 (ms)，到达后反向
SEARCH_Y_SPEED_RPM = 0               # 搜索时 Y 轴转速 (RPM)，0=不调Y轴

# =============================================================================
# ★★★ 外部信号配置 — 修改信号定义请从这里开始 ★★★
# =============================================================================

# ╔══════════════════════════════════════════════════════════════╗
# ║  ※ 外部触发信号定义（修改此处即可更改触发条件）              ║
# ╚══════════════════════════════════════════════════════════════╝
SIGNAL_START  = "01"                 # ★ 收到此信号 → 启动系统进入搜索模式
SIGNAL_STOP   = "00"                 # ★ 收到此信号 → 停止系统回到待机模式

# ╔══════════════════════════════════════════════════════════════╗
# ║  ※ UART 信号接收配置                                       ║
# ╚══════════════════════════════════════════════════════════════╝
SIGNAL_UART_NUM   = UART.UART2       # ★ 使用 UART2 接收外部信号
SIGNAL_UART_BAUD  = 115200           # ★ 波特率（需与外部信号源一致）
SIGNAL_UART_RX_PIN = 12              # ★ UART2 RX 引脚（接收信号）
SIGNAL_UART_TX_PIN = 11              # ★ UART2 TX 引脚（预留，可回传状态）

# ★★★ 外部信号配置 — 修改信号定义请从这里结束 ★★★

# ---------- 终端打印限速 ----------
last_print_time = 0

# =============================================================================
# 二、ZDT_Y42 自由协议函数（Emm 固件）
# =============================================================================

def motor_set_speed(uart, addr, speed_rpm, accel=None):
    """
    Emm 固件速度模式控制（自由协议，校验码固定 0x6B）

    帧格式: [Addr][0xF6][Direction][SpeedH][SpeedL][Accel][Sync][0x6B]

    参数:
        uart:       串口对象
        addr:       电机地址 (1-255)
        speed_rpm:  目标转速 (RPM)，范围 0~3000
                    正值=CCW，负值=CW，0=停止
        accel:      加速度档位 0-255，默认使用 MOTOR_ACCEL_GEAR
                    0=无曲线直接启动，值越大加速越快
    """
    if accel is None:
        accel = MOTOR_ACCEL_GEAR

    # 方向映射：符号决定方向
    if speed_rpm > 0:
        direction = 0x01      # CCW
    elif speed_rpm < 0:
        direction = 0x00      # CW
        speed_rpm = -speed_rpm
    else:
        direction = 0x00
        speed_rpm = 0

    speed_rpm = min(int(speed_rpm), 3000)

    frame = bytes([
        addr,                    # 字节0: 电机地址
        0xF6,                    # 字节1: 功能码 = 速度模式
        direction,               # 字节2: 方向 (00=CW, 01=CCW)
        (speed_rpm >> 8) & 0xFF, # 字节3: 速度高字节
        speed_rpm & 0xFF,        # 字节4: 速度低字节
        accel & 0xFF,            # 字节5: 加速度档位
        0x00,                    # 字节6: 同步标志 = 立即执行
        0x6B,                    # 字节7: 校验码 = 固定 0x6B
    ])
    uart.write(frame)


def motor_stop(uart, addr):
    """
    立即停止电机（FE 98 命令）
    帧格式: [Addr][0xFE][0x98][0x00][0x6B]
    """
    frame = bytes([addr, 0xFE, 0x98, 0x00, 0x6B])
    uart.write(frame)


# =============================================================================
# 三、初始化函数
# =============================================================================

def motors_init():
    """
    初始化电机 UART 端口（两个电机共用 UART3，通过地址区分）
    返回: motor_uart
    """
    motor_uart = UART(MOTOR_UART, MOTOR_UART_BAUD)
    return motor_uart


# =============================================================================
# 四、外部信号接收
# =============================================================================

def check_signal(uart):
    """
    检查 UART 接收缓冲区，返回收到的完整字符串
    无数据时返回 None
    """
    if uart is None:
        return None
    if uart.any():
        data = uart.read()
        if data:
            try:
                return data.decode('utf-8').strip()
            except:
                return None
    return None


# =============================================================================
# 五、辅助函数（视觉处理）
# =============================================================================

def vector_angle_diff(v1, v2):
    """计算两个向量之间的夹角（单位：度）"""
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    det = v1[0] * v2[1] - v1[1] * v2[0]
    angle = math.atan2(det, dot) * (180 / math.pi)
    return abs(angle)


def get_line_intersection(line1, line2):
    """计算两条直线的交点（克拉默法则）"""
    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2

    A1 = y2 - y1
    B1 = x1 - x2
    C1 = A1 * x1 + B1 * y1

    A2 = y4 - y3
    B2 = x3 - x4
    C2 = A2 * x3 + B2 * y3

    det = A1 * B2 - A2 * B1

    if det == 0:
        return ((x1 + x3) / 2, (y1 + y3) / 2)
    else:
        x = (B2 * C1 - B1 * C2) / det
        y = (A1 * C2 - A2 * C1) / det
        return (x, y)


# =============================================================================
# 六、主程序
# =============================================================================

# ---------- 状态机枚举 ----------
STATE_IDLE      = "IDLE"       # 待机：等待触发信号
STATE_SEARCHING = "SEARCHING"  # 搜索：X 轴旋转扫描寻找矩形
STATE_TRACKING  = "TRACKING"   # 追踪：正常比例控制追踪

try:
    # ---------- 状态变量 ----------
    current_state = STATE_IDLE
    sweep_direction = 1         # 搜索方向 (1=正转, -1=反转)，用 RPM 符号表示
    sweep_timer = 0             # 搜索换向计时器 (ms)
    laser_on = False            # 激光当前状态

    # ---------- PID 状态变量 ----------
    integral_x = 0.0            # X 轴积分累加器
    integral_y = 0.0            # Y 轴积分累加器
    prev_error_x = 0.0          # X 轴上一帧偏差（用于微分）
    prev_error_y = 0.0          # Y 轴上一帧偏差（用于微分）
    last_pid_time = 0           # PID 上次计算时间 (ms)

    print("=" * 60)
    print("  K230 Visual Tracking + ZDT_Y42 Motor Control v3.0")
    print("  (Emm Firmware, Free Protocol 0x6B)")
    print("=" * 60)
    print("  Target Point : {}".format(TARGET_POINT))
    print("  Deadzone     : {} px".format(DEADZONE))
    print("  PID: KP_X={}, KI_X={}, KD_X={}".format(KP_X, KI_X, KD_X))
    print("       KP_Y={}, KI_Y={}, KD_Y={}".format(KP_Y, KI_Y, KD_Y))
    print("  Integral Limit : {} RPM".format(MAX_INTEGRAL))
    print("  Start Signal : '{}' | Stop Signal: '{}'".format(SIGNAL_START, SIGNAL_STOP))
    print("  Search Speed : {} RPM".format(SEARCH_SPEED_RPM))
    print("=" * 60)

    # ---------- 硬件引脚映射 ----------
    fpioa = FPIOA()

    # 按键引脚
    fpioa.set_function(53, FPIOA.GPIO53)

    # UART2 引脚（接收外部信号）
    fpioa.set_function(SIGNAL_UART_RX_PIN, FPIOA.UART2_RXD)
    fpioa.set_function(SIGNAL_UART_TX_PIN, FPIOA.UART2_TXD)

    # 电机 UART3 引脚（两电机共用）
    fpioa.set_function(MOTOR_UART_TX, FPIOA.UART3_TXD)
    fpioa.set_function(MOTOR_UART_RX, FPIOA.UART3_RXD)

    # 激光控制引脚
    fpioa.set_function(LASER_PIN, FPIOA.GPIO49)

    # ---------- 按键 ----------
    key = Pin(53, Pin.IN, Pin.PULL_DOWN)

    # ---------- 外部信号 UART ----------
    signal_uart = UART(SIGNAL_UART_NUM, SIGNAL_UART_BAUD)
    print("Signal UART ready: UART2, {} baud".format(SIGNAL_UART_BAUD))

    # ---------- 激光控制引脚 ----------
    laser_pin = Pin(LASER_PIN, Pin.OUT)
    laser_pin.value(0)
    print("Laser control: GPIO{} (active={})".format(
        LASER_PIN, 'HIGH' if LASER_ACTIVE_LEVEL else 'LOW'))

    # ---------- 电机 UART 初始化 ----------
    motor_uart = motors_init()
    print("Motor UART ready: UART3, {} baud (X=addr{}, Y=addr{})".format(
        MOTOR_UART_BAUD, MOTOR_X_ADDR, MOTOR_Y_ADDR))
    print("  Protocol: Free Protocol (0x6B) | Accel Gear: {}".format(MOTOR_ACCEL_GEAR))
    print("  Speed cmd: [Addr][0xF6][Dir][SpdH][SpdL][Accel][Sync][0x6B]")

    # ---------- 摄像头初始化 ----------
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(Sensor.QVGA)       # 320x240
    sensor.set_pixformat(Sensor.RGB565)
    time.sleep(1)

    # ---------- 显示屏初始化 ----------
    Display.init(Display.ST7701, width=800, height=480, to_ide=True)
    MediaManager.init()
    sensor.run()
    clock = time.clock()

    # ---------- 追踪状态变量 ----------
    prev_min_corners = None
    prev_has_rect = False

    last_print_time = 0
    sweep_timer = time.ticks_ms()

    print("\nSystem ready. Waiting for start signal...")
    print("  -> Send '{}' via UART2 to start.".format(SIGNAL_START))
    print("  -> Send '{}' via UART2 to stop.".format(SIGNAL_STOP))
    print("  -> Press button (GPIO53) to force toggle.\n")

    # =====================================================================
    # 主循环
    # =====================================================================
    while True:
        # ---- 外部信号检测 ----
        signal = check_signal(signal_uart)
        if signal is not None:
            print("[SIGNAL] Received: '{}'".format(signal))

            if signal == SIGNAL_START:
                if current_state == STATE_IDLE:
                    current_state = STATE_SEARCHING
                    sweep_direction = 1
                    sweep_timer = time.ticks_ms()
                    print("[STATE] IDLE -> SEARCHING")
                else:
                    print("[STATE] Already active ({}), ignoring START".format(current_state))

            elif signal == SIGNAL_STOP:
                if current_state != STATE_IDLE:
                    current_state = STATE_IDLE
                    motor_stop(motor_uart, MOTOR_X_ADDR)
                    motor_stop(motor_uart, MOTOR_Y_ADDR)
                    laser_pin.value(0)
                    laser_on = False
                    detect_counter = 0
                    lost_counter = 0
                    flag_detected = False
                    prev_min_corners = None
                    integral_x = 0.0
                    integral_y = 0.0
                    prev_error_x = 0.0
                    prev_error_y = 0.0
                    last_pid_time = 0
                    print("[STATE] -> IDLE (stopped)")
                else:
                    print("[STATE] Already IDLE, ignoring STOP")

        # ---- 按键检测（手动强制切换 IDLE <-> SEARCHING）----
        if key.value() == 1:
            while key.value() == 1:
                pass
            time.sleep_ms(20)

            if current_state == STATE_IDLE:
                current_state = STATE_SEARCHING
                sweep_direction = 1
                sweep_timer = time.ticks_ms()
                print("[STATE] IDLE -> SEARCHING (button)")
            else:
                current_state = STATE_IDLE
                motor_stop(motor_uart, MOTOR_X_ADDR)
                motor_stop(motor_uart, MOTOR_Y_ADDR)
                laser_pin.value(0)
                laser_on = False
                detect_counter = 0
                lost_counter = 0
                flag_detected = False
                prev_min_corners = None
                integral_x = 0.0
                integral_y = 0.0
                prev_error_x = 0.0
                prev_error_y = 0.0
                last_pid_time = 0
                print("[STATE] -> IDLE (button)")

        clock.tick()
        os.exitpoint()

        # ---- IDLE 模式 ----
        if current_state == STATE_IDLE:
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            img.draw_string_advanced(10, 10, 15,
                                     "fps: {:.1f}".format(clock.fps()),
                                     color=(255, 0, 0))
            img.draw_string_advanced(10, 30, 18,
                                     "STATE: IDLE",
                                     color=(0, 0, 255))
            img.draw_string_advanced(10, 55, 14,
                                     "Waiting for signal...",
                                     color=(255, 255, 255))
            img.compressed_for_ide()
            Display.show_image(img, x=(800 - 320) // 2, y=(480 - 240) // 2)
            continue

        # ========== 以下为 SEARCHING / TRACKING 模式共享的图像处理 ==========

        # ---- 图像采集与预处理 ----
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        img_binary = img.to_grayscale(copy=True)
        img_binary = img_binary.binary([black])
        img_binary.dilate(2)
        rects = img_binary.find_rects(threshold=1000)

        # ---- 矩形筛选与验证 ----
        min_area = float('inf')
        min_corners = None
        survivors = []

        if rects is not None:
            for rect in rects:
                corners = rect.corners()
                if len(corners) != 4:
                    continue

                current_area = rect.w() * rect.h()
                if current_area < 1500:
                    continue

                # 角度验证
                angles = []
                max_angle_error = 0
                for i in range(4):
                    p0 = corners[(i - 1) % 4]
                    p1 = corners[i]
                    p2 = corners[(i + 1) % 4]
                    vec1 = (p0[0] - p1[0], p0[1] - p1[1])
                    vec2 = (p2[0] - p1[0], p2[1] - p1[1])
                    angle_error = abs(vector_angle_diff(vec1, vec2) - 90)
                    angles.append(angle_error)
                    if angle_error > max_angle_error:
                        max_angle_error = angle_error

                avg_angle_error = sum(angles) / len(angles)
                if max_angle_error > 45 or avg_angle_error > 30:
                    continue

                # 中心区域黑色占比验证
                center = get_line_intersection(
                    [corners[0], corners[2]], [corners[1], corners[3]]
                )
                center_x, center_y = int(center[0]), int(center[1])

                cx_start = max(0, center_x - rect.w() // 4)
                cx_end = min(img.width() - 1, center_x + rect.w() // 4)
                cy_start = max(0, center_y - rect.h() // 4)
                cy_end = min(img.height() - 1, center_y + rect.h() // 4)

                valid_pixels = 0
                total_pixels = 0
                step_size = 7
                for y in range(cy_start, cy_end, step_size):
                    for x in range(cx_start, cx_end, step_size):
                        pixel_value = img_binary.get_pixel(x, y)
                        if isinstance(pixel_value, tuple):
                            pixel_value = pixel_value[0]
                        if pixel_value == 0:
                            valid_pixels += 1
                        total_pixels += 1

                black_ratio = valid_pixels / total_pixels if total_pixels > 0 else 0.0
                if black_ratio < 0.4:
                    continue

                survivors.append((rect, corners))
                if current_area < min_area:
                    min_area = current_area
                    min_corners = corners

        # 选面积最小的矩形
        if len(survivors) > 0:
            min_area = float('inf')
            min_corners = None
            for rect, corners in survivors:
                area = rect.w() * rect.h()
                if area < min_area:
                    min_area = area
                    min_corners = corners
        else:
            min_corners = None

        current_has_rect = min_corners is not None

        # 丢帧补偿
        if not current_has_rect and prev_min_corners is not None:
            min_corners = prev_min_corners
            use_prev = True
        else:
            use_prev = False

        # 检测状态滤波
        if current_has_rect:
            if not flag_detected:
                detect_counter += 1
                if detect_counter >= min_detect_frames:
                    flag_detected = True
                    detect_counter = 0
            else:
                lost_counter = 0
        else:
            if flag_detected:
                lost_counter += 1
                if lost_counter >= min_lost_frames:
                    flag_detected = False
                    lost_counter = 0
            else:
                detect_counter = 0

        # ========== 状态机逻辑 ==========

        if current_state == STATE_SEARCHING and flag_detected:
            current_state = STATE_TRACKING
            integral_x = 0.0
            integral_y = 0.0
            prev_error_x = 0.0
            prev_error_y = 0.0
            last_pid_time = 0
            print("[STATE] SEARCHING -> TRACKING (rect found)")

        if current_state == STATE_TRACKING and not flag_detected and min_corners is None:
            current_state = STATE_SEARCHING
            sweep_direction = 1
            sweep_timer = time.ticks_ms()
            print("[STATE] TRACKING -> SEARCHING (rect lost)")

        # ---- 计算偏差 & 电机控制 ----
        dx_center = 0
        dy_center = 0
        x_speed = 0
        y_speed = 0

        if current_state == STATE_SEARCHING:
            # =============================================================
            # SEARCHING 模式：X 轴往复扫描
            # =============================================================

            now = time.ticks_ms()

            # 到达换向时间 → 反转方向
            if time.ticks_diff(now, sweep_timer) >= SEARCH_SWEEP_TIME:
                sweep_direction = -sweep_direction   # 1 <-> -1
                sweep_timer = now

            # 发送X轴搜索速度
            x_speed = SEARCH_SPEED_RPM * sweep_direction
            motor_set_speed(motor_uart, MOTOR_X_ADDR, x_speed)

            # Y 轴可选微调
            if SEARCH_Y_SPEED_RPM > 0:
                y_speed = SEARCH_Y_SPEED_RPM * sweep_direction
                motor_set_speed(motor_uart, MOTOR_Y_ADDR, y_speed)
            else:
                y_speed = 0
                motor_stop(motor_uart, MOTOR_Y_ADDR)

            # 搜索模式下关激光
            if laser_on:
                laser_pin.value(0)
                laser_on = False

            # 如有矩形则计算偏差仅供显示
            if min_corners is not None and not use_prev:
                diagonal1 = [min_corners[0], min_corners[2]]
                diagonal2 = [min_corners[1], min_corners[3]]
                center = get_line_intersection(diagonal1, diagonal2)
                center_x, center_y = int(center[0]), int(center[1])
                dx_center = TARGET_POINT[0] - center_x
                dy_center = TARGET_POINT[1] - center_y

        elif current_state == STATE_TRACKING:
            # =============================================================
            # TRACKING 模式：PID 速度控制追踪
            # 核心公式: P = error * KP, I = sum(error)*dt * KI, D = delta(error)/dt * KD
            #           speed_rpm = P + I + D
            #           正值=正转, 负值=反转
            # =============================================================

            if min_corners is not None and flag_detected:
                if not use_prev:
                    prev_min_corners = min_corners

                # 绘制矩形框 (绿色)
                if not use_prev:
                    img.draw_line(min_corners[0][0], min_corners[0][1],
                                  min_corners[1][0], min_corners[1][1],
                                  color=(0, 255, 0), thickness=2)
                    img.draw_line(min_corners[1][0], min_corners[1][1],
                                  min_corners[2][0], min_corners[2][1],
                                  color=(0, 255, 0), thickness=2)
                    img.draw_line(min_corners[2][0], min_corners[2][1],
                                  min_corners[3][0], min_corners[3][1],
                                  color=(0, 255, 0), thickness=2)
                    img.draw_line(min_corners[3][0], min_corners[3][1],
                                  min_corners[0][0], min_corners[0][1],
                                  color=(0, 255, 0), thickness=2)

                # 计算矩形中心
                diagonal1 = [min_corners[0], min_corners[2]]
                diagonal2 = [min_corners[1], min_corners[3]]
                center = get_line_intersection(diagonal1, diagonal2)
                center_x, center_y = int(center[0]), int(center[1])

                # 绘制中心点 (黄色)
                img.draw_circle(center_x, center_y, 2, color=(255, 255, 0), thickness=1)

                # 计算像素偏差
                dx_center = TARGET_POINT[0] - center_x
                dy_center = TARGET_POINT[1] - center_y

                # ---- PID 计算：像素偏差 → 目标转速 ----
                now = time.ticks_ms()
                if last_pid_time == 0:
                    dt = 0.03   # 首帧默认 30ms（~30fps）
                else:
                    dt = time.ticks_diff(now, last_pid_time) / 1000.0
                    if dt <= 0:
                        dt = 0.03
                last_pid_time = now

                # ---- X 轴 PID ----
                if abs(dx_center) <= DEADZONE:
                    x_speed = 0
                    integral_x = 0.0          # 死区内清零积分
                else:
                    # P: 比例项
                    p_term_x = dx_center * KP_X
                    # I: 积分项（累加偏差×时间）
                    integral_x += dx_center * dt
                    if integral_x > MAX_INTEGRAL:
                        integral_x = MAX_INTEGRAL
                    elif integral_x < -MAX_INTEGRAL:
                        integral_x = -MAX_INTEGRAL
                    i_term_x = integral_x * KI_X
                    # D: 微分项（偏差变化率）
                    d_term_x = (dx_center - prev_error_x) / dt * KD_X
                    prev_error_x = dx_center
                    # PID 输出
                    x_speed = p_term_x + i_term_x + d_term_x

                # 限幅
                if abs(x_speed) > MAX_SPEED_RPM:
                    x_speed = MAX_SPEED_RPM if x_speed > 0 else -MAX_SPEED_RPM
                elif abs(x_speed) < MIN_SPEED_RPM and x_speed != 0:
                    x_speed = MIN_SPEED_RPM if x_speed > 0 else -MIN_SPEED_RPM

                motor_set_speed(motor_uart, MOTOR_X_ADDR, int(x_speed))

                # ---- Y 轴 PID ----
                if abs(dy_center) <= DEADZONE:
                    y_speed = 0
                    integral_y = 0.0          # 死区内清零积分
                else:
                    # P: 比例项
                    p_term_y = dy_center * KP_Y
                    # I: 积分项
                    integral_y += dy_center * dt
                    if integral_y > MAX_INTEGRAL:
                        integral_y = MAX_INTEGRAL
                    elif integral_y < -MAX_INTEGRAL:
                        integral_y = -MAX_INTEGRAL
                    i_term_y = integral_y * KI_Y
                    # D: 微分项
                    d_term_y = (dy_center - prev_error_y) / dt * KD_Y
                    prev_error_y = dy_center
                    # PID 输出
                    y_speed = p_term_y + i_term_y + d_term_y

                if abs(y_speed) > MAX_SPEED_RPM:
                    y_speed = MAX_SPEED_RPM if y_speed > 0 else -MAX_SPEED_RPM
                elif abs(y_speed) < MIN_SPEED_RPM and y_speed != 0:
                    y_speed = MIN_SPEED_RPM if y_speed > 0 else -MIN_SPEED_RPM

                motor_set_speed(motor_uart, MOTOR_Y_ADDR, int(y_speed))

                # ---- 激光控制 ----
                if abs(dx_center) <= DEADZONE and abs(dy_center) <= DEADZONE:
                    if not laser_on:
                        laser_pin.value(LASER_ACTIVE_LEVEL)
                        laser_on = True
                else:
                    if laser_on:
                        laser_pin.value(0)
                        laser_on = False

            else:
                motor_stop(motor_uart, MOTOR_X_ADDR)
                motor_stop(motor_uart, MOTOR_Y_ADDR)
                x_speed = 0
                y_speed = 0
                integral_x = 0.0
                integral_y = 0.0
                prev_error_x = 0.0
                prev_error_y = 0.0
                if min_corners is None:
                    prev_min_corners = None
                if laser_on:
                    laser_pin.value(0)
                    laser_on = False

        prev_has_rect = current_has_rect

        # ---- 屏幕叠加显示 ----
        img.draw_string_advanced(10, 10, 15,
                                 "fps: {:.1f}".format(clock.fps()),
                                 color=(255, 0, 0))

        state_colors = {
            STATE_IDLE:      (0, 0, 255),
            STATE_SEARCHING: (255, 165, 0),
            STATE_TRACKING:  (0, 255, 0),
        }
        state_color = state_colors.get(current_state, (255, 255, 255))
        img.draw_string_advanced(10, 30, 18,
                                 "STATE: {}".format(current_state),
                                 color=state_color)

        img.draw_string_advanced(10, 52, 15,
                                 "track: {}".format(
                                     "OK" if flag_detected else "---"),
                                 color=(0, 255, 0) if flag_detected
                                 else (255, 0, 0))

        img.draw_string_advanced(10, 72, 15,
                                 "dx: {:+.0f} dy: {:+.0f}".format(
                                     dx_center, dy_center),
                                 color=(255, 255, 255))

        img.draw_string_advanced(10, 92, 15,
                                 "mX: {:+d} RPM  mY: {:+d} RPM".format(
                                     int(x_speed), int(y_speed)),
                                 color=(255, 255, 0))

        laser_color = (255, 0, 0) if laser_on else (128, 128, 128)
        img.draw_string_advanced(10, 112, 15,
                                 "LASER: {}".format("ON" if laser_on else "OFF"),
                                 color=laser_color)

        # ---- 终端打印（每秒一次）----
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_time) >= 1000:
            last_print_time = now
            print("[{}] track={} | dx={:+d} dy={:+d} | X={:+d}RPM Y={:+d}RPM | LASER={} | FPS={:.1f}".format(
                  current_state, flag_detected, dx_center, dy_center,
                  int(x_speed), int(y_speed),
                  'ON' if laser_on else 'OFF', clock.fps()))

        # ---- 显示输出 ----
        img.compressed_for_ide()
        Display.show_image(img, x=(800 - 320) // 2, y=(480 - 240) // 2)

# =============================================================================
# 七、资源清理
# =============================================================================
finally:
    print("Shutting down...")

    try:
        motor_stop(motor_uart, MOTOR_X_ADDR)
        motor_stop(motor_uart, MOTOR_Y_ADDR)
        laser_pin.value(0)
        print("Motors & laser stopped.")
    except:
        pass

    if isinstance(sensor, Sensor):
        sensor.stop()

    try:
        Display.deinit()
    except:
        pass
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
    print("Cleanup done. System ready for sleep.")
