# =============================================================================
# CanMV K230 视觉识别 + 激光控制
#   — 仅摄像头检测黑色矩形，通过 UART2 与外部芯片通信
#
# 功能：
#   1. 通过 UART2 接收外部触发信号，启动/停止系统
#   2. 摄像头实时检测黑色矩形目标，计算矩形中心与画面中心的像素偏差
#   3. 通过 UART2_TX 将偏差数据发送给其他芯片
#   4. 矩形中心与目标点重合后 → GPIO 输出信号控制激光发射
#
# 状态机：
#   IDLE ──(收到 START 信号)──▶ TRACKING ──(收到 STOP 信号)──▶ IDLE
#
# 硬件连接：
#   K230              外部设备
#   ────────────────────────────
#   UART2_RX (GPIO12) ←  外部信号源 TX
#   UART2_TX (GPIO11) →  外部芯片 RX
#   GPIO49         →  激光笔控制
#   GND            →  共地
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
TARGET_POINT = (156, 113)            # 目标点坐标（QVGA 画面中心 320x240）
DEADZONE = 5                         # 死区 (pixel)，偏差在此范围内判定重合

# ---------- 检测状态滤波 ----------
detect_counter = 0
lost_counter = 0
min_detect_frames = 2                # 连续检测 2 帧确认"发现"
min_lost_frames = 5                  # 连续丢失 5 帧确认"丢失"
flag_detected = False

# ---------- 激光控制引脚 ----------
LASER_PIN = 49                       # 激光笔控制信号（高电平发射）
LASER_ACTIVE_LEVEL = 1               # 激光发射时引脚电平（1=高电平有效）

# =============================================================================
# 二、外部信号 & 通信配置
# =============================================================================

# ╔══════════════════════════════════════════════════════════════╗
# ║  ※ 外部触发信号定义                                        ║
# ╚══════════════════════════════════════════════════════════════╝
SIGNAL_START  = "01"                 # ★ 收到此信号 → 启动追踪
SIGNAL_STOP   = "00"                 # ★ 收到此信号 → 停止追踪

# ╔══════════════════════════════════════════════════════════════╗
# ║  ※ UART 信号接收/发送配置                                  ║
# ╚══════════════════════════════════════════════════════════════╝
SIGNAL_UART_NUM   = UART.UART2       # ★ 使用 UART2 接收/发送
SIGNAL_UART_BAUD  = 115200           # ★ 波特率（需与外部设备一致）
SIGNAL_UART_RX_PIN = 12              # ★ UART2 RX 引脚（接收触发信号）
SIGNAL_UART_TX_PIN = 11              # ★ UART2 TX 引脚（发送偏差数据）

# ---------- 终端打印限速 ----------
last_print_time = 0

# =============================================================================
# 三、辅助函数
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


def send_data(uart, dx, dy, aligned):
    """
    通过 UART2_TX 向其他芯片发送偏差数据
    格式: "DX:+00123,DY:-00045,ALIGN:0\r\n"
    """
    if uart is None:
        return
    msg = "DX:{:+06d},DY:{:+06d},ALIGN:{:d}\r\n".format(dx, dy, 1 if aligned else 0)
    uart.write(msg)


# =============================================================================
# 四、主程序
# =============================================================================

# ---------- 状态机枚举 ----------
STATE_IDLE     = "IDLE"       # 待机：等待触发信号
STATE_TRACKING = "TRACKING"   # 追踪：检测矩形、发送数据、控制激光

try:
    # ---------- 状态变量 ----------
    current_state = STATE_IDLE
    laser_on = False            # 激光当前状态

    print("=" * 60)
    print("  K230 Visual Detection + Laser Control")
    print("=" * 60)
    print("  Target Point : {}".format(TARGET_POINT))
    print("  Deadzone     : {} px".format(DEADZONE))
    print("  Start Signal : '{}' | Stop Signal: '{}'".format(SIGNAL_START, SIGNAL_STOP))
    print("=" * 60)

    # ---------- 硬件引脚映射 ----------
    fpioa = FPIOA()

    # UART2 引脚（接收外部信号 + 发送数据）
    fpioa.set_function(SIGNAL_UART_RX_PIN, FPIOA.UART2_RXD)
    fpioa.set_function(SIGNAL_UART_TX_PIN, FPIOA.UART2_TXD)

    # 激光控制引脚
    fpioa.set_function(LASER_PIN, FPIOA.GPIO49)

    # ---------- 外部信号 UART ----------
    signal_uart = UART(SIGNAL_UART_NUM, SIGNAL_UART_BAUD)
    print("Signal UART ready: UART2, {} baud".format(SIGNAL_UART_BAUD))

    # ---------- 激光控制引脚 ----------
    laser_pin = Pin(LASER_PIN, Pin.OUT)
    laser_pin.value(0)
    print("Laser control: GPIO{} (active={})".format(
        LASER_PIN, 'HIGH' if LASER_ACTIVE_LEVEL else 'LOW'))

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
    last_print_time = 0

    print("\nSystem ready. Waiting for start signal...")
    print("  -> Send '{}' via UART2 to start.".format(SIGNAL_START))
    print("  -> Send '{}' via UART2 to stop.\n".format(SIGNAL_STOP))

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
                    current_state = STATE_TRACKING
                    detect_counter = 0
                    lost_counter = 0
                    flag_detected = False
                    prev_min_corners = None
                    print("[STATE] IDLE -> TRACKING")
                else:
                    print("[STATE] Already tracking, ignoring START")

            elif signal == SIGNAL_STOP:
                if current_state != STATE_IDLE:
                    current_state = STATE_IDLE
                    laser_pin.value(0)
                    laser_on = False
                    detect_counter = 0
                    lost_counter = 0
                    flag_detected = False
                    prev_min_corners = None
                    print("[STATE] -> IDLE (stopped)")
                else:
                    print("[STATE] Already IDLE, ignoring STOP")

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

        # ========== TRACKING 模式 ==========

        # ---- 图像采集与预处理 ----
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        img_binary = img.to_grayscale(copy=True)
        img_binary = img_binary.binary([black])
        img_binary.dilate(2)
        rects = img_binary.find_rects(threshold=1000)

        # ---- 矩形筛选与验证 ----
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

        # 选面积最小的矩形
        if len(survivors) > 0:
            min_area = float('inf')
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

        # ---- 更新 prev_min_corners ----
        if min_corners is not None and not use_prev:
            prev_min_corners = min_corners
        elif min_corners is None:
            prev_min_corners = None

        # ---- 计算中心偏移 ----
        dx_center = 0
        dy_center = 0

        if min_corners is not None and flag_detected and not use_prev:
            # 绘制矩形框 (绿色)
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

            # 计算像素偏差（画面中心 - 矩形中心）
            dx_center = TARGET_POINT[0] - center_x
            dy_center = TARGET_POINT[1] - center_y

        # ---- 激光控制 ----
        if flag_detected and min_corners is not None:
            aligned = (abs(dx_center) <= DEADZONE and abs(dy_center) <= DEADZONE)
        else:
            aligned = False

        if aligned:
            if not laser_on:
                laser_pin.value(LASER_ACTIVE_LEVEL)
                laser_on = True
        else:
            if laser_on:
                laser_pin.value(0)
                laser_on = False

        # ---- 发送数据到其他芯片 ----
        send_data(signal_uart, int(dx_center), int(dy_center), aligned)

        # ---- 屏幕叠加显示 ----
        img.draw_string_advanced(10, 10, 15,
                                 "fps: {:.1f}".format(clock.fps()),
                                 color=(255, 0, 0))

        state_colors = {
            STATE_IDLE:     (0, 0, 255),
            STATE_TRACKING: (0, 255, 0),
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

        laser_color = (255, 0, 0) if laser_on else (128, 128, 128)
        img.draw_string_advanced(10, 92, 15,
                                 "LASER: {}".format("ON" if laser_on else "OFF"),
                                 color=laser_color)

        # ---- 终端打印（每秒一次）----
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_time) >= 1000:
            last_print_time = now
            print("[{}] track={} | dx={:+d} dy={:+d} | LASER={} | FPS={:.1f}".format(
                  current_state, flag_detected, dx_center, dy_center,
                  'ON' if laser_on else 'OFF', clock.fps()))

        # ---- 显示输出 ----
        img.compressed_for_ide()
        Display.show_image(img, x=(800 - 320) // 2, y=(480 - 240) // 2)

# =============================================================================
# 五、资源清理
# =============================================================================
finally:
    print("Shutting down...")

    try:
        laser_pin.value(0)
        print("Laser stopped.")
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
