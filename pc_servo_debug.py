# -*- coding: utf-8 -*-
# pc_servo_debug.py —— PC 端视觉伺服调试(单口模式)
# 命令从 UART2/PB16 进(单片机 CMD_UART=2 时), 回显/心跳从 UART2/PB15 出,
# 所以一个 COM 口全双工即可: 既发帧又读回显。
#   COM 接线:  USB-TTL TX -> MCU PB16,  USB-TTL RX <- MCU PB15, GND 共地
# 用法: 改 PORT 为你的 COM 号 -> python pc_servo_debug.py
import serial, threading, math, time

PORT = 'COM22'        # 与单片机 UART2 全双工的那一个 COM
BAUD = 115200
PERIOD = 0.05         # 恒定/扫描/方波每帧间隔(s, 必须 < 单片机的300ms超时)
HOLD_SEC = 5.0        # 恒定模式时长
SWEEP_SEC = 20.0      # 正弦扫描时长
SOF0, SOF1 = 0xAA, 0x55

def pack_frame(pe, te):
    pe, te = int(pe), int(te)
    if pe < -32768 or pe > 32767 or te < -32768 or te > 32767:
        raise ValueError("pe/te 超出 int16")
    p, t = pe & 0xFFFF, te & 0xFFFF
    payload = bytes([p & 0xFF, (p >> 8) & 0xFF, t & 0xFF, (t >> 8) & 0xFF])
    cs = payload[0] ^ payload[1] ^ payload[2] ^ payload[3]
    return bytes([SOF0, SOF1]) + payload + bytes([cs])

def reader(ser):
    buf = b''
    while True:
        data = ser.read(64)
        if not data:
            time.sleep(0.01); continue
        buf += data
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            s = line.decode('ascii', errors='ignore').rstrip('\r')
            if s:
                print("   <<", s)

def open_serial():
    ser = serial.Serial(PORT, BAUD, timeout=0.01)
    threading.Thread(target=reader, args=(ser,), daemon=True).start()
    print("已打开 %s @ %d (全双工: 发命令+读回显/心跳)" % (PORT, BAUD))
    return ser

def mode_const(ser):
    print("恒定模式 %.1fs: 持续发同一 (pan,tilt) [自动每50ms一帧]" % HOLD_SEC)
    pe = int(input("pan(0.1°): ")); te = int(input("tilt(0.1°): "))
    end = time.time() + HOLD_SEC
    while time.time() < end:
        ser.write(pack_frame(pe, te)); time.sleep(PERIOD)

def mode_sine(ser):
    amp = int(input("半径(0.1°): ") or 300)
    period = float(input("一圈周期(s): ") or 4)
    print("正弦扫描 %.0fs: 半径%d 周期%.1fs" % (SWEEP_SEC, amp, period))
    end = time.time() + SWEEP_SEC; w = 2*math.pi/period; t0 = time.time()
    while time.time() < end:
        t = time.time() - t0
        ser.write(pack_frame(int(amp*math.sin(w*t)), int(amp*math.cos(w*t))))
        time.sleep(PERIOD)

def mode_square(ser):
    print("方波换向: 在 +幅度/-幅度 之间自动交替, 每50ms重发一帧(保持电机不停)")
    amp = int(input("pan 幅度(0.1°): ") or 30)
    tilt = int(input("tilt 幅度(0.1°): ") or 0)
    half_ms = int(input("每个方向的保持时间(ms, 建议 ≤250, 需<300): ") or 200)
    dur_s = float(input("持续(s): ") or 8)
    end = time.time() + dur_s
    pos = True; t0 = time.time()
    while time.time() < end:
        if (time.time() - t0) >= half_ms/1000.0:
            pos = not pos; t0 = time.time()
        pe = amp if pos else -amp
        te = tilt if pos else -tilt
        ser.write(pack_frame(pe, te))
        time.sleep(PERIOD)

def mode_manual(ser):
    print("手动: 输入 'pe,te' 发一帧; q 返回")
    while True:
        s = input("pe,te > ").strip()
        if s.lower() == 'q': break
        try:
            pe, te = map(int, s.split(','))
            ser.write(pack_frame(pe, te))
        except Exception as e:
            print("格式应为 int,int:", e)

def main():
    try:
        ser = open_serial()
    except Exception as e:
        print("打开串口失败:", e); return
    while True:
        print("\n[1] 固定误差  [2] 正弦扫描  [3] 手动发帧  [4] 方波换向  [q] 退出")
        k = input("> ").strip().lower()
        if k == 'q': break
        elif k == '1':
            try: mode_const(ser)
            except (ValueError, KeyboardInterrupt) as e: print("取消/参数错:", e)
        elif k == '2':
            try: mode_sine(ser)
            except (ValueError, KeyboardInterrupt) as e: print("取消/参数错:", e)
        elif k == '3': mode_manual(ser)
        elif k == '4':
            try: mode_square(ser)
            except (ValueError, KeyboardInterrupt) as e: print("取消/参数错:", e)
        else: print("无效选项")

if __name__ == '__main__':
    try: main()
    except KeyboardInterrupt: print("已退出")
    except serial.SerialException as e: print("串口异常:", e)
