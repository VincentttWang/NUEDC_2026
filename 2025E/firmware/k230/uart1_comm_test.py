# K230 UART1 communication test for MSPM0 Device Check.
# Wiring: K230 GPIO3/UART1_TXD -> MSPM0 K230 RX, K230 GPIO4/UART1_RXD -> MSPM0 K230 TX,
# and common GND. Both sides use 3.3 V logic, 115200 8N1.
import time

from machine import FPIOA
from machine import UART

K230_UART_TX_PIN = 3
K230_UART_RX_PIN = 4
K230_UART_BAUDRATE = 115200

ANGLE_FRAME_START0 = 0xA5
ANGLE_FRAME_START1 = 0x5A
ANGLE_SCALE = 100


def uart1_init():
    fpioa = FPIOA()
    fpioa.set_function(K230_UART_TX_PIN, fpioa.UART1_TXD)
    fpioa.set_function(K230_UART_RX_PIN, fpioa.UART1_RXD)

    return UART(UART.UART1,
                baudrate=K230_UART_BAUDRATE,
                bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE)


def append_i16_be(frame, value):
    value = max(-32768, min(32767, int(value))) & 0xFFFF
    frame.append((value >> 8) & 0xFF)
    frame.append(value & 0xFF)


def build_angle_test_frame(counter):
    # 伪随机、有符号角度误差：便于验证字节序、符号解析和 OLED 刷新。
    # 本脚本只应用于 Device Check 的 K230 页面；不要在瞄准任务中运行。
    seed = (counter * 1103515245 + 12345) & 0xFFFFFFFF
    yaw_error_deg = ((seed % 3001) - 1500) / 100.0       # -15.00 ~ +15.00 deg
    pitch_error_deg = (((seed >> 16) % 2001) - 1000) / 100.0  # -10.00 ~ +10.00 deg
    frame = bytearray()
    # K230 MicroPython 的 bytearray.extend() 不接受元组，逐字节追加。
    frame.append(ANGLE_FRAME_START0)
    frame.append(ANGLE_FRAME_START1)
    append_i16_be(frame, round(yaw_error_deg * ANGLE_SCALE))
    append_i16_be(frame, round(pitch_error_deg * ANGLE_SCALE))
    frame.append(sum(frame) & 0xFF)
    return frame, yaw_error_deg, pitch_error_deg


def main():
    uart = uart1_init()
    counter = 0
    print("K230 UART1 communication test started: GPIO3 TX, GPIO4 RX, 115200 8N1")

    while True:
        frame, yaw_error_deg, pitch_error_deg = build_angle_test_frame(counter)
        uart.write(frame)
        print("tx angle frame=%d yaw=%.2f pit=%.2f" %
              (counter, yaw_error_deg, pitch_error_deg))
        counter = (counter + 1) & 0xFF
        time.sleep_ms(100)


if __name__ == "__main__":
    main()
