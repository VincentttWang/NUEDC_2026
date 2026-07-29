# K230 UART1 communication test for MSPM0 Device Check.
# Wiring: K230 GPIO3/UART1_TXD -> MSPM0 K230 RX, K230 GPIO4/UART1_RXD -> MSPM0 K230 TX,
# and common GND. Both sides use 3.3 V logic, 115200 8N1.
import time

from machine import FPIOA
from machine import UART

K230_UART_TX_PIN = 11
K230_UART_RX_PIN = 12
K230_UART_BAUDRATE = 115200

FRAME_START = 0x12
FRAME_END = 0x5B

IMAGE_CENTER_X = 160
IMAGE_CENTER_Y = 120


def uart1_init():
    fpioa = FPIOA()
    fpioa.set_function(K230_UART_TX_PIN, fpioa.UART2_TXD)
    fpioa.set_function(K230_UART_RX_PIN, fpioa.UART2_RXD)

    return UART(UART.UART2,
                baudrate=K230_UART_BAUDRATE,
                bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE)


def append_u16_be(frame, value):
    value = max(0, min(65535, int(value)))
    frame.append((value >> 8) & 0xFF)
    frame.append(value & 0xFF)


def build_test_frame(counter):
    target_x = 100 + (counter % 80)
    target_y = 60 + ((counter * 3) % 80)
    rect_corners = (
        (20, 220),
        (300, 220),
        (300, 20),
        (20, 20),
    )

    frame = bytearray()
    frame.append(FRAME_START)
    frame.append(counter & 0xFF)

    # Laser segment: target_x, target_y, laser_x, laser_y.
    append_u16_be(frame, target_x)
    append_u16_be(frame, target_y)
    append_u16_be(frame, IMAGE_CENTER_X)
    append_u16_be(frame, IMAGE_CENTER_Y)

    for x, y in rect_corners:
        append_u16_be(frame, x)
        append_u16_be(frame, y)

    frame.append(FRAME_END)
    return frame, target_x, target_y


def main():
    uart = uart1_init()
    counter = 0
    print("K230 UART1 communication test started: GPIO3 TX, GPIO4 RX, 115200 8N1")

    while True:
        frame, target_x, target_y = build_test_frame(counter)
        uart.write(frame)
        print("tx frame=%d target=(%d,%d)" % (counter, target_x, target_y))
        counter = (counter + 1) & 0xFF
        time.sleep_ms(100)


if __name__ == "__main__":
    main()
