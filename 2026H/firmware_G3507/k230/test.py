import os
import sys
import time
import urandom

import network

from media.display import *
from media.media import *
from libs.WBCRtsp import WBCRtsp

DISPLAY_WIDTH = ALIGN_UP(1920, 16)
DISPLAY_HEIGHT = 1080

nic = None

#WIFI连接函数
def WIFI_Connect(ssid, passwd = None):
    global nic

    wlan = network.WLAN(network.STA_IF) #STA模式

    # RT-Smart 的 WLAN 接口始终处于 active 状态。先清理上一次失败的
    # 关联，避免紧接着运行脚本时 connect() 直接失败。
    if wlan.isconnected():
        wlan.disconnect()
        time.sleep(1)

    start_time=time.time() #记录时间做超时判断

    if not wlan.isconnected():
        print('connecting to network...')

        #输入WIFI账号密码（仅支持2.4G信号）, 连接超过5秒为超时
        access_points = wlan.scan(ssid)
        if not access_points:
            print("target SSID not found:", ssid)
            return False
        access_point = max(access_points, key=lambda item: item.rssi)
        print("selected access point:", access_point)
        print("connect request:", wlan.connect(None, passwd, info=access_point))

        while not wlan.isconnected():
            print("wifi status:", wlan.status())
            time.sleep(1)
            #超时判断,10秒没连接成功判定为超时
            if time.time()-start_time > 15 :
                print('WIFI Connected Timeout!')
                break

    if wlan.isconnected(): #连接成功
        print('connect success')
        #等待获取IP地址
        while wlan.ifconfig()[0] == '0.0.0.0':
            pass
        #串口打印信息
        print('network information:', wlan.ifconfig())

        nic = wlan
        return True
    else: #连接失败
        print("Connect wifi failed.")
        return False

#有线以太网连接函数
def Lan_Connect():
    global nic
    lan = network.LAN()

    if lan.isconnected(): #连接成功
        print('network information:', lan.ifconfig())

        nic = lan
    else: #连接失败
        print("Connect lan failed.")

def display_test():
    global nic

    print(f"virtual wbc rtsp stream on rtsp://{nic.ifconfig()[0]}:8554/test")

    # create image for drawing
    img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)

    # use lcd as display output
    Display.init(Display.VIRT, width = DISPLAY_WIDTH, height = DISPLAY_HEIGHT, fps = 60,to_ide=False)
    # init wbc
    WBCRtsp.configure(wbc_width=DISPLAY_WIDTH,wbc_height=DISPLAY_HEIGHT)
    # 启用wbc编码推流
    WBCRtsp.start()

    try:
        while True:
            img.clear()
            for i in range(10):
                x = (urandom.getrandbits(11) % img.width())
                y = (urandom.getrandbits(11) % img.height())
                r = (urandom.getrandbits(8))
                g = (urandom.getrandbits(8))
                b = (urandom.getrandbits(8))
                size = (urandom.getrandbits(30) % 64) + 32
                # If the first argument is a scaler then this method expects
                # to see x, y, and text. Otherwise, it expects a (x,y,text) tuple.
                # Character and string rotation can be done at 0, 90, 180, 270, and etc. degrees.
                img.draw_string_advanced(x,y,size, "Hello World!，你好世界！！！", color = (r, g, b),)

            # draw result to screen
            Display.show_image(img)

            time.sleep(1)
            os.exitpoint()
    except KeyboardInterrupt as e:
        print("user stop: ", e)
    except BaseException as e:
        import sys
        sys.print_exception(e)

    WBCRtsp.stop()  # stop wbc
    # deinit display
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)

if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)

    #执行WIFI连接函数
    if not WIFI_Connect("WAGA"):
        raise RuntimeError("Wi-Fi connection failed; RTSP server not started")

    #执行以太网连接函数
    # Lan_Connect()

    display_test()
