"""CanMV K230 camera-to-RTSP diagnostic stream for the WAGA hotspot."""

import _thread
import multimedia as mm
import network
import os
import time

from media.media import *
from media.sensor import *
from media.vencoder import *


WIFI_SSID = "WAGA"
WIFI_PASSWORD = None
RTSP_SESSION = "test"
RTSP_PORT = 8554
STREAM_WIDTH = 640
STREAM_HEIGHT = 360
STREAM_FPS = 60
STREAM_BIT_RATE = 1000
STREAM_GOP = 10
SENSOR_WIDTH = 1920
SENSOR_HEIGHT = 1080
SENSOR_FPS = 60
WIFI_CONNECT_TIMEOUT_S = 15
WIFI_RETRY_DELAY_S = 3


def connect_wifi(ssid, password, timeout_s=WIFI_CONNECT_TIMEOUT_S):
    sta = network.WLAN(network.STA_IF)
    while not sta.isconnected():
        access_points = sta.scan(ssid)
        if not access_points:
            print("Wi-Fi not found, retrying:", ssid)
            time.sleep(WIFI_RETRY_DELAY_S)
            continue

        access_point = max(access_points, key=lambda item: item.rssi)
        print("selected access point:", access_point)
        try:
            print("connect request:",
                  sta.connect(None, password, info=access_point))
        except Exception as error:
            print("Wi-Fi connect request failed:", error)
            time.sleep(WIFI_RETRY_DELAY_S)
            continue

        started = time.time()
        while not sta.isconnected() and time.time() - started <= timeout_s:
            time.sleep(1)
        if not sta.isconnected():
            print("Wi-Fi connection timed out, retrying")
            time.sleep(WIFI_RETRY_DELAY_S)

    print("network information:", sta.ifconfig())
    return sta


class CameraRtspServer:
    def __init__(self, logger=print):
        self.server = mm.rtsp_server()
        self.logger = logger
        self.streaming = False
        self.thread_over = False
        self.frame_count = 0
        self.byte_count = 0
        self.started_ms = time.ticks_ms()

    def start(self):
        width = ALIGN_UP(STREAM_WIDTH, 16)
        self.sensor = Sensor(
            width=SENSOR_WIDTH,
            height=SENSOR_HEIGHT,
            fps=SENSOR_FPS,
        )
        self.sensor.reset()
        self.sensor.set_framesize(
            width=width,
            height=STREAM_HEIGHT,
            alignment=12,
        )
        self.sensor.set_pixformat(Sensor.YUV420SP)

        self.encoder = Encoder()
        self.encoder.SetOutBufs(0, 8, width, STREAM_HEIGHT)
        attributes = ChnAttrStr(
            self.encoder.PAYLOAD_TYPE_H264,
            self.encoder.H264_PROFILE_MAIN,
            width,
            STREAM_HEIGHT,
        )
        # Use a native GC2093 mode, then scale in VICAP before VENC.
        attributes.src_frame_rate = SENSOR_FPS
        attributes.dst_frame_rate = STREAM_FPS
        attributes.bit_rate = STREAM_BIT_RATE
        attributes.gop_len = STREAM_GOP
        self.encoder.Create(0, attributes)
        self.link = MediaManager.link(
            self.sensor.bind_info()["src"],
            (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, 0),
        )

        self.server.rtspserver_init(RTSP_PORT)
        self.server.rtspserver_createsession(
            RTSP_SESSION,
            mm.multi_media_type.media_h264,
            False,
        )
        self.server.rtspserver_start()
        self.encoder.Start(0)
        self.sensor.run()

        self.streaming = True
        _thread.start_new_thread(self._stream_loop, ())
        self.logger("RTSP server started: %s" %
                    self.server.rtspserver_getrtspurl(RTSP_SESSION))

    def _stream_loop(self):
        stream_data = StreamData()
        frame_count = 0
        try:
            while self.streaming:
                os.exitpoint()
                if self.encoder.GetStream(0, stream_data) != 0:
                    continue
                frame_count += 1
                self.frame_count = frame_count
                for index in range(stream_data.pack_cnt):
                    self.byte_count += stream_data.data_size[index]
                    # The RTSP extension can consume the VENC physical buffer
                    # directly. Avoid copying every packet into a Python bytes
                    # object, which adds CPU time and garbage-collection jitter.
                    self.server.rtspserver_sendvideodata_byphyaddr(
                        RTSP_SESSION,
                        stream_data.phy_addr[index],
                        stream_data.data_size[index],
                        1000,
                    )
                self.encoder.ReleaseStream(0, stream_data)
                if frame_count == 1 or frame_count % 300 == 0:
                    self.logger(
                        "video frames=%d bytes=%d last_pack=%d" %
                        (frame_count, self.byte_count,
                         stream_data.data_size[0]
                         if stream_data.pack_cnt else 0))
        except BaseException as error:
            import sys
            sys.print_exception(error)
        finally:
            self.thread_over = True

    def status(self, nic):
        elapsed_ms = max(1, time.ticks_diff(time.ticks_ms(), self.started_ms))
        return ("STATUS ip=%s wifi=%d rtsp=%s frames=%d bytes=%d fps=%.1f" %
                (nic.ifconfig()[0], nic.isconnected(),
                 self.server.rtspserver_getrtspurl(RTSP_SESSION),
                 self.frame_count, self.byte_count,
                 self.frame_count * 1000.0 / elapsed_ms))

    def stop(self):
        self.streaming = False
        started = time.time()
        while not self.thread_over and time.time() - started < 2:
            time.sleep_ms(20)
        try:
            self.sensor.stop()
        except Exception:
            pass
        try:
            self.encoder.Stop(0)
            self.encoder.Destroy(0)
        except Exception:
            pass
        try:
            self.server.rtspserver_stop()
        except Exception:
            pass


if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)
    nic = connect_wifi(WIFI_SSID, WIFI_PASSWORD)
    rtsp = CameraRtspServer()
    try:
        rtsp.start()
        print("service ready: rtsp://%s:%d/%s" %
              (nic.ifconfig()[0], RTSP_PORT, RTSP_SESSION))
        last_heartbeat = time.ticks_ms()
        while True:
            time.sleep(1)
            os.exitpoint()
            if not nic.isconnected():
                print("Wi-Fi disconnected; reconnecting to", WIFI_SSID)
                nic = connect_wifi(WIFI_SSID, WIFI_PASSWORD)
                print("Wi-Fi reconnected:", nic.ifconfig())
            if time.ticks_diff(time.ticks_ms(), last_heartbeat) >= 5000:
                print(rtsp.status(nic))
                last_heartbeat = time.ticks_ms()
    except KeyboardInterrupt as error:
        print("user stop:", error)
    finally:
        rtsp.stop()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
