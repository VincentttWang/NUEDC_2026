"""Finite WAGA hotspot connection probe for CanMV K230."""

import network
import time


SSID = "WAGA"
PASSWORD = None
TIMEOUT_S = 15

wlan = network.WLAN(network.STA_IF)
if wlan.isconnected():
    wlan.disconnect()
    time.sleep(1)

access_points = wlan.scan(SSID)
print("WIFI_AP_COUNT", len(access_points))
if not access_points:
    raise RuntimeError("WAGA hotspot was not found")

access_point = max(access_points, key=lambda item: item.rssi)
print("WIFI_AP", access_point)
print("WIFI_CONNECT_REQUEST", wlan.connect(None, PASSWORD, info=access_point))

started = time.time()
while not wlan.isconnected() and time.time() - started < TIMEOUT_S:
    print("WIFI_STATUS", wlan.status())
    time.sleep(1)

print("WIFI_CONNECTED", wlan.isconnected())
print("WIFI_IFCONFIG", wlan.ifconfig())
if not wlan.isconnected():
    raise RuntimeError("WAGA hotspot connection timed out")
