import network
import time


wlan = network.WLAN(network.STA_IF)
wlan.active(True)
time.sleep_ms(500)

print("wifi scan start")
for item in wlan.scan():
    print("network:", item)
    print("fields:", dir(item))
print("wifi scan done")
