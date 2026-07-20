import time

from machine import FPIOA
from machine import UART
from machine import Pin

fpioa = FPIOA()
fpioa.set_function(2, FPIOA.GPIO2)
laser = Pin(2, Pin.OUT)
laser.value(1)

while 1:
    pass
