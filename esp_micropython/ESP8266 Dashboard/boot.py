try:
  import usocket as socket
except:
  import socket

from machine import Pin
import network

import esp
esp.osdebug(None)

import gc
gc.collect()

ssid = "ESP8266-MTE380-Group-4"
password = "12345678"

ap = network.WLAN(network.AP_IF)
# station = network.WLAN(network.STA_IF)

ap.active(True)
ap.config(essid=ssid, password=password)

# station.active(True)
# station.connect(ssid, password)

# while station.isconnected() == False:
#   pass

# print("Connection successful")
# print(station.ifconfig())

while ap.active() == False:
  pass

print("Connection successful")
print(ap.ifconfig())

led = Pin(2, Pin.OUT)