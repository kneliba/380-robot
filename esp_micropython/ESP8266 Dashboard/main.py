from machine import Pin, I2C
import sys
import time
import os
import webrepl_setup
# ------------SETUP--------------
# CONSTANTS
DEBUG = True

# construct an I2C bus
i2c = I2C(scl=Pin(5), sda=Pin(4), freq=100000)

# set led pin on ESP
led = Pin(2, Pin.OUT)
# -------------------------------


def i2c_read(address, buffer):
    i2c.readfrom_into(address, buffer)
    size = sys.getsizeof(buffer)
    if DEBUG:
        # print can't use f-strings for some reason
        print("{size} bytes read from address '{address}', payload is: {buffer}\n".format(size, address, buffer))
    return buffer


def i2c_write(address, buffer):
    if DEBUG:
        print("Buffer to send to address '{address}': {buffer}\n".format(address, buffer))
    i2c.writeto(address, buffer)

f = open('data.txt', 'w')
f.write('some data')
f.close()

while True:
    led.value(not led.value())
    time.sleep(0.5)
    f = open('data.txt')
    print(f.read())
    f.close()

# def main():
# for arg in sys.argv:
#     print(arg)

# if __name__ == '__main__':
#     main()

# def web_page():
#     if led.value() == 0:
#         gpio_state = "ON"
#     else:
#         gpio_state = "OFF"

#     html = """<html><head> <title>ESP Web Server</title> <meta name="viewport" content="width=device-width, initial-scale=1">
#   <link rel="icon" href="data:,"> <style>html{font-family: Helvetica; display:inline-block; margin: 0px auto; text-align: center;}
#   h1{color: #0F3376; padding: 2vh;}p{font-size: 1.5rem;}.button{display: inline-block; background-color: #e7bd3b; border: none; 
#   border-radius: 4px; color: white; padding: 16px 40px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}
#   .button2{background-color: #4286f4;}</style></head><body> <h1>ESP Web Server</h1> 
#   <p>GPIO state: <strong>""" + gpio_state + """</strong></p><p><a href="/?led=on"><button class="button">ON</button></a></p>
#   <p><a href="/?led=off"><button class="button button2">OFF</button></a></p></body></html>"""
#     return html


# def http_get(url):
#     import socket
#     _, _, host, path = url.split('/', 3)
#     addr = socket.getaddrinfo(host, 80)[0][-1]
#     s = socket.socket()
#     s.connect(addr)
#     s.send(bytes('GET /%s HTTP/1.0\r\nHost: %s\r\n\r\n' % (path, host), 'utf8'))
#     while True:
#         data = s.recv(100)
#         if data:
#             print(str(data, 'utf8'), end='')
#         else:
#             break
#     s.close()


# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.bind(('', 80))
# s.listen(5)

# while True:
#     conn, addr = s.accept()
#     print('Got a connection from %s' % str(addr))
#     request = conn.recv(1024)
#     request = str(request)
#     print('Content = %s' % request)
#     led_on = request.find('/?led=on')
#     led_off = request.find('/?led=off')
#     if led_on == 6:
#         print('LED ON')
#         led.value(0)
#     if led_off == 6:
#         print('LED OFF')
#         led.value(1)
#     response = web_page()
#     conn.send('HTTP/1.1 200 OK\n')
#     conn.send('Content-Type: text/html\n')
#     conn.send('Connection: close\n\n')
#     conn.sendall(response)
#     conn.close()
