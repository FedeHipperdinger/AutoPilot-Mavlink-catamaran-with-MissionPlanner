

from machine import Pin, Timer, ADC, UART
import time

led = Pin(25, Pin.OUT)
#timer = Timer()

# tx_pin = Pin(0, Pin.OUT)
# rx_pin = Pin(1, Pin.IN)

# Configurar la UART
#uart0 = UART(0, baudrate=57600, tx=tx_pin, rx=rx_pin)
uart0 = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
uart0.init(bits=8, parity=None, stop=1)
#tx_pin.on()

# Use a timer for sending packets
#timer.init(freq=1, mode=Timer.PERIODIC, callback=sendFloats)

# Keep looping to receive data
while True:
    
    #print(uart0.read())
    time.sleep(0.1)
    uart0.write(b'a')
    #tx_pin.toggle()
