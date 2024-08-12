'''
Proyecto Final: piloto automático para un dron acuático.

Archivo: main.py.

Autor: Federico Hipperdinger.
'''

# import gc    # Garbage collector, use if needed
# gc.enable()

from GPS_module import GPS															# Class that handles the GPS gps6mv2
from Mav_communication import Mavlink_communication									# Class that handles communication with Mission Planner (MP)
from pilot_module import Pilot														# Class that implements an autopilot

from machine import Pin, Timer, ADC, UART											# Lib to work witg RP2040 peripherals
import time																			# Time lib

timer = Timer()

gpsUart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5), timeout = 1)
gps = GPS(gpsUart) 																	# GPS object

updating = False

TESTING = False 																	# Flag for testing mode

### Mission Started ###
print('Starting mission')

if(TESTING):
    data_valid, position = True, {'longitude': -38.692230, 'latitude': -62.292370}	# If testing, define first position manually, also could be use to just define "home" coordinates manually
else:
    data_valid, position = gps.getCoordinates(timeout = 300) 						# Find current location with 5min timeout

if(not data_valid):
    raise ValueError(f"Cannot find gps signal at start.") 							# Raise error if no gps signal is found

startPosition = (position['latitude'],position['longitude'])						# "Home" coordinates

# Pilot Object
pilot = Pilot(gps = gps, waypoints_file = 'coordinates.txt', home = startPosition, left_motor_pin = 16, left_motor_dir_pinA = 17, left_motor_dir_pinB = 18, right_motor_pin = 19, right_motor_dir_pinA = 20, right_motor_dir_pinB = 21 )

# Mavlink communication object
mav_com = Mavlink_communication()

# Timer for updating pilot and communication with MP
def timerCalls(timer):
    global pilot, updating
    
    mav_com.sendFloats(timer,pilot.get_last_gpsData()) 								# Send last gps data to MP
    mav_com.checkForMessages(timer)					   								# Check for msgs from MP
    
    if(not updating):																# This flag is used to not generate a new pilot.update() if the last one hasnt finished, since it might take longer than 1s
        updating = True
#         print('updating')
#         pilot.update()    														# Update pilot, uncomment if using
#         print('updated')
        updating = False
    
    
    
# TIMER
timer.init(freq=1, mode=Timer.PERIODIC, callback=timerCalls)						# timer init with 1s frequency

while True:
    #Nothing
    time.sleep(0.01)
    
    
    
    
    



