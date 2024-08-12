'''
Proyecto Final: piloto automático para un dron acuático.

Archivo: pilot_module.py.

Descripción: Implementacion de la clase Pilot, un piloto automatico que a partir de un archivo con waypoints controla la velocidad y direccion de dos motores para recorrer tales waypoints en orden.
    Para el control de motores utiliza un PID, implementado en PID_module.py y al instanciarse toma como parametro un objeto GPS de GPS_module.py para orientarse.
    Desde afuera, solo deben llamarse a los metodos .update(), para que el piloto actualice su posición y velocidad de motores, .return_home(), para forzar el retorno al punto de inicio antes de finalizar la misión, y
    .get_last_gpsData() para obtener la ultima informacion que el piloto extrajo del modulo gps.
    
Autor: Federico Hipperdinger.
'''


import math
from PID_module import PID
from machine import Pin, PWM
import ujson
import utime

class Pilot:
    def __init__(self, gps, waypoints_file, home, left_motor_pin = 16, left_motor_dir_pinA = 17, left_motor_dir_pinB = 18, right_motor_pin = 19, right_motor_dir_pinA = 20, right_motor_dir_pinB = 21 ):
        
        # state Flags
        self.mission_finished = False
        self.coming_home = False
        
        # H Bridge config
        # Motors PWM Pins
        self.motor_left_pwm = PWM(Pin(left_motor_pin))
        self.motor_right_pwm = PWM(Pin(right_motor_pin))
        self.motor_left_pwm.freq(1000)
        self.motor_right_pwm.freq(1000)

        # H bridge direction control pins
        self.motor_left_dirA = Pin(left_motor_dir_pinA, Pin.OUT)
        self.motor_left_dirB = Pin(left_motor_dir_pinB, Pin.OUT)
        self.motor_right_dirA = Pin(right_motor_dir_pinA, Pin.OUT)
        self.motor_right_dirB = Pin(right_motor_dir_pinB, Pin.OUT)
        
        # Start with motors off
        self.motor_left_dirA.off()
        self.motor_left_dirB.off()
        self.motor_right_dirA.off()
        self.motor_right_dirB.off()
        
        # GPS related
        self.gps = gps
        self.waypoints = self.read_waypoints(waypoints_file)	# Get waypoints
        self.distance_margin = 4 								#margin of error in meters
        self.angle_margin = 1 									#margin for angle in radians, if greater the pilot slows down until it is corrected
        self.angle_PID = PID(0.1, 0.01, 0.005)					# Angle PID
        self.distance_PID = PID(0.1, 0.01, 0.005)				# Distance PID 
        self.home = home										# Home coordinates
        self.current_target = self.waypoints[0]					# First target
        
        self.prev_position = self.get_coordinates(timeout = 60) # Define an starting previous position as the current one, 60s timeout
        if self.prev_position == (-1,-1):
            raise ValueError(f"Cannot find gps signal at start.")
        
        # Move for 5s
        self.control_motors(1,1)
        utime.sleep_ms(5000)
        self.control_motors(0,0)
        
        self.position = self.get_coordinates(timeout = 60) 		#Current position, 60s timeout
        if self.position == (-1,-1):
            raise ValueError(f"Cannot find gps signal at start.")
        
        self.distance  = self.haversine(self.position[0], self.position[1], self.current_target[0], self.current_target[1]) 
        self.angle     = self.getAngle(self.position, self.prev_position, self.current_target)
        
        # Update PIDs
        self.distance_PID.update(self.distance) 
        self.angle_PID.update(self.angle)
            
        ### Mission started ###
        print('Mission started.')




    # Get mission waypoints from txt file
    def read_waypoints(self, file_path):
        with open(file_path, 'r') as file:
            coordinates = ujson.load(file)
        return coordinates

    # Get current GPS data from gps module
    def get_coordinates(self, timeout = 60):
        data_valid, self.gps_data = self.gps.getCoordinates(timeout)
        if data_valid:
            return self.gps_data['longitude'], self.gps_data['latitude']
        else:
            return -1,-1
    
    # Return last gsp data retrieved
    def get_last_gpsData(self):
        return self.gps_data
    
    # Obtain angle between BC and BA
    def getAngle(self, a, b, c):
        ang = math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0]) #Angle in radians
        return ang
 
    # Update Pilot
    def update(self):
        self.prev_position = self.position
        aux = self.get_coordinates()
        self.position  = aux if aux != (-1,-1) else self.position
        
        if(self.position == self.prev_position):
            moving = False
            print('Not moving')
        else:
            moving = True
            print('Moving')
        
        if(moving):
            # Update distance and angle
            self.distance  = self.haversine(self.position[0], self.position[1], self.current_target[0], self.current_target[1]) #math.sqrt((self.position[0]-self.current_target[0])**2 + (self.position[1]-self.current_target[1])**2) 
            self.angle     = self.getAngle(self.position, self.prev_position, self.current_target)
            
            print('Distance from target: ', self.distance, '. Angle: ', self.angle, '.')
            
            # Mission logic
            if(not self.mission_finished):
                target_reached = False
                
                if(self.distance < self.distance_margin):
                    target_reached = True
                    print('Target ', self.current_target, ' reached.')
                    if(self.coming_home):
                        self.mission_finished = True
                        print('Mission finished')
                        return target_reached, self.coming_home, self.mission_finished
                    
                    
                    self.waypoints.remove(self.current_target)
                    if(self.waypoints.empty()):
                        self.return_home()
                    else:
                        self.current_target = self.waypoints[0]
                        self.distance  = sqrt((self.position[0]-self.current_target[0])**2 + (self.position[1]-self.current_target[1])**2) 
                        self.angle     = getAngle(self.position, self.prev_position, self.current_target)
            
            # Update PIDs
            self.distance_PID.update(self.distance) 
            self.angle_PID.update(self.angle)
        
            # If the angle is bigger than the defined margin, adjust it first   
            if(self.angle > self.angle_margin):
                speed_right = 0.8*self.angle_PID.get() + 0.2*self.distance_PID.get() #Max value = 1
                speed_left = -0.8*self.angle_PID.get() + 0.2*self.distance_PID.get()
            else: 
                speed_right = 0.3*self.angle_PID.get() + 0.7*self.distance_PID.get()
                speed_left = -0.3*self.angle_PID.get() + 0.7*self.distance_PID.get()
            
            # Update motors speed
            self.control_motors(speed_left,speed_right)
            
        
        return target_reached, self.coming_home, self.mission_finished # Useful data to know the state of the pilot
        
    # Motor control
    def control_motors(self, speed_left, speed_right):
        self.motor_left_dirA.value(1 if speed_left >= 0 else 0) 
        self.motor_left_dirB.value(0 if speed_left >= 1 else 0)
        self.motor_right_dirA.value(1 if speed_right >= 0 else 0)
        self.motor_right_dirB.value(0 if speed_right >= 1 else 0)
        
        self.motor_left_pwm.duty_u16(min(65535, max(0, int(abs(speed_left) * 65535)))) 
        self.motor_right_pwm.duty_u16(min(65535, max(0, int(abs(speed_right) * 65535))))

    # Define current and final target as home
    def return_home(self):
        self.coming_home = True
        self.current_target = self.home
        print('Coming home')
        
    # Calculate distance in meters between two coordinates
    def haversine(self, lat1, lon1, lat2, lon2):
        # Earth radius in Km
        R = 6371.0
        
        # Degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Lat and long diff
        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad
        
        # Haversine formula
        a = math.sin(delta_lat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance_km = R * c
        
        # distance to meters
        distance_m = distance_km * 1000
        
        return distance_m

        
