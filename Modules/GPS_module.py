'''
Proyecto Final: piloto automático para un dron acuático.

Archivo: GPS_module.py.

Descripción: Implementacion de la clase GPS para el módulo gps6mv2. Se posee un unico metodo, getCoordinates(timeout), que devuelve un diccionario con latitud, longitud, altitud, velocidad, numero de satelitex, num de fix y tiempo.
             Este método se encarga de interpretar los mensages NMEA recibidos del módulo y obtener toda la informacion requerida, la cual está repartida entre los mensages $GPGGA y $GPRMC.
    
Autor: Federico Hipperdinger.
'''

import utime, time


TESTING = False # Testing mode, use to define lat and long manually


class GPS:
    def __init__(self, uart):
        self.gpsModule = uart
#         print(self.gpsModule)

        self.buff = bytearray(255) # Buffer
        self.data = {}

    
    def getCoordinates(self, timeout = 80):
        if TESTING:
            # define lat and long manually
            lat = float(input('latitude'))
            long = float(input('longitud'))
            self.data = {'latitude':lat, 'longitude':long, 'altitude': 0, 'fix': 1, 'satellites':3,'time':time.time()}
            
            return True, self.data
        else:
            self.data = {}
            data_counter = [0,0] # Equal to [1,1] when all the required data has been retrieved
            timeout = time.time() + timeout
            
            while True:
                buff = str(self.gpsModule.readline())
                parts = buff.split(',')
                buff = ''
                # Handle $GPGGA message
                if (parts[0] == "b'$GPGGA" and len(parts) == 15 and data_counter[0] == 0):
                    if(parts[1] and parts[2] and parts[3] and parts[4] and parts[5] and parts[6] and parts[7] and parts[9]):
                        
                        print(parts) # print list with data
                        
                        data_counter[0] = 1 # update counter
                        
                        # Process data
                        latitude_str = self.convertToDegree(parts[2])
                        if (parts[3] == 'S'):
                            latitude_str = '-'+latitude_str
                            
                        longitude_str = self.convertToDegree(parts[4])
                        if (parts[5] == 'W'):
                            longitude_str = '-'+longitude_str
                            
                        latitude = float(latitude_str)
                        latitude_int = round(float(latitude)*10**7)
#                         print(latitude_int)
                        longitude = float(longitude_str)
                        longitude_int = round(float(longitude)*10**7)
#                         print(longitude_int)
                        fix = round(float(parts[6]))
                        satellites = round(float(parts[7]))
                        GPStime_str = parts[1][0:2] + ":" + parts[1][2:4] + ":" + parts[1][4:6]
                        GPStime = round(float(parts[1]))
                        altitude = round(float(parts[9]))
                        
                        # Useful print
                        '''
                        print("Printing GPS data...")
                        print(" ")
                        print("Latitude: "+latitude_str)
                        print("Longitude: "+longitude_str)
                        print("Satellites: " +str(satellites))
                        print("Time: "+GPStime_str)
                        print("----------------------")
                        '''
                        
                        # Assign data for return
                        self.data['latitude'] 		= latitude
                        self.data['longitude'] 		= longitude
                        self.data['latitude_int'] 	= latitude_int
                        self.data['longitude_int'] 	= longitude_int
                        self.data['altitude'] 		= altitude
                        self.data['time'] 			= GPStime
                        self.data['fix'] 			= fix
                        self.data['satellites'] 	= satellites
                        

                # Handle $GPRMC message
                elif ((parts[0] == "b'$GPRMC" or parts[0] == "b'$G$GPRMC") and parts[2] == 'A' and data_counter[1] == 0 and len(parts) >= 8 ):
                    if(parts[1] and parts[2] and parts[3] and parts[4] and parts[5] and parts[6] and parts[7] ):

                        print(parts) # print list with data
                        
                        data_counter[1] = 1 # update counter
                        
                        # Process data
                        latitude_str = self.convertToDegree(parts[3])
                        if (parts[4] == 'S'):
                            latitude_str = '-'+latitude_str
                            
                        longitude_str = self.convertToDegree(parts[5])
                        if (parts[6] == 'W'):
                            longitude_str = '-'+longitude_str
                            
                        latitude = float(latitude_str)
                        latitude_int = round(float(latitude)*10**7)
#                         print(latitude_int)
                        longitude = float(longitude_str)
                        longitude_int = round(float(longitude)*10**7)
#                         print(longitude_int)
                        
                        if(parts[7][-1] == "'"):
                            parts[7] = parts[7][:-1]
                        try:
                            speed = round(float(parts[7])* 5.144) #knots per hour --->>> cm per second
                        except:
                            speed = 0
                        GPStime_str = parts[1][0:2] + ":" + parts[1][2:4] + ":" + parts[1][4:6]
                        GPStime = round(float(parts[1]))
                        
                        # Useful print
                        '''
                        print("Printing GPS data...")
                        print(" ")
                        print("Latitude: "+latitude_str)
                        print("Longitude: "+longitude_str)
                        print("Speed: " +str(speed))
                        print("Time: "+GPStime_str)
                        print("----------------------")
                        '''
                        
                        # Assign data for return
                        self.data['latitude'] 		= latitude
                        self.data['longitude'] 		= longitude
                        self.data['latitude_int'] 	= latitude_int
                        self.data['longitude_int'] 	= longitude_int
                        self.data['speed'] 		= speed 
                        self.data['time'] = GPStime
                        

                    
                # Timeout
                if (time.time() > timeout):
                    print("No GPS data is found.")
                    return False ,self.data
                
                # All required data has been retrieved
                if(data_counter == [1,1]):
                    return True, self.data
            
                utime.sleep_ms(200)
                
                
    # Function to obtain lat and long value from the raw data provided by the gps module
    def convertToDegree(self, RawDegrees):

        RawAsFloat = float(RawDegrees)
        firstdigits = int(RawAsFloat/100) 
        nexttwodigits = RawAsFloat - float(firstdigits*100) 
        
        Converted = float(firstdigits + nexttwodigits/60.0)
        Converted = '{0:.6f}'.format(Converted) 
        return str(Converted)
        
    
