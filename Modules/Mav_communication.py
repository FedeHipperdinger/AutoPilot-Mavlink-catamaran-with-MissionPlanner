'''
Proyecto Final: piloto automático para un dron acuático.

Archivo: Mav_communication.py.

Descripción: Implementacion de la clase Mavlink_communication. Esta clase permite la comunicacion con Mission Planner a partir de la libreria auto generada pymavminimal.py obtenida de https://github.com/stephendade/mav_rp2040.
             Se utilizan 3 pines ADC, el led montado de la raspberry pico y la Uart 0 con los pines tx=0 y rx=1. Aqui se definen dos metodos: sendFloats(), para la transmision de datos ADC y GPS,
             y checkForMessages() para establecer la comunicacion con MP a partir de los Heartbeats y manejar los mensajes tipo STATUSTEXT, de los cuales se pueden recibir mensajes personalizados
             e interpretarlos como comandos (Se implementó un ejemplo con el control del Led On-board).
    
Autor: Federico Hipperdinger.
'''


from machine import Pin, UART, ADC
import pymavminimal as pymav		# Lib with classes and functions for handling the Mavlink protocol
import time




class Mavlink_communication:
    def __init__(self, tx = 0, rx = 1):
        self.led = Pin(25, Pin.OUT) # On-board Led

        # UART config
        tx_pin = Pin(tx, Pin.OUT)
        rx_pin = Pin(rx, Pin.IN)
        self.uart0 = UART(0, baudrate=57600, tx=tx_pin, rx=rx_pin)
        self.uart0.init(bits=8, parity=None, stop=1)

        self.f = open("msgs.txt", "w")

        # MAVLink
        self.mavobj = pymav.MAVLink(self.f)
        self.mavobj.robust_parsing = True

        # ADC pins
        self.adc2_value = ADC(28)
        self.adc1_value = ADC(27)
        self.adc0_value = ADC(26)
        self.conversion_factor = 3.3/(65536)
        
        # Heartbeat detection flag
        self.seen_heartbeat = False

    # Msg with data
    def sendFloats(self, timer, gpsData):
        if self.seen_heartbeat:
            #send floats
            msgs = []
            # Heartbeat
            msgs.append(self.mavobj.heartbeat_encode(pymav.MAV_TYPE_ONBOARD_CONTROLLER, 8, 0, 0, 0))
            # Transform GPS data to message
            gps_msg = pymav.MAVLink_gps_raw_int_message(gpsData['time'], gpsData['fix'],gpsData['latitude_int'], gpsData['longitude_int'], gpsData['altitude'], 0, 0, gpsData['speed'], 0, gpsData['satellites'])  #time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible
            msgs.append(gps_msg)
            # Append ADC values to message
            msgs.append(self.mavobj.named_value_float_encode(time.ticks_ms(), "ADC0", self.adc0_value.read_u16() * self.conversion_factor))
            msgs.append(self.mavobj.named_value_float_encode(time.ticks_ms(), "ADC1", self.adc1_value.read_u16() * self.conversion_factor))
            msgs.append(self.mavobj.named_value_float_encode(time.ticks_ms(), "ADC2", self.adc2_value.read_u16() * self.conversion_factor))

            # Transmit message
            for msg in msgs:
                self.uart0.write(msg.pack(self.mavobj))
#             print("Sent at {0}".format(time.ticks_ms()))
            
    # Check for new messages and handle them
    def checkForMessages(self, timer):
        num = self.uart0.any()
        
        # Receive data and process into Mavlink packets
        if num > 0:
            rxData = self.uart0.read(num)
            pkts = self.mavobj.parse_buffer(bytearray(rxData))
            
            if pkts is not None:
                for pkt in pkts:
#                     print(pkt)
                    pkt_type = pkt.get_type()
#                     print(pkt_type)
                    if pkt_type == 'HEARTBEAT': #  Handle heartbeat message
                        self.led.toggle()
                        if not self.seen_heartbeat:
                            print("Got heartbeat from {0}:{1}".format(pkt.get_srcSystem(), pkt.get_srcComponent()))
                            self.mavobj.srcSystem = pkt.get_srcSystem()
                            self.mavobj.srcComponent = 158 #MAV_COMP_ID_PERIPHERAL
                            self.seen_heartbeat = True
                    elif pkt_type == 'STATUSTEXT': #Handle Status message. Useful for receiveng commands from Mission Planner.
#                         print(pkt)
#                         print('msg: ', pkt.text)
                        msg = pkt.text 		# Msg text content
                        parts = msg.split()
                        
                        #Handle commands. Change to define custom commands.
                        if(len(parts)>0):
                            
                            if(parts[0] == 'command1'):
                                N = int(parts[1])
                                for i in range(5):
                                    self.led.on()
                                    time.sleep(N*0.1)
                                    self.led.off()
                                    time.sleep(N*0.1)
                            elif(parts[0] == 'command2'):
                                N = int(parts[1])
                                for i in range(5):
                                    self.led.on()
                                    time.sleep(N*0.2)
                                    self.led.off()
                                    time.sleep(N*0.2)
                            elif(parts[0] == 'command3'):
                                N = int(parts[1])
                                for i in range(5):
                                    self.led.on()
                                    time.sleep(N*0.3)
                                    self.led.off()
                                    time.sleep(N*0.3)        
                        


### For Testing ###
                            
# mav = Mavlink_communication()
# 
# while True:
#     mav.checkForMessages(0)
#     mav.sendFloats(0,1)
#     time.sleep(1)
