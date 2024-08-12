'''
Proyecto Final: piloto automático para un dron acuático.

Archivo: PID_module.py.

Descripción: Implementacion de la clase PID. Esta clase implementa un PID simple, con dos metodos: get(), para obtener la ultima salida, y update(), para actualizar con un nuevo valor de entrada.
    
Autor: Federico Hipperdinger.
'''

class PID:
    def __init__(self, kp, ki, kd):
        # PID constants
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # PID vars
        self.setpoint = 0
        self._prev_error = 0
        self._integral = 0
        self.output = 0

    def update(self, current_value):
        # Calculate output and store useful data for next update
        error = self.setpoint - current_value
        self._integral += error
        derivative = error - self._prev_error
        self.output = max(1, self.kp * error + self.ki * self._integral + self.kd * derivative)  #From 0 to 1
        self._prev_error = error
        
        return self.output
    
    def get(self):
        return self.output #return last output
