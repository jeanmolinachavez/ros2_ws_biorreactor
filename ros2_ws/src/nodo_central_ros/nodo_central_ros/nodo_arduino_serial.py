import rclpy
from rclpy.node import Node
import serial
import json
import requests
from datetime import datetime

class NodoArduinoSerial(Node):
    def __init__(self):
        super().__init__('nodo_arduino_serial')

        self.serial_port = 'COM7' # Cambiar por el puerto de linux
        self.baud_rate = 9600
        self.api_sensores = 'https://biorreactor-app-api.onrender.com/api/sensores'
        self.api_comida = 'https://biorreactor-app-api.onrender.com/api/registro_comida'

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Conectado a {self.serial_port} a {self.baud_rate} baudios")
        except Exception as e:
            self.get_logger().error(f"No se pudo abrir el puerto serial: {e}")
            return
        
        self.create_timer(0.1, self.leer_serial) # Llama cada 100 ms

    def leer_serial(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f"Recibido: {line}")

                try:
                    data = json.loads(line)

                    # Enviar a la URL correspondiente
                    if "evento" in data:
                        url = self.api_comida
                    else:
                        url = self.api_sensores

                    # Enviar el JSON a la API
                    response = requests.post(url, json=data)
                    if response.status_code == 201:
                        self.get_logger().info("Datos enviados correctamente")
                    else:
                        self.get_logger().warn(f"Error al enviar: {response.text}")

                except json.JSONDecodeError:
                    self.get_logger().warn("Linea recibida no es JSON valido")

        except Exception as e:
            self.get_logger().error(f"Error general: {e}")

def main(args=None):
    rclpy.init(args=args)
    nodo = NodoArduinoSerial()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()