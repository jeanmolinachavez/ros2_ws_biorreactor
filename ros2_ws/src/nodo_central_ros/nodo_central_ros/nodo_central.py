import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import requests
from datetime import datetime

API_URL = "https://biorreactor-app-api.onrender.com/api/sensores"

def safe_format(value):
    return f"{value:.2f}" if isinstance(value, (int, float)) else "N/A"

class NodoCentral(Node):
    def __init__(self):
        super().__init__('nodo_central')
        self.subscription = self.create_subscription(
            String,
            'datos_sensores',
            self.listener_callback,
            10)
        self.ultimo_json = None
        self.ultima_hora = None # Para verificar si el dato es reciente
        self.timer = self.create_timer(60.0, self.publicar_datos) # Cada 60 segundos verifica la hora

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.ultimo_json = data
            self.ultima_hora = datetime.now()
            #self.get_logger().info("Mensaje recibido y guardado")
        except json.JSONDecodeError:
            self.get_logger().warn("Mensaje recibido no es JSON válido:")
            self.get_logger().warn(msg.data)
        except Exception as e:
            self.get_logger().error(f"Error procesando mensaje: {e}")

    def publicar_datos(self):
        ahora = datetime.now()
        minuto = ahora.minute

        if minuto in [0, 30] and self.ultimo_json is not None:
            tiempo_dato = (ahora - self.ultima_hora).total_seconds()

            # Solo publicar si el dato no es muy antiguo
            if tiempo_dato < 300:
                # Enviar json directamente a la API
                try:
                    response = requests.post(API_URL, json=self.ultimo_json, timeout=5)
                    if response.status_code == 201:
                        self.get_logger().info(f"[{ahora.strftime('%H:%M')}] Datos enviados correctamente a la API")
                    else:
                        self.get_logger().warn(f"[{ahora.strftime('%H:%M')}] Error al enviar a la API: {response.status_code} - {response.text}")
                except requests.RequestException as e:
                    self.get_logger().error(f"Error de conexion al enviar a la API: {e}")

                # Mostrar en consola
                self.get_logger().info(
                    f"Id_dispositivo: {self.ultimo_json.get('id_dispositivo')} | "
                    f"Dominio: {self.ultimo_json.get('dominio')} | "
                    f"Temperatura: {safe_format(self.ultimo_json.get('temperatura'))} | "
                    f"pH: {safe_format(self.ultimo_json.get('ph'))} | "
                    f"Oxigeno: {safe_format(self.ultimo_json.get('oxigeno'))} | "
                    f"Turbidez: {safe_format(self.ultimo_json.get('turbidez'))} | "
                    f"Conductividad: {safe_format(self.ultimo_json.get('conductividad'))} | "
                )
            else:
                self.get_logger().warn(f"[{ahora.strftime('%H:%M')}] Dato demasiado antiguo ({int(tiempo_dato)}s). No se publica.")
        #else:
        #    self.get_logger().info(f"[{ahora.strftime('%H:%M')}] No se publica. Minuto actual: {minuto}")

def main(args=None):
    rclpy.init(args=args)
    nodo = NodoCentral()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

