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
            10
        )
        self.datos_por_dispositivo = {}  # Almacena últimos datos por dispositivo
        self.timer = self.create_timer(60.0, self.publicar_datos)  # Ejecuta cada 60 segundos

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            id_disp = data.get("id_dispositivo")
            if id_disp:
                self.datos_por_dispositivo[id_disp] = (data, datetime.now())
                #self.get_logger().info(f"✅ Dato recibido de {id_disp}")
            else:
                self.get_logger().warn("⚠️ Mensaje recibido sin 'id_dispositivo'")
        except json.JSONDecodeError:
            self.get_logger().warn("⚠️ Mensaje no es JSON válido:")
            self.get_logger().warn(msg.data)
        except Exception as e:
            self.get_logger().error(f"❌ Error procesando mensaje: {e}")

    def publicar_datos(self):
        ahora = datetime.now()
        minuto = ahora.minute

        if minuto in [0, 30]:
            self.get_logger().info(f"⏱️ Verificando datos a las {ahora.strftime('%H:%M')}")

            for id_disp, (json_data, hora) in self.datos_por_dispositivo.items():
                tiempo_dato = (ahora - hora).total_seconds()

                if tiempo_dato < 300:
                    try:
                        response = requests.post(API_URL, json=json_data, timeout=5)
                        if response.status_code == 201:
                            self.get_logger().info(f"[{ahora.strftime('%H:%M')}] ✅ Datos de {id_disp} enviados correctamente")
                        else:
                            self.get_logger().warn(f"[{ahora.strftime('%H:%M')}] ⚠️ Error al enviar {id_disp}: {response.status_code} - {response.text}")
                    except requests.RequestException as e:
                        self.get_logger().error(f"❌ Error al enviar datos de {id_disp}: {e}")

                    self.get_logger().info(
                        f"{id_disp} | "
                        f"Dominio: {json_data.get('dominio')} | "
                        f"🌡️ Temp: {safe_format(json_data.get('temperatura'))} | "
                        f"pH: {safe_format(json_data.get('ph'))} | "
                        f"🫁 Oxígeno: {safe_format(json_data.get('oxigeno'))} | "
                        f"🧪 Turbidez: {safe_format(json_data.get('turbidez'))} | "
                        f"⚡ Conductividad: {safe_format(json_data.get('conductividad'))}"
                    )
                else:
                    self.get_logger().warn(f"[{ahora.strftime('%H:%M')}] ❌ Dato de {id_disp} demasiado antiguo ({int(tiempo_dato)}s). No se publica.")
        #else:
        #    self.get_logger().debug(f"[{ahora.strftime('%H:%M')}] Esperando próximo intervalo de publicación...")

def main(args=None):
    rclpy.init(args=args)
    nodo = NodoCentral()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


