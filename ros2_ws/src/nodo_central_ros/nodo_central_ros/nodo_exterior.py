import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

def formato_seguro(value):
    return f"{value:.2f}" if isinstance(value, (int, float)) else "N/A"

class NodoExterior(Node):
    def __init__(self):
        super().__init__('nodo_exterior')

        # Suscripción al topic específico para este nodo
        self.subscription = self.create_subscription(
            String,
            'datos_sensores_exteriores',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            id_disp = data.get("id_dispositivo")
            if id_disp:
                self.get_logger().info(

                    # Log de los datos recibidos
                    f"✅ Dato recibido de {id_disp}:\n"
                    f"🆔 ID: {id_disp} | "
                    f"🌐 Dominio: {data.get('dominio')}\n"
                    f"🌡️ Temp. Nuevo Sensor: {formato_seguro(data.get('temperatura_sht'))} °C | "
                    f"💧 Humedad: {formato_seguro(data.get('humedad'))} %\n"
                )
            else:
                self.get_logger().warn("⚠️ Mensaje recibido sin 'id_dispositivo'")
        except json.JSONDecodeError:
            self.get_logger().warn("⚠️ Mensaje no es JSON válido:")
            self.get_logger().warn(msg.data)
        except Exception as e:
            self.get_logger().error(f"❌ Error procesando mensaje: {e}")

# Función principal del nodo
def main(args=None):
    rclpy.init(args=args)
    nodo = NodoExterior()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
