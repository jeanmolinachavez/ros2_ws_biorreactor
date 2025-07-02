// =============================
//      Biorreactor ESP32
//   Nodo micro-ROS Publisher
// =============================
//
// Lee sensores analógicos y de temperatura digital (Dallas),
// envía los datos formateados en JSON vía micro-ROS a un nodo central ROS 2,
// usando conexión WiFi con agente micro-ROS por protocolo UDP.
//

#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Configuración de pines de sensores
#define PIN_TEMP 4        // Sensor de temperatura DS18B20
#define PIN_PH 32         // Sensor de pH DiyMore PH-4502C
#define PIN_TDS 33        // Sensor de conductividad TDS
#define PIN_O2 34         // Sensor de oxigeno ME2-O2 Winsen
#define PIN_TURBIDEZ 35   // Sensor de turbidez MJKDZ

// Configuración WiFi
char ssid[] = "ESP_UCN";
char password[] = "3sp32UCN@";

// IP del agente ROS y puerto UDP (PC con ROS 2)
char agent_ip[] = "192.168.31.200";
size_t agent_port = 8888;

// Identificación del dispositivo (ID y dominio)
const char* id_dispositivo = "esp32_01";
const char* dominio = "dominio_ucn";

// Constantes de ESP32
const float VREF = 3.3; // Voltaje de referencia ADC
const int RESOLUCION_ADC = 4095; // Resolución ADC (12 bits)

// Instancias del sensor de temperatura
OneWire oneWire(PIN_TEMP);
DallasTemperature sensors(&oneWire);

// Datos de calibración del sensor de pH
const float v1 = 3.02, ph1 = 4.02;
const float v2 = 2.65, ph2 = 6.88;
const float v3 = 2.41, ph3 = 9.05;

// Cálculo de coeficientes para regresión cuadrática: ph = a*v^2 + b*v + c
float denom = (v1 - v2) * (v1 - v3) * (v2 - v3);

float a = (v3*(ph2 - ph1) + v2*(ph1 - ph3) + v1*(ph3 - ph2)) / denom;
float b = (v3*v3*(ph1 - ph2) + v2*v2*(ph3 - ph1) + v1*v1*(ph2 - ph3)) / denom;
float c = (v2*v3*(v2 - v3)*ph1 + v3*v1*(v3 - v1)*ph2 + v1*v2*(v1 - v2)*ph3) / denom;

// Calibración sensor O2 (se asume por fabricante que 2V para 21% O2) sin acceso a gases de referencia
const float o2_aire = 0.21;
const float volt_aire = 2.0;

// Variables de micro-ROS
rcl_publisher_t publisher;
std_msgs__msg__String msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;
rcl_timer_t timer;
char json_buffer[256];

// Callback del Timer: lectura de sensores
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //  Sensor de temperatura
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);

    // Sensor de turbidez
    int valorTurbidez = analogRead(PIN_TURBIDEZ);
    float voltajeTurbidez = valorTurbidez * VREF / RESOLUCION_ADC;

    // Se asume que: 0 V = 100% turbidez, 3.3 V = 0% turbidez
    float turbidez = (1.0 - (voltajeTurbidez / 3.3)) * 100.0;

    // Limitar el valor entre 0% y 100%
    turbidez = constrain(turbidez, 0.0, 100.0);

    // Sensor de pH
    const int muestrasPH = 32;
    int sumaPH = 0;
    for (int i = 0; i < muestrasPH; i++) {
      sumaPH += analogRead(PIN_PH);
      delay(10);
    }
    float promedioPH = sumaPH / (float)muestrasPH;
    float voltajePH = promedioPH * VREF / RESOLUCION_ADC;
    float ph = a * voltajePH * voltajePH + b * voltajePH + c;

    // Sensor de oxígeno disuelto
    const int muestrasO2 = 12;
    long sumaO2 = 0;
    for (int i = 0; i < muestrasO2; i++) {
      sumaO2 += analogRead(PIN_O2);
    }
    float promedioO2 = sumaO2 / (float)muestrasO2;
    float voltajeO2 = promedioO2 * VREF / RESOLUCION_ADC;
    float concentracionO2 = voltajeO2 * o2_aire / volt_aire;
    float o2_porcentaje = concentracionO2 * 100;

    // Limitar rango a 0–25% para poder leer en el rango del sensor de oxigeno
    o2_porcentaje = constrain(o2_porcentaje, 0.0, 25.0);

    // Sensor de conductividad (TDS)
    int lecturaTDS = analogRead(PIN_TDS);
    float voltajeTDS = lecturaTDS * VREF / RESOLUCION_ADC;

    // Formula para sensor de TDS con compensación por temperatura
    float tds = (133.42 * pow(voltajeTDS, 3) - 255.86 * pow(voltajeTDS, 2) + 857.39 * voltajeTDS) //Convierte el voltaje medido a un valor TDS en ppm
                * (1.0 + 0.02 * (25.0 - tempC)); // Se aplica la compensación con el sensor de temperatura

    // Empaquetar en JSON con datos
    snprintf(json_buffer, sizeof(json_buffer),
      "{\"id_dispositivo\":\"%s\",\"dominio\":\"%s\",\"turbidez\":%.2f,\"ph\":%.2f,"
      "\"temperatura\":%.2f,\"oxigeno\":%.2f,\"conductividad\":%.2f}",
      id_dispositivo, dominio, turbidez, ph, tempC, o2_porcentaje, tds);

    // Enviar por micro-ROS
    msg.data.data = json_buffer;
    msg.data.size = strlen(json_buffer);
    msg.data.capacity = sizeof(json_buffer);
    rcl_publish(&publisher, &msg, NULL);

    Serial.print("Publicado por micro-ROS: ");
    Serial.println(json_buffer);
  }
}

// Configuración inicial
void setup() {
  Serial.begin(115200);

  // Conexión Wifi
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi conectado");
  Serial.println(WiFi.localIP());

  // Configurar micro-ROS vía Wifi UDP
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  delay(2000);

  // Inicializar sensores
  sensors.begin();

  // Inicializar micro-ROS
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "nodo_esp32_01", "", &support);
  rclc_publisher_init_default(&publisher, &node,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                              "datos_sensores");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(30000), timer_callback); // cada 30 segundos
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
}

// Bucle principal
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
