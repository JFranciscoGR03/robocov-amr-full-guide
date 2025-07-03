#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>

#include <VescUart.h>
#include <HardwareSerial.h>

// UARTs para VESC
HardwareSerial VESCSerial1(2);  ///< UART para rueda izquierda (RX=16, TX=17)
HardwareSerial VESCSerial2(1);  ///< UART para rueda derecha (RX=23, TX=22)

VescUart vesc1;  ///< Instancia de VESC para rueda izquierda
VescUart vesc2;  ///< Instancia de VESC para rueda derecha

// ROS
rcl_subscription_t subscriber;         ///< Suscripción al tópico /cmd_vel
rcl_publisher_t vel_left_publisher;    ///< Publicador de velocidad izquierda
rcl_publisher_t vel_right_publisher;   ///< Publicador de velocidad derecha
rcl_timer_t velocity_timer;            ///< Temporizador para publicación alternada

rcl_node_t node;                       ///< Nodo ROS
rclc_executor_t executor;             ///< Executor para callbacks
rclc_support_t support;               ///< Soporte para inicialización
rcl_allocator_t allocator;            ///< Allocador de memoria

geometry_msgs__msg__Twist twist_msg;        ///< Mensaje recibido de cmd_vel
std_msgs__msg__Float32 vel_left_msg;        ///< Mensaje de velocidad izquierda
std_msgs__msg__Float32 vel_right_msg;       ///< Mensaje de velocidad derecha

#define LED_BUILTIN 2  ///< LED indicador de error

// Robot físico
const float wheel_radius = 0.08;         ///< Radio de rueda en metros
const float wheel_separation = 0.6 / 2;  ///< Mitad de distancia entre ruedas
const int motor_poles = 16;              ///< Número de polos del motor

float velocityEncL = 0.0;  ///< Velocidad de la rueda izquierda por encoder
float velocityEncR = 0.0;  ///< Velocidad de la rueda derecha por encoder

bool publish_left_next = true;  ///< Alternancia para publicar primero izquierda y luego derecha

/**
 * @brief Loop infinito que parpadea el LED en caso de error crítico.
 */
void error_loop() {
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Macros para manejo de errores de funciones ROS
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

/**
 * @brief Callback para el temporizador. Alterna entre publicar la velocidad
 *        de la rueda izquierda y la derecha, usando los datos de VESC.
 * 
 * @param timer Puntero al temporizador.
 * @param last_call_time Tiempo de la última llamada (no usado).
 */
void publish_encoder_velocity_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  if (publish_left_next) {
    if (vesc1.getVescValues()) {
      velocityEncL = (vesc1.data.rpm / motor_poles) / (60.0 / (2 * PI));
      vel_left_msg.data = velocityEncL;
      RCSOFTCHECK(rcl_publish(&vel_left_publisher, &vel_left_msg, NULL));
    }
  } else {
    if (vesc2.getVescValues()) {
      velocityEncR = (vesc2.data.rpm / motor_poles) / (60.0 / (2 * PI));
      vel_right_msg.data = velocityEncR;
      RCSOFTCHECK(rcl_publish(&vel_right_publisher, &vel_right_msg, NULL));
    }
  }

  publish_left_next = !publish_left_next;  // Alternar publicación
}

/**
 * @brief Callback del tópico /cmd_vel. Convierte la velocidad lineal y angular
 *        en velocidades por rueda, calcula RPM y las envía a los VESCs.
 * 
 * @param msgin Puntero al mensaje recibido de tipo Twist.
 */
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;

  float v = msg->linear.x;
  float w = msg->angular.z;

  float w_left = (v / wheel_radius) - (w * (wheel_separation / wheel_radius));
  float w_right = (v / wheel_radius) + (w * (wheel_separation / wheel_radius));

  float rpm_left = (60.0 / (2 * PI)) * w_left;
  float rpm_right = (60.0 / (2 * PI)) * w_right;

  float erpm_left = rpm_left * motor_poles * -1;
  float erpm_right = rpm_right * motor_poles * -1;

  vesc1.setRPM((int)erpm_left);
  vesc2.setRPM((int)erpm_right);
}

/**
 * @brief Función setup. Inicializa los VESC, micro-ROS y suscriptores/publicadores.
 */
void setup() {
  VESCSerial1.begin(115200, SERIAL_8N1, 16, 17);
  VESCSerial2.begin(115200, SERIAL_8N1, 23, 22);
  vesc1.setSerialPort(&VESCSerial1);
  vesc2.setSerialPort(&VESCSerial2);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // micro-ROS setup
  set_microros_transports();
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "vesc_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_publisher_init_default(
    &vel_left_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "velocityEncL"));

  RCCHECK(rclc_publisher_init_default(
    &vel_right_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "velocityEncR"));

  RCCHECK(rclc_timer_init_default(
    &velocity_timer,
    &support,
    RCL_MS_TO_NS(100),
    publish_encoder_velocity_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &velocity_timer));
}

/**
 * @brief Función loop. Ejecuta el executor de micro-ROS y espera un poco.
 */
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}

