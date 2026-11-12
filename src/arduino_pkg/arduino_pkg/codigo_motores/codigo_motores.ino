// Código motores

#include <Sabertooth.h>
#include <Arduino.h>
#include <math.h>

// librerias micro ros
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/Int32.h>


// DRIVER MOTORES
Sabertooth ST(128, Serial2);   // se crea "ST" que representa el sabertooth con address 128 y en serial2 (deberia ser serial1, pero parece que no xd)
// DIP sw: 1-OFF 2-OFF 3-OFF 4-ON 5-ON 6-ON
// 1,2 OFF: libreria sabertooth, 3 OFF: proteccion bateria, 4,5 ON: comunicacion gpio
// 6 ON: emergency stop off -> detenerse si un enable se desactiva (A1 y A2 deben estar high para que M1 y M2 funcionen)

// PARAMETROS DE CONTROL
const int VEL_BASE = 40;    // -127 full reversa, 0 stop, 127 full avance
const int VEL_MAX = 120;
const int VEL_CORRECCION_MAX = 10;

// IMPORTANTE: cambiar pulsos por vuelta
const float PULSOS_VUELTA_LEFT = 20.0;
const float PULSOS_VUELTA_RIGHT = 20.0;

const unsigned long DT_MS = 50;
const unsigned long TIMEOUT_RASPBERRY_MS = 500;

// PID de velocidad: valores iniciales, requieren calibracion.
const float KP_VEL = 1.5;
const float KI_VEL = 0.005;
const float KD_VEL = 0.08;

// Ganancias de trayectoria: valores iniciales, requieren calibracion.
const float KP_LATERAL = 60.0;
const float KP_ANGULAR = 60.0;

// ENCODERS -> pins y funciones
const int PIN_ENC_L_A = 2;   // ENC_L_A -> GIGA D2   encoder A de motor izquierdo
const int PIN_ENC_L_B = 3;   // ENC_L_B -> GIGA D3   encoder B de motor izquierdo
const int PIN_ENC_R_A = 4;   // ENC_R_A -> GIGA D4   encoder A de motor derecho
const int PIN_ENC_R_B = 5;   // ENC_R_B -> GIGA D5   encoder B de motor derecho

volatile long cuentaL = 0;   // cuenta actual de los encoders R y L
volatile long cuentaR = 0;

long anteriorL = 0;    // cuenta pasada de los encoders R y L
long anteriorR = 0;

void encL() { cuentaL += digitalRead(PIN_ENC_L_B) ? 1 : -1; }  // a cuentaL se le suma el resultado de: if el pin esta high (1), entonces es 1 y si no entonces es -1
void encR() { cuentaR += digitalRead(PIN_ENC_R_B) ? 1 : -1; }

// PID DE VELOCIDAD
class PIDVelocidad {
    public:
        float integral = 0;
        float errorAnterior = 0;

        int calcular(float objetivo, float actual, float dt) {
            float error = objetivo - actual;

            float derivada = (error - errorAnterior) / dt;
            float integralNueva = integral + error * dt;

            float salida = KP_VEL * error + KI_VEL * integralNueva + KD_VEL * derivada;

            // Limitar salida PWM y evitar acumulacion excesiva.
            if (salida >= 0 && salida <= 255) {
                integral = integralNueva;
            }

            salida = KP_VEL * error + KI_VEL * integral + KD_VEL * derivada;

            errorAnterior = error;
            return constrain((int)round(salida), 0, 255);
        }

        void reiniciar() {
            integral = 0;
            errorAnterior = 0;
        }
};

// PID Distancia
class PIDDistancia {
    public:
        float integral = 0;
        float errorAnterior = 0;

        float calcular(float objetivo, float actual, float dt) {

            float error = objetivo - actual;
            float derivada = (error - errorAnterior) / dt;

            float integralNueva = integral + error * dt;

            float salida = KP_DIST * error + KI_DIST * integralNueva + KD_DIST * derivada;

            // Anti-windup básico.
            if (salida >= -PWM_MAX && salida <= PWM_MAX) {
                integral = integralNueva;
            }

            salida = KP_DIST * error + KI_DIST * integral + KD_DIST * derivada;
            errorAnterior = error;
            return constrain(salida, -PWM_MAX, PWM_MAX);
        }
        void reiniciar() {
            integral = 0;
            errorAnterior = 0;
        }
};
PIDDistancia pidDistancia;

PIDVelocidad pidLeft, pidRight;

// ESTADO
float errorLateral = 0;
float errorAngular = 0;
float anchoCara = 0;

unsigned long ultimaOrden = 0;
unsigned long ultimoControl = 0;
long anteriorLeft = 0, anteriorRight = 0;

bool usuarioDetectado = false;
bool ordenRecibida = false;


// VARIABLES MICRO-ROS
rcl_node_t nodo_arduino;
rclc_support_t ros_support;
rcl_allocator_t ros_allocator;
rclc_executor_t ros_executor;

rcl_subscription_t sub_target_speed;
rcl_publisher_t pub_remote_btn;

std_msgs__msg__Float32 msg_recv_speed;
std_msgs__msg__Bool msg_pub_btn;

float bufferControl[2];
float bufferEstado[4];

// ============================================================
// CALLBACK DEL TOPIC DE CONTROL
//
// /control_error
//
// data[0] = error lateral
// data[1] = error angular
// ============================================================

void callbackControl(const void *msgin) {
    const std_msgs__msg__Float32 *msg =
        (const std_msgs__msg__Float32 *)msgin;

    if (msg->data.size < 2) return;

    float lat = msg->data.data[0];
    float ang = msg->data.data[1];

    if (isfinite(lat) && isfinite(ang) &&
        lat >= -1.0 && lat <= 1.0 &&
        ang >= -1.0 && ang <= 1.0) {

        errorLateral = lat;
        errorAngular = ang;

        ultimaOrden = millis();
        ordenRecibida = true;
    }
}


// SETUP
void setup() {
    pinMode(PIN_ENC_L_A, INPUT);
    pinMode(PIN_ENC_L_B, INPUT);
    pinMode(PIN_ENC_R_A, INPUT);
    pinMode(PIN_ENC_R_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), encL, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), encR, RISING);

    // Serial reservado para ros (no se le hace begin y no se puede usar para imprimir)
    Serial2.begin(9600); // serial2 para motores. 9600 es el default   (pin D18 arduino conectado a S1 sabertooth)
    Serial3.begin(115200); // serial3 para comunicacion esp            (pin D17 esp con arduino o con raspi)
    //ST.setTimeout(1000); // para que si no llegan comandos nuevos dentro de 1 segundo se detienen los motores (settimeout(0) para desactivar)
                       // un delay en el codigo puede activar el timeout!!
    ST.motor(1,0);  // se detienen los motores
    ST.motor(2,0);

    // MICRO-ROS
    set_microros_serial_transports(Serial);  // solo usar serial para microros y no serial.print
    delay(2000);

    allocator = rcl_get_default_allocator();

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "control_motores", "", &support);

    // Subscriber
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/control_error"
    );

    // Publisher
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/motor_status"
    );

    // CONFIGURACIÓN DE MENSAJES
    mensajeControl.data.data = bufferControl;
    mensajeControl.data.capacity = 2;
    mensajeControl.data.size = 2;

    mensajeEstado.data.data = bufferEstado;
    mensajeEstado.data.capacity = 4;
    mensajeEstado.data.size = 4;

    // EXECUTOR
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &mensajeControl, &callbackControl, ON_NEW_DATA);

    ultimoControl = millis();
}




// LOOP
void loop() {
    // Procesar mensajes ROS 2 entrantes
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

    unsigned long ahora = millis();

    // Si se pierde la comunicacion, detener el robot.
    if (!ordenRecibida || ahora - ultimaOrden > TIMEOUT_RASPBERRY_MS) {
        detenerMotores();
        pidLeft.reiniciar();
        pidRight.reiniciar();
        return;
    }

    if (ahora - ultimoControl < DT_MS) return;

    float dt = (ahora - ultimoControl) / 1000.0;
    ultimoControl = ahora;

    float rpmLeft = deltaLeft * 60.0 / (PULSOS_VUELTA_LEFT * dt);
    float rpmRight = deltaRight * 60.0 / (PULSOS_VUELTA_RIGHT * dt);

    // La Raspberry determina el sentido de la correccion.
    float correccion = KP_LATERAL * errorLateral + KP_ANGULAR * errorAngular;

    correccion = constrain(correccion, -RPM_CORRECCION_MAX, RPM_CORRECCION_MAX);

    float objetivoLeft = constrain(RPM_BASE + correccion, 0.0, RPM_MAX);
    float objetivoRight = constrain(RPM_BASE - correccion, 0.0, RPM_MAX);

    // PID independiente para cada rueda.
    int velLeft = pidLeft.calcular(objetivoLeft, rpmLeft, dt);
    int velRight = pidRight.calcular(objetivoRight, rpmRight, dt);

    // para avanzar, ambos valores son positivos
    ST.motor(1, velLeft);
    ST.motor(2, velRight);

    // se leen encoders cada 0.1 segundos
    static uint32_t ultimo_read_enc = 0;
    if (millis() - ultimo_read_enc >= 100) {
        ultimo_read_enc = millis();
        noInterrupts();
        long l = cuentaL, r = cuentaR;
        interrupts();
    }
    long deltaL = cuentaL - anteriorL;  // se calcula que tanto aumento el valor de los encoders
    long deltaR = cuentaR - anteriorR;
    anteriorL = cuentaL;
    anteriorR = cuentaR;
}



