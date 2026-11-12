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

// DRIVER MOTORES
Sabertooth ST(128, Serial2);   // se crea "ST" que representa el sabertooth con address 128 y en serial2 (deberia ser serial1, pero parece que no xd)
// DIP sw: 1-OFF 2-OFF 3-OFF 4-ON 5-ON 6-ON
// 1,2 OFF: libreria sabertooth, 3 OFF: proteccion bateria, 4,5 ON: comunicacion gpio
// 6 ON: emergency stop off -> (A1 y A2: si reciben un high) detenerse si un enable se desactiva. 

// PARAMETROS DE CONTROL
const int VEL_BASE = 100;    // -127 full reversa, 0 stop, 127 full avance
const int VEL_MAX = 125;
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

volatile long cuentaL = 0;
volatile long cuentaR = 0;

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
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rclc_executor_t executor;

std_msgs__msg__Float32MultiArray mensajeControl;
std_msgs__msg__Float32MultiArray mensajeEstado;

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
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;

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

// MOTORES
void moverMotor(int in1, int in2, int pwm, int velocidad) {
    velocidad = constrain(velocidad, -255, 255);

    if (velocidad > 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(pwm, velocidad);
    } else if (velocidad < 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(pwm, -velocidad);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        analogWrite(pwm, 0);
    }
}

void detenerMotores() {
    moverMotor(IN1_LEFT, IN2_LEFT, PWM_LEFT, 0);
    moverMotor(IN1_RIGHT, IN2_RIGHT, PWM_RIGHT, 0);
}

// COMUNICACION RASPBERRY
void leerRaspberry() {
    static String buffer = "";

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n') {
            buffer.trim();

            if (buffer == "STOP") {
                detenerMotores();
                ordenRecibida = false;
                errorLateral = 0;
                errorAngular = 0;
            } else {
                int separador = buffer.indexOf(',');

                if (separador > 0) {
                    float lat = buffer.substring(0, separador).toFloat();
                    float ang = buffer.substring(separador + 1).toFloat();

                    if (isfinite(lat) && isfinite(ang) && lat >= -1.0 && lat <= 1.0 && ang >= -1.0 && ang <= 1.0) {
                        errorLateral = lat;
                        errorAngular = ang;
                        ultimaOrden = millis();
                        ordenRecibida = true;
                    }
                }
            }

            buffer = "";
        } else if (c != '\r') {
            if (buffer.length() < 60) {
                buffer += c;
            } else {
                buffer = "";
            }
        }
    }
}


// SETUP
void setup() {
    pinMode(IN1_LEFT, OUTPUT);
    pinMode(IN2_LEFT, OUTPUT);
    pinMode(PWM_LEFT, OUTPUT);
    pinMode(IN1_RIGHT, OUTPUT);
    pinMode(IN2_RIGHT, OUTPUT);
    pinMode(PWM_RIGHT, OUTPUT);

    pinMode(ENC_LEFT, INPUT_PULLUP);
    pinMode(ENC_RIGHT, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_LEFT), contarLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), contarRight, RISING);

    Serial.begin(115200);
    detenerMotores();

    // MICRO-ROS
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
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

    // ----------------------------
    // Publisher
    // ----------------------------

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

    // Leer contadores de forma atomica.
    noInterrupts();
    long cuentaLeft = pulsosLeft;
    long cuentaRight = pulsosRight;
    interrupts();

    long deltaLeft = cuentaLeft - anteriorLeft;
    long deltaRight = cuentaRight - anteriorRight;
    anteriorLeft = cuentaLeft;
    anteriorRight = cuentaRight;

    float rpmLeft = deltaLeft * 60.0 / (PULSOS_VUELTA_LEFT * dt);
    float rpmRight = deltaRight * 60.0 / (PULSOS_VUELTA_RIGHT * dt);

    // La Raspberry determina el sentido de la correccion.
    float correccion = KP_LATERAL * errorLateral + KP_ANGULAR * errorAngular;

    correccion = constrain(correccion, -RPM_CORRECCION_MAX, RPM_CORRECCION_MAX);

    float objetivoLeft = constrain(RPM_BASE + correccion, 0.0, RPM_MAX);
    float objetivoRight = constrain(RPM_BASE - correccion, 0.0, RPM_MAX);

    // PID independiente para cada rueda.
    int pwmLeft = pidLeft.calcular(objetivoLeft, rpmLeft, dt);
    int pwmRight = pidRight.calcular(objetivoRight, rpmRight, dt);

    // Conserva el sentido original: izquierda positiva, derecha negativa para avanzar.
    moverMotor(IN1_LEFT, IN2_LEFT, PWM_LEFT, pwmLeft);
    moverMotor(IN1_RIGHT, IN2_RIGHT, PWM_RIGHT, -pwmRight);

    // Diagnostico por USB Serial.
    Serial.print("RPM L: ");
    Serial.print(rpmLeft);
    Serial.print(" / ");
    Serial.print(objetivoLeft);
    Serial.print(" | RPM R: ");
    Serial.print(rpmRight);
    Serial.print(" / ");
    Serial.print(objetivoRight);
    Serial.print(" | Correccion: ");
    Serial.println(correccion);
}