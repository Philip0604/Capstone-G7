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
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/vector3.h>
// #include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
// #include <std_msgs/msg/Int32.h>


// DRIVER MOTORES
Sabertooth ST(128, Serial2);   // se crea "ST" que representa el sabertooth con address 128 y en serial2 (deberia ser serial1, pero parece que no xd)
// DIP sw: 1-OFF 2-OFF 3-OFF 4-ON 5-ON 6-ON
// 1,2 OFF: libreria sabertooth, 3 OFF: proteccion bateria, 4,5 ON: comunicacion gpio
// 6 ON: emergency stop off -> detenerse si un enable se desactiva (A1 y A2 deben estar high para que M1 y M2 funcionen)

// PARAMETROS DE CONTROL
const int RPM_BASE = 40;    // -127 full reversa, 0 stop, 127 full avance
const int RPM_MAX = 120;
const int RPM_CORRECCION_MAX = 120.0f;

const float PULSOS_VUELTA_LEFT = 480.0f;    // cantidad de pulsos medidos por encoder por revolucion (valor para pololu 4752 de 1:30)
const float PULSOS_VUELTA_RIGHT = 480.0f;

const unsigned long DT_MS = 50;
const unsigned long TIMEOUT_RASPBERRY_MS = 500;

// PID de velocidad: valores iniciales, requieren calibracion (f para Float32; calculos mas rapidos)
const float KP_VEL = 1.5f;
const float KI_VEL = 0.8f;
const float KD_VEL = 0.08f;

// PID de distancia: valores iniciales, requieren calibracion
const float KP_DIST = 1.5f;
const float KI_DIST = 0.8f;
const float KD_DIST = 0.08f;

const float DISTANCIA_OBJETIVO = 35.0f;   // equivale a alrededor de 2.5 metros de distancia

// Ganancias de trayectoria: valores iniciales, requieren calibracion
const float KP_LATERAL = 20.0f;
const float KP_ANGULAR = 20.0f;

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
void encR() { cuentaR += digitalRead(PIN_ENC_R_B) ? 1 : -1; }  // cambiar de lugar el 1 con el -1 si un encoder da valores negativos (muy probable)

// PID DE VELOCIDAD
class PIDVelocidad {
    public:
        float integral = 0;
        float errorAnterior = 0;

        int calcular(float objetivo, float actual, float dt) {
            float error = objetivo - actual;

            float derivada = (error - errorAnterior) / dt;
            float integralNueva = integral + error * dt;

            float salida = (KP_VEL * error) + (KI_VEL * integralNueva) + (KD_VEL * derivada);

            // Limitar salida PWM y evitar acumulacion excesiva (anti windup basico)
            if (salida >= 0 && salida <= RPM_MAX) {
                integral = integralNueva;
            }

            salida = (KP_VEL * error) + (KI_VEL * integral) + (KD_VEL * derivada);

            errorAnterior = error;
            return constrain((int)round(salida), 0, RPM_MAX);
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

        float calcular(float error, float dt) {
            float derivada = (error - errorAnterior) / dt;

            float integralNueva = integral + error * dt;

            float salida = (KP_DIST * error) + (KI_DIST * integralNueva) + (KD_DIST * derivada);

            // Limitar salida PWM y evitar acumulacion excesiva (anti windup basico)
            if (salida >= 0 && salida <= RPM_MAX) {
                integral = integralNueva;
            }

            salida = (KP_DIST * error) + (KI_DIST * integral) + (KD_DIST * derivada);
            errorAnterior = error;
            return constrain(salida, 0, RPM_MAX);
        }
        void reiniciar() {
            integral = 0;
            errorAnterior = 0;
        }
};
PIDDistancia pidDistancia;

PIDVelocidad pidLeft, pidRight;

// ESTADO
volatile float errorLateral = 0.0f;   // Vector3.x    // f para indicar que es Float32 (sin f es Float64)
volatile float errorAngular = 0.0f;   // Vector3.y
volatile int distancia = 0;           // Vector3.z    // en realidad es el ancho de la cara

unsigned long ultimaOrden = 0;
unsigned long ultimoControl = 0;
long anteriorLeft = 0, anteriorRight = 0;

volatile bool usuarioDetectado = false;
bool ordenRecibida = false;


// VARIABLES MICRO-ROS
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_subscription_t sub_valores_pid;
geometry_msgs__msg__Vector3 vec_para_pid;

rcl_subscription_t sub_cara_detectada;
std_msgs__msg__Bool msg_cara_detectada;

rcl_publisher_t pub_btn_control;    // editar para publicar estado de botones A.B,C y D (tal vez hacer un vector de bools si se puede)
std_msgs__msg__Bool msg_pub_btn;

float bufferControl[2];
float bufferEstado[4];

void callback_valores_pid(const void * msgin) {
    const geometry_msgs__msg__Vector3 * msg =
        (const geometry_msgs__msg__Vector3 *)msgin;

    errorLateral = constrain((float)msg->x, -1.0f, 1.0f);
    errorAngular = constrain((float)msg->y, -1.0f, 1.0f);
    distancia = (int)msg->z;

    ultimaOrden = millis();
    ordenRecibida = true;
}

void callback_cara_detectada(const void * msgin) {
    const std_msgs__msg__Bool * msg =
        (const std_msgs__msg__Bool *)msgin;

    usuarioDetectado = msg->data;
}

// rmw_uros_ping_agent --------------------------------------------------------------------------------------
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
    ST.setTimeout(1000); // para que si no llegan comandos nuevos dentro de 1 segundos se detienen los motores (settimeout(0) para desactivar)
                       // un delay en el codigo puede activar el timeout!!

    // Iniciacion MICRO-ROS (nodos, subscribers, publishers, etc)
    set_microros_transports();  // solo usar serial para microros y no serial.print
    delay(1000);

    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {  // se espera a que el agente de microros en la raspi este activa
        ST.motor(1, 0);
        ST.motor(2, 0);
        delay(100);
    }

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "control_motores", "", &support);

    // Subscriber
    rclc_subscription_init_default(
        &sub_valores_pid,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "/pid_data"
    );
    rclc_subscription_init_default(
        &sub_cara_detectada,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/user_detected"
    );

    // Publisher
    // rclc_publisher_init_default(
    //     &publisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    //     "/motor_status"
    // );

    // CONFIGURACIÓN DE MENSAJES  --> no usado para geometry..Vector3
    // mensajeEstado.data.data = bufferEstado;
    // mensajeEstado.data.capacity = 4;
    // mensajeEstado.data.size = 4;

    // EXECUTOR
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &sub_valores_pid, &vec_para_pid, &callback_valores_pid, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_cara_detectada, &msg_cara_detectada, &callback_cara_detectada, ON_NEW_DATA);

    ultimoControl = millis();
}



// LOOP
void loop() {
    // Procesar mensajes ROS 2 entrantes
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

    unsigned long ahora = millis();

    // Si se pierde la comunicacion, detener el robot.
    if (!ordenRecibida || ahora - ultimaOrden > TIMEOUT_RASPBERRY_MS) {
        ST.motor(1, 0);
        ST.motor(2, 0);
        pidLeft.reiniciar();
        pidRight.reiniciar();
        pidDistancia.reiniciar();
        return;
    }

    if (ahora - ultimoControl < DT_MS) return;  // se sale del loop si han pasado menos de DT milisegundos ya que sigue con otra tarea

    float dt = (ahora - ultimoControl) / 1000.0f;  // se updatea dt por si acaso hizo return antes
    ultimoControl = ahora;

    if (dt <= 0.0f) return;   // se sale del loop por si acaso dt es 0 (se haria division por 0 en rpmleft y right)

    noInterrupts(); // para que al obtener l y r no se detenga el codigo
    long l = cuentaL, r = cuentaR;
    interrupts();

    long deltaL = l- anteriorL;  // se calcula que tanto aumento el valor de los encoders
    long deltaR = r - anteriorR;
    anteriorL = l;
    anteriorR = r;

    float rpmLeft = deltaL * 60.0 / (PULSOS_VUELTA_LEFT * dt);
    float rpmRight = deltaR * 60.0 / (PULSOS_VUELTA_RIGHT * dt);

    // La Raspberry determina el sentido de la correccion con los errores.
    float correccion = KP_LATERAL * errorLateral + KP_ANGULAR * errorAngular;

    if (!usuarioDetectado) {
        ST.motor(1, 0);
        ST.motor(2, 0);
        pidLeft.reiniciar();
        pidRight.reiniciar();
        pidDistancia.reiniciar();
        return;
    }

    float rpmBaseDinamica = 0.0f;
    float errorDist = (float)distancia - DISTANCIA_OBJETIVO;  // actual - objetivo (mientras mas cerca esta la persona, "distancia" aumenta por ser el ancho de la cara)
    rpmBaseDinamica = pidDistancia.calcular(errorDist, dt);

    float objetivoLeft = constrain(rpmBaseDinamica - correccion, 0.0, RPM_MAX);   // invertir signos si dobla a direccion opuesta
    float objetivoRight = constrain(rpmBaseDinamica + correccion, 0.0, RPM_MAX);  // cambiar por RPM_BASE para testear pid de rpmleft y right

    // PID independiente para cada rueda.
    int velLeft = pidLeft.calcular(objetivoLeft, rpmLeft, dt);
    int velRight = pidRight.calcular(objetivoRight, rpmRight, dt);

    // para avanzar, ambos valores son positivos
    ST.motor(1, velLeft);
    ST.motor(2, velRight);
}

