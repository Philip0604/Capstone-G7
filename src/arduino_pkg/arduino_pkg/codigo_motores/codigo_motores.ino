#include <micro_ros_arduino.h>
#include <std_msgs/msg/string.h>

// Código motores con micro-ROS, reconexión automática con el agente y control de velocidad y distancia con PID.
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
const unsigned long TIMEOUT_RASPBERRY_MS = 500;   // sin /pid_data por este tiempo -> detenerse
const unsigned long TIMEOUT_CARA_MS = 1000;       // sin /user_detected por este tiempo -> se considera sin usuario
const unsigned long REPETIR_STOP_MS = 250;        // mientras esta detenido, se repite la orden de stop cada este tiempo

// Logs: cada cuanto se publica /arduino/estado (leer en la Raspy con: ros2 topic echo /arduino/estado)
const unsigned long PERIODO_LOG_MS = 500;

// Reconexion con el agente micro-ROS
const unsigned long PING_ESPERANDO_MS = 500;      // cada cuanto se busca al agente si no hay conexion
const unsigned long PING_CONECTADO_MS = 500;      // cada cuanto se revisa que el agente siga vivo

// Ganancias PID y objetivos
const float KP_VEL = 1.5f; // PID de velocidad: valores iniciales, requieren calibracion (f para Float32; calculos mas rapidos)
const float KI_VEL = 0.8f;
const float KD_VEL = 0.08f;

const float KP_DIST = 1.5f; // PID de distancia: valores iniciales, requieren calibracion
const float KI_DIST = 0.8f;
const float KD_DIST = 0.08f;

const float KP_LATERAL = 2.0f; // Ganancias de trayectoria: valores iniciales, requieren calibracion
const float KP_ANGULAR = 2.0f;

const float DISTANCIA_OBJETIVO = 35.0f;   // equivale a alrededor de 2.5 metros de distancia

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

class PIDVelocidad { // PID DE VELOCIDAD
    public:
        float integral = 0;
        float errorAnterior = 0;
        bool primero = true;   // en la primera llamada no hay derivada (evita un "salto")

        int calcular(float objetivo, float actual, float dt) {
            float error = objetivo - actual;

            float derivada = primero ? 0.0f : (error - errorAnterior) / dt;
            primero = false;
            float integralNueva = integral + error * dt;

            // El objetivo es la velocidad deseada, actual es la velocidad medida, ambas en RPM. Se calcula la salida del PID.
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
            primero = true;
        }
};

// PID Distancia
class PIDDistancia {
    public:
        float integral = 0;
        float errorAnterior = 0;
        bool primero = true;

        float calcular(float error, float dt) {
            float derivada = primero ? 0.0f : (error - errorAnterior) / dt;
            primero = false;
            float integralNueva = integral + error * dt;

            float salida = (KP_DIST * error) + (KI_DIST * integralNueva) + (KD_DIST * derivada);
            // Limitar salida PWM y evitar acumulacion excesiva (anti windup basico)
            if (salida >= 0 && salida <= RPM_MAX) {
                integral = integralNueva;
            }
            salida = (KP_DIST * error) + (KI_DIST * integral) + (KD_DIST * derivada);

            errorAnterior = error;
            return constrain(salida, 0.0f, (float)RPM_MAX);
        }

        void reiniciar() {
            integral = 0;
            errorAnterior = 0;
            primero = true;
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
unsigned long ultimaCara = 0;
unsigned long ultimoStop = 0;
long anteriorLeft = 0, anteriorRight = 0;

volatile bool usuarioDetectado = false;
bool ordenRecibida = false;
bool motoresDetenidos = false;   // false para que el primer detenerMotores() mande el stop altiro

// VARIABLES MICRO-ROS
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_subscription_t sub_valores_pid;
geometry_msgs__msg__Vector3 vec_para_pid;

rcl_subscription_t sub_cara_detectada;
std_msgs__msg__Bool msg_cara_detectada;

// LOGS -> /arduino/estado (std_msgs/String)
rcl_publisher_t pub_estado;
std_msgs__msg__String msg_estado;
char buffer_estado[300];
unsigned long ultimoLog = 0;
const char * motivo = "iniciando";   // por que el robot esta (o no) moviendose
// ultimos valores del control, solo para el log
float logBase = 0, logObjL = 0, logObjR = 0, logRpmL = 0, logRpmR = 0;
int logVelL = 0, logVelR = 0;

// Estados de la conexion con el agente (Raspberry)
enum EstadoConexion { ESPERANDO_AGENTE, AGENTE_DISPONIBLE, CONECTADO, DESCONECTADO };
EstadoConexion estadoConexion = ESPERANDO_AGENTE;
unsigned long ultimoPing = 0;

rcl_publisher_t pub_btn_control;    // editar para publicar estado de botones A.B,C y D (tal vez hacer un vector de bools si se puede)
std_msgs__msg__Bool msg_pub_btn;    // No se usa aún

float bufferControl[2];             // No se usa aún
float bufferEstado[4];              // No se usa aún

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
    ultimaCara = millis();
}

// Creacion - destruccion de las entidades micro-ROS (nodo, suscripciones, executor) cada vez que se (re)conecta el agente.
// Esto permite que si el agente se cierra, se pierda la conexion y luego se vuelva a abrir, el Arduino pueda reconectarse sin reiniciarse.
// Obtenido de: https://github.com/micro-ROS/micro_ros_arduino/blob/kilted/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
bool crearEntidades() {
    allocator = rcl_get_default_allocator();

    // Todo en "cero" antes de crear, para que destruirEntidades() sea seguro si algo falla a medias
    node = rcl_get_zero_initialized_node();
    sub_valores_pid = rcl_get_zero_initialized_subscription();
    sub_cara_detectada = rcl_get_zero_initialized_subscription();
    executor = rclc_executor_get_zero_initialized_executor();
    pub_estado = rcl_get_zero_initialized_publisher();

    // Crear nodo y suscripciones y verificar que no haya errores. Si hay error, se sale y se destruyen las entidades (si es que se crearon).
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "control_motores", "", &support) != RCL_RET_OK) return false;
    if (rclc_subscription_init_default(
            &sub_valores_pid, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "/pid_data") != RCL_RET_OK) return false;
    if (rclc_subscription_init_default(
            &sub_cara_detectada, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "/user_detected") != RCL_RET_OK) return false;

    // Publisher de logs (texto)
    if (rclc_publisher_init_default(
            &pub_estado, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/arduino/estado") != RCL_RET_OK) return false;
    msg_estado.data.data = buffer_estado;
    msg_estado.data.capacity = sizeof(buffer_estado);
    msg_estado.data.size = 0;

    if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) return false; // EXECUTOR (2 = cantidad de suscripciones)
    if (rclc_executor_add_subscription(&executor, &sub_valores_pid, &vec_para_pid,
            &callback_valores_pid, ON_NEW_DATA) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(&executor, &sub_cara_detectada, &msg_cara_detectada,
            &callback_cara_detectada, ON_NEW_DATA) != RCL_RET_OK) return false;

    return true;
}

void destruirEntidades() {
    // Timeout 0: no esperar respuesta del agente (puede que ya no exista)
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    (void) rcl_subscription_fini(&sub_valores_pid, &node);
    (void) rcl_subscription_fini(&sub_cara_detectada, &node);
    (void) rcl_publisher_fini(&pub_estado, &node);
    (void) rclc_executor_fini(&executor);
    (void) rcl_node_fini(&node);
    (void) rclc_support_fini(&support);
}

//  Maquina de estados de la conexion. Se llama en cada vuelta del loop.
//   ESPERANDO_AGENTE  -> ping cada 0.5 s hasta que el agente responda
//   AGENTE_DISPONIBLE -> crear nodo y suscripciones
//   CONECTADO         -> recibir mensajes; ping cada 0.5 s para saber si sigue vivo
//   DESCONECTADO      -> destruir todo y volver a esperar
void gestionarConexion() {
    unsigned long ahora = millis();

    switch (estadoConexion) {
        case ESPERANDO_AGENTE:
            if (ahora - ultimoPing >= PING_ESPERANDO_MS) {
                ultimoPing = ahora;
                if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                    estadoConexion = AGENTE_DISPONIBLE;
                }
            }
            break;

        case AGENTE_DISPONIBLE:
            if (crearEntidades()) {
                estadoConexion = CONECTADO;
                ultimoPing = ahora;
            } else {
                destruirEntidades();
                estadoConexion = ESPERANDO_AGENTE;
            }
            break;

        case CONECTADO:
            if (ahora - ultimoPing >= PING_CONECTADO_MS) {
                ultimoPing = ahora;
                if (rmw_uros_ping_agent(50, 2) != RMW_RET_OK) {
                    estadoConexion = DESCONECTADO;
                    break;
                }
            }
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)); // procesar mensajes ROS 2 entrantes
            break;

        case DESCONECTADO:
            destruirEntidades(); // Olvidar los ultimos datos: al reconectar se espera informacion nueva
            ordenRecibida = false;
            usuarioDetectado = false;
            estadoConexion = ESPERANDO_AGENTE;
            break;
    }
}

// Motores
void detenerMotores(unsigned long ahora) {  // Detiene los motores. Se repite cada REPETIR_STOP_MS (no cada vuelta del loop, o llenaria el Serial2 y frenaria el loop).
    if (!motoresDetenidos || ahora - ultimoStop >= REPETIR_STOP_MS) {
        ST.motor(1, 0);
        ST.motor(2, 0);
        ultimoStop = ahora;
    }
    if (!motoresDetenidos) {
        pidLeft.reiniciar();
        pidRight.reiniciar();
        pidDistancia.reiniciar();
    }
    motoresDetenidos = true;
}

// Se crea una función encargada de realizar el control de los motores, que se llama en cada vuelta del loop.
// Esto permite que si se pierde la comunicación con la Raspberry, el robot se detenga automáticamente.
void controlar(unsigned long ahora) {
    // Al volver de estar detenido: partir el control "desde cero" (si no, dt y la diferencia de encoders incluirian todo el tiempo detenido).
    if (motoresDetenidos) {
        noInterrupts();
        anteriorL = cuentaL;
        anteriorR = cuentaR;
        interrupts();
        ultimoControl = ahora;
        motoresDetenidos = false;
        return;
    }

    if (ahora - ultimoControl < DT_MS) return;  // sale del loop si han pasado menos de DT milisegundos ya que sigue con otra tarea

    float dt = (ahora - ultimoControl) / 1000.0f;  // se updatea dt por si acaso hizo return antes
    ultimoControl = ahora;

    if (dt <= 0.0f) return;   // se sale del loop por si acaso dt es 0 (se haria division por 0 en rpmleft y right)

    noInterrupts(); // para que al obtener l y r sus valores no cambien mientras se leen
    long l = cuentaL, r = cuentaR;
    interrupts();

    long deltaL = l - anteriorL;  // se calcula que tanto aumento el valor de los encoders
    long deltaR = r - anteriorR;
    anteriorL = l;
    anteriorR = r;

    float rpmLeft = deltaL * 60.0f / (PULSOS_VUELTA_LEFT * dt);
    float rpmRight = deltaR * 60.0f / (PULSOS_VUELTA_RIGHT * dt);

    // Velocidad base segun la distancia al usuario (mientras mas cerca esta la persona, "distancia" aumenta por ser el ancho de la cara)
    float errorDist = (float)distancia - DISTANCIA_OBJETIVO;  // actual - objetivo (mientras mas cerca esta la persona, "distancia" aumenta por ser el ancho de la cara)
    float rpmBaseDinamica = pidDistancia.calcular(errorDist, dt);

    // La Raspberry determina el sentido de la correccion con los errores.
    float correccion = KP_LATERAL * errorLateral + KP_ANGULAR * errorAngular;
    // La correccion no puede ser mayor que la velocidad base: si el robot esta esperando al usuario (base = 0) no gira en el lugar, y al avanzar lento gira suave.
    correccion = constrain(correccion, -rpmBaseDinamica, rpmBaseDinamica);

    float objetivoLeft = constrain(rpmBaseDinamica - correccion, 0.0, RPM_MAX);   // invertir signos si dobla a direccion opuesta
    float objetivoRight = constrain(rpmBaseDinamica + correccion, 0.0, RPM_MAX);  // cambiar por RPM_BASE para testear pid de rpmleft y right

    // PID independiente para cada rueda.
    int velLeft = pidLeft.calcular(objetivoLeft, rpmLeft, dt);
    int velRight = pidRight.calcular(objetivoRight, rpmRight, dt);

    // valores para el log
    logBase = rpmBaseDinamica; logObjL = objetivoLeft; logObjR = objetivoRight;
    logRpmL = rpmLeft; logRpmR = rpmRight; logVelL = velLeft; logVelR = velRight;
    if (rpmBaseDinamica <= 0.0f) {
        motivo = "QUIETO: velocidad base 0 (usuario lejos o a la distancia objetivo: cara <= 35 px)";
    } else {
        motivo = "MOVIENDO";
    }

    // para avanzar, ambos valores son positivos
    ST.motor(1, velLeft);
    ST.motor(2, velRight);
}

// LOGS: publica un texto en /arduino/estado con el motivo y los valores del control.
// Los decimales se mandan x100 como enteros (ej: lat=30 -> 0.30), porque printf
// con %f no siempre funciona en el Arduino.
void publicarEstado(unsigned long ahora) {
    if (estadoConexion != CONECTADO || ahora - ultimoLog < PERIODO_LOG_MS) return;
    ultimoLog = ahora;

    long hacePid = ordenRecibida ? (long)(ahora - ultimaOrden) : -1;
    long haceCara = (ultimaCara > 0) ? (long)(ahora - ultimaCara) : -1;
    bool detenido = motoresDetenidos;

    int n = snprintf(buffer_estado, sizeof(buffer_estado),
        "%s | pid_data hace %ldms lat=%d ang=%d cara=%dpx | usuario=%d (hace %ldms) | "
        "base=%d objL=%d objR=%d rpmL=%d rpmR=%d cmdL=%d cmdR=%d",
        motivo, hacePid,
        (int)(errorLateral * 100), (int)(errorAngular * 100), distancia,
        (int)usuarioDetectado, haceCara,
        detenido ? 0 : (int)logBase, detenido ? 0 : (int)logObjL, detenido ? 0 : (int)logObjR,
        detenido ? 0 : (int)logRpmL, detenido ? 0 : (int)logRpmR,
        detenido ? 0 : logVelL, detenido ? 0 : logVelR);
    if (n < 0) return;
    if (n >= (int)sizeof(buffer_estado)) n = sizeof(buffer_estado) - 1;
    msg_estado.data.size = n;
    (void) rcl_publish(&pub_estado, &msg_estado, NULL);
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
    Serial2.begin(9600);    // serial2 para motores. 9600 es el default (pin D18 arduino conectado a S1 sabertooth)
    Serial3.begin(115200);  // serial3 para comunicacion esp            (pin D17 esp con arduino o con raspi)
    ST.setTimeout(1000);    // para que si no llegan comandos nuevos dentro de 1 segundos se detienen los motores (settimeout(0) para desactivar)
                            // un delay en el codigo puede activar el timeout!!
    ST.motor(1, 0);
    ST.motor(2, 0);

    // MICRO-ROS: solo se configura el transporte. La conexion con el agente la
    // maneja gestionarConexion() en el loop (y se reconecta sola si se pierde).
    set_microros_transports();  // solo usar serial para microros y no serial.print
    delay(100);
}

// LOOP
void loop() {
    gestionarConexion();
    unsigned long ahora = millis();
    publicarEstado(ahora);   // log cada PERIODO_LOG_MS (solo si hay conexion)

    if (estadoConexion != CONECTADO) { // Sin conexion con la Raspberry -> detenido
        motivo = "DETENIDO: sin conexion con el agente";
        detenerMotores(ahora);
        return;
    }

    if (!ordenRecibida || ahora - ultimaOrden > TIMEOUT_RASPBERRY_MS) { // Sin ordenes recientes (pista perdida o nodo caido) -> detenido
        motivo = ordenRecibida
            ? "DETENIDO: /pid_data dejo de llegar (pista perdida o pid_message_node caido)"
            : "DETENIDO: nunca llego /pid_data (pista no detectada o pid_message_node no corre)";
        detenerMotores(ahora);
        return;
    }

    if (!usuarioDetectado || (ahora - ultimaCara > TIMEOUT_CARA_MS)) { // Sin usuario (o sin noticias del nodo de la cara hace rato) -> detenido
        motivo = (ahora - ultimaCara > TIMEOUT_CARA_MS)
            ? "DETENIDO: no llega /user_detected (face_detection_node no corre?)"
            : "DETENIDO: /user_detected = false (no se ve al usuario)";
        detenerMotores(ahora);
        return;
    }

    controlar(ahora);
}