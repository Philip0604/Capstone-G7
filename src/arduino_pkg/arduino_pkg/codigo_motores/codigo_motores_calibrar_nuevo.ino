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
const float RPM_MAX = 160.0f;  // rpm max estimada multiplicando 0.75 por el rpm max sin ruedas (al medirlo da alrededor de 225 rpm con vel = 100)
                               // para llegar a 6km/h (1,67m/s) se necesita como 200 rpm, entonces con 160 se llega a una velocidad de 4,8km/h (1,34m/s)

const int CMD_MAX = 100;  // valor maximo para comandos a motores (va de -127 a 127)

const float PULSOS_VUELTA = 480.0f;    // cantidad de pulsos medidos por encoder por revolucion (valor para pololu 4752 de 1:30)

const unsigned long DT_MS = 50;
const unsigned long TIMEOUT_RASPBERRY_MS = 500;   // sin /pid_data por este tiempo -> detenerse
const unsigned long TIMEOUT_CARA_MS = 1000;       // sin /user_detected por este tiempo -> se considera sin usuario
const unsigned long REPETIR_STOP_MS = 250;        // mientras esta detenido, se repite la orden de stop cada este tiempo

// Logs: cada cuanto se publica /arduino/estado (leer en la Raspy con: ros2 topic echo /arduino/estado)
const unsigned long PERIODO_LOG_MS = 500;
const unsigned long PERIODO_LOG_PRUEBA_MS = 100;  // mas seguido durante una prueba de calibracion
const unsigned long DURACION_PRUEBA_MS = 8000;    // las pruebas de calibracion se detienen solas

// Reconexion con el agente micro-ROS
const unsigned long PING_ESPERANDO_MS = 500;      // cada cuanto se busca al agente si no hay conexion
const unsigned long PING_CONECTADO_MS = 500;      // cada cuanto se revisa que el agente siga vivo

// ======================= GANANCIAS (CALIBRACION) =======================
// Son VARIABLES: se pueden cambiar en vivo desde la Raspberry, sin volver a cargar el codigo:
//   ros2 topic pub --once /arduino/ajuste std_msgs/String "data: 'kp_vel 0.5'"
//   ros2 topic pub --once /arduino/ajuste std_msgs/String "data: 'ver'"      (muestra todas)
//   ros2 topic echo /arduino/estado                                            (ver resultados)
// Los cambios se pierden al reiniciar el Arduino: cuando queden bien, copiar los valores aca.
//
// PRUEBAS (el robot SE MUEVE sin mirar la pista ni al usuario: ruedas en el aire o espacio libre;
// se detienen solas a los 8 s, o con 'stop'):
//   'prueba_cmd 50'  comando fijo a ambas ruedas, sin PID -> medir rpm  => kf_vel = 50 / rpm medido
//   'prueba_rpm 80'  ambas ruedas a 80 rpm con el PID   -> ajustar kp_vel / ki_vel
//
// Orden recomendado: 1) kf_vel  2) kp_vel, ki_vel  3) distancia  4) direccion (kp_lat, kp_ang)

// 1-2. Velocidad de cada rueda:  comando (0..CMD_MAX) = KF_VEL*objetivo + PID(objetivo - rpm medido)
float KF_VEL = 0.55f;     // comando por rpm (feedforward). Inicial: 100 cmd / (0.75 * 225 rpm) ~ 0.59
float KP_VEL = 0.40f;     // comando por rpm de error
float KI_VEL = 1.00f;     // comando por (rpm de error * s): elimina el error que deja KF
float KD_VEL = 0.00f;
float FILTRO_RPM = 0.5f;  // filtro de la medicion de rpm (1 = sin filtro). 1 pulso en 50 ms = 2.5 rpm de ruido

// 3. Distancia al usuario -> velocidad base de ambas ruedas (rpm)
// El ancho de la cara (px) NO es lineal con la distancia: ancho * distancia ~ constante.
// Medido: 33 px ~ 2.7 m y 37 px ~ 2.4 m  ->  K_CARA ~ 89 px*m. Recalibrar si se cambia la webcam o su resolucion.
float K_CARA = 89.0f;               // px * m
float DISTANCIA_OBJETIVO_M = 2.2f;  // m (equivale a 40 px, el objetivo anterior)
float ZONA_MUERTA_M = 0.10f;        // si el usuario esta a menos de 10 cm de mas cerca que el objetivo no se avanza
float KP_DIST = 80.0f;              // rpm por metro que el usuario esta MAS CERCA que el objetivo
float KI_DIST = 0.0f;               // rpm por (m * s): con > 0 se elimina la diferencia al caminar a velocidad constante
float KD_DIST = 0.0f;
float ACEL_MAX = 150.0f;            // rpm/s: la velocidad base sube como maximo esto por segundo (arranque suave)
float DECEL_MAX = 400.0f;           // rpm/s: frena mas rapido de lo que acelera

// 4. Direccion (seguimiento de pista). Los errores llegan en [-1, 1].
// La correccion es una FRACCION de la velocidad base: asi el robot dobla igual de "cerrado" rapido o lento.
//   correccion (rpm) = base * (KP_LATERAL*errorLateral + KP_ANGULAR*errorAngular), limitada a +-CORRECCION_MAX
// Fraccion 1.0 => una rueda se detiene y la otra va al doble.
float KP_LATERAL = 0.35f;
float KP_ANGULAR = 0.50f;
float CORRECCION_MAX = 0.80f;

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
void encR() { cuentaR += digitalRead(PIN_ENC_R_B) ? -1 : 1; }

// PID con anti-windup y derivada sobre la medicion (no "salta" al cambiar el objetivo).
// Las ganancias son punteros para poder cambiarlas en vivo con /arduino/ajuste.
class PID {
    public:
        float *kp, *ki, *kd;
        float minSalida, maxSalida;
        float integral = 0;
        float medicionAnterior = 0;
        bool primero = true;   // en la primera llamada no hay derivada

        PID(float *p, float *i, float *d, float mn, float mx)
            : kp(p), ki(i), kd(d), minSalida(mn), maxSalida(mx) {}

        // error = objetivo - medicion. 'extra' = feedforward (se suma a la salida)
        float calcular(float error, float medicion, float dt, float extra = 0.0f) {
            float derivada = primero ? 0.0f : -(medicion - medicionAnterior) / dt;
            primero = false;
            medicionAnterior = medicion;

            float sinIntegral = (*kp) * error + (*kd) * derivada + extra;
            float integralNueva = integral + error * dt;
            float salida = sinIntegral + (*ki) * integralNueva;

            // Anti windup: no acumular si la salida esta saturada y el error la empuja mas alla
            bool saturadoAlto = (salida > maxSalida && error > 0);
            bool saturadoBajo = (salida < minSalida && error < 0);
            if (!saturadoAlto && !saturadoBajo) {
                integral = integralNueva;
            }
            return constrain(sinIntegral + (*ki) * integral, minSalida, maxSalida);
        }

        void reiniciar() {
            integral = 0;
            medicionAnterior = 0;
            primero = true;
        }
};

PID pidDistancia(&KP_DIST, &KI_DIST, &KD_DIST, 0.0f, RPM_MAX);
PID pidLeft(&KP_VEL, &KI_VEL, &KD_VEL, 0.0f, (float)CMD_MAX);
PID pidRight(&KP_VEL, &KI_VEL, &KD_VEL, 0.0f, (float)CMD_MAX);

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
float rpmFiltL = 0, rpmFiltR = 0;   // rpm medidos (filtrados)
float rpmBase = 0;                  // velocidad base despues de la rampa de aceleracion
float distUsuarioM = 0;             // distancia estimada al usuario (solo para el log)

// Pruebas de calibracion (comandos 'prueba_cmd' y 'prueba_rpm' por /arduino/ajuste)
enum TipoPrueba { SIN_PRUEBA, PRUEBA_CMD, PRUEBA_RPM };
TipoPrueba prueba = SIN_PRUEBA;
float valorPrueba = 0;
unsigned long inicioPrueba = 0;

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
char buffer_estado[400];
unsigned long ultimoLog = 0;
const char * motivo = "iniciando";   // por que el robot esta (o no) moviendose
// ultimos valores del control, solo para el log
float logBase = 0, logObjL = 0, logObjR = 0, logRpmL = 0, logRpmR = 0;
int logVelL = 0, logVelR = 0;

// Estados de la conexion con el agente (Raspberry)
enum EstadoConexion { ESPERANDO_AGENTE, AGENTE_DISPONIBLE, CONECTADO, DESCONECTADO };
EstadoConexion estadoConexion = ESPERANDO_AGENTE;
unsigned long ultimoPing = 0;

// AJUSTE EN VIVO <- /arduino/ajuste (std_msgs/String)
rcl_subscription_t sub_ajuste;
std_msgs__msg__String msg_ajuste;
char buffer_ajuste[64];

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

// ---------------------------------------------------------------- AJUSTE EN VIVO
struct Ajuste { const char *nombre; float *valor; float minimo; float maximo; };
Ajuste AJUSTES[] = {
    {"kf_vel", &KF_VEL, 0.0f, 5.0f},        {"kp_vel", &KP_VEL, 0.0f, 10.0f},
    {"ki_vel", &KI_VEL, 0.0f, 20.0f},       {"kd_vel", &KD_VEL, 0.0f, 2.0f},
    {"filtro_rpm", &FILTRO_RPM, 0.05f, 1.0f},
    {"k_cara", &K_CARA, 10.0f, 500.0f},     {"dist_obj", &DISTANCIA_OBJETIVO_M, 0.5f, 6.0f},
    {"zona_muerta", &ZONA_MUERTA_M, 0.0f, 1.0f},
    {"kp_dist", &KP_DIST, 0.0f, 400.0f},    {"ki_dist", &KI_DIST, 0.0f, 200.0f},
    {"kd_dist", &KD_DIST, 0.0f, 100.0f},
    {"acel", &ACEL_MAX, 10.0f, 2000.0f},    {"decel", &DECEL_MAX, 10.0f, 5000.0f},
    {"kp_lat", &KP_LATERAL, 0.0f, 3.0f},    {"kp_ang", &KP_ANGULAR, 0.0f, 3.0f},
    {"corr_max", &CORRECCION_MAX, 0.0f, 1.0f},
};
const int N_AJUSTES = sizeof(AJUSTES) / sizeof(AJUSTES[0]);

// "1.234" sin usar printf("%f") (no siempre funciona en el Arduino)
void floatATexto(float v, char *out, size_t n) {
    long milesimas = lround(v * 1000.0f);
    const char *signo = milesimas < 0 ? "-" : "";
    milesimas = labs(milesimas);
    snprintf(out, n, "%s%ld.%03ld", signo, milesimas / 1000, milesimas % 1000);
}

void publicarTexto(const char *texto) {
    if (estadoConexion != CONECTADO) return;
    size_t n = strlen(texto);
    if (n >= sizeof(buffer_estado)) n = sizeof(buffer_estado) - 1;
    memmove(buffer_estado, texto, n);
    buffer_estado[n] = '\0';
    msg_estado.data.size = n;
    (void) rcl_publish(&pub_estado, &msg_estado, NULL);
}

void publicarAjustes() {
    char texto[400];
    char num[16];
    size_t largo = snprintf(texto, sizeof(texto), "AJUSTES:");
    for (int i = 0; i < N_AJUSTES && largo < sizeof(texto); i++) {
        floatATexto(*AJUSTES[i].valor, num, sizeof(num));
        largo += snprintf(texto + largo, sizeof(texto) - largo, " %s=%s", AJUSTES[i].nombre, num);
    }
    publicarTexto(texto);
}

void iniciarPrueba(TipoPrueba tipo, float valor) {
    prueba = (valor > 0) ? tipo : SIN_PRUEBA;
    valorPrueba = valor;
    inicioPrueba = millis();
}

// Comandos: "ver" | "stop" | "prueba_cmd <cmd>" | "prueba_rpm <rpm>" | "<nombre> <valor>"
void procesarAjuste(char *linea) {
    char respuesta[120];
    char num[16];
    char *nombre = strtok(linea, " =\t\r\n");
    char *valorTxt = strtok(NULL, " =\t\r\n");
    if (nombre == NULL) return;

    if (strcmp(nombre, "ver") == 0) { publicarAjustes(); return; }
    if (strcmp(nombre, "stop") == 0) { prueba = SIN_PRUEBA; publicarTexto("AJUSTE: prueba detenida"); return; }

    if (valorTxt == NULL) {
        snprintf(respuesta, sizeof(respuesta), "AJUSTE: falta el valor de '%s'", nombre);
        publicarTexto(respuesta);
        return;
    }
    char *fin;
    float valor = strtof(valorTxt, &fin);
    if (fin == valorTxt) {
        snprintf(respuesta, sizeof(respuesta), "AJUSTE: '%s' no es un numero", valorTxt);
        publicarTexto(respuesta);
        return;
    }

    if (strcmp(nombre, "prueba_cmd") == 0 || strcmp(nombre, "prueba_rpm") == 0) {
        bool esCmd = strcmp(nombre, "prueba_cmd") == 0;
        valor = constrain(valor, 0.0f, esCmd ? (float)CMD_MAX : RPM_MAX);
        iniciarPrueba(esCmd ? PRUEBA_CMD : PRUEBA_RPM, valor);
        floatATexto(valor, num, sizeof(num));
        snprintf(respuesta, sizeof(respuesta), "AJUSTE: %s %s por %lu ms (CUIDADO: el robot se mueve)",
                 nombre, num, DURACION_PRUEBA_MS);
        publicarTexto(respuesta);
        return;
    }

    for (int i = 0; i < N_AJUSTES; i++) {
        if (strcmp(nombre, AJUSTES[i].nombre) == 0) {
            *AJUSTES[i].valor = constrain(valor, AJUSTES[i].minimo, AJUSTES[i].maximo);
            floatATexto(*AJUSTES[i].valor, num, sizeof(num));
            snprintf(respuesta, sizeof(respuesta), "AJUSTE: %s = %s", nombre, num);
            publicarTexto(respuesta);
            return;
        }
    }
    snprintf(respuesta, sizeof(respuesta), "AJUSTE: no existe '%s' (usar 'ver')", nombre);
    publicarTexto(respuesta);
}

void callback_ajuste(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    char linea[sizeof(buffer_ajuste)];
    size_t n = msg->data.size < sizeof(linea) - 1 ? msg->data.size : sizeof(linea) - 1;
    memcpy(linea, msg->data.data, n);
    linea[n] = '\0';
    procesarAjuste(linea);
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
    sub_ajuste = rcl_get_zero_initialized_subscription();
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
    if (rclc_subscription_init_default(
            &sub_ajuste, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/arduino/ajuste") != RCL_RET_OK) return false;
    msg_ajuste.data.data = buffer_ajuste;
    msg_ajuste.data.capacity = sizeof(buffer_ajuste);
    msg_ajuste.data.size = 0;

    // Publisher de logs (texto)
    if (rclc_publisher_init_default(
            &pub_estado, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/arduino/estado") != RCL_RET_OK) return false;
    msg_estado.data.data = buffer_estado;
    msg_estado.data.capacity = sizeof(buffer_estado);
    msg_estado.data.size = 0;

    if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK) return false; // EXECUTOR (3 = cantidad de suscripciones)
    if (rclc_executor_add_subscription(&executor, &sub_valores_pid, &vec_para_pid,
            &callback_valores_pid, ON_NEW_DATA) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(&executor, &sub_cara_detectada, &msg_cara_detectada,
            &callback_cara_detectada, ON_NEW_DATA) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(&executor, &sub_ajuste, &msg_ajuste,
            &callback_ajuste, ON_NEW_DATA) != RCL_RET_OK) return false;

    return true;
}

void destruirEntidades() {
    // Timeout 0: no esperar respuesta del agente (puede que ya no exista)
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    (void) rcl_subscription_fini(&sub_valores_pid, &node);
    (void) rcl_subscription_fini(&sub_cara_detectada, &node);
    (void) rcl_subscription_fini(&sub_ajuste, &node);
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
            prueba = SIN_PRUEBA;
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
        rpmBase = 0;
        rpmFiltL = rpmFiltR = 0;
    }
    motoresDetenidos = true;
}

// Velocidad base (rpm) segun la distancia al usuario, con zona muerta y rampa de aceleracion.
// La webcam es TRASERA: si el usuario se acerca (cara mas ancha) el robot avanza; si se aleja, espera.
float calcularBase(float dt) {
    float objetivo = 0.0f;
    if (distancia > 0) {
        distUsuarioM = K_CARA / (float)distancia;
        float error = DISTANCIA_OBJETIVO_M - distUsuarioM;   // > 0: usuario mas cerca que el objetivo -> avanzar
        if (error > ZONA_MUERTA_M) {
            // se resta la zona muerta para que la velocidad parta desde 0 (sin saltos)
            objetivo = pidDistancia.calcular(error - ZONA_MUERTA_M, distUsuarioM, dt);
        } else {
            pidDistancia.reiniciar();
        }
    } else {
        distUsuarioM = 0;
        pidDistancia.reiniciar();
    }
    // Rampa: limita cuanto cambia la velocidad base en cada ciclo
    rpmBase += constrain(objetivo - rpmBase, -DECEL_MAX * dt, ACEL_MAX * dt);
    if (objetivo <= 0.0f && rpmBase < 0.5f) rpmBase = 0.0f;   // evita quedar en 0.001 rpm
    return rpmBase;
}

// PID de una rueda: feedforward + PID. Con objetivo 0 se manda 0 (sin reversa).
int comandoRueda(PID &pid, float objetivo, float rpm, float dt) {
    if (objetivo <= 0.0f) {
        pid.reiniciar();
        return 0;
    }
    return (int)lroundf(pid.calcular(objetivo - rpm, rpm, dt, KF_VEL * objetivo));
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
        pidDistancia.reiniciar();
        pidLeft.reiniciar();
        pidRight.reiniciar();
        rpmBase = 0;
        rpmFiltL = rpmFiltR = 0;
        ultimoControl = ahora;
        motoresDetenidos = false;
        return;
    }

    if (ahora - ultimoControl < DT_MS) return;  // sale si han pasado menos de DT milisegundos ya que sigue con otra tarea

    float dt = (ahora - ultimoControl) / 1000.0f;  // se updatea dt por si acaso hizo return antes
    ultimoControl = ahora;

    if (dt <= 0.0f) return;   // se sale por si acaso dt es 0 (se haria division por 0 en rpmleft y right)

    noInterrupts(); // para que al obtener l y r sus valores no cambien mientras se leen
    long l = cuentaL, r = cuentaR;
    interrupts();

    long deltaL = l - anteriorL;  // se calcula que tanto aumento el valor de los encoders
    long deltaR = r - anteriorR;
    anteriorL = l;
    anteriorR = r;

    // revoluciones por minuto de cada rueda medidas con valores de encoders (filtradas: 1 pulso = 2.5 rpm de ruido)
    float rpmLeft = deltaL * 60.0f / (PULSOS_VUELTA * dt);
    float rpmRight = deltaR * 60.0f / (PULSOS_VUELTA * dt);
    rpmFiltL += FILTRO_RPM * (rpmLeft - rpmFiltL);
    rpmFiltR += FILTRO_RPM * (rpmRight - rpmFiltR);

    float base = 0.0f, objetivoLeft = 0.0f, objetivoRight = 0.0f;
    int velLeft = 0, velRight = 0;

    if (prueba == PRUEBA_CMD) {
        // Lazo abierto: para medir cuantas rpm da cada comando (=> KF_VEL)
        velLeft = velRight = constrain((int)lroundf(valorPrueba), 0, CMD_MAX);
        motivo = "PRUEBA: comando fijo (sin PID)";
    } else {
        if (prueba == PRUEBA_RPM) {
            objetivoLeft = objetivoRight = valorPrueba;
            motivo = "PRUEBA: rpm fijo (PID de ruedas)";
        } else {
            base = calcularBase(dt);
            // La Raspberry determina el sentido de la correccion con los errores
            // (correccion > 0 => rueda derecha mas rapida => gira a la izquierda).
            float fraccion = constrain(KP_LATERAL * errorLateral + KP_ANGULAR * errorAngular,
                                       -CORRECCION_MAX, CORRECCION_MAX);
            float correccion = base * fraccion;   // si base = 0 (esperando al usuario) no gira en el lugar
            objetivoLeft = constrain(base - correccion, 0.0f, RPM_MAX);   // invertir signos si dobla a direccion opuesta
            objetivoRight = constrain(base + correccion, 0.0f, RPM_MAX);
            motivo = (base > 0.0f) ? "MOVIENDO"
                   : "QUIETO: velocidad base 0 (usuario lejos o a la distancia objetivo)";
        }
        velLeft = comandoRueda(pidLeft, objetivoLeft, rpmFiltL, dt);
        velRight = comandoRueda(pidRight, objetivoRight, rpmFiltR, dt);
    }

    // para avanzar, ambos valores son positivos
    ST.motor(1, velLeft);
    ST.motor(2, velRight);

    // valores para el log
    logBase = base; logObjL = objetivoLeft; logObjR = objetivoRight;
    logRpmL = rpmFiltL; logRpmR = rpmFiltR; logVelL = velLeft; logVelR = velRight;
}

// LOGS: publica un texto en /arduino/estado con el motivo y los valores del control.
// Los decimales se mandan x100 como enteros (ej: lat=30 -> 0.30), porque printf
// con %f no siempre funciona en el Arduino.
void publicarEstado(unsigned long ahora) {
    unsigned long periodo = (prueba != SIN_PRUEBA) ? PERIODO_LOG_PRUEBA_MS : PERIODO_LOG_MS;
    if (estadoConexion != CONECTADO || ahora - ultimoLog < periodo) return;
    ultimoLog = ahora;

    long hacePid = ordenRecibida ? (long)(ahora - ultimaOrden) : -1;
    long haceCara = (ultimaCara > 0) ? (long)(ahora - ultimaCara) : -1;
    bool detenido = motoresDetenidos;

    int n = snprintf(buffer_estado, sizeof(buffer_estado),
        "%s | t=%lu pid_data hace %ldms lat=%d ang=%d cara=%dpx (%dcm) | usuario=%d (hace %ldms) | "
        "base=%d objL=%d objR=%d rpmL=%d rpmR=%d cmdL=%d cmdR=%d",
        motivo, ahora, hacePid,
        (int)(errorLateral * 100), (int)(errorAngular * 100), distancia, (int)(distUsuarioM * 100),
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
    ST.setTimeout(0);    // para que si no llegan comandos nuevos dentro de 1 segundos se detienen los motores (settimeout(0) para desactivar)
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
        prueba = SIN_PRUEBA;
        detenerMotores(ahora);
        return;
    }

    if (prueba != SIN_PRUEBA) { // Prueba de calibracion: ignora pista y usuario, termina sola
        if (ahora - inicioPrueba < DURACION_PRUEBA_MS) {
            controlar(ahora);
            return;
        }
        prueba = SIN_PRUEBA;
        publicarTexto("AJUSTE: prueba terminada");
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