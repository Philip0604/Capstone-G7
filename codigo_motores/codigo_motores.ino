// Código motores

#include <Arduino.h>
#include <math.h>

// PINES DE MOTORES cambiar por hardware
const int IN1_LEFT = 2, IN2_LEFT = 3, PWM_LEFT = 4;
const int IN1_RIGHT = 5, IN2_RIGHT = 6, PWM_RIGHT = 7;

// Pines de encoders CAMBIAR POR HARDWARE
const int ENC_LEFT = 8, ENC_RIGHT = 9;

// PARAMETROS DE CONTROL
const float RPM_BASE = 300.0;
const float RPM_MAX = 360.0;
const float RPM_CORRECCION_MAX = 120.0;

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

// ENCODERS
volatile long pulsosLeft = 0;
volatile long pulsosRight = 0;

void contarLeft() { pulsosLeft++; }
void contarRight() { pulsosRight++; }

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

PIDVelocidad pidLeft, pidRight;

// ESTADO
float errorLateral = 0;
float errorAngular = 0;
unsigned long ultimaOrden = 0;
unsigned long ultimoControl = 0;
long anteriorLeft = 0, anteriorRight = 0;

bool ordenRecibida = false;

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
    ultimoControl = millis();
}

// LOOP
void loop() {
    leerRaspberry();

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