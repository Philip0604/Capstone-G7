#include <Sabertooth.h>

const int PIN_ENC_L_A = 2;   // ENC_L_A -> GIGA D2   encoder A de motor izquierdo
const int PIN_ENC_L_B = 3;   // ENC_L_B -> GIGA D3   encoder B de motor izquierdo
const int PIN_ENC_R_A = 4;   // ENC_R_A -> GIGA D4   encoder A de motor derecho
const int PIN_ENC_R_B = 5;   // ENC_R_B -> GIGA D5   encoder B de motor derecho

//const int PIN_EN_GIGA = 7;  // enable de giga para el and

const int VEL_STOP   = 0;   // -127 max reversa, 0 stop, 127 max avance
const int VEL_RAPIDO = 100;   
const int VEL_MEDIO  = 75;
const int VEL_LENTO  = 50;

int velActualL = VEL_STOP;
int velObjetivoL = VEL_STOP;
int velActualR = VEL_STOP;
int velObjetivoR = VEL_STOP;
const int VEL_RAMPA = 1;   // aumenta 1 cada MS_RAMPA milisegundos
const unsigned long MS_RAMPA = 50;

volatile long cuentaL = 0;
volatile long cuentaR = 0;

long anteriorL = 0;    // cuenta pasada de los encoders R y L
long anteriorR = 0;

const float PULSOS_VUELTA = 480.0f;

Sabertooth ST(128, Serial2);   // se crea el objeto ST que representa el sabertooth con el address 128
                               // serial1 para ocupar el pin d18 tx1 conectado a s1
// DIP sw: 1-OFF 2-OFF 3-OFF 4-ON 5-ON 6-OFF
// 1,2 off: libreria sabertooth, 3 off: proteccion bateria, 4 on: escucha packet serial,
// 5 on: address a 128, 6 off: emergency stop on -> (S2) detenerse si un enable se desactiva (apagar emergency stop hace que no importe el valor en S2)

void encL() { cuentaL += digitalRead(PIN_ENC_L_B) ? 1 : -1; }  // a cuentaL se le suma el resultado del: if el pin esta high, entonces es 1 y si no entonces es -1
void encR() { cuentaR += digitalRead(PIN_ENC_R_B) ? -1 : 1; }

void ponerLed(bool r, bool g, bool b) {
  digitalWrite(LEDR, r ? LOW : HIGH);
  digitalWrite(LEDG, g ? LOW : HIGH);
  digitalWrite(LEDB, b ? LOW : HIGH);
}

void actualizarLed() {
  if (velActualR <= VEL_STOP)       ponerLed(true, false, false);
  else if (velActualR <= VEL_MEDIO) ponerLed(true, true, false);
  else                             ponerLed(false, true, false);
}


void setup() {
  Serial.begin(115200); // 9600?  serial para consola (printf)
  Serial2.begin(9600); // serial2 para motores. 9600 es el rate default (se cambia en DEScribe)
  Serial3.begin(115200); // 9600?  serial3 para esp
  //ST.setTimeout(1000); // para que si no llegan comandos nuevos dentro de 1 segundo se detienen los motores (settimeout(0) para desactivar)
                       // un delay en el codigo puede activar el timeout!!
  
  //pinMode(PIN_EN_GIGA, OUTPUT);
  //digitalWrite(PIN_EN_GIGA, HIGH); // enable de giga en 1 (para que funcione "en" de rpi y esp deben ser high)

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  actualizarLed();

  pinMode(PIN_ENC_L_A, INPUT);
  pinMode(PIN_ENC_L_B, INPUT);
  pinMode(PIN_ENC_R_A, INPUT);
  pinMode(PIN_ENC_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), encL, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), encR, RISING);

  Serial.println("GIGA listo. Esperando A/B/C/D por Serial3.");
  
}

void loop() {
  if (Serial3.available()) {
    char c = Serial3.read();
    int velL = VEL_STOP;
    int velR = VEL_STOP;
    switch (c) {
      // Test girar:
      // case 'A': velL = VEL_RAPIDO, velR = VEL_RAPIDO; break;
      // case 'B': velL = VEL_RAPIDO, velR = VEL_MEDIO;  break;
      // case 'C': velL = VEL_MEDIO, velR = VEL_RAPIDO;  break;

      // Test velocidades
      case 'A': velL = VEL_RAPIDO, velR = VEL_RAPIDO; break;
      case 'B': velL = VEL_MEDIO, velR = VEL_MEDIO;  break;
      case 'C': velL = VEL_LENTO, velR = VEL_LENTO;  break;
      case 'D': velL, velR = VEL_STOP;   break;
      default:  break;
    }
    if (c == 'A' || c == 'B' || c == 'C' || c == 'D') {
      velObjetivoL = velL;
      velObjetivoR = velR;
      if (c == 'D') {
        velActualL = VEL_STOP;
        velActualR = VEL_STOP;
        ST.motor(1, VEL_STOP);   // paran ambos motores (motor 1 y 2)
        ST.motor(2, VEL_STOP);
        actualizarLed();
      }
      Serial.print("Comando ");
      Serial.print(c);
      Serial.print(" -> velocidad ");
      Serial.println(velR);
      Serial.println(velL);
    }
  }

  static uint32_t ultimaRampaL = 0;
  if (millis() - ultimaRampaL >= MS_RAMPA && velActualL != velObjetivoL) {
    ultimaRampaL = millis();
    if (velActualL < velObjetivoL)   velActualL = min(velActualL + VEL_RAMPA, velObjetivoL);
    else                           velActualL = max(velActualL - VEL_RAMPA, velObjetivoL);
    ST.motor(1, velActualL);
    actualizarLed();
  }
  static uint32_t ultimaRampaR = 0;
  if (millis() - ultimaRampaR >= MS_RAMPA && velActualR != velObjetivoR) {
    ultimaRampaR = millis();
    if (velActualR < velObjetivoR)   velActualR = min(velActualR + VEL_RAMPA, velObjetivoR);
    else                           velActualR = max(velActualR - VEL_RAMPA, velObjetivoR);
    ST.motor(2, velActualR);
    actualizarLed();
  }

  static uint32_t ultimo = 0;
  if (millis() - ultimo >= 500) {
    ultimo = millis();
    noInterrupts();
    long l = cuentaL, r = cuentaR;
    interrupts();

    long deltaL = l - anteriorL;  // se calcula que tanto aumento el valor de los encoders
    long deltaR = r - anteriorR;
    anteriorL = l;
    anteriorR = r;

    // revoluciones por minuto de cada rueda medidas con valores de encoders
    float rpmLeft = deltaL * 60.0f / (PULSOS_VUELTA * 0.5);
    float rpmRight = deltaR * 60.0f / (PULSOS_VUELTA * 0.5);

    Serial.print("Encoders  L=");
    Serial.print(l);
    Serial.print("  R=");
    Serial.print(r);
    // Serial.print("  Diferencia=");
    // Serial.print(deltaR - deltaL);
    Serial.print("  RPML=");
    Serial.print(rpmLeft);
    Serial.print("  RPMR=");
    Serial.println(rpmRight);
  }

}
