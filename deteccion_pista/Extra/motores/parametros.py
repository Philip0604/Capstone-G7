# CONFIGURACIÓN DE LA MISIÓN
FPS = 5                 # Cantidad de FPS que procesará la cámara
SEQ_STEP_SIZE = 5.0     # [cm] Distancia a recorrer antes de cada foto
SEQ_MAX_LIMIT = 15      # [cm] Distancia total #TODO: CAMBIAR A DISTANCIA TOTAL PARA DEMO
AUTO_MODE = False       # TODO: CAMBIAR A MODO AUTÓNOMO. Solo al ya tener todo calibrado pq ya no se podría detener por código
AUTO_MODE_DELAY = 2000  # [ms] Tiempo que espera luego de encenderse para comenzar a moverse

# CONFIGURACION
# Pines de direccion y PWM: adaptar al driver utilizado
IN1_LEFT = 2
IN2_LEFT = 3
PWM_LEFT = 4

IN1_RIGHT = 5
IN2_RIGHT = 6
PWM_RIGHT = 7

# Pines de los encoders: adaptar al hardware
ENC_LEFT = 8
ENC_RIGHT = 9

# Velocidad objetivo en pulsos por segundo
VELOCIDAD_OBJETIVO = 100.0

# Periodo de muestreo del PID
DT = 0.05  # 50 ms

# Ganancias PID iniciales (requieren ajuste experimental)
KP = 1.50 
KI = 0.005
KD = 0.8

# PWM MicroPython: 0 a 65535
PWM_MAX = 65535
PWM_MIN = 0

# PARAMETROS MECANISMO CÁMARA


# PARAMETROS TRASLACIÓN (PID)
MAX_GLOBAL_VEL = 10.0 #TODO: SUBIR si vel máxima es muy lenta (no va a subir si no le da el torque => está limitado por MAX_CMD_DRIVER) no creo q haya q subirlo
MAX_ACCEL      = 5 #TODO: BAJAR si (des)acelera muy de golpe. SUBIR si no alcanza a llegar a una buena velocidad (dejarlo de último)    
CM_PER_PULSE   = 100.0 / 11960.0 

KP_POSITION = 0.8 #TODO: BAJAR si hay overshoot (o aumentar un poco PID_KP para que frene más rápido)     
MIN_VEL_REQ = 1.2 #TODO: BAJAR si al final vibra mucho. SUBIR si al final se queda corto
POSITION_DEADZONE = 0.2
MAX_CMD_DRIVER = 750 #TODO: SUBIR si al final corrige mal (los motores suenan pero no mueven el robot). Evitar subirlo (aumenta corriente)

# --- Ganancias ---
