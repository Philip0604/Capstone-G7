from machine import Pin, PWM
from time import ticks_ms, ticks_diff, sleep_ms
from machine import Pin, UART, PWM
import time
import sys
import select
from encoders import Encoder
import parametros as p

# CLASES
class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.error_anterior = 0.0

    def calcular(self, referencia, medida):
        error = referencia - medida
        self.integral += error * self.dt

        # if self.integral > self.max_int: 
        #     self.integral = self.max_int
        # elif self.integral < -self.max_int: 
        #     self.integral = -self.max_int

        derivada = (error - self.error_anterior) / self.dt
        salida = (self.kp * error + self.ki * self.integral + self.kd * derivada)

        self.error_anterior = error

        # if output > self.max_out: 
        #     output = self.max_out
        # elif output < -self.max_out: 
        #     output = -self.max_out
        # return output
        return max(p.PWM_MIN, min(p.PWM_MAX, salida))

    def reset(self):
        self.integral = 0.0
        self.error_anterior = 0.0

# INICIALIZACIÓN HARDWARE
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
enc_1 = Encoder(0, pin_a=3, pin_b=2) 
enc_2 = Encoder(1, pin_a=5, pin_b=4)

pid_vel_m1 = PID(p.K_P, p.K_I, p.K_D, p.D_T)
pid_vel_m2 = PID(p.K_P, p.K_I, p.K_D, p.D_T)

poller = select.poll()
poller.register(sys.stdin, select.POLLIN)

# CONFIGURACION DE PINES
in1_left = Pin(p.IN1_LEFT, Pin.OUT)
in2_left = Pin(p.IN2_LEFT, Pin.OUT)
pwm_left = PWM(Pin(p.PWM_LEFT))
pwm_left.freq(1000)

in1_right = Pin(p.IN1_RIGHT, Pin.OUT)
in2_right = Pin(p.IN2_RIGHT, Pin.OUT)
pwm_right = PWM(Pin(p.PWM_RIGHT))
pwm_right.freq(1000)

enc_left = Pin(p.ENC_LEFT, Pin.IN, Pin.PULL_UP)
enc_right = Pin(p.ENC_RIGHT, Pin.IN, Pin.PULL_UP)

# ENCODERS
pulsos_left = 0
pulsos_right = 0

# FUNCIONES AUXILIARES
def contar_left(pin):
    global pulsos_left
    pulsos_left += 1

def contar_right(pin):
    global pulsos_right
    pulsos_right += 1

enc_left.irq(trigger=Pin.IRQ_RISING, handler=contar_left)
enc_right.irq(trigger=Pin.IRQ_RISING, handler=contar_right)

# CONTROL DE MOTORES
def mover_motor(in1, in2, pwm, sentido, velocidad):
    velocidad = max(p.PWM_MIN, min(p.PWM_MAX, int(velocidad)))

    if sentido > 0:
        in1.value(1)
        in2.value(0)
    elif sentido < 0:
        in1.value(0)
        in2.value(1)
    else:
        in1.value(0)
        in2.value(0)
        velocidad = 0

    pwm.duty_u16(velocidad)

def detener_motores():
    pwm_left.duty_u16(0)
    pwm_right.duty_u16(0)

    in1_left.value(0)
    in2_left.value(0)
    in1_right.value(0)
    in2_right.value(0)
        
# VARIABLES DE ESTADO
# --- Física Traslación ---
pos_m1_cm = 0.0; pos_m2_cm = 0.0
avg_pos = 0.0; target_position = 0.0     
current_vel_setpoint = 0.0 

# --- Estados del Sistema General ---
SYS_IDLE = 0
SYS_MOVING = 1      
SYS_CAMERA = 2      
SYS_FINISHED = 3    
SYS_RETURNING_HOME = 4 

current_sys_state = SYS_IDLE

# --- Estados Internos Cámara ---
CAM_INIT_CHECK = 0  
CAM_EXITING = 1     
CAM_SEARCHING = 2   
CAM_CAPTURING = 3   
CAM_DONE = 4        

cam_state = CAM_INIT_CHECK
cam_timer = 0

# Variables lógicas cámara
cam_direction = 1       # 1: CCW, -1: CW
first_sweep_ever = True 
current_seq_list = []   
current_seq_idx = 0     

# Variables para retorno
return_list = []
return_idx = 0

# --- Timers Loop ---
last_count_1 = enc_1.get_count()
last_count_2 = enc_2.get_count()
last_time_pid = time.ticks_ms()
print_timer = time.ticks_ms()

try:
    while True:
        current_time = time.ticks_ms()

        # 1. COMANDOS    
        if poller.poll(0):
            line = sys.stdin.readline().strip()
            
            if (line == "s") or (line == "stop"):
                current_sys_state = SYS_IDLE; stop_servo()
                print_pc("\nSTOP MANUAL")

            elif line == "reset":
                current_sys_state = SYS_IDLE; 
                stop_servo()

                pos_m1_cm = 0
                pos_m2_cm = 0
                avg_pos = 0
                target_position = 0

                pid_vel_m1.reset()
                pid_vel_m2.reset()

                cam_direction = 1
                first_sweep_ever = True

                print_pc("\nRESET COMPLETO")

            elif line == "start":
                if target_position >= SEQ_MAX_LIMIT:
                    print_pc("\nLIMITE ALCANZADO.")
                else:
                    next_pos = target_position + SEQ_STEP_SIZE
                    if next_pos > SEQ_MAX_LIMIT: 
                        next_pos = SEQ_MAX_LIMIT
                    target_position = next_pos
                    current_sys_state = SYS_MOVING
                    print_pc(f"\nINICIO: Moviendo a {target_position:.1f} cm")

        # -------------------------------
        # 2. LÓGICA CÁMARA (BARRIDO FOTOS)
        # -------------------------------
        if current_sys_state == SYS_CAMERA:
            
            if cam_state == CAM_INIT_CHECK:
                # Elegir lista según dirección
                if cam_direction == 1:
                    current_seq_list = MAGNET_SEQ_CCW
                    print_pc(">> CAM: Secuencia ANTIHORARIA (CCW)")
                else:
                    current_seq_list = MAGNET_SEQ_CW
                    print_pc(">> CAM: Secuencia HORARIA (CW)")
                
                current_seq_idx = 0
                
                # --- Lógica Primera Vez (Imán 1 o 2) ---
                if first_sweep_ever and cam_direction == 1:

                    if is_magnet_detected(): # Parte justo donde está el primer imán
                        mag_id = current_seq_list[0] # Imán 1
                        print_pc(f">> CAM: Inicio en Imán {mag_id}")
                        trigger_photo_rpi4(avg_pos, mag_id)
                        cam_timer = current_time
                        cam_state = CAM_CAPTURING
                        current_seq_idx = 1
                        first_sweep_ever = False
                    else:
                        print_pc(f">> CAM: Buscando Imán 2...")
                        current_seq_idx = 1 # Ir directo al 2do imán
                        cam_state = CAM_SEARCHING
                        set_servo_move(cam_direction)
                        first_sweep_ever = False
                else:
                    # Lógica Normal: Se asume que es el 1er imán
                    mag_id = current_seq_list[0]
                    print_pc(f">> CAM: Inicio en Imán {mag_id}")
                    trigger_photo_rpi4(avg_pos, mag_id)
                    cam_timer = current_time
                    cam_state = CAM_CAPTURING
                    current_seq_idx = 1 
            
            elif cam_state == CAM_CAPTURING:
                if time.ticks_diff(current_time, cam_timer) > CAM_TIME_CAPTURE:
                    if current_seq_idx >= len(current_seq_list):
                        print_pc(">> CAM: Fin de secuencia.")
                        cam_state = CAM_DONE
                    else:
                        set_servo_move(cam_direction)
                        cam_state = CAM_EXITING

            elif cam_state == CAM_EXITING:
                if not is_magnet_detected():
                    cam_state = CAM_SEARCHING

            elif cam_state == CAM_SEARCHING:
                if is_magnet_detected():
                    stop_servo()
                    mag_id = current_seq_list[current_seq_idx]
                    print_pc(f">> CAM: Foto Imán {mag_id}")
                    trigger_photo_rpi4(avg_pos, mag_id)
                    current_seq_idx += 1
                    cam_timer = current_time
                    cam_state = CAM_CAPTURING

            elif cam_state == CAM_DONE:
                stop_servo()
                # Invertir dirección para la siguiente
                cam_direction *= -1 
                cam_state = CAM_INIT_CHECK 
                
                if target_position >= SEQ_MAX_LIMIT - 0.5:
                    print_pc(">> MISION COMPLETADA. Verificando retorno...")
                    current_sys_state = SYS_FINISHED
                else:
                    next_pos = target_position + SEQ_STEP_SIZE
                    if next_pos > SEQ_MAX_LIMIT: next_pos = SEQ_MAX_LIMIT
                    target_position = next_pos
                    current_sys_state = SYS_MOVING
                    print_pc(f">> SIGUIENTE: {target_position:.1f} cm")

        # -------------------------------
        # 3. LÓGICA RETORNO A IMÁN 1
        # -------------------------------
        elif current_sys_state == SYS_FINISHED:
            # Mensaje a la raspi de que terminó el avance y no habrá más fotos
            print(f"-1,{SEQ_MAX_LIMIT}")

            # "cam_direction" ya fue invertida al final de CAM_DONE.
            # Si ahora es 1 (CCW), => la última fue -1 (CW, terminó en Imán 1)
            # Si ahora es -1 (CW), => la última fue 1 (CCW, terminó en Imán 4)

            if cam_direction == 1: 
                # CASO A: Terminó en Imán 1
                print_pc(">> FIN: Ya estamos en el Imán 1. Sistema detenido.")
                stop_servo()
                current_sys_state = SYS_IDLE
            else:
                # CASO B: Terminó Imán 4 (o último de la lista CCW)
                #Hay q volver por todos hasta llegar al imán 1
                print_pc(">> FIN: Estamos en el final. Retornando a Imán 1...")
                
                # Crear lista de retorno excluyendo el actual (el 4)
                # MAGNET_SEQ_CW es [4, 3, 2, 1]. Hay que ir a 3, luego 2, luego 1
                return_list = MAGNET_SEQ_CW[1:] # Slice: toma desde el índice 1 hasta el final
                return_idx = 0
                
                set_servo_move(-1) # Forzar Horario (CW)
                cam_state = CAM_EXITING # Primero salir del imán actual (4)
                current_sys_state = SYS_RETURNING_HOME

        elif current_sys_state == SYS_RETURNING_HOME:
            # Ciclo rápido para volver sin tomar fotos
            
            # 1. Salir del imán (sea el 4, 3 o 2)
            if cam_state == CAM_EXITING:
                if not is_magnet_detected():
                    cam_state = CAM_SEARCHING
            
            # 2. Buscar el siguiente
            elif cam_state == CAM_SEARCHING:
                if is_magnet_detected():
                    found_id = return_list[return_idx]
                    print_pc(f">> RETORNO: Detectado Imán {found_id}")
                    
                    if found_id == 1: # Es el imán 1
                        stop_servo()
                        print_pc(">> RETORNO COMPLETO: Home (Imán 1).")
                        current_sys_state = SYS_IDLE

                    else: # Es un imán intermedio (3 o 2), seguir de largo
                        return_idx += 1
                        cam_state = CAM_EXITING

        # -------------------------------
        # 4. LÓGICA TRASLACIÓN (PID)
        # -------------------------------
        dt_ms = time.ticks_diff(current_time, last_time_pid)
        if dt_ms >= 50:
            dt_sec = dt_ms / 1000.0
            last_time_pid = current_time
            
            c1 = enc_1.get_count()
            c2 = enc_2.get_count()

            d1 = c1 - last_count_1
            d2 = c2 - last_count_2
            
            last_count_1 = c1
            last_count_2 = c2
            
            dist_1 = d1 * CM_PER_PULSE
            dist_2 = d2 * CM_PER_PULSE

            pos_m1_cm += dist_1
            pos_m2_cm += dist_2

            avg_pos = (pos_m1_cm + pos_m2_cm) / 2.0
            vel_real_1 = dist_1 / dt_sec; vel_real_2 = dist_2 / dt_sec

            target_vel_req = 0.0
            
            if current_sys_state == SYS_MOVING:
                pos_error = target_position - avg_pos

                if abs(pos_error) < POSITION_DEADZONE:
                    target_vel_req = 0.0

                    if abs(vel_real_1) < 0.5 and abs(vel_real_2) < 0.5:
                        print_pc(f">> LLEGADA: {avg_pos:.1f} cm")
                        current_sys_state = SYS_CAMERA
                        cam_state = CAM_INIT_CHECK 
                else:
                    calculated_vel = pos_error * KP_POSITION

                    if calculated_vel > 0:
                        target_vel_req = max(calculated_vel, MIN_VEL_REQ)
                    else:
                        target_vel_req = min(calculated_vel, -MIN_VEL_REQ)

                    if target_vel_req > MAX_GLOBAL_VEL: 
                        target_vel_req = MAX_GLOBAL_VEL
                    if target_vel_req < -MAX_GLOBAL_VEL: 
                        target_vel_req = -MAX_GLOBAL_VEL

            max_step = MAX_ACCEL * dt_sec
            if current_vel_setpoint < target_vel_req:
                current_vel_setpoint += max_step
                if current_vel_setpoint > target_vel_req: 
                    current_vel_setpoint = target_vel_req
            elif current_vel_setpoint > target_vel_req:
                current_vel_setpoint -= max_step
                if current_vel_setpoint < target_vel_req: 
                    current_vel_setpoint = target_vel_req
            
            if current_sys_state != SYS_MOVING and abs(current_vel_setpoint) < 0.1 and abs(vel_real_1) < 0.5:
                out1 = 0; out2 = 0
                pid_vel_m1.reset(); pid_vel_m2.reset()
            else:
                out1 = pid_vel_m1.compute(current_vel_setpoint, vel_real_1, dt_sec)
                out2 = pid_vel_m2.compute(current_vel_setpoint, vel_real_2, dt_sec)

            uart.write(f"M1:{int(out1)}\r\n".encode())
            time.sleep_us(200)
            uart.write(f"M2:{int(out2)}\r\n".encode())

            if time.ticks_diff(current_time, print_timer) > 1000:
                if current_sys_state == SYS_MOVING:
                    print_pc(f">> MOV: {avg_pos:5.1f}/{target_position:.1f} cm")
                print_timer = current_time
        
        time.sleep_ms(1)

except KeyboardInterrupt:
    print_pc("\nFin.")
finally:
    uart.write(f"M1:0\r\n".encode())
    uart.write(f"M2:0\r\n".encode())
    stop_servo()



##############################################
##############################################
##############################################
##############################################










pid_left = PID(p.KP, p.KI, p.KD, p.DT)
pid_right = PID(p.KP, p.KI, p.KD, p.DT)

# ============================================================
# BUCLE PRINCIPAL
# ============================================================

try:
    detener_motores()
    sleep_ms(1000)

    pulsos_prev_left = 0
    pulsos_prev_right = 0
    tiempo_anterior = ticks_ms()

    while True:
        sleep_ms(int(DT * 1000))

        tiempo_actual = ticks_ms()
        dt_real = ticks_diff(tiempo_actual, tiempo_anterior) / 1000.0

        if dt_real <= 0:
            continue

        tiempo_anterior = tiempo_actual

        # Obtener pulsos acumulados
        actual_left = pulsos_left
        actual_right = pulsos_right

        # Pulsos registrados durante el intervalo
        delta_left = actual_left - pulsos_prev_left
        delta_right = actual_right - pulsos_prev_right

        pulsos_prev_left = actual_left
        pulsos_prev_right = actual_right

        # Velocidad medida en pulsos por segundo
        velocidad_left = delta_left / dt_real
        velocidad_right = delta_right / dt_real

        # Calcular PWM mediante los PID
        control_left = pid_left.calcular(
            VELOCIDAD_OBJETIVO, velocidad_left
        )

        control_right = pid_right.calcular(
            VELOCIDAD_OBJETIVO, velocidad_right
        )

        # Sentidos opuestos para avanzar
        mover_motor(
            in1_left, in2_left, pwm_left, 1, control_left
        )

        mover_motor(
            in1_right, in2_right, pwm_right, -1, control_right
        )

        print(
            "Izq: ref={:.1f}, medida={:.1f}, PWM={:.0f} | "
            "Der: ref={:.1f}, medida={:.1f}, PWM={:.0f}".format(
                VELOCIDAD_OBJETIVO, velocidad_left, control_left,
                VELOCIDAD_OBJETIVO, velocidad_right, control_right
            )
        )

except KeyboardInterrupt:
    print("Programa detenido.")

finally:
    detener_motores()