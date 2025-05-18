from rplidar import RPLidar
from rpi_hardware_pwm import HardwarePWM
import time


pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
pwm_stop_prop = 8.17
point_mort_prop = 0.13
delta_pwm_max_prop = 1.5
direction_prop = -1
vitesse_max_m_s_hard = 8
vitesse_max_m_s_soft = 2
pwm_prop.start(pwm_stop_prop)

def set_vitesse_m_s(vitesse_m_s):
    vitesse_m_s = max(-vitesse_max_m_s_hard, min(vitesse_m_s, vitesse_max_m_s_soft))
    if vitesse_m_s == 0:
        pwm_prop.change_duty_cycle(pwm_stop_prop)
    elif vitesse_m_s > 0:
        pwm = pwm_stop_prop + direction_prop * (point_mort_prop + vitesse_m_s * delta_pwm_max_prop / vitesse_max_m_s_hard)
        pwm_prop.change_duty_cycle(pwm)
    else:
        pwm = pwm_stop_prop - direction_prop * (point_mort_prop - vitesse_m_s * delta_pwm_max_prop / vitesse_max_m_s_hard)
        pwm_prop.change_duty_cycle(pwm)


pwm_dir = HardwarePWM(pwm_channel=1, hz=50)
angle_pwm_min = 6
angle_pwm_max = 9
angle_pwm_centre = 7.5
angle_degre_max = 18
direction = 1
pwm_dir.start(angle_pwm_centre)

def set_direction_degre(angle_degre):
    angle_pwm = angle_pwm_centre + direction * (angle_pwm_max - angle_pwm_min) * angle_degre / (2 * angle_degre_max)
    angle_pwm = max(angle_pwm_min, min(angle_pwm_max, angle_pwm))
    pwm_dir.change_duty_cycle(angle_pwm)


lidar = RPLidar('/dev/ttyUSB0', baudrate=256000)
lidar.connect()
print('Info LIDAR:', lidar.get_info())
lidar.start_motor()
time.sleep(1)

tableau_lidar_mm = [0] * 360


try:
    print("Début de la conduite autonome")
    for scan in lidar.iter_scans(scan_type='express'):
        for (_, angle, distance) in scan:
            angle = int(angle) % 360
            tableau_lidar_mm[angle] = distance

        
        front = tableau_lidar_mm[0]
        gauche = tableau_lidar_mm[60]
        droite = tableau_lidar_mm[300]  

        
        angle_degre = 0.02 * (gauche - droite)
        set_direction_degre(angle_degre)

        
        if front < 400:
            set_vitesse_m_s(0)
        elif front < 800:
            set_vitesse_m_s(0.2)
        else:
            set_vitesse_m_s(0.5)

        time.sleep(0.1)

except KeyboardInterrupt:
    print(\"\\nArrêt par CTRL+C\")

finally:
    print(\"Arrêt des moteurs et du LIDAR\")
    set_vitesse_m_s(0)
    pwm_dir.change_duty_cycle(angle_pwm_centre)
    lidar.stop_motor()
    lidar.stop()
    time.sleep(1)
    lidar.disconnect()
    pwm_prop.stop()
    pwm_dir.stop()
