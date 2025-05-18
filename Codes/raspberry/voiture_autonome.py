from rplidar import RPLidar
import numpy as np
import time
from rpi_hardware_pwm import HardwarePWM

direction = -1  
angle_pwm_min = 6.6
angle_pwm_max = 8.9
angle_pwm_centre = 7.75
angle_degre_max = 18  

pwm_dir = HardwarePWM(pwm_channel=1, hz=50)
pwm_dir.start(angle_pwm_centre)

def set_direction_degre(angle_degre):
    angle_pwm = angle_pwm_centre + direction * (angle_pwm_max - angle_pwm_min) * angle_degre / (2 * angle_degre_max)
    angle_pwm = min(max(angle_pwm, angle_pwm_min), angle_pwm_max)
    pwm_dir.change_duty_cycle(angle_pwm)

direction_prop = 1
pwm_stop_prop = 7.7
point_mort_prop = 0.5
delta_pwm_max_prop = 1.0

vitesse_max_m_s_hard = 8
vitesse_max_m_s_soft = 2

pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
pwm_prop.start(pwm_stop_prop)

def set_vitesse_m_s(vitesse_m_s):
    if vitesse_m_s > vitesse_max_m_s_soft:
        vitesse_m_s = vitesse_max_m_s_soft
    elif vitesse_m_s < -vitesse_max_m_s_hard:
        vitesse_m_s = -vitesse_max_m_s_hard

    if vitesse_m_s == 0:
        pwm_prop.change_duty_cycle(pwm_stop_prop)
    elif vitesse_m_s > 0:
        vitesse = vitesse_m_s * (delta_pwm_max_prop) / vitesse_max_m_s_hard
        pwm_prop.change_duty_cycle(pwm_stop_prop + direction_prop * (point_mort_prop + vitesse))
    elif vitesse_m_s < 0:
        vitesse = vitesse_m_s * (delta_pwm_max_prop) / vitesse_max_m_s_hard
        pwm_prop.change_duty_cycle(pwm_stop_prop - direction_prop * (point_mort_prop - vitesse))


PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(PORT_NAME)


seuil_obstacle = 1000  
distance_avant = 180   
V_consigne = 1.5      
K = 0.002            
K_angle = 0.1         


try:
    print("Mode automatique activé. Le robot suit la piste.")
    for scan in lidar.iter_scans(max_buf_meas=500):

        distances = [0] * 360
        for (_, angle, distance) in scan:
            angle = int(angle)
            if 0 <= angle < 360:
                distances[angle] = distance

        delta_x_g = distances[90]
        delta_x_d = distances[270]
        delta_x_avant = distances[distance_avant]

        print(f"delatgx : {delta_x_g} mm | delatdx : {delta_x_d} mm | delatavantx : {delta_x_avant} mm")

        if delta_x_avant < seuil_obstacle:
            angle_corr = K_angle * (delta_x_g - delta_x_d)
        else:
            angle_corr = K_angle * (distances[60] - distances[300])

        angle_degre = max(-18, min(18, angle_corr))
        set_direction_degre(angle_degre)

        v_g = V_consigne - K * (delta_x_d - delta_x_g)
        v_d = V_consigne + K * (delta_x_g - delta_x_d)
        vitesse_moyenne = (v_g + v_d) / 2

        set_vitesse_m_s(vitesse_moyenne)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Arret manuel")

finally:
    print("Arret du LiDAR et moteurs")
    set_vitesse_m_s(0)
    set_direction_degre(0)
    lidar.stop()
    lidar.disconnect()
