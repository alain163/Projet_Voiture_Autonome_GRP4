from rplidar import RPLidar
import numpy as np
import time
import matplotlib.pyplot as plt

# Connexion et démarrage du LIDAR
PORT = "/dev/ttyUSB0"
BAUDRATE = 256000

lidar = RPLidar(PORT, baudrate=BAUDRATE)

try:
    print(lidar.get_info())
    lidar.start_motor()
    time.sleep(1)

    tableau_lidar_mm = np.zeros(360)  # Tableau des distances
    teta = np.radians(np.arange(360))  # Tableau des angles en radians

    scan_count = 0  # Compteur de scans
    max_scans = 20  # Nombre de scans à réaliser

    # Acquisition des données
    for scan in lidar.iter_scans():
        print(f"Scan {scan_count + 1}/{max_scans} - nb pts : {len(scan)}")

        for (_, angle, distance) in scan:
            index = int(round(angle)) % 360  # Angle entre 0 et 359
            tableau_lidar_mm[index] = distance  # Mise à jour du tableau

        scan_count += 1
        if scan_count >= max_scans:
            break  # Arrêt après 20 scans

except KeyboardInterrupt:
    print("Fin des acquisitions")

finally:
    # Arrêt et déconnexion du LIDAR
    lidar.stop_motor()
    lidar.stop()

# Affichage en mode polar
fig = plt.figure()
ax = plt.subplot(111, projection='polar')
ax.scatter(teta, tableau_lidar_mm, s=5)
ax.set_rmax(8000)
ax.grid(True)
plt.show()
