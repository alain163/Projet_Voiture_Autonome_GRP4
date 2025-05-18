from vehicle import Driver
from controller import Lidar
import csv


driver = Driver()
basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

# Initialisation du LiDAR
lidar = Lidar("RpLidarA2")
lidar.enable(sensorTimeStep)
lidar.enablePointCloud()

# Initialisation des constantes
vitesse_max = 27.5  # km/h
angle_limite = 0.27  # rad
seuil_obstacle = 1.05  # m
distance_evitem = 0.25  # m
V_consigne = 3.0  # km/h
K = 0.5

# Variables de contrôle
direction = 0
vitesse = 0
auto_actif = True
obstacle_pres = False

# Indice LiDAR pour l'avant
distance_avant = 0  # 0° correspond à l'avant

# Ouvrir le fichier CSV (dans le dossier courant)
with open("donnees_robot_fusion.csv", mode="w", newline="", encoding="utf-8") as fichier_csv:
    writer = csv.writer(fichier_csv)
    writer.writerow(["Δx_g (m)", "Δx_d (m)", "Δx_avant (m)", "V_g (km/h)", "V_d (km/h)"])

    print("🚗 Mode autonome activé avec enregistrement CSV")

    # Boucle principale
    while driver.step() != -1:
        donnees_lidar = lidar.getRangeImage()

        if donnees_lidar is None or len(donnees_lidar) < 360:
            print("Erreur : Données LiDAR invalides")
            continue

        delta_x_g = donnees_lidar[90] if donnees_lidar[90] else float("inf")
        delta_x_d = donnees_lidar[270] if donnees_lidar[270] else float("inf")
        delta_x_avant = donnees_lidar[distance_avant] if donnees_lidar[distance_avant] else float("inf")

        print(f"[LIDAR] Avant: {delta_x_avant:.2f} m | Gauche: {delta_x_g:.2f} m | Droite: {delta_x_d:.2f} m")

        # Détection obstacle
        obstacle_pres = any(
            donnees_lidar[i] < seuil_obstacle or donnees_lidar[359 - i] < seuil_obstacle for i in range(20)
        )

        # Fonctions pour analyser l'espace
        def gauche_dispo():
            espace_g = sum(max(0, donnees_lidar[i] - donnees_lidar[359 - i]) for i in range(90))
            espace_d = sum(max(0, donnees_lidar[359 - i] - donnees_lidar[i]) for i in range(90))
            return espace_d >= espace_g

        def droite_dispo():
            espace_g = sum(max(0, donnees_lidar[i] - donnees_lidar[359 - i]) for i in range(90))
            espace_d = sum(max(0, donnees_lidar[359 - i] - donnees_lidar[i]) for i in range(90))
            return espace_d < espace_g

        if auto_actif:
            if not obstacle_pres:
                direction = (donnees_lidar[50] - donnees_lidar[310]) * 2
                direction = max(-angle_limite, min(direction, angle_limite))
                vitesse = max(0.6, 1.9 + abs(direction) * 3.5)
            else:
                if gauche_dispo():
                    vitesse = 1.5
                    direction = -0.28
                    for i in range(15, 160):
                        if donnees_lidar[i] < distance_evitem:
                            direction = -0.30
                elif droite_dispo():
                    vitesse = 1.2
                    direction = 0.28
                    for i in range(190, 355):
                        if donnees_lidar[i] < distance_evitem:
                            direction = 0.30
        else:
            vitesse = 0
            direction = 0

        # Calcul des vitesses de roue gauche/droite
        V_g = V_consigne - K * (delta_x_d - delta_x_g)
        V_d = V_consigne + K * (delta_x_g - delta_x_d)

        # Écrire dans le CSV
        writer.writerow([delta_x_g, delta_x_d, delta_x_avant, V_g, V_d])

        # Limites physiques
        vitesse = max(min(vitesse, vitesse_max), -vitesse_max)
        direction = max(min(direction, angle_limite), -angle_limite)

        driver.setCruisingSpeed(vitesse)
        driver.setSteeringAngle(direction)