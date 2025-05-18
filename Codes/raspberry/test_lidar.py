from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np

PORT_NAME = '/dev/ttyUSB0'

lidar = RPLidar(PORT_NAME)

plt.ion()
fig, ax = plt.subplots()
scan_plot, = ax.plot([], [], 'b.', markersize=2)
ax.set_xlim(-5000, 5000)
ax.set_ylim(-5000, 5000)
ax.set_title("Visualisation des points LiDAR")
ax.set_xlabel("X (mm)")
ax.set_ylabel("Y (mm)")

try:
    for scan in lidar.iter_scans(max_buf_meas=500):
        angles = []
        distances = []
        for (_, angle, distance) in scan:
            angles.append(np.radians(angle))
            distances.append(distance)

        xs = [d * np.cos(a) for d, a in zip(distances, angles)]
        ys = [d * np.sin(a) for d, a in zip(distances, angles)]

        scan_plot.set_xdata(xs)
        scan_plot.set_ydata(ys)
        fig.canvas.draw()
        fig.canvas.flush_events()
except KeyboardInterrupt:
    print("Arret par l'utilisateur")
finally:
    print("Fermeture du LiDAR...")
    lidar.stop()
    lidar.disconnect()