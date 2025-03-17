import matplotlib.pyplot as plt
import numpy as np

# timeHistory: Array de tiempos (eje X)
# netForceHistory: Magnitud de la fuerza neta en cada instante
# netTorqueHistory: Magnitud del torque neto en cada instante
# fingerAngles1History: Ángulos [ang1, ang2, ang3] para el dedo 1
# fingerForceSumHistory: Fuerzas resultantes [Fx, Fy, Fz] de los dedos
timeHistory = np.linspace(0, 10, 101)
netForceHistory = 10 + 2*np.sin(timeHistory)
netTorqueHistory = 0.5 + 0.2*np.cos(timeHistory)
fingerAngles1History = [[0.1*t, 0.05*t, 0.02*t] for t in timeHistory]
fingerForceSumHistory = [[0.1*np.sin(t), 0.05*np.sin(2*t), 0.02*np.cos(3*t)] for t in timeHistory]

# Configuración de la figura y organización de subplots
plt.figure(figsize=(12, 8))

# Gráfica: Evolución de la fuerza neta (incluye peso)
plt.subplot(2, 2, 1)
plt.plot(timeHistory, netForceHistory, label='Fuerza Neta (incluye peso)', color='darkblue')
plt.xlabel('Tiempo (s)')
plt.ylabel('Magnitud de Fuerza')
plt.title('Evolución de la Fuerza Neta')
plt.legend()
plt.grid(True)

# Gráfica: Evolución del torque neto
plt.subplot(2, 2, 2)
plt.plot(timeHistory, netTorqueHistory, label='Torque Neto', color='darkgreen')
plt.xlabel('Tiempo (s)')
plt.ylabel('Magnitud de Torque')
plt.title('Evolución del Torque Neto')
plt.legend()
plt.grid(True)

# Gráfica: Evolución de los ángulos del dedo 1
plt.subplot(2, 2, 3)
plt.plot(timeHistory, [a[0] for a in fingerAngles1History], label='Ángulo 1', color='magenta')
plt.plot(timeHistory, [a[1] for a in fingerAngles1History], label='Ángulo 2', color='orange')
plt.plot(timeHistory, [a[2] for a in fingerAngles1History], label='Ángulo 3', color='purple')
plt.xlabel('Tiempo (s)')
plt.ylabel('Ángulo (rad)')
plt.title('Evolución de los Ángulos del Dedo 1')
plt.legend()
plt.grid(True)

# Gráfica: Suma de fuerzas de los dedos (debe aproximarse a cero)
plt.subplot(2, 2, 4)
fingerForceSumHistory = np.array(fingerForceSumHistory)
plt.plot(timeHistory, fingerForceSumHistory[:, 0], label='Fuerza Fx', color='red')
plt.plot(timeHistory, fingerForceSumHistory[:, 1], label='Fuerza Fy', color='blue')
plt.plot(timeHistory, fingerForceSumHistory[:, 2], label='Fuerza Fz', color='green')
plt.xlabel('Tiempo (s)')
plt.ylabel('Fuerza (N)')
plt.title('Suma de Fuerzas de los Dedos')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
