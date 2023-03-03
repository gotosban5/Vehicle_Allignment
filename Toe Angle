import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import matplotlib.pyplot as plt
import scipy as sp
import sympy as smp
from scipy.integrate import quad
from scipy.integrate import cumulative_trapezoid
from scipy.integrate import trapz
from scipy.integrate import cumtrapz

DatosGiroscopoX = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
DatosGiroscopoY = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
DatosGiroscopoZ = [120, 120, 120, 120, 120, 119, 120, 120, 120, 121, 120, 121, 120, 120, 120, 121, 120, 120, 120, 120]

import matplotlib.pyplot as plt
plt.close('all')
y1 = np.linspace(0,20,20)
X = np.array(DatosGiroscopoX)
Y = np.array(DatosGiroscopoY)
Z = np.array(DatosGiroscopoZ)
plt.figure(figsize=[15,4])
plt.subplot(131)
plt.title(label = "Instrumento rueda constante en eje Z")
plt.plot(y1, X, label = "Giroscopo en X")
plt.plot(y1, Y, label = "Giroscopo en Y")
plt.plot(y1, Z, label = "Giroscopo en Z")
plt.grid(True)
plt.legend()

#DatosAcelerometroX = [0.04, 0.02, 0.00, 0.00, 0.00, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01, -0.02, -0.02, -0.02, -0.02, -0.02, -0.03, -0.03, -0.03]
#DatosAcelerometroY = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,]
#DatosAcelerometroZ = [0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98,]

#y2 = np.linspace(0,20,20)
#X = np.array(DatosAcelerometroX)
#Y = np.array(DatosAcelerometroY)
#Z = np.array(DatosAcelerometroZ)
#plt.subplot(132)
#plt.plot(y2, X, label = "Acelerometro en X")
#plt.plot(y2, Y, label = "Acelerometro en Y")
#plt.plot(y2, Z, label = "Acelerometro en Z")
#plt.legend()

#Datos de giroscopio, eje Z a una dirección
#DatosGiroscopoX1 = [0.00, -0.10, -1.02, -2.61, -3.53, -5.27, -5.99, -6.52, -6.03, -5.33, -4.55, -3.84, -3.43, -2.99, -2.44, -2.06, -1.77, -1.20, -0.45, 0.00]
DatosGiroscopoX1 = [0.01, 0.00, 0.00, 0.01, -0.01, 0.00, 0.00, 0.00, 0.00, 0.02, 0.00, 0.01, 0.00, 0.00, -0.01, 0.00, 0.00, 0.00, -0.01, 0.00]
DatosGiroscopoZ1 = [0.00, 0.02, 0.00, 0.01, 0.00, 0.00, 0.00, -0.01, 0.00, 0.00, -0.01, 0.00, 0.00, 0.02, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00]
DatosGiroscopoY1 = [0.00, -0.10, -0.66, -1.25, -1.90, -2.34, -2.77, -3.21, -3.02, -2.68, -2.41, -2.01, -1.83, -1.39, -1.02, -0.81, -0.55, -0.33, -0.09, 0.00]
#DatosGiroscopoY1 = [0.00, 0.10, 1.02, 2.61, 3.53, 5.27, 5.99, 6.52, 6.03, 5.33, 4.55, 3.84, 3.43, 2.99, 2.44, 2.06, 1.77, 1.20, 0.45, 0.00]

y1 = np.linspace(0,20,20)
X = np.array(DatosGiroscopoX1)
Y = np.array(DatosGiroscopoY1)
Z = np.array(DatosGiroscopoZ1)
plt.subplot(132)
plt.title(label = "Instrumento dirección 1")
plt.plot(y1, X, label = "Giroscopo en X")
plt.plot(y1, Y, label = "Giroscopo en Y")
plt.plot(y1, Z, label = "Giroscopo en Z")
plt.grid(True)
plt.legend()

#Datos de giroscopio, eje Z a otra dirección
#DatosGiroscopoX2 = [0.00, 0.90, 1.82, 3.05, 3.54, 5.11, 5.49, 6.22, 5.46, 4.88, 4.49, 3.75, 3.30, 2.89, 2.22, 1.86, 1.35, 0.74, 0.13, 0.00]
DatosGiroscopoX2 = [0.00, 0.00, 0.01, -0.01, 0.00, -0.01, 0.00, 0.00, -0.01, 0.00, 0.00, 0.02, 0.00, 0.01, 0.00, -0.02, 0.00, -0.01, 0.00, 0.02]
DatosGiroscopoZ2 = [0.02, 0.00, 0.00, -0.01, 0.00, 0.00, 0.00, 0.02, 0.00, 0.02, 0.00, -0.01, 0.00, -0.01, 0.00, 0.01, 0.00, 0.00, 0.02, 0.00]
#DatosGiroscopoY2 = [0.00, 0.90, 1.82, 3.05, 3.54, 5.11, 5.49, 6.22, 5.46, 4.88, 4.49, 3.75, 3.30, 2.89, 2.22, 1.86, 1.35, 0.74, 0.13, 0.00]
DatosGiroscopoY2 = [0.00, -0.04, -0.08, -0.12, -0.22, -0.38, -0.50, -0.54, -0.51, -0.40, -0.32, -0.28, -0.20, -0.11, -0.04, -0.00, -0.00, -0.00, -0.00, 0.00]

y1 = np.linspace(0,20,20)
X = np.array(DatosGiroscopoX2)
Y = np.array(DatosGiroscopoY2)
Z = np.array(DatosGiroscopoZ2)
plt.subplot(133)
plt.title(label = "Instrumento dirección 2")
plt.plot(y1, X, label = "Giroscopo en X")
plt.plot(y1, Y, label = "Giroscopo en Y")
plt.plot(y1, Z, label = "Giroscopo en Z")
plt.legend()
plt.grid(True)
plt.show()

DatosAcelerometroX = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
DatosAcelerometroY = [0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98]
DatosAcelerometroZ = [0.12, 0.10, 0.13, 0.11, 0.12, 0.13, 0.12, 0.12, 0.13, 0.12, 0.11, 0.12, 0.12, 0.12, 0.11, 0.12, 0.12, 0.12, 0.11, 0.12]

y2 = np.linspace(0,20,20)
X = np.array(DatosAcelerometroX)
Y = np.array(DatosAcelerometroY)
Z = np.array(DatosAcelerometroZ)
plt.plot(y2, X, label = "Acelerometro en X")
plt.plot(y2, Y, label = "Acelerometro en Y")
plt.plot(y2, Z, label = "Acelerometro en Z")
plt.legend()
plt.grid(True)
plt.show()

PromedioX = sum(DatosGiroscopoX)/20
PromedioY = sum(DatosGiroscopoY)/20
PromedioZ = sum(DatosGiroscopoZ)/20

PromedioX1 = sum(DatosGiroscopoX1)/20
PromedioY1 = sum(DatosGiroscopoY1)/20
PromedioZ1 = sum(DatosGiroscopoZ1)/20

PromedioX2 = sum(DatosGiroscopoX2)/20
PromedioY2 = sum(DatosGiroscopoY2)/20
PromedioZ2 = sum(DatosGiroscopoZ2)/20

V0 = [1, 0, 0] #V0 es la coordenada principal cuando el giroscopo se encuentra quieto
VX = [PromedioX, PromedioY, PromedioZ] #Vector de la rueda girando constante
V1 = [PromedioX1, PromedioY1, PromedioZ1]
V2 = [PromedioX2, PromedioY2, PromedioZ2]

print(V0)
print('------------------------------------')
print(VX)
print(V1)
print(V2)

DeltaV1 = (np.asarray(V1) - np.asarray(V0))
DeltaV2 = (np.asarray(V2) - np.asarray(V0))

print(np.matmul(np.asarray(V1).T, np.asarray(V2)))
print('-----------------------------------------')

vectKZ = np.cross(DeltaV2, DeltaV1)
vz2 = np.matmul(vectKZ.T, vectKZ)
Z = vectKZ/np.sqrt(vz2)

print(DeltaV1)
print(DeltaV2)
print(vectKZ)
print(vz2) 
print('-----------------------------------------')
print(Z) #Coordenada en el eje Z Normalizado

vectX = V0/np.linalg.norm(V0)
print(vectX) #Coordenada en el eje X Normalizado

vectY = np.cross(vectX, Z)
print(vectY) #Coordenada en el eje Y Normalizado

# Integrando datos del giroscopio en eje Y
datos1 = DatosGiroscopoY1
datos2 = DatosGiroscopoY2

# Definir dt (tiempo de muestra)
dt = 1 # 1 segundo

integral1 = trapz(datos1, dx=dt)
integral2 = trapz(datos2, dx=dt)

print("Integral Lado 1:", integral1)
print("Integral Lado 2:", integral1)

# Utilizamos los datos del giroscopio y acelerometro para hallar el ángulo
gyro_x = DatosGiroscopoX2
gyro_y = DatosGiroscopoY2
gyro_z = DatosGiroscopoZ2
accel_x = DatosAcelerometroX
accel_y = DatosAcelerometroY
accel_z = DatosAcelerometroZ

# Definen coeficientes de filtro complementario (paso-bajo acelerometro y paso-alto giroscopio) y el dt
dt = 1 # 1 segundo
alpha = 0.95 

# Posición inicial
roll = 0
pitch = 0
yaw = 0
toe_angle = 0

# Almacenar en listas los datos con respecto al tiempo los cambios de ángulo
roll_list = [roll]
pitch_list = [pitch]
yaw_list = [yaw]
toe_angle_list = [toe_angle]

# Integramos las velocidades angulares
for i in range(1, len(gyro_x)):
    roll += gyro_x[i] * dt
    pitch += gyro_y[i] * dt
    yaw += gyro_z[i] * dt

 # Se calcula la orientación del acelerometro (Eje Z)
    roll_acc = math.atan2(accel_y[i], accel_z[i]) # Determina dirección positivo o negativo
    pitch_acc = math.atan2(-accel_x[i], math.sqrt(accel_y[i]**2 + accel_z[i]**2))

    # Se aplica el filtro complementario (paso bajo)
    roll = (alpha * roll) + ((1 - alpha) * roll_acc)
    pitch = (alpha * pitch) + ((1 - alpha) * pitch_acc)

    #roll = roll
    #pitch = pitch

    # Calculo del ángulo Toe (promedio de los ejes longitudinales y )
    toe_angle = (roll - pitch) / 2

    # Unir la orientación y el ángulo Toe a las listas vacias iniciales
    roll_list.append(roll)
    pitch_list.append(pitch)
    yaw_list.append(yaw)
    toe_angle_list.append(toe_angle)

# Hallar el ángulo acumulativo con el paso del tiempo dt
# Acumulativo = cumtrapz(toe_angle_list, dx=dt, initial=0)

# Vector tiempo para graficar
t = np.arange(0, len(gyro_x) * dt, dt)

fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex=True, figsize=(8, 10))
ax1.plot(t, roll_list, 'r-', label='Roll')
ax1.set_ylabel('Roll (°Grados/Eje X)')

ax1.legend()
ax2.plot(t, pitch_list, 'g-', label='Pitch')
ax2.set_ylabel('Pitch (°Grados/Eje Y)')

ax2.legend()
ax3.plot(t, yaw_list, 'b-', label='Yaw')
ax3.set_ylabel('Yaw (°Grados/Eje Z)')

ax3.legend()
ax4.plot(t, toe_angle_list, 'm-', label='Toe angle')
ax4.set_ylabel('Ángulo')
plt.grid(True)

print(roll)
print(pitch)
print('---------------------')
print(toe_angle)
