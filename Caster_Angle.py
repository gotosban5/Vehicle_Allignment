import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import matplotlib.pyplot as plt

DatosGiroscopoX = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
DatosGiroscopoY = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
DatosGiroscopoZ = [120, 120, 120, 120, 120, 119, 120, 120, 120, 121, 120, 121, 120, 120, 120, 121, 120, 120, 120, 120]

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

#Datos de giroscopio, segunda dirección
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

DatosAcelerometroX = [0.965015, 0.963738, 0.963179, 0.962082, 0.960082, 0.964082, 0.959082, 0.962082, 0.964082, 0.962082, 0.965082, 0.959082, 0.962082, 0.958082, 0.959082, 0.958082, 0.960082, 0.964082, 0.962082, 0.960082]
DatosAcelerometroY = [-0.030235, -0.029975, -0.029483, -0.031146, -0.032146, -0.033146, -0.032146, -0.031146, -0.028146, -0.029146, -0.034146, -0.030146, -0.032146, -0.033146, -0.030146, -0.028146, -0.028146, -0.029146, -0.033146, -0.032146]
DatosAcelerometroZ = [0.377509, 0.371002, 0.376355, 0.375429, 0.37043, 0.377431, 0.368432, 0.374433, 0.378434, 0.371435, 0.377436, 0.369437, 0.375438, 0.378439, 0.36944, 0.374441, 0.372442, 0.369443, 0.372444, 0.377445]

#DatosAcelerometroX = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
#DatosAcelerometroY = [0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98, 0.98]
#DatosAcelerometroZ = [0.12, 0.10, 0.13, 0.11, 0.12, 0.13, 0.12, 0.12, 0.13, 0.12, 0.11, 0.12, 0.12, 0.12, 0.11, 0.12, 0.12, 0.12, 0.11, 0.12]


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

gyro_x = DatosGiroscopoX2
gyro_y = DatosGiroscopoY2
gyro_z = DatosGiroscopoZ2
accel_x = DatosAcelerometroX
accel_y = DatosAcelerometroY
accel_z = DatosAcelerometroZ

def calculate_caster_angle_with_imu(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, wheelbase, track_width, alpha):
    
    # Convert accelerometer data to inclination angle in radians
    inclination_angle = [math.atan2(accel_y[i], math.sqrt(accel_x[i] ** 2 + accel_z[i] ** 2)) for i in range(len(accel_x))]
    
    # Convert gyroscope data to inclination angle rate in radians per second
    inclination_angle_rate = gyro_x
    
    # Filtro complementario
    dt = 1 
    angulo = 0.0
    for i in range(len(inclination_angle)):
        angulo = alpha * (angulo + inclination_angle_rate[i] * dt) + (1 - alpha) * inclination_angle[i]
    
    # Calculamos el Caster de acuerdo a las dimensiones del vehículo y sus ángulos de inclinación
    caster_angle_rad = math.atan(math.tan(angulo) * wheelbase / track_width)
    caster_angle_deg = math.degrees(caster_angle_rad)
    
    return caster_angle_deg
    
wheelbase = 2.443 # Dimensiones de un Chevrolet Corsa (longitud entre rueda delantera y trasera)
track_width = 1.387 # (longitud entre rueda izquierda y derecha)
alpha = 0.9
caster_angle = calculate_caster_angle_with_imu(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, wheelbase, track_width, alpha)
print("El ángulo definitivo Caster es:" , caster_angle)

wheelbase = 2.443 # Dimensiones de un Chevrolet Corsa (longitud entre rueda delantera y trasera)
track_width = 1.387 # (longitud entre rueda izquierda y derecha)
alpha = 0.85
caster_angle2 = calculate_caster_angle_with_imu(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, wheelbase, track_width, alpha)
print("El ángulo definitivo Caster es:" , caster_angle2)

wheelbase = 2.443 # Dimensiones de un Chevrolet Corsa (longitud entre rueda delantera y trasera)
track_width = 1.387 # (longitud entre rueda izquierda y derecha)
alpha = 0.95
caster_angle3 = calculate_caster_angle_with_imu(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, wheelbase, track_width, alpha)
print("El ángulo definitivo Caster es:" , caster_angle3)

wheelbase = 2.443 # Dimensiones de un Chevrolet Corsa (longitud entre rueda delantera y trasera)
track_width = 1.387 # (longitud entre rueda izquierda y derecha)
alpha = 0.98
caster_angle4 = calculate_caster_angle_with_imu(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, wheelbase, track_width, alpha)
print("El ángulo definitivo Caster es:" , caster_angle4)

wheelbase = 2.443 # Dimensiones de un Chevrolet Corsa (longitud entre rueda delantera y trasera)
track_width = 1.387 # (longitud entre rueda izquierda y derecha)
alpha = 0.875
caster_angle5 = calculate_caster_angle_with_imu(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, wheelbase, track_width, alpha)
print("El ángulo definitivo Caster es:" , caster_angle5)

wheelbase = 2.443 # Dimensiones de un Chevrolet Corsa (longitud entre rueda delantera y trasera)
track_width = 1.387 # (longitud entre rueda izquierda y derecha)
alpha = 0.92
caster_angle6 = calculate_caster_angle_with_imu(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, wheelbase, track_width, alpha)
print("El ángulo definitivo Caster es:" , caster_angle6)

wheelbase = 2.443 # Dimensiones de un Chevrolet Corsa (longitud entre rueda delantera y trasera)
track_width = 1.387 # (longitud entre rueda izquierda y derecha)
alpha = 0.94
caster_angle7 = calculate_caster_angle_with_imu(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, wheelbase, track_width, alpha)
print("El ángulo definitivo Caster es:" , caster_angle7)

wheelbase = 2.443 # Dimensiones de un Chevrolet Corsa (longitud entre rueda delantera y trasera)
track_width = 1.387 # (longitud entre rueda izquierda y derecha)
alpha = 0.97
caster_angle8 = calculate_caster_angle_with_imu(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, wheelbase, track_width, alpha)
print("El ángulo definitivo Caster es:" , caster_angle8)

y1 = caster_angle
y2 = caster_angle2
y3 = caster_angle3
y4 = caster_angle4
y5 = caster_angle5
y6 = caster_angle6
y7 = caster_angle7
y8 = caster_angle8

plt.scatter(0.9, y1)
plt.scatter(0.85, y2)
plt.scatter(0.95, y3)
plt.scatter(0.98, y4)
plt.scatter(0.875, y5)
plt.scatter(0.92, y6)
plt.scatter(0.94, y7)
plt.scatter(0.97, y8)
plt.xlabel('Señal')
plt.ylabel('Ángulo')
plt.grid(True)

plt.legend(["Punto Filtrada (0.9)" , "Punto Filtrada (0.85)", "Punto Filtrada (0.95)", "Punto Filtrada (0.98)", "Punto Filtrada (0.875)", "Punto Filtrada (0.92)", "Punto Filtrada (0.94)", "Punto Filtrada (0.97)"])
plt.show()
