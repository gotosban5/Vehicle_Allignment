import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

ax = 0.981
ay = -0.0324
az = 0.155

if (ax <= 1.1 and ax >= -1.1) and (ay <= 1.1 and ay >= -1.1) and (az <= 1.1 and az >= -1.1):
  print("Ax = ", ax)
  print("Ay = ", ay)
  print("Az = ", az)
else:
   print("Los datos no son coincidentes con el acelerometro. Ingresar un numero de -1 a 1")
   
import scipy as sp
yaw_angle = math.degrees(-(math.atan(ax/az)))
camber_angle = math.degrees(math.atan(ay/(math.sqrt((ax**2)+(az**2)))))

ángulo = camber_angle/ 470
camber_angle_degrees = ángulo * 180 / math.pi
print(camber_angle)
print("Ángulo de Camber: {:.2f} Grados".format(camber_angle_degrees))

import csv

file = open('Libro1.csv')
matriz = np.genfromtxt(file, delimiter=",") #Open a file with the accelerometer data

print(matriz)
# csvreader = csv.reader(file)
# rows = []
# for row in csvreader: 
        # rows.append(row)
# rows

def column(matrix, i):
    return [row[i] for row in matrix]

CoordenadaX = column(matriz, 0)
CoordenadaY = column(matriz, 1)
CoordenadaZ = column(matriz, 2)

print("Coordenadas en X", CoordenadaX)
print("Coordenadas en Y", CoordenadaY)
print("Coordenadas en Z",CoordenadaZ)

import matplotlib.pyplot as plt
plt.close('all')
y = np.linspace(0,101,101)
X = np.array(CoordenadaX)
Y = np.array(CoordenadaY)
Z = np.array(CoordenadaZ)
plt.plot(y, X, label = "Coordenada en X")
plt.plot(y, Y, label = "Coordenada en Y")
plt.plot(y, Z, label = "Coordenada en Z")
plt.legend()
plt.show()

transpuesta = np.transpose(matriz)

datos = len(CoordenadaX)
print(datos)

Valor_esperadoX = sum(CoordenadaX)/datos
print(Valor_esperadoX)

Valor_esperadoY = sum(CoordenadaY)/datos
print(Valor_esperadoY)

Valor_esperadoZ = sum(CoordenadaZ)/datos
print(Valor_esperadoZ)

X2 = [n**2 for n in CoordenadaX]
Y2 = [m**2 for m in CoordenadaY]
Z2 = [p**2 for p in CoordenadaZ]

print(X2)
print(Y2)
print(Z2)

from itertools import zip_longest

vector_denominador = [sum(n) for n in zip_longest(X2, Y2, Z2, fillvalue=0)]
vector = sum(vector_denominador)/datos
print(vector)

import math
FuncionLambda = 101*(((math.sqrt((Valor_esperadoX)**2)+((Valor_esperadoY)**2)+((Valor_esperadoZ)**2))/vector)-1)
print(FuncionLambda) #Lambda Function for Lagrange ecuation in optimization problem

Valor_optimoX = (Valor_esperadoX*datos)/(datos+FuncionLambda)
Valor_optimoY = (Valor_esperadoY*datos)/(datos+FuncionLambda)
Valor_optimoZ = (Valor_esperadoZ*datos)/(datos+FuncionLambda)

print(Valor_optimoX)
print(Valor_optimoY)
print(Valor_optimoZ)

print(Valor_esperadoX, Valor_optimoX)
print(Valor_esperadoY, Valor_optimoY)
print(Valor_esperadoZ, Valor_optimoZ)

Lineaoptima = math.sqrt(((Valor_optimoX)**2)+((Valor_optimoY)**2)+((Valor_optimoZ)**2))
Lineapromedio = math.sqrt(((Valor_esperadoX)**2)+((Valor_esperadoY)**2)+((Valor_esperadoZ)**2))

print(Lineaoptima)
print(Lineapromedio)

plt.close('all')
plt.plot(vector_denominador, 'k.')
plt.plot([0,101], [Lineapromedio, Lineapromedio], 'r', label = "Valor Esperado")
plt.plot([0,101], [Lineaoptima, Lineaoptima], 'g', label = "Valor Optimo")
plt.plot([0,101], [vector, vector], 'b', label = "Valor Promedio de datos")
plt.legend() 
plt.show()

import math

ángulo = math.degrees(math.atan(Valor_optimoY/(math.sqrt((Valor_optimoX**2)+(Valor_optimoZ**2)))))
camber_angle = ángulo / 470 #Tomando un meumatico de 18.5 pulgadas
camber_angle_degrees = camber_angle * 180 / math.pi
print("Ángulo de Camber: {:.2f} Grados".format(camber_angle_degrees))
