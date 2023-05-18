import math

Camber1 = float(input("Ingresa el valor del Camber 1 en grados: "))
Camber2 = float(input("Ingresa el valor del Camber 2 en grados: "))

print("------------------------")
print("Camber 1:", Camber1)
print("Camber 2:", Camber2)

angle_rad1 = Camber1 * math.pi / 180
angle_rad2 = Camber2 * math.pi / 180
angle_rad3 = 15 * math.pi / 180
sin1 = math.sin(angle_rad1)
sin2 = math.sin(angle_rad2)
sin3 = math.sin(angle_rad3) * 2
CasterRad = math.atan((sin1 - sin2) / sin3)
Caster = CasterRad * 180 / math.pi

print("-------------------------------------")
print("Caster en grados:", Caster)
print("Caster en Radianes:", CasterRad)
