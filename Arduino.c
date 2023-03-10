#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Datos de Calibración
float accelXzero = -0.1;
float accelYzero = 0.2;
float accelZzero = 10.1;

// Filtro complementario Paso Bajo
float alpha = 0.98; // Constante

// Output angles
float camber = 0;
float toe = 0;
float caster = 0;
float casterTotal = 0;
float wheelBase = 2.443;
float trackWidth = 1.338;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  mpu.setFullScaleAccelRange(2); // rango del acelerometro +/- 2g
}

void loop() {
  
  int16_t accelXraw, accelYraw, accelZraw;
  int16_t gyroXraw, gyroYraw, gyroZraw;
  mpu.getMotion6(&accelXraw, &accelYraw, &accelZraw, &gyroXraw, &gyroYraw, &gyroZraw);

  // Conversión de aceleración a m/s^2 y velocidad angular en grad/s
  float accelX = (float)accelXraw / 16384.0 * 9.81;
  float accelY = (float)accelYraw / 16384.0 * 9.81;
  float accelZ = (float)accelZraw / 16384.0 * 9.81;
  float gyroX = (float)gyroXraw / 131.0;
  float gyroY = (float)gyroYraw / 131.0;
  float gyroZ = (float)gyroZraw / 131.0;

  // Aplicación de la calibración
  accelX -= accelXzero;
  accelY -= accelYzero;
  accelZ -= accelZzero;

  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;

  // Integración del giroscopio a grados

  float dt = 0.01;

  roll += gyroX * dt;
  pitch += gyroY * dt;
  yaw += gyroZ * dt;

  // Ejes direccionales

  float rollAccel = atan(accelY / accelZ) * 180.0 / PI;
  // float rollAccel = atan(accelY / ((pow(accelX, 2) + pow(accelZ, 2))) * 180.0 / PI;
  float pitchAccel = atan(-accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180.0 / PI;

  // Filtro complementario

  float rollAngle = (0.98 * roll) + (0.02 * rollAccel);
  float pitchAngle = (0.98 * pitch) + (0.02 * pitchAccel);

  // Camber
  camber = atan(accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180.0 / PI;

  // Toe
  toe = atan2(tan(roll), cos(pitch) - yaw) * 180.0 / PI;
  //toe = ((rollAngle - pitchAngle) / 2);
  
  // Caster
  caster = alpha * (caster + yaw) + (1 - alpha) * atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2)))* 180.0 / PI;
  casterTotal = atan(tan(caster) * wheelBase / trackWidth);
  

  // Resultados
  Serial.print("Camber angle: ");
  Serial.print(camber);
  Serial.print(" degrees, Toe angle: ");
  Serial.print(toe);
  Serial.print(" degrees, Caster angle: ");
  Serial.print(casterTotal);
  Serial.println(" degrees");

  delay(500);
}
