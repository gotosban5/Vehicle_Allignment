#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Datos de Calibración
float accelXzero = 0;
float accelYzero = 0;
float accelZzero = 0;

// Filtro complementario Paso Bajo
float alpha = 0.95; // Constante

// Output angles
float camber = 0;
float toe = 0;
float caster = 0;
float casterTotal = 0;
float wheelBase = 2.443;
float trackWidth = 1.338;
float dt = 0.01;

volatile unsigned long loopStartTime = 0;
volatile unsigned long loopEndTime = 0;
volatile bool newData = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  mpu.setFullScaleAccelRange(2); // rango del acelerometro +/- 2g

  attachInterrupt(digitalPinToInterrupt(2), newDataISR, RISING);
}

void loop() {

if (newData) {
    noInterrupts();
    unsigned long startTime = loopStartTime;
    unsigned long endTime = loopEndTime;
    bool newDataFlag = newData;
    newData = false;
    interrupts();

    // Calculate loop time
    unsigned long loopTime = endTime - startTime;

  
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

   accelX -= accelXzero;
   accelY -= accelYzero;
   accelZ -= accelZzero;

   float roll = 0.0;
   float pitch = 0.0;
   float yaw = 0.0;

   // Integración del giroscopio a grados

   roll += gyroX * dt;
   pitch += gyroY * dt;
   yaw += gyroZ * dt;

   // Ejes direccionales

   float rollAccel = atan(accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180.0 / PI;
   // float rollAccel = atan(accelY / ((pow(accelX, 2) + pow(accelZ, 2))) * 180.0 / PI;
   float pitchAccel = atan(accelZ / sqrt(pow(accelX, 2) + pow(accelY, 2))) * 180.0 / PI;
   float yawAccel = atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2)))* 180.0 / PI;

   // Filtro complementario

   // float rollAngle = (0.95 * roll) + (0.05 * rollAccel);
   // float pitchAngle = (0.95 * pitch) + (0.05 * pitchAccel);
   // float yawAngle = (0.95 * yaw) + (0.05 * yawAccel);

   float rollAngle = (0.95 * rollAccel) + (0.05 * roll);
   float pitchAngle = (0.95 * pitchAccel) + (0.05 * pitch);
   float yawAngle = (0.95 * yawAccel) + (0.05 * yaw);

   // Camber
   camber = rollAccel;

   // Toe
   toe = (pitchAngle - rollAngle) / 2;
   //toe = (rollAngle - yawAngle) / 2;
  
   // Caster
   //caster = pitchAngle;
   //casterTotal = yawAccel;
   casterTotal = (yawAngle * wheelBase) / trackWidth;
   //casterTotal = atan((gyroY * cos(pitchAngle) + gyroZ * sin(yawAngle)) / sqrt(pow(accelX, 2) + pow(accelY, 2)));
  

   // Resultados
   Serial.print("Camber: ");
   Serial.print(camber);
   Serial.print(" °, Toe: ");
   Serial.print(toe);
   Serial.print(" °, Caster: ");
   Serial.print(casterTotal);
   Serial.println(" °");

  }
}

void newDataISR() {
  loopStartTime = micros();
  loopEndTime = loopStartTime + (dt * 1000000);
  newData = true;
}
