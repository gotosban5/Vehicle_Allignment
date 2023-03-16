#include <Wire.h>
#include <MPU6050.h>
#include <TimerOne.h>
#include <SoftwareSerial.h>

MPU6050 mpu;

volatile int interruptFlag = 0;
unsigned long previousTime = 0;
unsigned long interval = 20; // intervalo en milisegundos

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
float dt;
unsigned long millisOld;

SoftwareSerial analyzer(2, 3);

void timerISR() {
  interruptFlag = 1;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  mpu.setFullScaleAccelRange(2);
  mpu.setFullScaleGyroRange(250); // rango del acelerometro +/- 2g
  attachInterrupt(digitalPinToInterrupt(4), timerISR, FALLING); // Interruptor en pin 4
  Timer1.initialize(interval * 1000UL); // Inicia el temporizador
  Timer1.attachInterrupt(timerISR);
  Timer1.start();

  analyzer.begin(9600);

}

void loop() {
  if (interruptFlag) {
    noInterrupts();
    interruptFlag = 0;
    interrupts();

    unsigned long currentTime = millis();
    double elapsedTime = (currentTime - previousTime) / 1000.0; // convert to seconds
    previousTime = currentTime;
  
    volatile int16_t accelXraw, accelYraw, accelZraw;
    volatile int16_t gyroXraw, gyroYraw, gyroZraw;
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

    roll += gyroX * dt;
    pitch += gyroY * dt;
    yaw += gyroZ * dt;

    // Ejes direccionales

    float rollAccel = atan(accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180.0 / PI;
    // float rollAccel = atan(accelY / ((pow(accelX, 2) + pow(accelZ, 2))) * 180.0 / PI;
    // float pitchAccel = atan(accelZ / sqrt(pow(accelX, 2) + pow(accelY, 2))) * 180.0 / PI;
    float pitchAccel = atan(-accelX / accelZ) * 180.0 / PI;
     //float pitchAccel = atan(accelZ / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180.0 / PI;
    float yawAccel = atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2)))* 180.0 / PI;

    // Filtro complementario

    float rollAngle = (0.95 * roll) + (0.05 * rollAccel);
    float pitchAngle = (0.95 * pitch) + (0.05 * pitchAccel);
    float yawAngle = (0.95 * yaw) + (0.05 * yawAccel);

    // float rollAngle = (0.95 * rollAccel) + (0.05 * roll);
    // float pitchAngle = (0.95 * pitchAccel) + (0.05 * pitch);
    // float yawAngle = (0.95 * yawAccel) + (0.05 * yaw);

    // Camber
    camber = rollAccel;

    // Toe
    //toe = atan(pitchAngle / pitchAccel) * 180.0 / PI;
    toe = (pitchAngle - rollAngle) / 2;
    //toe = atan(pitchAngle / pitchAccel);
  
    // Caster
    //caster = pitchAngle;
    //casterTotal = yawAccel;
    casterTotal = (yawAngle * wheelBase) / trackWidth;
    //casterTotal = atan((gyroY * cos(pitchAngle) + gyroZ * sin(yawAngle)) / sqrt(pow(accelX, 2) + pow(accelY, 2)));
  

    // Resultados
    Serial.print("Camber: ");
    Serial.print(camber);
    Serial.print("°, Toe: ");
    Serial.print(toe);
    Serial.print("°, Caster: ");
    Serial.print(casterTotal);
    Serial.print("°");
    Serial.print(" Accel Y: ");
    Serial.println(accelY);

    analyzer.write(0x55);
  }
}
