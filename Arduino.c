#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Calibration values
float accelXzero = -0.1;
float accelYzero = 0.2;
float accelZzero = 10.1;

// Filter parameters
float alpha = 0.95; // Complementary filter constant

// Output angles
float camber = 0;
float toe = 0;
float caster = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  mpu.setFullScaleAccelRange(2); // Set accelerometer range to +/- 2g
}

void loop() {
  // Read accelerometer and gyroscope values
  int16_t accelXraw, accelYraw, accelZraw;
  int16_t gyroXraw, gyroYraw, gyroZraw;
  mpu.getMotion6(&accelXraw, &accelYraw, &accelZraw, &gyroXraw, &gyroYraw, &gyroZraw);

  // Convert raw values to acceleration in m/s^2 and angular velocity in deg/s
  float accelX = (float)accelXraw / 16384.0 * 9.81;
  float accelY = (float)accelYraw / 16384.0 * 9.81;
  float accelZ = (float)accelZraw / 16384.0 * 9.81;
  float gyroX = (float)gyroXraw / 131.0;
  float gyroY = (float)gyroYraw / 131.0;
  float gyroZ = (float)gyroZraw / 131.0;

  // Apply calibration offsets to accelerometer values
  accelX -= accelXzero;
  accelY -= accelYzero;
  accelZ -= accelZzero;

  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;

  // Integración del giroscopio

  float dt = 0.01;

  roll += gyroX * dt;
  pitch += gyroY * dt;
  yaw += gyroZ * dt;

  // Ejes direccionales

  float rollAccel = atan(accelY / accelZ) * 180.0 / PI;
  float pitchAccel = atan(-accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180.0 / PI;

  // Filtro complementario

  float rollAngle = (0.95 * roll) + (0.05 * rollAccel);
  float pitchAngle = (0.95 * pitch) + (0.05 * pitchAccel);

  // Calculate Camber angle
  camber = atan(accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180.0 / PI;

  // Calculate Toe angle
  toe = (((rollAngle - pitchAngle) / 2) * 180.0 / PI);
  
  // Apply Complementary Filter to Gyroscope values
  caster = alpha * (caster + gyroZ * 0.01) + (1 - alpha) * atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180.0 / PI;

  // Print angles to Serial Monitor
  Serial.print("Camber angle: ");
  Serial.print(camber);
  Serial.print(" degrees, Toe angle: ");
  Serial.print(toe);
  Serial.print(" degrees, Caster angle: ");
  Serial.print(caster);
  Serial.println(" degrees");

  delay(100);
}
