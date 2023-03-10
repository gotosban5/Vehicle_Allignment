#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Calibration variables
int samples = 1000;
float accelXoffset = 0;
float accelYoffset = 0;
float accelZoffset = 0;
float gyroXoffset = 0;
float gyroYoffset = 0;
float gyroZoffset = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
}

void loop() {
  // Collect accelerometer and gyroscope data for calibration
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  for (int i = 0; i < samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    accelXoffset += ax;
    accelYoffset += ay;
    accelZoffset += az;
    gyroXoffset += gx;
    gyroYoffset += gy;
    gyroZoffset += gz;
    delay(10);
  }

  // Calculate offsets
  accelXoffset /= samples;
  accelYoffset /= samples;
  accelZoffset /= samples;
  gyroXoffset /= samples;
  gyroYoffset /= samples;
  gyroZoffset /= samples;

  // Set MPU6050 offsets
  mpu.setXAccelOffset(-accelXoffset);
  mpu.setYAccelOffset(-accelYoffset);
  mpu.setZAccelOffset(16384 - accelZoffset);
  mpu.setXGyroOffset(-gyroXoffset);
  mpu.setYGyroOffset(-gyroYoffset);
  mpu.setZGyroOffset(-gyroZoffset);

  // Print offsets to serial monitor
  Serial.print("Accelerometer offsets: X = ");
  Serial.print(-accelXoffset);
  Serial.print(", Y = ");
  Serial.print(-accelYoffset);
  Serial.print(", Z = ");
  Serial.println(16384 - accelZoffset);
  Serial.print("Gyroscope offsets: X = ");
  Serial.print(-gyroXoffset);
  Serial.print(", Y = ");
  Serial.print(-gyroYoffset);
  Serial.print(", Z = ");
  Serial.println(-gyroZoffset);

  while (1); // Stop program after calibration
}
