#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(9600);

  while (!Serial) {}

  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while (1) {}
  }

  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  float dt = 0.1; // intervalo de tiempo

  float gyroXrate = gyroX;
  float gyroYrate = gyroY;
  float gyroZrate = gyroZ;

  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;
  
  // Integración del giroscopip

  roll += gyroXrate * dt;
  pitch += gyroYrate * dt;
  yaw += gyroZrate * dt;
  
  // Ejes direccionales

  float rollAccel = atan2(accelY, accelZ) * 180.0 / PI;
  float pitchAccel = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  
  // Filtro complementario

  float rollAngle = 0.96 * (roll + gyroXrate * dt) + 0.04 * rollAccel;
  float pitchAngle = 0.96 * (pitch + gyroYrate * dt) + 0.04 * pitchAccel;

  // Ángulo de Camber
  float camberAngle = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;

  // Ángulo de Toe
  float toeAngle = (rollAngle - pitchAngle) / 2;

  // Ángulo de Caster
  float casterAngle = atan2(pitchAngle, rollAngle) * 180.0 / PI;

  Serial.print("Camber Angle: ");
  Serial.println(camberAngle);
  Serial.print("Toe Angle: ");
  Serial.println(toeAngle);
  Serial.print("Caster Angle: ");
  Serial.println(casterAngle);

  delay(10);
}
