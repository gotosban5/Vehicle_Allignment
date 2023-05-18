#include <basicMPU6050.h>
#include <MPU6050.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <TimerOne.h>
#include <SoftwareSerial.h>
#include <MechaQMC5883.h>

//Ratios de conversion
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250
 
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
 
//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
MPU6050 mpu;
MechaQMC5883 qmc;

const int NUM_READINGS = 100;
int x_offset = 0, y_offset = 0, z_offset = 0;
volatile int16_t ax, ay, az, gx, gy, gz;
 
//Angulos
float Acc[2];
float Gy[3];
float Angle[3];

int x, y, z;
int x_offset_global, y_offset_global, z_offset_global;
float azimuth;
float adjustedAzimuth;

String valores;

unsigned long tiempo_prev;
volatile int interruptFlag = 0;
unsigned long interval = 10; // intervalo en milisegundos

const byte INTERRUPT_PIN = 3;
volatile unsigned int timerOverflowCount = 0;
float dt;

SoftwareSerial analyzer(2, 3);

bool LED_STATE = true;

#define TCAADDR 0x70 // TCA9548A address
#define MPUADDR 0x6B // MPU6050 address
#define HMCADDR 0x1E // HMC5883L address

void selectChannel(int channel) {
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void readMPU() {
  mpu.initialize();
  mpu.setFullScaleAccelRange(2);
  mpu.setFullScaleGyroRange(250);
  mpu.setDLPFMode(3);

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

void readHMC() {
  Wire.beginTransmission(HMCADDR);
  Wire.write(0x03); // Start reading from register 0x03
  Wire.endTransmission(false);
  Wire.requestFrom(HMCADDR, 6, true);
  qmc.init();
  qmc.read(&x, &y, &z);
}

void calibrateMagnetometer() {
  selectChannel(1); // Select channel 1 for HMC5883L
  
  int numSamples = 100; // Number of samples to collect for calibration
  int x_offset = 0, y_offset = 0, z_offset = 0;
  
  for (int i = 0; i < numSamples; i++) {
    readHMC();
    x_offset += x;
    y_offset += y;
    z_offset += z;
    delay(10);
  }
  
  x_offset /= numSamples;
  y_offset /= numSamples;
  z_offset /= numSamples;
  
  // Update the global offset variables
  x_offset_global = x_offset;
  y_offset_global = y_offset;
  z_offset_global = z_offset;
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  
  pinMode(2, OUTPUT);        //Pin 2 de Salida del analizador
  cli();

  TCCR1A = 0;                // Reset del temporizador a 0 
  TCCR1B = 0;

  TCCR1B |= B00000011;

  TIMSK1 |= B00000010;

  OCR1A = 31250;

  sei();  

  analyzer.begin(9600);

}

void loop() {
  selectChannel(0); // Select channel 0 for MPU6050
  readMPU();
  
  calibrateMagnetometer();
}

ISR(TIMER1_COMPA_vect){
  
  timerOverflowCount++;
  TCNT1  = 0;                  //First, set the timer back to 0 so it resets for next interrupt
  LED_STATE = !LED_STATE;      //Invert LED state
  digitalWrite(2,LED_STATE);

  //A partir de los valores del acelerometro, se calculan los angulos Y, X
  //respectivamente, con la formula de la tangente.
  Acc[1] = atan(-1*(ax/A_R)/sqrt(pow((ay/A_R),2) + pow((az/A_R),2)))*RAD_TO_DEG;
  Acc[0] = atan((ay/A_R)/sqrt(pow((ax/A_R),2) + pow((az/A_R),2)))*RAD_TO_DEG;

  //Calculo del angulo del Giroscopio
  Gy[0] = gx/G_R;
  Gy[1] = gy/G_R;
  Gy[2] = gz/G_R;

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();
 
  //Aplicar el Filtro Complementario
  Angle[0] = 0.96 *(Angle[0]+Gy[0]*dt) + 0.04*Acc[0];
  Angle[1] = 0.96 *(Angle[1]+Gy[1]*dt) + 0.04*Acc[1];

  double wheelDiameter = 330.2; // Diametro de la rueda bmw 318i
  double radius = wheelDiameter / 2;
  double milimetros = (M_PI * radius * Angle[1]) / 180; // conversión de grados a milimetros.
  double Camber = asin(milimetros / wheelDiameter); // ángulo de Camber definitivo.
  double CamberT = Camber * (180.0 / M_PI);

   //Mostrar los valores por consola
  //valores = "Caster: " +String(Angle[0]) + ", Camber: " + String(Angle[1]) + ", Toe: " + String(Angle[2]);
  //Serial.println(valores);

  x -= x_offset_global;
  y -= y_offset_global;
  z -= z_offset_global;

  azimuth = qmc.azimuth(&y, &x);
  azimuth -= 180.0; // Adjust for magnetic declination

  if (azimuth < 0) {
    azimuth -= 0;
  }
  
  double milimetros2 = (M_PI * radius * azimuth) / 180; // conversión de grados a milimetros.
  double distance = milimetros2 / wheelDiameter;
  double toeAngle = asin(distance); // ángulo de Toe definitivo.
  double toeAngleDegT = toeAngle * (180.0 / M_PI);
  
  Serial.print("X:");
  Serial.print(Angle[1]);
  Serial.print(", Y:");
  Serial.print(Angle[0]);
  Serial.print(", Z:");
  Serial.print(azimuth);
  Serial.print(", TRad:");
  Serial.print(toeAngle);
  Serial.print(", CaRad:");
  Serial.print(Camber);
  Serial.print(", TGrad:");
  Serial.print(toeAngleDegT);
  Serial.print(", CaGrad:");
  Serial.println(CamberT);

  analyzer.write(0x55);

}
