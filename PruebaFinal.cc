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
float azimuth;

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
  
  selectChannel(1); // Select channel 1 for HMC5883L
  readHMC();
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

   //Mostrar los valores por consola
  //valores = "Caster: " +String(Angle[0]) + ", Camber: " + String(Angle[1]) + ", Toe: " + String(Angle[2]);
  //Serial.println(valores);

  x -= x_offset;
  y -= y_offset;
  z -= z_offset;
  
  azimuth = qmc.azimuth(&y, &x);
  azimuth = azimuth - 180.0; // adjust for magnetic declination
  
  if (azimuth < 0) {
    azimuth += 360.0;
  }

  Serial.print("Mx: ");
  Serial.print(x);
  Serial.print(", My: ");
  Serial.print(y);
  Serial.print(", Mz: ");
  Serial.print(z);
  Serial.print(", T: ");
  Serial.print(azimuth);
  Serial.print(", Ca:");
  Serial.print(Angle[1]);
  Serial.print(", Cs:");
  Serial.println(Angle[0]);

  analyzer.write(0x55);

}



