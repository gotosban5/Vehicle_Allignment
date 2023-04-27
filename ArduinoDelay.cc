#include <Wire.h>
#include <TimerOne.h>
#include <SoftwareSerial.h>
 
//Direccion I2C de la IMU
#define MPU 0x68
 
//Ratios de conversion
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250
 
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
 
//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
volatile int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
 
//Angulos
float Acc[2];
float Gy[3];
float Angle[3];

String valores;

unsigned long tiempo_prev;
volatile int interruptFlag = 0;
unsigned long interval = 10; // intervalo en milisegundos

const byte INTERRUPT_PIN = 3;
volatile unsigned int timerOverflowCount = 0;
float dt;

SoftwareSerial analyzer(2, 3);

bool LED_STATE = true;

void setup() {
  Wire.begin(); // remove arguments from begin() method call
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
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
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);   //A partir del 0x3B, se piden 6 registros
  AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();

  //Leer los valores del Giroscopio
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);   //A partir del 0x43, se piden 6 registros
  GyX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();

}

ISR(TIMER1_COMPA_vect){
  
  timerOverflowCount++;
  TCNT1  = 0;                  //First, set the timer back to 0 so it resets for next interrupt
  LED_STATE = !LED_STATE;      //Invert LED state
  digitalWrite(2,LED_STATE);

  //A partir de los valores del acelerometro, se calculan los angulos Y, X
  //respectivamente, con la formula de la tangente.
  Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
  Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
  //Acc[2] = atan(-1*(AcZ/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcY/A_R),2)))*RAD_TO_DEG;

  //Calculo del angulo del Giroscopio
  Gy[0] = GyX/G_R;
  Gy[1] = GyY/G_R;
  Gy[2] = GyZ/G_R;

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();
 
  //Aplicar el Filtro Complementario
  Angle[0] = 0.95 *(Angle[0]+Gy[0]*dt) + 0.05*Acc[0];
  Angle[1] = 0.95 *(Angle[1]+Gy[1]*dt) + 0.05*Acc[1];

  //Integración respecto del tiempo paras calcular el YAW
  Angle[2] = 0.95*(Angle[2] + Gy[2]*dt) + 0.05*Acc[2];
  //Angle[2] = Angle[2]+Gy[2]*dt;
 
  //Mostrar los valores por consola
  valores = "Caster: " +String(Angle[0]) + ", Camber: " + String(Angle[1]) + ", Toe: " + String(Angle[2]);
  Serial.println(valores);

  analyzer.write(0x55);

}
