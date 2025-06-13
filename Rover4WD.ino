/* File: Rover4WD.ino
   Rover 4WD Autonomo & Manuale con sensori multipli - Arduino
   By Bocaletto Luca
*/

// ===== Librerie =====
#include <Wire.h>
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <QTRSensors.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <BH1750.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ===== Pin e Config =====
// TB6612FNG #1 (M1/M2)
#define AIN1   22
#define AIN2   23
#define PWMA   24
// TB6612FNG #2 (M3/M4)
#define BIN1   25
#define BIN2   26
#define PWMB   27
#define STBY   30

// Bluetooth HC-05
#define BT_RX  19
#define BT_TX  18
SoftwareSerial bt(BT_RX, BT_TX);

// Line Sensor QTR-8A
#define QTR_COUNT   8
#define QTR_PIN_EN  30
const uint8_t QTR_PIN[QTR_COUNT] = {A0,A1,A2,A3,A4,A5,A6,A7};
QTRSensorsRC qtr(rc, QTR_PIN, QTR_COUNT, QTR_PIN_EN, 2000);

// Ultrasonic
#define US_FRONT_TRIG 2
#define US_FRONT_ECHO 3
#define US_LEFT_TRIG  4
#define US_LEFT_ECHO  5
#define US_RIGHT_TRIG 6
#define US_RIGHT_ECHO 7
NewPing usFront(US_FRONT_TRIG,US_FRONT_ECHO,200);
NewPing usLeft(US_LEFT_TRIG,US_LEFT_ECHO,200);
NewPing usRight(US_RIGHT_TRIG,US_RIGHT_ECHO,200);

// MPU-6050 Tilt
MPU6050 imu;
const float TILT_LIMIT = 30.0;

// BH1750 (lux)
BH1750 lightMeter;

// DS18B20 Temp
#define ONEWIRE_PIN 8
OneWire oneWire(ONEWIRE_PIN);
DallasTemperature ds18b20(&oneWire);

// LED RGB + Buzzer
#define LED_R 9
#define LED_G 10
#define LED_B 11
#define BUZZER 12

// Modalità
volatile uint8_t mode = 0; // 0=standby,1=BT,2=line,FOLL,3=avoid

// ===== Setup =====
void setup() {
  // Motori
  pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT); pinMode(PWMA,OUTPUT);
  pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT); pinMode(PWMB,OUTPUT);
  pinMode(STBY,OUTPUT); digitalWrite(STBY,HIGH);

  // BT
  bt.begin(9600);

  // Line sensor
  pinMode(QTR_PIN_EN, OUTPUT);
  digitalWrite(QTR_PIN_EN, LOW);
  qtr.reset();

  // Ultrasonic
  // (NewPing inizializzati in globale)

  // MPU-6050
  Wire.begin();
  imu.initialize();

  // BH1750
  lightMeter.begin();

  // DS18B20
  ds18b20.begin();

  // LED/Buzzer
  pinMode(LED_R,OUTPUT); pinMode(LED_G,OUTPUT); pinMode(LED_B,OUTPUT);
  pinMode(BUZZER,OUTPUT);

  // Accensione
  setRGB(255,255,255);
  delay(500);
  setRGB(0,0,0);
}

// ===== Loop =====
void loop() {
  // 1) Controllo tilt
  if (isTiltExceeded()) {
    stopAll();
    alarmRGB(255,0,0,3);
    return;
  }
  // 2) Gestione modalità da BT
  if (bt.available()) {
    char c = bt.read();
    parseBT(c);
  }
  // 3) Esecuzione modalità
  switch (mode) {
    case 0: stopAll(); break;
    case 1: // Manuale
      manualDrive();
      break;
    case 2: // Line follow
      lineFollow();
      break;
    case 3: // Avoidance
      obstacleAvoid();
      break;
  }
}

// ===== Funzioni di Guida =====
void stopAll() {
  digitalWrite(AIN1,LOW); digitalWrite(AIN2,LOW); analogWrite(PWMA,0);
  digitalWrite(BIN1,LOW); digitalWrite(BIN2,LOW); analogWrite(PWMB,0);
}

void drive(int dir, int speed) {
  // dir:  1=F,2=B,3=L,4=R,5=S
  switch(dir) {
    case 1: // avanti
      digitalWrite(AIN1,HIGH); digitalWrite(AIN2,LOW);
      digitalWrite(BIN1,HIGH); digitalWrite(BIN2,LOW);
      analogWrite(PWMA,speed);  analogWrite(PWMB,speed);
      break;
    case 2: // indietro
      digitalWrite(AIN1,LOW); digitalWrite(AIN2,HIGH);
      digitalWrite(BIN1,LOW); digitalWrite(BIN2,HIGH);
      analogWrite(PWMA,speed);  analogWrite(PWMB,speed);
      break;
    case 3: // sinistra (spin)
      digitalWrite(AIN1,LOW); digitalWrite(AIN2,HIGH);
      digitalWrite(BIN1,HIGH); digitalWrite(BIN2,LOW);
      analogWrite(PWMA,speed);  analogWrite(PWMB,speed);
      break;
    case 4: // destra (spin)
      digitalWrite(AIN1,HIGH); digitalWrite(AIN2,LOW);
      digitalWrite(BIN1,LOW); digitalWrite(BIN2,HIGH);
      analogWrite(PWMA,speed);  analogWrite(PWMB,speed);
      break;
    default: stopAll(); break;
  }
}

// ===== Modalità Manuale =====
char lastCmd = 'S';
void parseBT(char c) {
  switch (c) {
    case 'F': mode=1; lastCmd='F'; break;
    case 'B': mode=1; lastCmd='B'; break;
    case 'L': mode=1; lastCmd='L'; break;
    case 'R': mode=1; lastCmd='R'; break;
    case 'S': mode=1; lastCmd='S'; break;
    case 'N': mode=2; break;
    case 'O': mode=3; break;
    case 'P': mode=0; break;
  }
}
void manualDrive() {
  int speed = 200;
  if (lastCmd=='F') drive(1,speed);
  if (lastCmd=='B') drive(2,speed);
  if (lastCmd=='L') drive(3,speed);
  if (lastCmd=='R') drive(4,speed);
  if (lastCmd=='S') stopAll();
}

// ===== Mode 2: Line Follow =====
void lineFollow() {
  unsigned int data[QTR_COUNT];
  qtr.read(data);
  long position = qtr.readLine(data);
  // Simple P-controller
  int error = position - 3500; // centro ~3500
  int turn  = error / 20;
  int base  = 200;
  int l = constrain(base - turn, 0, 255);
  int r = constrain(base + turn, 0, 255);
  // Mappatura su motori A (l) e B (r)
  digitalWrite(AIN1, l>0); digitalWrite(AIN2, l<=0);
  digitalWrite(BIN1, r>0); digitalWrite(BIN2, r<=0);
  analogWrite(PWMA, abs(l)); analogWrite(PWMB, abs(r));
}

// ===== Mode 3: Obstacle Avoidance =====
void obstacleAvoid() {
  unsigned int dF = usFront.ping_cm();
  if (dF>0 && dF<20) {
    stopAll(); delay(100);
    // reversa
    drive(2,200); delay(400);
    // turn random
    if (random(0,2)) drive(3,200);
    else drive(4,200);
    delay(500);
  } else {
    drive(1,180);
  }
}

// ===== Tilt Safety =====
bool isTiltExceeded() {
  int16_t ax,ay,az;
  imu.getAcceleration(&ax,&ay,&az);
  float xg=ax/16384.0, yg=ay/16384.0, zg=az/16384.0;
  float roll  = atan2(yg,zg)*57.3;
  float pitch = atan2(-xg,sqrt(yg*yg+zg*zg))*57.3;
  return (abs(roll)>TILT_LIMIT || abs(pitch)>TILT_LIMIT);
}

// ===== LED, Buzzer & Utility =====
void setRGB(uint8_t r,uint8_t g,uint8_t b) {
  analogWrite(LED_R,r);
  analogWrite(LED_G,g);
  analogWrite(LED_B,b);
}
void alarmRGB(uint8_t r,uint8_t g,uint8_t b,int times) {
  for(int i=0;i<times;i++){
    setRGB(r,g,b); tone(BUZZER,1000);
    delay(200);
    setRGB(0,0,0); noTone(BUZZER);
    delay(200);
  }
}
