#include <Encoder.h>
#include "EEPROMAnything.h"

// Encoder A, B
Encoder mEncoder(2, 4);

// IN1, IN2, PWM
const int in1 = 7;
const int in2 = 6;
const int pwm_pin = 5;
const int BRK_LOW = 9;
const int BRK_HIGH = 10;

//Motor constants
const float gear_ratio = 230.0;
const float ppt = 500.0; // Pulses per Turn
const float extra_constants = 3.0784 * 4; //Extra coefficents (Pulley * 4(?))
const float Kp = 9.0;
const float Kd = 0.01;
const float f = 30.0; // 2.5-50 range
const float Kdob = 0.0;

//Program
float refAngle = 0;

//Other
long oldPosition  = -999;
long initMillis = 0;
float premeaAngle = 0.0;
float meaAngle = 0.0;
float zaman = 0.0;
//float prezaman = 0.0;
float meaangVel = 0.0;
float premeaangVel = 0.0;
float preFmeaangVel = 0.0;
float FmeaangVel = 0.0;
float dt2 = 0.11;
float a1 = 0;
float fa1 = 0;
float prefa1 = 0;
float Tdis = 0;

void setup() {
  Serial.begin(115200);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  pinMode(BRK_LOW, OUTPUT);
  pinMode(BRK_HIGH, OUTPUT);

  // Increase frequency of PWM
  TCCR0B = (TCCR0B & 0b11111000) | 0x01;

  //Restore Encoder Position
  long pos = 0;
  if (true)
    EEPROM_readAnything(0, pos);
  else  //Reset Encoder
    EEPROM_writeAnything(0, pos);
  mEncoder.write(pos);
  //Serial.println(getDegree());

  while (!Serial) {
    /* Wait for USB connection */
  }
  initMillis = millis();
}

void loop() {
  
  float dt = 0.0002;
  float Jm = 136; //gcm^2
  Jm = Jm / 10000000.0 * pow(gear_ratio, 2);
  float baudr = 115200;
  //prezaman = zaman;
  
  premeaAngle = meaAngle;
  zaman = getTime()/baudr;
  meaAngle = getDegree();
  premeaangVel = meaangVel;
  meaangVel = (meaAngle - premeaAngle) / dt;
  
  float g20 = 2*PI*f;
  float g20dt = g20*dt;
  
  float Fan = 0.2;
  float Fam = 10;

  //refAngle = -10;//Fam*sin(2*PI*Fan*zaman);
  float refangVel = 1;//Fam*2*PI*Fan*cos(2*PI*Fan*zaman);
  float Err = refAngle - meaAngle;
  float dErr = refangVel - meaangVel;
  
  float motVol = Kp * Err + Kd * dErr - Kdob*Tdis;
  
  a1 = meaangVel*Jm*g20 + motVol;
  prefa1 = fa1;
  fa1 = g20dt*a1 + (1-g20dt)*prefa1;
  Tdis = meaangVel*Jm*g20 -fa1; //if set, reduce Kp (espicially) and Kd

  if (abs(Err) <= 0.2) {
    Tdis=0;
    fa1=0;
    motVol=0;
  } else {
    Serial.print(refAngle);
    Serial.print(" ");
    Serial.println(meaAngle);
  }
  
  setVolts(motVol);
}

int setVolts(float volt) {
  if (abs(volt) > 12.0) {
    volt = volt > 0 ? 12 : -12;
  }
  if (volt == 0) {
    digitalWrite(BRK_HIGH, LOW);
    digitalWrite(BRK_LOW, HIGH);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(BRK_HIGH, HIGH);
    digitalWrite(BRK_LOW, LOW);   
  }
  if (volt > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (volt < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  int pwm = int(255.0 * pow((abs(volt) / 12.0), 2));
  analogWrite(pwm_pin, pwm);
  return pwm;
}

long oldTime = 0;
float getDegree() {
  long newPosition = mEncoder.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    if (getTime() - oldTime > 1000) {
      EEPROM_writeAnything(0, newPosition);
      oldTime = getTime();
    }
    return newPosition * (360.0 / (gear_ratio * ppt * extra_constants));
  }
}

long getTime() {
  return millis() - initMillis;
}
