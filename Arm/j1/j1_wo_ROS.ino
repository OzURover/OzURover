/* 
  IMPORTANT! 
  This code uses an external library DIO2, it needs to be downloaded from: 
  http://www.codeproject.com/Articles/732646/Fast-digital-I-O-for-Arduino
  If the library is not installed properly, the code will not compile and/or 
  the readings coming from the encoder will not be correct.
*/

#include <PID_v1.h>
#include <avr/wdt.h>
#include "EEPROMAnything.h"
#include <Encoder.h>

#define GPIO2_PREFER_SPEED 1
#define c_EncoderPinA 2
#define c_EncoderPinB 4

Encoder enc(2, 4);

#define gearRatio 156 

volatile bool _EncoderBSet;
volatile long _EncoderTicks = 0;
int direction1 = 1; // the direction that the motor is rotating to
double angle1 = 1.5708; // angle of the motor wrt to the initial position

//PID Library Setup
//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
double PID_V; //Input that will be sent to the motor as PWM
double GoalPosition = (90.0+0.0) / (180/PI); //The angle to be reached

//Timer
long oldTime = 0;

//6.65 0723 0.51
PID armPID (&angle1, &PID_V, &GoalPosition,120,0,0, DIRECT); //angle1 is encoder reading

void setup() {
  //Encoder Settings
  pinMode(c_EncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(c_EncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_EncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(c_EncoderPinB, LOW);  // turn on pullup resistors
  //attachInterrupt(0, MotorInterruptA, RISING); //assignment of the interrupt pin
  
  //Motor settings
  pinMode(6, OUTPUT); 
  pinMode(7, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  
  halt(); //Halt the motor 
   
  //digitalWrite(13,HIGH);
  //delay(2500);
  //digitalWrite(13,LOW);
  
  //Enable PID
  armPID.SetOutputLimits(-12,12);
  armPID.SetMode(AUTOMATIC);

  Serial.begin(115200);
  
  wdt_disable();
  
  //Start timer and restore Encoder
  EEPROM_readAnything(0, _EncoderTicks);
  enc.write(_EncoderTicks);
  angle1 = 1.5708 + (1.0/(500.0*156.0*7.5))*5.49*_EncoderTicks;
  /*EEPROM.write(0, 0);
  EEPROM.write(1, 0);
  EEPROM.write(2, 0);
  EEPROM.write(3, 0);
  EEPROM.write(4, 0);
  EEPROM.write(5, 0);
  EEPROM.write(6, 0);
  EEPROM_readAnything(0, _EncoderTicks);*/
  oldTime = millis();
}

void loop() {
/*
  -----------------------------------------------
  THE CONTROL LOOP
  -----------------------------------------------
  
  By using the angle1, direction1, CW, CCW and halt a control loop will be implemented here. 
  The loop needs to take a goal position for the motor through serial input. 
*/

  armPID.Compute();
  
  //CCW(PID_PWM);
  if(abs(GoalPosition-angle1) <= 0.01){
    PID_V=0;
  }
  //PID_V = -5;
  int PID_PWM = int(255.0 * pow(PID_V/12.0, 2));
  if(PID_V > 0) CW(abs(PID_PWM));
  else if(PID_V < 0) CCW(abs(PID_PWM));
  else halt();

  /*Serial.print(GoalPosition * 180/PI);
  Serial.print(" ");
  Serial.print(angle1 * 180/PI);
  Serial.print(" ");
  Serial.print(PID_V);
  Serial.print(" ");
  Serial.println(PID_PWM);*/
  Serial.print(_EncoderTicks);
  Serial.print(" ");
  Serial.println(angle1 * 180/PI);

  if (millis() - oldTime > 1000) {
    EEPROM_writeAnything(0, _EncoderTicks);
    oldTime = millis();
  }

  _EncoderTicks = enc.read();
  // Coefficient -> [1/cpt]*3.14*7.5(reduktor)*[motor gear ratio]
  //cpt: counts per turn 500 for HEDL 5540 A02
  angle1 = 1.5708 + (1.0/(500.0*156.0*7.5))*5.49*_EncoderTicks;
  
  //delay(5);
}

//This method sets the motor to turn in clockwise direction
void CW(double PID_PWM){
  digitalWrite(10, HIGH);
  digitalWrite(9, LOW);
  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);
  analogWrite(5, PID_PWM);
}

//This method sets the motor to turn in counter-clockwise direction
void CCW(double PID_PWM){
  digitalWrite(10, HIGH);
  digitalWrite(9, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, HIGH);
  analogWrite(5, PID_PWM);
}

//This method stops the motor
void halt(){
  digitalWrite(10,LOW);
  digitalWrite(9, HIGH);  //ınverted signal
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  analogWrite(5, 0);
}
