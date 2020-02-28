#include <ros.h>
#include <std_msgs/Int32.h>

const int D_ActuatorInA = 8;
const int D_ActuatorInB = 9;
const int D_ActuatorPWM = 2;

const int S_ActuatorInA = 48;
const int S_ActuatorInB = 49;
const int S_ActuatorPWM = 5;

const int DrillInA = 12;
const int DrillInB = 13;
const int DrillPWM = 4;

const int WaterInA = 52;
const int WaterInB = 53;
const int WaterPWM = 7;

const int stepStep = 47;
const int stepDirection = 46;

int Index;

ros::NodeHandle nh;

void drillActuator(const std_msgs::Int32 &msg, std_msgs::Int32) {
    if (msg.data == 1) {
        D_Actuator_Up();
    }
    else if (msg.data == -1) {
        D_Actuator_Down();
    }
    else {
        D_Actuator_Halt();
    }
}

void drill(const std_msgs::Int32 &msg, std_msgs::Int32) {
    if (msg.data == 1) {
        Drill_Move();
    }
    else {
        Drill_Halt();
    }
}

void water(const std_msgs::Int32 &msg, std_msgs::Int32) {
   if(msg.data== 1){  
        Water_Move();
}
   else {
        Water_Halt();
   } 
}

void sensorActuator(const std_msgs::Int32 &msg, std_msgs::Int32) {
    if (msg.data == 1) {
        S_Actuator_Up();
    }
    else if (msg.data == -1) {
        S_Actuator_Down();
    }
    else {
        S_Actuator_Halt();
    }
}

void stepper(const std_msgs::Int32 &msg, std_msgs::Int32) {
  if (msg.data == 1) {
    Step_Right();
  }
  else if (msg.data == -1) {
    Step_Left();
  }
  else if (msg.data == 2) {
    Step_Shuffle();
  }
}

ros::Subscriber<std_msgs::Int32> sub1("drillActuator", &drillActuator);
ros::Subscriber<std_msgs::Int32> sub2("drill", &drill);
ros::Subscriber<std_msgs::Int32> sub3("water", &water);
ros::Subscriber<std_msgs::Int32> sub4("sensorActuator", &sensorActuator);
ros::Subscriber<std_msgs::Int32> sub5("stepper", &stepper);

void setup()
{
    pinMode(D_ActuatorInA, OUTPUT);
    pinMode(D_ActuatorInB, OUTPUT);
    pinMode(D_ActuatorPWM, OUTPUT);
    pinMode(S_ActuatorInA, OUTPUT);
    pinMode(S_ActuatorInB, OUTPUT);
    pinMode(S_ActuatorPWM, OUTPUT);
    pinMode(DrillInA, OUTPUT);
    pinMode(DrillInB, OUTPUT);
    pinMode(DrillPWM, OUTPUT);
    pinMode(WaterInA, OUTPUT);
    pinMode(WaterInB, OUTPUT);
    pinMode(WaterPWM, OUTPUT);
    pinMode(stepStep, OUTPUT);
    pinMode(stepDirection, OUTPUT);

    D_Actuator_Halt();
    S_Actuator_Halt();
    Drill_Halt();
    Water_Halt();

    nh.initNode();
    nh.subscribe(sub1);
    nh.subscribe(sub2);
    nh.subscribe(sub3);
    nh.subscribe(sub4);
    nh.subscribe(sub5);
    
}

void loop()
{
    nh.spinOnce();
    delay(1);
}

void D_Actuator_Halt() {
    analogWrite(D_ActuatorPWM, 0);
}

void S_Actuator_Halt() {
    analogWrite(S_ActuatorPWM, 0);
}

void Drill_Halt() {
    analogWrite(DrillPWM, 0);
}

void Water_Halt() {
    analogWrite(WaterPWM, 0);
}

void D_Actuator_Up() {
  digitalWrite(D_ActuatorInA, HIGH);
  digitalWrite(D_ActuatorInB, LOW);
  analogWrite(D_ActuatorPWM, 255);
}

void D_Actuator_Down() {
  digitalWrite(D_ActuatorInA, LOW);
  digitalWrite(D_ActuatorInB, HIGH);
  analogWrite(D_ActuatorPWM, 255);
}

void S_Actuator_Up() {
  digitalWrite(S_ActuatorInA, HIGH);
  digitalWrite(S_ActuatorInB, LOW);
  analogWrite(S_ActuatorPWM, 255);
}

void S_Actuator_Down() {
  digitalWrite(S_ActuatorInA, LOW);
  digitalWrite(S_ActuatorInB, HIGH);
  analogWrite(S_ActuatorPWM, 255);
}

void Drill_Move() {
  digitalWrite(DrillInA, HIGH);
  digitalWrite(DrillInB, LOW);
  analogWrite(DrillPWM, 255);
}

void Water_Move() {
  digitalWrite(WaterInA, HIGH);
  digitalWrite(WaterInB, LOW);
  analogWrite(WaterPWM, 255);
}

void Step_Right() {
    digitalWrite(stepDirection, HIGH);
  
  for(Index = 0; Index <100; Index++) {
    digitalWrite(stepStep, HIGH);
    delayMicroseconds(500);
    digitalWrite(5, LOW);
    delayMicroseconds(500);
  }
}

void Step_Left() {
    digitalWrite(stepDirection, LOW);
  
  for(Index = 0; Index <100; Index++) {
    digitalWrite(stepStep, HIGH);
    delayMicroseconds(500);
    digitalWrite(5, LOW);
    delayMicroseconds(500);
  }
}

void Step_Shuffle() {
  int shuffleCount = 0;

  while (shuffleCount<50){
  digitalWrite(stepDirection, HIGH);

  for(Index = 0; Index < 20; Index++)
  {
    digitalWrite(5,HIGH);
    delayMicroseconds(500);
    digitalWrite(5,LOW);
    delayMicroseconds(500);
  }
  delay(100);

  digitalWrite(stepDirection, LOW);

  for(Index = 0; Index < 20; Index++)
  {
    digitalWrite(5,HIGH);
    delayMicroseconds(500);
    digitalWrite(5,LOW);
    delayMicroseconds(500);
  }
  delay(100);

  shuffleCount++;
  }
}
