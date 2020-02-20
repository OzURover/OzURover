#include <ros.h>
#include <std_msgs/Int32.h>

const int D_ActuatorInA = x;
const int D_ActuatorInB = x;
const int D_ActuatorPWM = x;

const int S_ActuatorInA = x;
const int S_ActuatorInB = x;
const int S_ActuatorPWM = x;

const int DrillInA = x;
const int DrillInB = x;
const int DrillPWM = x;

const int Water1InA = x;
const int Water1InB = x;
const int Water1PWM = x;

const int Water2InA = x;
const int Water2InB = x;
const int Water2PWM = x;

const int Water3InA = x;
const int Water3InB = x;
const int Water3PWM = x;

const int Water4InA = x;
const int Water4InB = x;
const int Water4PWM = x;

const int stepEnable = x;
const int stepStep = x;
const int stepDirection = x;

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
    Water_Move( msg.data );   
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
    pinMode(Water1InA, OUTPUT);
    pinMode(Water1InB, OUTPUT);
    pinMode(Water1PWM, OUTPUT);
    pinMode(Water2InA, OUTPUT);
    pinMode(Water2InB, OUTPUT);
    pinMode(Water2PWM, OUTPUT);
    pinMode(Water3InA, OUTPUT);
    pinMode(Water3InB, OUTPUT);
    pinMode(Water3PWM, OUTPUT);
    pinMode(Water4InA, OUTPUT);
    pinMode(Water4InB, OUTPUT);
    pinMode(Water4PWM, OUTPUT);
    pinMode(stepEnable, OUTPUT);
    pinMode(stepStep, OUTPUT);
    pinMode(stepDirection, OUTPUT);

    digitalWrite(stepEnable, LOW);

    D_Actuator_Halt();
    S_Actuator_Halt();
    Drill_Halt();

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
    analogWrite(D_ActuatorPWM, 0);
}

void Drill_Halt() {
  analogWrite(DrillPWM, 0);
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

void Water_Move(int motor) {
  int InA;
  int InB;
  int PWM;

  if (motor == 1) {
    InA = Water1InA;
    InB = Water1InB;
    PWM = Water1PWM;
  }

  else if (motor == 2) {
    InA = Water2InA;
    InB = Water2InB;
    PWM = Water2PWM;
  }

  else if (motor == 3) {
    InA = Water3InA;
    InB = Water3InB;
    PWM = Water3PWM;
  }

  else {
    InA = Water4InA;
    InB = Water4InB;
    PWM = Water4PWM;
  }

  digitalWrite(InA, HIGH);
  digitalWrite(InB, LOW);
  analogWrite(PWM, 255);
  delay(5000);
  analogWrite(PWM, 0);
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