#include <ros.h>
#include <std_msgs/String.h>

#define S_ActuatorInA x;
#define S_ActuatorInB x;
#define S_ActuatorPWM x;

#define D_ActuatorInA x;
#define D_ActuatorInB x;
#define D_ActuatorPWM x;

#define D_DrillInA x;
#define D_DrillInB x;
#define D_DrillPWM x;

#define C_WaterInA x;
#define C_WaterInB x;
#define C_WaterPWM x;

ros::NodeHandle nh;

void messageCb(const std_msgs::String &msg, std_msgs::String) {
  if (msg.data.substring(0,1) == "1") {
    S_Actuator_Up();
  }
  else if (msg.data.substring(2,3) == "1"){
    S_Actuator_Down();
  }
  else {
    S_Actuator_Halt();
  }
  if (msg.data.substring(4,5) == "1") {
    //TURN STEP MOTOR FASTLY
  }
  else if (msg.data.substring(6,7) == "1") {
    //STEP WITH STEP MOTOR
  }
  else {
    //STOP STEP MOTOR
  }
  if (msg.data.substring(8,9) == "1") {
    C_Water_Move();
  }
  if (msg.data.substring(10,11) == "1") {
    D_Drill_Move();
  }
  else {
    D_Drill_Halt();
  }
  if (msg.data.substring(12,13) == "1") {
    D_Actuator_Up();
  }
  else if (msg.data.substring(14,15) == "1") {
    D_Actuator_Down();
  }
  else {
    D_Actuator_Halt();
  }
  
  //soil distributor motor's degree = msg.data.substring(16)
}

ros::Subscriber<std_msgs::String> sub("ScienceController", &messageCb);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}

void S_Actuator_Halt() {
  analogWrite(S_ActuatorPWM, 0);
}

void D_Actuator_Halt() {
  analogWrite(D_ActuatorPWM, 0);
}

void D_Drill_Halt() {
  analogWrite(D_DrillPWM, 0);
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

void D_Drill_Move() {
  digitalWrite(D_DrillInA, HIGH);
  digitalWrite(D_DrillInB, LOW);
  analogWrite(D_DrillPWM, 255);
}

void C_Water_Move() {
  digitalWrite(C_WaterInA, HIGH);
  digitalWrite(C_WaterInB, LOW);
  analogWrite(C_WaterPWM, 255);
  delay(x);
  analogWrite(C_WaterPWM, 0);
}