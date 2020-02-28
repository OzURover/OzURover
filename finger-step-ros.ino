#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

const int step_pin = 3;
const int dir_pin = 2;

const int trigPin = A0;
const int echoPin = A1;

const byte M1CWPin = 7; // INA:
const byte M1CCWPin = 8; // INB:
const byte M1PWMPin = 5; // PWM çıkışı
const byte M1CurrentSensePin =11; // CS: durum çıkışı
const byte M1EnablePin = 13; // EN: KONTROL
ros::NodeHandle nh;

void messageCb(const std_msgs::Int32& data){
  if(data.data > 0) {
    digitalWrite(dir_pin, LOW);
    for(int i = 0; i <=abs(data.data); i++) {
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(3000);
      digitalWrite(step_pin, LOW);
      delayMicroseconds(3000);
    }
  }if(data.data < 0) {
    digitalWrite(dir_pin, HIGH);
    for(int i = 0; i <= abs(data.data); i++) {
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(3000);
      digitalWrite(step_pin, LOW);
      delayMicroseconds(3000);
    }
  }
}
void finger(const std_msgs::Int32& data){
  if(data.data == 1) {
      
digitalWrite(M1CWPin, LOW);
digitalWrite(M1CCWPin, HIGH);
delay(400);
digitalWrite(M1CWPin, LOW);
digitalWrite(M1CCWPin, LOW);
//
// BMOTOR TAM HIZDA
analogWrite(M1PWMPin, 30);
//
// 2 SN BEKLE
delay (250);
digitalWrite(M1CWPin, HIGH);
digitalWrite(M1CCWPin, LOW);
delay(400);
digitalWrite(M1CWPin, LOW);
digitalWrite(M1CCWPin, LOW);
//
// MOTOR TAM HIZDA
analogWrite(M1PWMPin, 30);
  }else if (data.data == 0){
    
  }
}
ros::Subscriber<std_msgs::Int32>sub("step_claw", &messageCb);
ros::Subscriber<std_msgs::Int32>sub2("FingerUrMom", &finger);
std_msgs::Float32 distanceMsg;
ros::Publisher distance("arm_distance", &distanceMsg);

void setup() {
  pinMode(M1CWPin, OUTPUT);
pinMode(M1CCWPin, OUTPUT);
pinMode(M1CurrentSensePin, INPUT);
pinMode(M1EnablePin, OUTPUT);
digitalWrite(M1CWPin, LOW);
digitalWrite(M1CCWPin, LOW);
digitalWrite(M1EnablePin, HIGH);
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  nh.getHardware()->setBaud(76800);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.advertise(distance);
}

void loop() {
  long duration, cm;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  
  distanceMsg.data = duration/29.0,2.0;
  distance.publish(&distanceMsg);
  
  nh.spinOnce();
  delay(100);
}
