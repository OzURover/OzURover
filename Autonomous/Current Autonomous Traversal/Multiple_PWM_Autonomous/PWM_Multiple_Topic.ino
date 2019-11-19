bool Halt_fr;
double f_ratio=0.2;
double r_ratio=0.2;

#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

int l3pwm = 5;
int l3inA = 23;
int l3inB = 22;

int l2pwm = 6;
int l2inA = 25;
int l2inB = 24;

int l1pwm = 7;
int l1inA = 27;
int l1inB = 26;

int r3pwm = 2;
int r3inA = 53;
int r3inB = 52;

int r2pwm = 3;
int r2inA = 51;
int r2inB = 50;

int r1pwm = 4;
int r1inA = 49;
int r1inB = 48;

ros::NodeHandle nh;

void messageCb(const std_msgs::Float32MultiArray& joyinputx){
    if(abs(joyinputx.data[0]) == 0){
        halt(0);
    }
    if(joyinputx.data[0] < 0) {
        Halt_fr = false;
        reverse(joyinputx);
    }
    if(joyinputx.data[0] > 0){
        Halt_fr = false;
        forward(joyinputx);
    }
}

void messageCb1(const std_msgs::Float32MultiArray& joyinputy){
    if(abs(joyinputy.data[0]) == 0){
        halt(0);
    }
    if(joyinputy.data[0] != 0) {
        Halt_fr = false;
        if(joyinputy.data[0] > 0){
            enableright();
            turnRight(joyinputy);
        }
        if(joyinputy.data[0] < 0){
            enableleft();
            turnLeft(joyinputy);
        }
    }
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("joyinputx", &messageCb);
ros::Subscriber<std_msgs::Float32MultiArray> sub1("joyinputy", &messageCb1);

void setup(){
    for(int i=22; i<=27; i++){
        pinMode(i, OUTPUT);
    }
    for(int i=48; i<=53; i++){
        pinMode(i, OUTPUT);
    }
    halt(0);
    nh.getHardware()->setBaud(76800);
    nh.initNode();
    nh.subscribe(sub);
    nh.subscribe(sub1);
}

void loop(){
    nh.spinOnce();
    delay(1);
}

void forward(const std_msgs::Float32MultiArray& joyinputx){
     enableforward();
     analogWrite(r2pwm, joyinputx.data[0] * 255);
     analogWrite(r3pwm, joyinputx.data[1] * 255);
     analogWrite(r1pwm, joyinputx.data[2] * 255);
     analogWrite(l1pwm, joyinputx.data[3] * 255);
     analogWrite(l2pwm, joyinputx.data[4] * 255);
     analogWrite(l3pwm, joyinputx.data[5] * 255);
}

void reverse(const std_msgs::Float32MultiArray& joyinputx){
     enablereverse();
     analogWrite(r2pwm, joyinputx.data[0] * -255);
     analogWrite(r3pwm, joyinputx.data[1] * -255);
     analogWrite(r1pwm, joyinputx.data[2] * -255);
     analogWrite(l1pwm, joyinputx.data[3] * -255);
     analogWrite(l2pwm, joyinputx.data[4] * -255);
     analogWrite(l3pwm, joyinputx.data[5] * -255);
}

void turnRight(const std_msgs::Float32MultiArray& joyinputy){
     analogWrite(r2pwm, joyinputy.data[0] * 255);
     analogWrite(r3pwm, joyinputy.data[1] * 255);
     analogWrite(r1pwm, joyinputy.data[2] * 255);
     analogWrite(l1pwm, joyinputy.data[3] * 255);
     analogWrite(l2pwm, joyinputy.data[4] * 255);
     analogWrite(l3pwm, joyinputy.data[5] * 255);
}

void turnLeft(const std_msgs::Float32MultiArray& joyinputy){
     analogWrite(r2pwm, joyinputy.data[0] * -255);
     analogWrite(r3pwm, joyinputy.data[1] * -255);
     analogWrite(r1pwm, joyinputy.data[2] * -255);
     analogWrite(l1pwm, joyinputy.data[3] * -255);
     analogWrite(l2pwm, joyinputy.data[4] * -255);
     analogWrite(l3pwm, joyinputy.data[5] * -255);
}

void halt(int pwm){
    for(int i=22; i<=27; i++){
        digitalWrite(i, LOW);
    }
    for(int i=2; i<=7; i++){
        analogWrite(i, pwm);
    }
}


void enableleft(){
   digitalWrite(r1inB,LOW);
   digitalWrite(r2inB,LOW);
   digitalWrite(r3inB,LOW);
   digitalWrite(l1inB,LOW);
   digitalWrite(l2inB,LOW);
   digitalWrite(l3inB,LOW);

   digitalWrite(r1inA,HIGH);
   digitalWrite(r2inA,HIGH);
   digitalWrite(r3inA,HIGH);
   digitalWrite(l1inA,HIGH);
   digitalWrite(l2inA,HIGH);
   digitalWrite(l3inA,HIGH);
}

void enableright(){
   digitalWrite(r1inB,HIGH);
   digitalWrite(r2inB,HIGH);
   digitalWrite(r3inB,HIGH);
   digitalWrite(l1inB,HIGH);
   digitalWrite(l2inB,HIGH);
   digitalWrite(l3inB,HIGH);

   digitalWrite(r1inA,LOW);
   digitalWrite(r2inA,LOW);
   digitalWrite(r3inA,LOW);
   digitalWrite(l1inA,LOW);
   digitalWrite(l2inA,LOW);
   digitalWrite(l3inA,LOW);
}

void enablereverse(){
   digitalWrite(r1inB,LOW);
   digitalWrite(r2inB,LOW);
   digitalWrite(r3inB,LOW);
   digitalWrite(l1inB,HIGH);
   digitalWrite(l2inB,HIGH);
   digitalWrite(l3inB,HIGH);

   digitalWrite(r1inA,HIGH);
   digitalWrite(r2inA,HIGH);
   digitalWrite(r3inA,HIGH);
   digitalWrite(l1inA,LOW);
   digitalWrite(l2inA,LOW);
   digitalWrite(l3inA,LOW);
}

void enableforward(){
   digitalWrite(r1inB,HIGH);
   digitalWrite(r2inB,HIGH);
   digitalWrite(r3inB,HIGH);
   digitalWrite(l1inB,LOW);
   digitalWrite(l2inB,LOW);
   digitalWrite(l3inB,LOW);

   digitalWrite(r1inA,LOW);
   digitalWrite(r2inA,LOW);
   digitalWrite(r3inA,LOW);
   digitalWrite(l1inA,HIGH);
   digitalWrite(l2inA,HIGH);
   digitalWrite(l3inA,HIGH);
}
