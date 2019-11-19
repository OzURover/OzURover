bool Halt_fr;
double f_ratio=0.2;
double r_ratio=0.2;

#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

int r1pwm = 2;
int r1inA = 53;
int r1inB = 52;

int r2pwm = 3;
int r2inA = 51;
int r2inB = 50;

int r3pwm = 4;
int r3inA = 49;
int r3inB = 48;

int l1pwm = 6;
int l1inA = 25;
int l1inB = 24;

int l2pwm = 5;
int l2inA = 23;
int l2inB = 22;

int l3pwm = 7;
int l3inA = 27;
int l3inB = 26;

ros::NodeHandle nh;

void forward(int pwm);
void reverse(int pwm);
void turn(int pwm);
void halt(int pwm);
void enableforward();
void enablereverse();
void enableleft();
void enableright();

void messageCb(const std_msgs::Float32& joyinputy){
    if(joyinputy.data == 0){
        halt(0);
    }
    if(joyinputy.data < 0) {
        Halt_fr = false;
        enablereverse();
        reverse(-255*joyinputy.data);
    }
    if(joyinputy.data > 0){
        Halt_fr = false;
        enableforward();
        forward(255*joyinputy.data);
    }
}

void messageCb1(const std_msgs::Float32& joyinputx){
    if(joyinputx.data == 0){
        halt(0);
    }
    if(joyinputx.data != 0) {
        Halt_fr = false;
        if(joyinputx.data > 0){
            enableright();
            turn(255*joyinputx.data);
        }
        if(joyinputx.data < 0){
            enableleft();
            turn(-255*joyinputx.data);
        }
    }
}

ros::Subscriber<std_msgs::Float32> sub("joyinputy", &messageCb);
ros::Subscriber<std_msgs::Float32> sub1("joyinputx", &messageCb1);

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

void forward(int pwm){
    analogWrite(2,0.57*pwm);
    analogWrite(3,0.6*pwm);
    analogWrite(4,pwm);
    analogWrite(5,0.57*pwm);
    analogWrite(6,0.6*pwm);
    analogWrite(7,pwm);
}

void reverse(int pwm){
    analogWrite(2,0.57*pwm);
    analogWrite(3,0.6*pwm);
    analogWrite(4,pwm);
    analogWrite(5,0.57*pwm);
    analogWrite(6,0.6*pwm);
    analogWrite(7,pwm);
}

void turn(int pwm){
    analogWrite(2,0.57*pwm);
    analogWrite(3,0.6*pwm);
    analogWrite(4,pwm);
    analogWrite(5,0.57*pwm);
    analogWrite(6,0.6*pwm);
    analogWrite(7,pwm);
}

void halt(int pwm){
    for(int i=22; i<=27; i++){
        digitalWrite(i, LOW);
    }
    for(int i=48; i<=53; i++){
        digitalWrite(i, LOW);
    }
    for(int i=2; i<=7; i++){
        analogWrite(i, pwm);
    }
}


void enableforward(){
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

void enablereverse(){
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

void enableleft(){
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

void enableright(){
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
