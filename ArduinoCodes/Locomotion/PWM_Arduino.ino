bool Halt_fr;
double f_ratio=0.2;
double r_ratio=0.2;

#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

void messageCb(const std_msgs::Float32& joyinputy){
    if(abs(joyinputy.data) == 0){
        halt(0);
    }
    if(joyinputx.data != 0) {
        Halt_fr = false;
        if(joyinputx.data > 0){
            enabletorward();
        }
        if(joyinputx.data < 0){
            enablereverse();
        }
        move(255*joyinputx.data);
    }
}

void messageCb1(const std_msgs::Float32& joyinputx){
    if(abs(joyinputx.data) == 0){
        halt(0);
    }
    if(joyinputx.data != 0) {
        Halt_fr = false;
        if(joyinputx.data > 0){
            enableright();
        }
        if(joyinputx.data < 0){
            enableleft();
        }
        turn(255*joyinputx.data);
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
    nh.initNode();
    nh.subscribe(sub);
    nh.subscribe(sub1);
}

void loop(){  
    nh.spinOnce();
    delay(1);
}

void move(int pwm){
     enableforward();
     for(int i=2; i<=7; i++){
         analogWrite(i, pwm);
    }
}

void turn(int pwm){
    for(int i=2; i<=7; i++){
        analogWrite(i, pwm);
    }
}

void halt(int pwm){
    for(int i=22; i<=27; i++){
        digitalWrite(i, LOW);
    }
    for(int i=2; i<=7; i++){
        analogWrite(i, pwm);
    }
}


void enableforward(){
   digitalWrite(23,LOW);
   digitalWrite(25,LOW);
   digitalWrite(27,LOW);
   digitalWrite(53,LOW);
   digitalWrite(51,LOW);
   digitalWrite(49,LOW);
   
   digitalWrite(22,HIGH);
   digitalWrite(24,HIGH);
   digitalWrite(26,HIGH);
   digitalWrite(52,HIGH);
   digitalWrite(50,HIGH);
   digitalWrite(48,HIGH);
}

void enablereverse(){
   digitalWrite(23,HIGH);
   digitalWrite(25,HIGH);
   digitalWrite(27,HIGH);
   digitalWrite(53,HIGH);
   digitalWrite(51,HIGH);
   digitalWrite(49,HIGH);
   
   digitalWrite(22,LOW);
   digitalWrite(24,LOW);
   digitalWrite(26,LOW);
   digitalWrite(52,LOW);
   digitalWrite(50,LOW);
   digitalWrite(48,LOW);
}

void enableLeft(){
   digitalWrite(23,LOW);
   digitalWrite(25,LOW);
   digitalWrite(27,LOW);
   digitalWrite(53,HIGH);
   digitalWrite(51,HIGH);
   digitalWrite(49,HIGH);
   
   digitalWrite(22,LOW);
   digitalWrite(24,LOW);
   digitalWrite(26,LOW);
   digitalWrite(52,HIGH);
   digitalWrite(50,HIGH);
   digitalWrite(48,HIGH);
}

void enableRight(){
   digitalWrite(23,HIGH);
   digitalWrite(25,HIGH);
   digitalWrite(27,HIGH);
   digitalWrite(53,LOW);
   digitalWrite(51,LOW);
   digitalWrite(49,LOW);
   
   digitalWrite(22,HIGH);
   digitalWrite(24,HIGH);
   digitalWrite(26,HIGH);
   digitalWrite(52,LOW);
   digitalWrite(50,LOW);
   digitalWrite(48,LOW);
}
