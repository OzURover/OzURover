#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;   //if you have any work with ros in arduino ide, you need to write this

void messageCb(const std_msgs::Float32 & joyinputy, std_msgs::Float32){   // determines if the rover goes forward or backward
    if(abs(joyinputy.data) < 0.15){
        halt();
    }
    if(joyinputy.data != 0) {
        if(joyinputy.data > 0){
            enableforward();
        }
        if(joyinputy.data < 0){
            enablereverse();
        }
        move(255*abs(joyinputy.data));
    }
}

void messageCb1(const std_msgs::Float32& joyinputx, const std_msgs::Float32){  // determines if the rover turns right or left
    if(abs(joyinputx.data) < 0.15){
        halt();
    }
    if(joyinputx.data != 0) {
        if(joyinputx.data > 0){
            enableRight();
        }
        if(joyinputx.data < 0){
            enableLeft();
        }
        move(255*abs(joyinputx.data));
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
    for(int i=2; i<=7; i++){
        pinMode(i,OUTPUT);
    }

    halt();  //it is for stopping rover
    nh.initNode();
    nh.subscribe(sub);  //creating subscribers
    nh.subscribe(sub1);
    Serial.begin(9600);
}
void loop(){
    nh.spinOnce();   //ROS only processes your callbacks when you tell it to with ros::spinOnce()
    delay(1);
}

void move(int pwm){
     for(int i=2; i<=7; i++){
         analogWrite(i, pwm);
    }
}

void halt(){
    for(int k=2; k<=7; k++){
        analogWrite(k, 0);
    }
    for(int i=48; i<=53; i++){
        digitalWrite(i, LOW);
    }

    for(int j=22; j<=27; j++){
        digitalWrite(j, LOW);
    }
}


void enablereverse(){
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

void enableforward(){
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

void enableRight(){
   digitalWrite(23,LOW);
   digitalWrite(25,LOW);
   digitalWrite(27,LOW);
   digitalWrite(53,HIGH);
   digitalWrite(51,HIGH);
   digitalWrite(49,HIGH);
   
   digitalWrite(22,HIGH);
   digitalWrite(24,HIGH);
   digitalWrite(26,HIGH);
   digitalWrite(52,LOW);
   digitalWrite(50,LOW);
   digitalWrite(48,LOW);
}

void enableLeft(){
   digitalWrite(23,HIGH);
   digitalWrite(25,HIGH);
   digitalWrite(27,HIGH);
   digitalWrite(53,LOW);
   digitalWrite(51,LOW);
   digitalWrite(49,LOW);
   
   digitalWrite(22,LOW);
   digitalWrite(24,LOW);
   digitalWrite(26,LOW);
   digitalWrite(52,HIGH);
   digitalWrite(50,HIGH);
   digitalWrite(48,HIGH);
}


/*
 * 22-23 left-mid
 * 24-25 left-front
 * 27-26 left-back
 * 53-52 right-front
 * 51-50 right-mid
 * 48-49 right-back
 */
