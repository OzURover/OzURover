#include <ros.h>
#include <std_msgs/Float32.h>
 
//create the ros node nh. The node will be used to publish to Arduino
ros::NodeHandle nh;
 

void messageCb(const std_msgs::Float32& msg)
{
  
  Serial.println(msg.data);
  if(msg.data >=0){
    movebitch();
  }else{
    stopbitch();
  }
 
}
 
ros::Subscriber<std_msgs::Float32> sub("joyinputx", &messageCb);
 
void setup()
{
  Serial.begin(9600);
  nh.initNode();
  nh.subscribe(sub);
}
 
void loop()
{
  Serial.println("aaaa");
  nh.spinOnce();
  delay(250);
}

  int motor_falan=13;
  
void movebitch(){

  
  digitalWrite(motor_falan,HIGH);
}

void stopbitch(){
  
  digitalWrite(motor_falan,LOW);
}
