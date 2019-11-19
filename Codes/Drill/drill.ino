#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <HX711.h>

ros::NodeHandle nh;

const int actuator_inA = 3;
const int actuator_inB = 4;
const int actuator_pwm = 2;

const int motor_inA = 8;
const int motor_inB = 9;
const int motor_pwm = 10;

const int motor2_inA = 6;
const int motor2_inB = 7;
const int motor2_pwm = 5;

const int loadcell_dout = 26;
const int loadcell_sck = 25;
const int calibration_factor = ?;

HX711 scale;


void actuatorCb(const std_msgs::Int32 & drillActuator, std_msgs::Int32){
  if(drillActuator.data == 1){
      actuator_move();
    }
  else{
      actuator_halt();
    }
}

void motorCb(const std_msgs::Int32 & drillMotor, std_msgs::Int32){
  if(drillMotor.data == 1){
    motor_up();
  }
  else if(drillMotor.data == -1){
    motor_down();
  }
  else{
    motor_halt();
  }
}

void motor2Cb(const std_msgs::Int32 & drillMotor2, std_msgs::Int32){
  if(drillMotor2.data == 1){
    motor2_up();
  }
  else if(drillMotor2.data == -1){
    motor2_down();
  }
  else{
    motor2_halt();
  }
}

std_msgs::Float32 loadcell_msg;

ros::Publisher drillLoadcell("drillLoadcell", &loadcell_msg);
ros::Subscriber<std_msgs::Int32> sub("drillActuator", &actuatorCb);
ros::Subscriber<std_msgs::Int32> sub1("drillMotor", &motorCb);
ros::Subscriber<std_msgs::Int32> sub2("drillMotor2", &motor2Cb);

void setup(){
  pinMode(actuator_inA, OUTPUT);
  pinMode(actuator_inB, OUTPUT);
  pinMode(actuator_pwm, OUTPUT);
  pinMode(motor_inA, OUTPUT);
  pinMode(motor_inB, OUTPUT);
  pinMode(motor_pwm, OUTPUT);
  pinMode(motor2_inA, OUTPUT);
  pinMode(motor2_inB, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);

  scale.begin(loadcell_dout, loadcell_sck);
  scale.set_scale(calibration_factor);
  scale.tare();

  actuator_halt();
  motor_halt();
  motor2_halt();

  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.advertise(drillLoadcell);
}

void loop(){
  loadcell_msg.data = scale.get_units();
  drillLoadcell.publish(&loadcell_msg);
  nh.spinOnce();
  delay(1);
}

void actuator_halt() {
  analogWrite(actuator_pwm, 0);
}

void motor_halt() {
  analogWrite(motor_pwm, 0);
}

void motor2_halt() {
  analogWrite(motor2_pwm, 0);
}

void actuator_move() {
  digitalWrite(actuator_inA, HIGH);
  digitalWrite(actuator_inB, LOW);
  analogWrite(actuator_pwm, 255);
}

void motor_up() {
  digitalWrite(motor_inA, HIGH);
  digitalWrite(motor_inB, LOW);
  analogWrite(motor_pwm, 255);
}

void motor_down() {
  digitalWrite(motor_inA, LOW);
  digitalWrite(motor_inB, HIGH);
  analogWrite(motor_pwm, 255);
}

void motor2_up() {
  digitalWrite(motor2_inA, HIGH);
  digitalWrite(motor2_inB, LOW);
  analogWrite(motor2_pwm, 255);
}

void motor2_down() {
  digitalWrite(motor2_inA, LOW);
  digitalWrite(motor2_inB, HIGH);
  analogWrite(motor2_pwm, 255);
}
