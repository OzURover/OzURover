#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <HX711_ADC.h>

const int actuator_inA = 3;
const int actuator_inB = 4;
const int actuator_pwm = 2;

const int motor_inA = 8;
const int motor_inB = 9;
const int motor_pwm = 10;

const int loadcell_1_dout = 5;
const int loadcell_1_sck = 7;
const int loadcell_2_dout = 6;
const int loadcell_2_sck = 11;

int timer = 30000;
//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(5, 7); //this returns negative results AND the one that points downwards should be connected
HX711_ADC LoadCell_2(6, 11); //this returns positive AND the one pointing upward should be connected
double loadCell_1_CAL = 244;
double loadCell_2_CAL = 293;

long t;

ros::NodeHandle nh;

std_msgs::Float32 load_msg1;
std_msgs::Float32 load_msg2;

ros::Publisher load("drill_loadcell1", &load_msg1);
ros::Publisher load2("drill_loadcell2", &load_msg2);

void actuatorCb(const std_msgs::Int32& data) {
  if(data.data == 1) {
    actuator_up();
  } else if (data.data == -1) {
    actuator_down();
  } else {
    actuator_halt();
  }
}

void motorCb(const std_msgs::Int32& data) {
  if(data.data == 1) {
    motor_up();
  } else if (data.data == -1) {
    motor_down();
  } else {
    motor_halt();
  }
}

ros::Subscriber<std_msgs::Int32> sub("drillActuator", &actuatorCb);
ros::Subscriber<std_msgs::Int32> sub1("drillMotor", &motorCb);

void setup() {
  pinMode(actuator_inA, OUTPUT);
  pinMode(actuator_inB, OUTPUT);
  pinMode(actuator_pwm, OUTPUT);
  pinMode(motor_inB, OUTPUT);
  pinMode(motor_inB, OUTPUT);
  pinMode(motor_inB, OUTPUT);

  actuator_halt();
  motor_halt();

  LoadCell_1.begin();
  LoadCell_2.begin();
  long stabilisingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilising time
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;
  while ((loadcell_1_rdy + loadcell_2_rdy) < 2) { //run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilisingtime);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilisingtime);
  }
  LoadCell_1.setCalFactor(loadCell_1_CAL); // user set calibration factor (float)
  LoadCell_2.setCalFactor(loadCell_2_CAL); // user set calibration factor (float)
  
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub1);
}

void loop() {
  //update() should be called at least as often as HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS
  //longer delay in scetch will reduce effective sample rate (be carefull with delay() in loop)
  LoadCell_1.update();
  LoadCell_2.update();
  
  //get smoothed value from data set + current calibration factor
  if (millis() > t + 250) {
    float a = LoadCell_1.getData();
    float b = LoadCell_2.getData();
    load_msg1.data = a;
    load_msg2.data = b;
    load.publish(&load_msg1);
    load2.publish(&load_msg2);
    t = millis();
  }
  
  nh.spinOnce();
  delay(1);
}

void actuator_halt() {
  analogWrite(actuator_pwm, 0);
}

void motor_halt() {
  analogWrite(motor_pwm, 0);
}

void actuator_up() {
  digitalWrite(actuator_inA, HIGH);
  digitalWrite(actuator_inB, LOW);
  analogWrite(actuator_pwm, 125);
}

void actuator_down() {
  digitalWrite(actuator_inA, LOW);
  digitalWrite(actuator_inB, HIGH);
  analogWrite(actuator_pwm, 125);
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

