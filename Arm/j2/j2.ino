/* 
  IMPORTANT! 
  This code uses an external library DIO2, it needs to be downloaded from: 
  http://www.codeproject.com/Articles/732646/Fast-digital-I-O-for-Arduino
  If the library is not installed properly, the code will not compile and/or 
  the readings coming from the encoder will not be correct.
*/

#include <DIO2.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <avr/wdt.h>

#define GPIO2_PREFER_SPEED 1
#define c_EncoderPinA 2
#define c_EncoderPinB 4
#define EncoderIsReversed


#define gearRatio 230 

volatile bool _EncoderBSet;
volatile long _EncoderTicks = 0;
int direction1 = 1; // the direction that the motor is rotating to
double angle1 = 1.5708; // angle of the motor wrt to the initial position

//PID Library Setup
//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
double PID_PWM; //Input that will be sent to the motor as PWM
double GoalPosition = 1.5708; //The angle to be reached

//Balcının PID: 6.65 0.723, 0.51
PID armPID (&angle1, &PID_PWM, &GoalPosition,660.0,90.0,0.0, DIRECT); //angle1 is encoder reading

ros::NodeHandle nh;

std_msgs::Float32 angle_msg;
std_msgs::Float32 u_outm;

ros::Publisher curr_pos1("curr_pos_j2", &angle_msg);
ros::Publisher u_out1("u_out_j2", &u_outm);

void messageCb(const std_msgs::Float32& goal_pos3){
  GoalPosition = goal_pos3.data;
}

void messageCb1(const std_msgs::Float32MultiArray& constants){
  armPID.SetTunings(constants.data[0], constants.data[1], constants.data[2]);
}
   
ros::Subscriber<std_msgs::Float32> sub("goal_pos3", &messageCb );
ros::Subscriber<std_msgs::Float32MultiArray> sub1("constants", &messageCb1 );

void setup() {
  //Encoder Settings
  pinMode(c_EncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(c_EncoderPinA, LOW);  // turn on pullup resistors
  pinMode2(c_EncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(c_EncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(0, MotorInterruptA, RISING); //assignment of the interrupt pin
  
  //Motor settings
  pinMode(6, OUTPUT); 
  pinMode(7, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  
  halt(); //Halt the motor 
   
  //digitalWrite(13,HIGH);
  //delay(2500);
  //digitalWrite(13,LOW);
  
  //Enable PID
  armPID.SetOutputLimits(-125,125);
  armPID.SetMode(AUTOMATIC);
  
  nh.getHardware()->setBaud(76800);
  nh.initNode();
  nh.advertise(curr_pos1);
  nh.advertise(u_out1);
  nh.subscribe(sub);
  nh.subscribe(sub1);
  wdt_disable();
}

void loop() {
/*
  -----------------------------------------------
  THE CONTROL LOOP
  -----------------------------------------------
  
  By using the angle1, direction1, CW, CCW and halt a control loop will be implemented here. 
  The loop needs to take a goal position for the motor through serial input. 
*/
  armPID.Compute();
  
  //CCW(PID_PWM);
  if(abs(GoalPosition-angle1) <= 0.005){
    PID_PWM=0;  
  }
  
  if(PID_PWM > 0) CCW(abs(PID_PWM));
  else if(PID_PWM < 0) CW(abs(PID_PWM));
  else halt();
  
  u_outm.data = PID_PWM;
  angle_msg.data = angle1;
  curr_pos1.publish( &angle_msg );
  u_out1.publish( &u_outm );
  
  
  nh.spinOnce();
  delay(5);
}


//This method reads the changes in the encoder and updates _EncoderTicks, direction1 and angle1 count
void MotorInterruptA() {
  _EncoderBSet = digitalRead2(c_EncoderPinB);   
  if(digitalRead2(c_EncoderPinB) == HIGH) {
    _EncoderTicks--;
  } else if(digitalRead2(c_EncoderPinB) == LOW) {
    _EncoderTicks++;
  } 
  // Coefficient -> [1/cpt]*3.14*3(kasnak)*[motor gear ratio]
  //cpt: counts per turn 500 for HEDL 5540 A02
  angle1 = 1.5708 + (1.0/(500.0*230.0*3.0))*6.28*_EncoderTicks;
}

//This method sets the motor to turn in clockwise direction
void CW(double PID_PWM){
  digitalWrite(10, HIGH);
  digitalWrite(9, LOW);
  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);
  analogWrite(5, PID_PWM);
}

//This method sets the motor to turn in counter-clockwise direction
void CCW(double PID_PWM){
  digitalWrite(10, HIGH);
  digitalWrite(9, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, HIGH);
  analogWrite(5, PID_PWM);
}

//This method stops the motor
void halt(){
  digitalWrite(10, LOW);
  digitalWrite(9, HIGH);  //ınverted signal
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  analogWrite(5, 0);
}
