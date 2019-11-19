#include <arduino2.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <avr/wdt.h>

#define c_EncoderPinA 2
#define c_EncoderPinB 4


volatile long _EncoderTicks = 0;
double angle1 = 0.0; // angle of the motor wrt to the initial position

double PID_PWM; //Input that will be sent to the motor as PWM
double GoalPosition = 0.0; //The angle to be reached
//200 135 20 
PID armPID (&angle1, &PID_PWM, &GoalPosition,320.0,19.5,3.0, DIRECT); //angle1 is encoder reading

ros::NodeHandle nh;

std_msgs::Float32 angle_msg;
std_msgs::Float32 u_outm;

ros::Publisher curr_pos_j0("curr_pos_j0", &angle_msg);
ros::Publisher u_out_j0("u_out_j0", &u_outm);

void messageCb(const std_msgs::Float32& goal_pos1){
  GoalPosition = goal_pos1.data;
}

void messageCb1(const std_msgs::Float32MultiArray& constants0){
  armPID.SetTunings(constants0.data[0], constants0.data[1], constants0.data[2]);
}
   
ros::Subscriber<std_msgs::Float32> sub("goal_pos1", &messageCb );
ros::Subscriber<std_msgs::Float32MultiArray> sub1("constants0", &messageCb1 );

void setup() {
  pinMode(c_EncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(c_EncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_EncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(c_EncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(0, MotorInterruptA, RISING); //assignment of the interrupt pin
  attachInterrupt(0, MotorInterruptA, FALLING);
  
  pinMode(6, OUTPUT); 
  pinMode(7, OUTPUT);
  pinMode(5, OUTPUT);

  halt();

  armPID.SetOutputLimits(-125,125);
  armPID.SetMode(AUTOMATIC);

  nh.getHardware()->setBaud(76800);
  nh.initNode();
  nh.advertise(curr_pos_j0);
  nh.advertise(u_out_j0);
  nh.subscribe(sub);
  nh.subscribe(sub1);
  
  wdt_disable();
}

void loop() {
  armPID.Compute();
 
  //CCW(PID_PWM);
  if(abs(GoalPosition-angle1) <= 0.005){
    PID_PWM=0;
  }
  
  if(PID_PWM > 0) CW(abs(PID_PWM));
  else if(PID_PWM < 0) CCW(abs(PID_PWM));
  else halt();
  
  u_outm.data = PID_PWM;
  angle_msg.data = angle1;
  curr_pos_j0.publish( &angle_msg );
  u_out_j0.publish( &u_outm );
  
  nh.spinOnce();
  delay(5);
}

void MotorInterruptA() {  
   if ( ((PIND >> 4) & 1) == 1) _EncoderTicks++;
  else _EncoderTicks--;
  // Coefficient -> [1/cpr]*3.14*[motor gear ratio]
  // cpr: 28 ppr: 7 ,,, 546 reduction93.14 500ppr
  angle1 = 0.0 + _EncoderTicks/42176.0;
  //angle1 = 0.00059655045/20.0*546.0*_EncoderTicks;
}

//This method sets the motor to turn in clockwise direction
void CW(double PID_PWM){
  digitalWrite(7, HIGH);
  digitalWrite(6, LOW);
  analogWrite(5, PID_PWM);
}

//This method sets the motor to turn in counter-clockwise direction
void CCW(double PID_PWM){
  digitalWrite(7, LOW);
  digitalWrite(6, HIGH);
  analogWrite(5, PID_PWM);
}

//This method stops the motor
void halt(){
  digitalWrite(7, LOW);
  digitalWrite(6, LOW);
  analogWrite(5, 0);
}
