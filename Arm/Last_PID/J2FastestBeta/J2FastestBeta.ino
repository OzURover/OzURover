/* 
  IMPORTANT! 
  This code uses an external library DIO2, it needs to be downloaded from: 
  http://www.codeproject.com/Articles/732646/Fast-digital-I-O-for-Arduino
  If the library is not installed properly, the code will not compile and/or 
  the readings coming from the encoder will not be correct.
*/


#include <PID_v1.h>
#include <arduino2.h>  
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <avr/wdt.h> 

#define c_EncoderPinA 2
#define c_EncoderPinB 4


volatile long _EncoderTicks = 0;
double angle1 = 1.5708; // angle of the motor wrt to the initial position


//PID Library Setup
//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
double PID_PWM; //Input that will be sent to the motor as PWM
double GoalPosition = 1.5708; //The angle to be reached

//Balcının PID: 6.65 0.723, 0.51   220 183 350
PID armPID (&angle1, &PID_PWM, &GoalPosition,3000,2500,45, DIRECT); //angle1 is encoder reading

ros::NodeHandle nh;

std_msgs::Float32 angle_msg;
std_msgs::Float32 u_outm;

ros::Publisher curr_pos_j2("curr_pos_j2", &angle_msg);
ros::Publisher u_out_j2("u_out_j2", &u_outm);

void messageCb(const std_msgs::Float32& goal_pos3){
  GoalPosition = goal_pos3.data;
}

void messageCb1(const std_msgs::Float32MultiArray& constants2){
  armPID.SetTunings(constants2.data[0], constants2.data[1], constants2.data[2]);
}


ros::Subscriber<std_msgs::Float32> sub("goal_pos3", &messageCb );
ros::Subscriber<std_msgs::Float32MultiArray> sub1("constants2", &messageCb1 );

void setup() {
  //Encoder Settings
  pinMode(c_EncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(c_EncoderPinA, LOW);  // turn on pullup resistors
  pinMode2(c_EncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(c_EncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(0, MotorInterruptA, RISING); //assignment of the interrupt pin
  attachInterrupt(0, MotorInterruptA, FALLING);
  //Motor settings
  pinMode(6, OUTPUT); 
  pinMode(7, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  
  halt(); //Halt the motor 
   
  //Enable PID
  armPID.SetOutputLimits(-175,175);
  armPID.SetMode(AUTOMATIC);
  
  nh.getHardware()->setBaud(76800);
  nh.initNode();
  nh.advertise(curr_pos_j2);
  nh.advertise(u_out_j2);
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
  angle1 = 1.5708 + (1.0/(500.0*230.0*3.0))*6.28*_EncoderTicks;

  armPID.Compute();
  

  if(abs(GoalPosition-angle1) <= 0.001){
    PID_PWM=0;
    GoalPosition = angle1;
  }

  
  if(PID_PWM > 0) CW(abs(PID_PWM));
  else if(PID_PWM < 0) CCW(abs(PID_PWM));
  else halt();



  u_outm.data = PID_PWM;
  angle_msg.data = angle1;
  curr_pos_j2.publish( &angle_msg );
  u_out_j2.publish( &u_outm );
  
  
  nh.spinOnce();
  delay(5);
}


//This method reads the changes in the encoder and updates _EncoderTicks, direction1 and angle1 count
void MotorInterruptA() { 
   if ( ((PIND >> 4) & 1) == 1) _EncoderTicks++;
  else _EncoderTicks--;
  // Coefficient -> [1/cpt]*3.14*3(kasnak)*[motor gear ratio]
  //cpt: counts per turn 500 for HEDL 5540 A02
}

//This method sets the motor to turn in clockwise direction
void CCW(double PID_PWM){
  digitalWrite(10, HIGH);
  digitalWrite(9, LOW);
  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);
  analogWrite(5, PID_PWM);
}

//This method sets the motor to turn in counter-clockwise direction
void CW(double PID_PWM){
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
