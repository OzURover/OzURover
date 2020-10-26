/**
 	*        /-----RIGHT-----\
 	*        |               |
 	*  FRONT |               | BACK
 	*        |   PDB    BATT |
 	*        \------LEFT-----/
 	*/

/**
	* reverse_low 46
	* reverse_high 20
	* 
	* forward_low 48
	* forward_high 74
	*/

#include <ros.h>
#include <geometry_msgs/Twist.h>

#define LEFT 7
#define RIGHT 8

#define FL 47
#define FH 74
#define RL 46
#define RH 20

#define LOWB 0.0
#define HIGHB 500.0

#define WHEEL_D_H 30.0
#define WHEEL_D_V 45.0
#define WHEEL_D_X sqrt(pow(WHEEL_D_H, 2) + pow(WHEEL_D_V, 2))
#define SEPERATION_ANGLE 1.5708 - atan(WHEEL_D_H / WHEEL_D_V)

#define GAIN_A 0.15

#define ROS

float goal[2] = {0, 0};

#ifdef ROS
void msgCb(const geometry_msgs::Twist &data)
{
	move(data.linear.x, data.linear.z);
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &msgCb);
#endif

void move(float Lraw, float Araw)
{
	float linear = Lraw;
	float angular = (Araw * WHEEL_D_X / cos(SEPERATION_ANGLE)) * GAIN_A;

	goal[0] = linear + angular;
	goal[1] = linear - angular;
}

void setup()
{
	pinMode(LEFT, OUTPUT);
	pinMode(RIGHT, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	TCCR4B = TCCR4B & B11111000 | B00000100;

#ifdef ROS
	nh.getHardware()->setBaud(115200);
	nh.initNode();
	nh.subscribe(sub);
#else
  Serial.begin(9600);
#endif

	// Wait for init and tell SRX's that we are sending PWM
	delay(1500);
	analogWrite(LEFT, 10);
	analogWrite(RIGHT, 10);
	delay(500);
	analogWrite(LEFT, 0);
	analogWrite(RIGHT, 0);
	digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
	if (goal[0] < 0)
	{
		analogWrite(LEFT, int(map(-goal[0], LOWB, HIGHB, RL, RH)));
	}
	else
	{
		analogWrite(LEFT, int(map(goal[0], LOWB, HIGHB, FL, FH)));
	}
	if (goal[1] < 0)
	{
		analogWrite(RIGHT, int(map(-goal[1], LOWB, HIGHB, FL, FH)));
	}
	else
	{
		analogWrite(RIGHT, int(map(goal[1], LOWB, HIGHB, RL, RH)));
	}
#ifdef ROS
	nh.spinOnce();
#else
	move(150.0, 1.54);
#endif
}
