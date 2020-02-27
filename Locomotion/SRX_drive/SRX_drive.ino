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
#include <std_msgs/Float32.h>

#define LEFT 7
#define RIGHT 8

#define FL 48
#define FH 74
#define RL 46
#define RH 20

#define LOWB 0.0
#define HIGHB 1.0

double goal;

void msgCb(const std_msgs::Flaot32& data) {
  goal = data.data;
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float32> sub("loco", &msgCb);

void setup()
{
	pinMode(LEFT, OUTPUT);
	pinMode(RIGHT, OUTPUT);
	TCCR4B = TCCR4B & B11111000 | B00000100;

  nh.getHardware()->setBaud(115200);
  nh.initNode();

	// Wait for init and tell SRX's that we are sending PWM
	delay(1500);
	analogWrite(LEFT, 10);
	analogWrite(RIGHT, 10);
	delay(500);
	analogWrite(LEFT, 0);
	analogWrite(RIGHT, 0);
}

void loop()
{
  if (goal < 10) {
    if (goal > 0) {
      forward(goal);
    } else {
      reverse(abs(goal));
    }
  } else {
    if (goal-20 < 0) {
      right(abs(goal-20));
    } else {
      left(goal-20);
    }
  }
  nh.spinOnce();
  /*
	if (1)
	{
		// Demo1: Show each movement
		for (int i = 0; i < 255; i++)
		{
			forward(i);
			delay(20);
		}
		for (int i = 255; i > 0; i--)
		{
			forward(i);
			delay(20);
		}
		delay(3000);
		for (int i = 0; i < 255; i++)
		{
			reverse(i);
			delay(20);
		}
		for (int i = 255; i > 0; i--)
		{
			reverse(i);
			delay(20);
		}
		delay(3000);
		for (int i = 0; i < 255; i++)
		{
			left(i);
			delay(20);
		}
		for (int i = 255; i > 0; i--)
		{
			left(i);
			delay(20);
		}
		delay(3000);
		for (int i = 0; i < 255; i++)
		{
			right(i);
			delay(20);
		}
		for (int i = 255; i > 0; i--)
		{
			right(i);
			delay(20);
		}
		delay(3000);
	}
	else
	{
		//Demo2: gradually increase and lower forward speed
		int l = 48;
		int r = 46;
		for (; l < 74 && r > 20;)
		{
			analogWrite(RIGHT, r--); //35-70
			analogWrite(LEFT, l++);  //35-70
			delay(300);
		}
		for (; l > 48 && r < 46;)
		{
			analogWrite(RIGHT, r++); //35-70
			analogWrite(LEFT, l--);  //35-70
			delay(300);
		}
	}*/
}

void forward(float pwr)
{
	analogWrite(LEFT, int(map(pwr, LOWB, HIGHB, FL, FH)));
	analogWrite(RIGHT, int(map(pwr, LOWB, HIGHB, RL, RH)));
}

void reverse(float pwr)
{
	analogWrite(LEFT, int(map(pwr, LOWB, HIGHB, RL, RH)));
	analogWrite(RIGHT, int(map(pwr, LOWB, HIGHB, FL, FH)));
}

void right(float pwr)
{
	analogWrite(LEFT, int(map(pwr, LOWB, HIGHB, FL, FH)));
	analogWrite(RIGHT, int(map(pwr, LOWB, HIGHB, FL, FH)));
}

void left(float pwr)
{
	analogWrite(LEFT, int(map(pwr, LOWB, HIGHB, RL, RH)));
	analogWrite(RIGHT, int(map(pwr, LOWB, HIGHB, RL, RH)));
}
