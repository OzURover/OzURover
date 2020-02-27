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

#define LEFT 7
#define RIGHT 8

#define FL 48
#define FH 74
#define RL 46
#define RH 20

#define LOWB 0
#define HIGHB 255

void setup()
{
	pinMode(LEFT, OUTPUT);
	pinMode(RIGHT, OUTPUT);
	TCCR4B = TCCR4B & B11111000 | B00000100;

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
	if (1) {
		// Demo1: Show each movement
		forward(255);
		delay(2000);
		reverse(255);
		delay(2000);
		left(255);
		delay(2000);
		right(255);
		delay(2000);
	} else {
		//Demo2: gradually increase and lower forward speed
		int l = 48;
		int r = 46;
		for (; l < 74 && r > 20;)
		{
			analogWrite(RIGHT, r--); //35-70
			analogWrite(LEFT, l++); //35-70
			delay(300);
		}
		for (; l > 48 && r < 46;)
		{
			analogWrite(RIGHT, r++); //35-70
			analogWrite(LEFT, l--); //35-70
			delay(300);
		}
	}
}

void forward(float pwr) {
	analogWrite(LEFT, int(map(pwr, LOWB, HIGHB, FL, FH)));
	analogWrite(RIGHT, int(map(pwr, LOWB, HIGHB, RL, RH)));
}

void reverse(float pwr) {
	analogWrite(LEFT, int(map(pwr, LOWB, HIGHB, RL, RH)));
	analogWrite(RIGHT, int(map(pwr, LOWB, HIGHB, FL, FH)));
}

void left(float pwr) {
	analogWrite(LEFT, int(map(pwr, LOWB, HIGHB, FL, FH)));
	analogWrite(RIGHT, int(map(pwr, LOWB, HIGHB, FL, FH)));
}

void right(float pwr) {
	analogWrite(LEFT, int(map(pwr, LOWB, HIGHB, RL, RH)));
	analogWrite(RIGHT, int(map(pwr, LOWB, HIGHB, RL, RH)));
}