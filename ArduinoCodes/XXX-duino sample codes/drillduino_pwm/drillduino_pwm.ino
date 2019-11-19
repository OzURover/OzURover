int linearPWM = 3;
int linearINB = 4;
int linearINA = 2;

int dc1PWM = 10;
int dc1INB = 9;
int dc1INA = 8;

int dc2PWM = 5;
int dc2INB = 6;
int dc2INA = 7;

int timer = 30000;

void setup() {
  // put your setup code here, to run once:
  pinMode(linearPWM, OUTPUT);
  pinMode(linearINB, OUTPUT);
  pinMode(linearINA, OUTPUT);
  pinMode(dc1PWM, OUTPUT);
  pinMode(dc1INB, OUTPUT);
  pinMode(dc1INA, OUTPUT);
  pinMode(dc2PWM, OUTPUT);
  pinMode(dc2INB, OUTPUT);
  pinMode(dc2INA, OUTPUT);
}

void runMotor(int pwmPin, int inb, int ina){
  digitalWrite(inb, HIGH);
  digitalWrite(ina, LOW);
  for (int i=0; i < timer; i++){
    analogWrite(pwmPin,255);
  }

  digitalWrite(inb, LOW);
  digitalWrite(ina, HIGH);
  for (int i=0; i < timer; i++){
    analogWrite(pwmPin,255);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  runMotor(linearPWM, linearINB, linearINA);
  runMotor(dc1PWM, dc1INB, dc1INA);
  runMotor(dc2PWM, dc2INB, dc2INA);
}
