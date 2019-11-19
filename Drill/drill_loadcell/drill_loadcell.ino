/*WARNING
HX711_ADC library HAS TO BE INSTALLED.*/
#include <HX711_ADC.h>

int linearPWM = 3;
int linearINB = 4;
int linearINA = 2;


int dc1PWM = 10;
int dc1INB = 9;
int dc1INA = 8;

int loadcell1dout = 5;
int loadcell1sck = 7;
int loadcell2dout = 6;
int loadcell2sck = 11;


int timer = 30000;
//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(5, 7); //this returns negative results AND the one that points downwards should be connected
HX711_ADC LoadCell_2(6, 11); //this returns positive AND the one pointing upward should be connected
double loadCell_1_CAL = 244;
double loadCell_2_CAL = 293;

long t;

void setup() {
  // put your setup code here, to run once:
  pinMode(linearPWM, OUTPUT);
  pinMode(linearINB, OUTPUT);
  pinMode(linearINA, OUTPUT);
  pinMode(dc1PWM, OUTPUT);
  pinMode(dc1INB, OUTPUT);
  pinMode(dc1INA, OUTPUT);
  Serial.begin(115200);
  Serial.println("Wait...");
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
  Serial.println("Startup + tare is complete");
}

void runMotor(int pwmPin, int inb, int ina){
  digitalWrite(inb, HIGH);
  digitalWrite(ina, LOW);
  //for (int i=0; i < timer; i++){
    analogWrite(pwmPin,200);
  //}

  /*digitalWrite(inb, LOW);
  digitalWrite(ina, HIGH);
  for (int i=0; i < timer; i++){
    analogWrite(pwmPin,255);
  }*/
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
    Serial.print("Load_cell 1 output val: ");
    Serial.print(a);
    Serial.print("    Load_cell 2 output val: ");
    Serial.println(b);
    t = millis();
  }

  /*//receive from serial terminal
  if (Serial.available() > 0) {
    float i;
    char inByte = Serial.read();
    if (inByte == 't') {
      LoadCell_1.tareNoDelay();
      LoadCell_2.tareNoDelay();
    }
  }

  //check if last tare operation is complete
  if (LoadCell_1.getTareStatus() == true) {
    Serial.println("Tare load cell 1 complete");
  }
  if (LoadCell_2.getTareStatus() == true) {
    Serial.println("Tare load cell 2 complete");
  }*/
  
  runMotor(linearPWM, linearINB, linearINA);
  runMotor(dc1PWM, dc1INB, dc1INA);
}
