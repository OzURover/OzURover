#include <Wire.h>
#define SLAVE_ADD 0x08

void setup() {

Serial.begin(9600);
Wire.begin(SLAVE_ADD);
Wire.onReceive(receiveEvent);

}
int number;
String total = "";
float new_number;

void receiveEvent(int bytes){

number = Wire.read();
if(number == 100){
total = "-";
}

else if(number == 101){
total += "1";
}

else if(number == 102){
if((total!="1")&&(total!="-1")){
if(total.substring(0,1)=="-"){
  total = "-0." + total.substring(1,-1);
  }
else{
  total = "0."+total;
  }
  new_number = total.toFloat();
}
else{
  new_number = float(total.toInt());
  }
Serial.println(new_number);
total = "";
new_number = 0;
}

else{
total+=String(number);
}


}

void loop() {
}
