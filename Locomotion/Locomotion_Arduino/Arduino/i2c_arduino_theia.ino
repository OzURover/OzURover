#include <Wire.h>
#define SLAVE_ADD 0x08

void setup() {

    for(int i=22; i<=27; i++){
        pinMode(i, OUTPUT);
    }
    for(int i=48; i<=53; i++){
        pinMode(i, OUTPUT);
    }
	for(int i=2; i<=7; i++){
		pinMode(i,OUTPUT);
	}

    halt();  //it is for stopping rover

	Wire.begin(SLAVE_ADD); //for starting i2c connection
	Wire.onReceive(receiveEvent); //runs receiveEvent function when message comes

}

int number;
String total = "";
float new_number;
String x_y;

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

		if(x_y == "x"){
			if(new_number < 0.2 || new_number > -0.2){
		    	halt();
			}
			if(new_number != 0) {
		    	if(new_number > 0){
		        	enableRight();
		    	}
		    	if(new_number < 0){
		        	enableLeft();
		    	}
		    	move(255*abs(new_number));
			}
		}

		if(x_y == "y"){
			if(new_number < 0.2 || new_number > -0.2){
		    	halt();
			}
			if(new_number != 0) {
		    	if(new_number > 0){
		        	enableforward();
		    	}
		    	if(new_number < 0){
		        	enablereverse();
		    	}
		    	move(255*abs(new_number));
			}
		}

		total = "";
		new_number = 0;
	}

	else if(number == 103){
		x_y = "x";
	}

	else if(number == 104){
		x_y = "y";
	}

	else{
		total+=String(number);
	}
}

void move(int pwm){
     for(int i=2; i<=7; i++){
         analogWrite(i, pwm);
    }
}

void halt(){
    for(int k=2; k<=7; k++){
        analogWrite(k, 0);
    }
	for(int i=48; i<=53; i++){
        digitalWrite(i, LOW);
    }

    for(int j=22; j<=27; j++){
        digitalWrite(j, LOW);
    }
}

void enableforward(){
   digitalWrite(23,LOW);
   digitalWrite(25,LOW);
   digitalWrite(27,LOW);
   digitalWrite(53,LOW);
   digitalWrite(51,LOW);
   digitalWrite(49,LOW);
   
   digitalWrite(22,HIGH);
   digitalWrite(24,HIGH);
   digitalWrite(26,HIGH);
   digitalWrite(52,HIGH);
   digitalWrite(50,HIGH);
   digitalWrite(48,HIGH);
}

void enablereverse(){
   digitalWrite(23,HIGH);
   digitalWrite(25,HIGH);
   digitalWrite(27,HIGH);
   digitalWrite(53,HIGH);
   digitalWrite(51,HIGH);
   digitalWrite(49,HIGH);
   
   digitalWrite(22,LOW);
   digitalWrite(24,LOW);
   digitalWrite(26,LOW);
   digitalWrite(52,LOW);
   digitalWrite(50,LOW);
   digitalWrite(48,LOW);
}

void enableLeft(){
   digitalWrite(23,LOW);
   digitalWrite(25,LOW);
   digitalWrite(27,LOW);
   digitalWrite(53,HIGH);
   digitalWrite(51,HIGH);
   digitalWrite(49,HIGH);
   
   digitalWrite(22,HIGH);
   digitalWrite(24,HIGH);
   digitalWrite(26,HIGH);
   digitalWrite(52,LOW);
   digitalWrite(50,LOW);
   digitalWrite(48,LOW);
}

void enableRight(){           //52 - 53
   digitalWrite(23,HIGH);     //24 - 25
   digitalWrite(25,HIGH);     //problems
   digitalWrite(27,HIGH);
   digitalWrite(53,LOW);
   digitalWrite(51,LOW);
   digitalWrite(49,LOW);
   
   digitalWrite(22,LOW);
   digitalWrite(24,LOW);
   digitalWrite(26,LOW);
   digitalWrite(52,HIGH);
   digitalWrite(50,HIGH);
   digitalWrite(48,HIGH);
}

void loop() {
}
