
const int AOUTpin=A0;//the AOUT pin of the CO sensor goes into analog pin A0 of the arduino
const int DOUTpin=3;//the DOUT pin of the CO sensor goes into digital pin D8 of the arduino

int value;

void setup() {
Serial.begin(9600);//sets the baud rate
pinMode(DOUTpin, INPUT);//sets the pin as an input to the arduino
}

void loop()
{
value= analogRead(AOUTpin);//reads the analaog value from the CO sensor's AOUT pin
Serial.print("CO value: ");
Serial.println(value);//prints the CO value
delay(100);
}
