int pin = 2;
unsigned long d1;
unsigned long d2;
unsigned long d3;
unsigned long d4;

void setup() {
  Serial.begin(9600);
  pinMode(pin, INPUT);
}

void loop() {
  d1 = pulseIn(pin, HIGH);
  d2 = pulseIn(pin, LOW);
  d3 = pulseIn(pin, HIGH);
  d4 = pulseIn(pin, LOW);
  Serial.println(d1 + d2 + d3 + d4);
  delay(10);
}
