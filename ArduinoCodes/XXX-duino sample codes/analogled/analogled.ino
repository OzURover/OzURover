void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(A0, 250);
  delay(500);
  analogWrite(A0, 5);
  delay(500);
}
