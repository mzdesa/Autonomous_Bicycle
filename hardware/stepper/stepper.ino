int stepPin = 9; //define pins
int dirPin = 8;
void setup() {
  // put your setup code here, to run once:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //write the diretion pin high and the step pin high once a loop
  digitalWrite(dirPin, 1); //write direction pin high: 1 is forward, 0 is backwards
  digitalWrite(stepPin, 1);
  delay(3); //have a 15ms delay
  digitalWrite(stepPin, 0);
  delay(3);
}
