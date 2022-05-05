int throttlePin = 8;
void setup() {
  // put your setup code here, to run once:
  pinMode(throttlePin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:(CODE FOR THE BRUSHED DC)
  //do a repeating throttle loop
  int max = 200;
  for (int i = 1; i<=max; i++){
    analogWrite(throttlePin, i); //increase the throttle
    delay(10);
  }
  for (int i = max; i>0; i--){
    analogWrite(throttlePin, i);
    delay(10);
  }
}
