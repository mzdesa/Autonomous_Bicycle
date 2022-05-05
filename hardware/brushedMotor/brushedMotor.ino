int throttlePin = 8;
int state = 4;
int max = 200;
void setup() {
  // put your setup code here, to run once:
//  pinmode(throttlePin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:(CODE FOR THE BRUSHED DC)
  //do a repeating throttle loop
  Serial.println("enter 1 (forward) or 2 (stop)");
  while (Serial.available() == 0) {
  }
  state = Serial.parseInt();
  switch (state) {
    case 1:
      Serial.println("Increasing Throttle");
      for (int i = 1; i <= max; i++) {
        analogWrite(throttlePin, i); //increase the throttle
        delay(10);
      }
      
      break;
    case 2:
      Serial.println("Stopping Throttle");
      for (int i = max; i > 0; i--) {
        analogWrite(throttlePin, i);
        delay(10);
      }
      break;

  }

}
