#include <Servo.h>
Servo escSignal;
#define escPin 9 //check this
int state = 3;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  escSignal.attach(escPin);
  escSignal.write(40); //command to arm the esc, otherwise it won't start up
  delay(3000); //esc init delay
}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.println("enter 1 or 2");
  while (Serial.available() == 0) {
  }
  
  int state = Serial.parseInt();
  switch (state) {
    case 1:
      escSignal.write(30);
      Serial.println("Turning off ESC");
      break;

    case 2:
      Serial.println("Turning on");
//      escSignal.write(30);
      for (int i = 70; i < 130; i++) {
        escSignal.write(i); //write between 40 and 130 according to website
        delay(15);
      }
      Serial.println("turned on");

      break;
  }


}
