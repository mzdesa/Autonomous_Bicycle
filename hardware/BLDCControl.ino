#include <Servo.h>
Servo escSignal;
void setup() {
  // put your setup code here, to run once:
  escSignal.attach(12);
  escSignal.write(30); //command to arm the esc, otherwise it won't start up
  delay(3000); //esc init delay
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 40; i<130; i++){
    escSignal.write(i); //write between 40 and 130 according to website
    delay(15);
  }
}
