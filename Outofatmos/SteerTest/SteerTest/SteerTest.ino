#include <Servo.h>

Servo servo1;
Servo servo2; 

void setup() {
  servo1.attach(11); 
  servo2.attach(12); 

  servo1.write(90);
  servo2.write(90);
}




void loop() {
  steer(0);
  delay(3000);
  steer(180);
  delay(3000);


}

void steer(int angle) {

  angle = constrain(angle, -90, 90);
  int center = 90;

  // Mirror angle for second servo
  int angle1 = center + angle;
  int angle2 = center + angle;

  servo1.write(angle1);
  servo2.write(angle2);
}