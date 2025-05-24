void steer(int angle) {
  angle = constrain(angle, -90, 90);
  int center = 90;

  int angle1 = center + angle;
  int angle2 = center + angle;

  servo1.write(angle1);
  servo2.write(angle2);
}
