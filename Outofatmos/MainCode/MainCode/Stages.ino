
bool checkDeployment() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float accelMagnitude = sqrt(a.acceleration.x * a.acceleration.x +
                              a.acceleration.y * a.acceleration.y +
                              a.acceleration.z * a.acceleration.z);
  return accelMagnitude > 3.0; // Tune based on test
}

bool checkTiltDown() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  return abs(pitch) > 45; // Tune threshold
}

void deployCanSat() {
  servoLeft.write(0);
  servoRight.write(180);
  delay(1000);  // Ensure it's released
  servoLeft.write(90);
  servoRight.write(90);
}