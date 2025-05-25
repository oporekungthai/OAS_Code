
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

  // Calculate raw pitch angle in degrees
  float rawPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  // Filter the pitch angle with Kalman filter
  float filteredPitch = kalmanPitch.updateEstimate(rawPitch);
  // Check if filtered pitch exceeds threshold
  return abs(filteredPitch) > 45; // Tune threshold as needed
}

void deployCanSat() {
  servoLeft.write(0);
  servoRight.write(180);
  delay(1000);  // Ensure it's released
  servoLeft.write(90);
  servoRight.write(90);
}