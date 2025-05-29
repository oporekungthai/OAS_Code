// --- PD Steering Control ---
// int calculateSteeringAngle(float targetBearing, float currentHeading) {
//   float error = targetBearing - currentHeading;
//   if (error > 180) error -= 360;
//   if (error < -180) error += 360;

//   float derivative = error - lastError;
//   lastError = error;

//   float control = Kp * error + Kd * derivative;

//   int angle = 90 + control;   
//   angle = constrain(angle, 45, 135); // Safe range
//   return angle;
// }


int calculateSteeringAngle(float targetBearing, float currentHeading) {
  float error = targetBearing - currentHeading;

  // Normalize error to [-180, 180]
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  float derivative = error - lastError;
  lastError = error;

  float control = Kp * error + Kd * derivative;

  // Map control output to servo range
  int angle = 90 + control;
  angle = constrain(angle, 0, 180); // Full servo range (0° to 180°)
  return angle;
}

void steerToTarget() {
  static unsigned long lastSteerTime = 0;
  if (millis() - lastSteerTime < STEER_INTERVAL) return;
  lastSteerTime = millis();

  if (gps.location.isValid()) {
    float currentLat = gps.location.lat();
    float currentLon = gps.location.lng();
    float currentHeading = qmc.getAzimuth();

    // Use TinyGPSPlus to calculate bearing
    float targetBearing = gps.courseTo(currentLat, currentLon, targetLat, targetLon);

    int servoAngle = calculateSteeringAngle(targetBearing, currentHeading);

    // Apply the same angle to both servos for symmetric control
    servoLeft.write(servoAngle);
    servoRight.write(servoAngle);
  }
}