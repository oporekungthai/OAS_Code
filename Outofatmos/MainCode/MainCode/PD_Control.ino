// --- PD Steering Control ---
int calculateSteeringAngle(float targetBearing, float currentHeading) {
  float error = targetBearing - currentHeading;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  float derivative = error - lastError;
  lastError = error;

  float control = Kp * error + Kd * derivative;

  int angle = 90 + control;   
  angle = constrain(angle, 45, 135); // Safe range
  return angle;
}