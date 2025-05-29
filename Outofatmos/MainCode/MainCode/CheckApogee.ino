bool checkApogeeAndTilt() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float bmpAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  // Calculate pitch
  float rawPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float filteredPitch = kalmanPitch.updateEstimate(rawPitch);

  // Calculate roll
  float rawRoll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float filteredRoll = kalmanRoll.updateEstimate(rawRoll);

  // Check tilt on both axes
  bool isTilted = (abs(filteredPitch) > TILT_THRESHOLD_DEG) || (abs(filteredRoll) > TILT_THRESHOLD_DEG);

  // Rest of your existing checks...
  float accelMagnitude = sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z);
  bool isAccelDrop = accelMagnitude < ACCEL_DROP_THRESHOLD;

  if (bmpAltitude > maxAltitude) maxAltitude = bmpAltitude;
  bool isAltitudeDrop = (maxAltitude - bmpAltitude) > ALTITUDE_DROP_MARGIN;

  unsigned long timeSinceDeploy = millis() - deployTime;
  bool isTimePassed = timeSinceDeploy > MIN_TIME_SINCE_DEPLOY;

  bool passed = true;
  if (CHECK_TILT_ANGLE) passed &= isTilted;
  if (CHECK_ACCEL_DROP) passed &= isAccelDrop;
  if (CHECK_ALTITUDE_DROP) passed &= isAltitudeDrop;
  if (CHECK_MIN_TIME) passed &= isTimePassed;

  return passed;
}
