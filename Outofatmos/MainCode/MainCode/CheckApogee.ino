bool checkApogeeAndTilt() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float bmpAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  // --- 1. TILT CHECK ---
  float rawPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float filteredPitch = kalmanPitch.updateEstimate(rawPitch);
  bool isTilted = abs(filteredPitch) > TILT_THRESHOLD_DEG;

  // --- 2. ACCEL DROP CHECK ---
  float accelMagnitude = sqrt(
    a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z
  );
  bool isAccelDrop = accelMagnitude < ACCEL_DROP_THRESHOLD;

  // --- 3. ALTITUDE DROP CHECK ---
  if (bmpAltitude > maxAltitude) {
    maxAltitude = bmpAltitude;  // Update peak
  }
  bool isAltitudeDrop = (maxAltitude - bmpAltitude) > ALTITUDE_DROP_MARGIN;

  // --- 4. TIME CHECK ---
  unsigned long timeSinceDeploy = millis() - deployTime;
  bool isTimePassed = timeSinceDeploy > MIN_TIME_SINCE_DEPLOY;

  // --- Combine enabled checks ---
  bool passed = true;

  if (CHECK_TILT_ANGLE)    passed &= isTilted;
  if (CHECK_ACCEL_DROP)    passed &= isAccelDrop;
  if (CHECK_ALTITUDE_DROP) passed &= isAltitudeDrop;
  if (CHECK_MIN_TIME)      passed &= isTimePassed;

  return passed;
}