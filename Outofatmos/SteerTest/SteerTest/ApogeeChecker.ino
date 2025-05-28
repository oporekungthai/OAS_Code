bool checkApogeeAndTilt() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float bmpAltitude = bmp.readAltitude(1013.25);

  if (isnan(bmpAltitude)) {
    Serial.println("ERROR: BMP altitude read failed.");
    return false;
  }

  float accelMagnitude = sqrt(
    a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z
  );

  if (!hasLaunched && accelMagnitude > LAUNCH_ACCEL_THRESHOLD) {
    hasLaunched = true;
    deployTime = millis();
    maxAltitude = bmpAltitude;
    Serial.println("LAUNCH DETECTED!");
  }

  if (!hasLaunched) {
    static unsigned long lastDisplay = 0;
    if (millis() - lastDisplay > 300) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("WAITING LAUNCH...");
      display.print("\nAccel: ");
      display.print(accelMagnitude, 1);
      display.display();
      lastDisplay = millis();
    }
    return false;
  }

  float rawPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float filteredPitch = pitchKalmanFilter.updateEstimate(rawPitch);
  bool isTilted = abs(filteredPitch) > TILT_THRESHOLD_DEG;
  bool isAccelDrop = accelMagnitude < ACCEL_DROP_THRESHOLD;
  if (bmpAltitude > maxAltitude) maxAltitude = bmpAltitude;
  bool isAltitudeDrop = (maxAltitude - bmpAltitude) > ALTITUDE_DROP_MARGIN;

  unsigned long timeSinceDeploy = millis() - deployTime;
  bool isTimePassed = timeSinceDeploy > MIN_TIME_SINCE_DEPLOY;

  bool passed = true;
  if (ENABLE_TILT_CHECK)     passed &= isTilted;
  if (ENABLE_ACCEL_DROP)     passed &= isAccelDrop;
  if (ENABLE_ALTITUDE_DROP)  passed &= isAltitudeDrop;
  if (ENABLE_TIME_CHECK)     passed &= isTimePassed;

  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 300) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Launched: "); display.println(hasLaunched ? "YES" : "NO");
    display.print("Pitch: "); display.println(filteredPitch, 1);
    display.print("Accel: "); display.println(accelMagnitude, 1);
    display.print("Δt: "); display.println(timeSinceDeploy / 1000.0, 1);
    display.print("PASS: "); display.println(passed ? "YES" : "NO");
    display.display();
    lastDisplay = millis();
  }

  return passed;
}