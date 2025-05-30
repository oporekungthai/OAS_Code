void steerToTarget() {
  static unsigned long lastPIDTime = 0;
  const unsigned long PID_INTERVAL = 1000;  // 1 second

  if (millis() - lastPIDTime >= PID_INTERVAL && gps.location.isValid()) {
    lastPIDTime = millis();  // Update timestamp

    float currentLat = gps.location.lat();
    float currentLon = gps.location.lng();
    float currentHeading = qmc.getAzimuth();

    float targetBearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
    int steeringOffset = calculateSteeringAngle(targetBearing, currentHeading) - 90;

    int leftServo = 90 + steeringOffset;
    int rightServo = 90 - steeringOffset;

    servoLeft.write(constrain(leftServo, 0, 180));
    servoRight.write(constrain(rightServo, 0, 180));
  }
}