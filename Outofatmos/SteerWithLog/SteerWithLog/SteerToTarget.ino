void steerToTarget() {
  if (gps.location.isValid()) {
    float currentLat = gps.location.lat();
    float currentLon = gps.location.lng();
    float currentHeading = qmc.getAzimuth();

    lastTargetBearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
    lastServoAngle = calculateSteeringAngle(lastTargetBearing, currentHeading);

    // Apply same angle to both since they're side-swapped
    servoLeft.write(lastServoAngle);
    servoRight.write(lastServoAngle);
  }
}