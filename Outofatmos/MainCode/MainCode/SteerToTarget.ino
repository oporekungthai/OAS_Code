void steerToTarget() {
  if (gps.location.isValid()) {
    float currentLat = gps.location.lat();
    float currentLon = gps.location.lng();
    float currentHeading = qmc.getAzimuth();

    float targetBearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
    int servoAngle = calculateSteeringAngle(targetBearing, currentHeading);

    // Apply same angle to both since they're side-swapped
    servoLeft.write(servoAngle);
    servoRight.write(servoAngle);
  }
}