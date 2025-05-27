void displaySensorData() {
  int azimuth = qmc.getAzimuth();
  double latitude = gps.location.isValid() ? gps.location.lat() : 0;
  double longitude = gps.location.isValid() ? gps.location.lng() : 0;

  // Calculate direction from last steering
  int angleDiff = (int)(lastTargetBearing - azimuth);
  if (angleDiff < -180) angleDiff += 360;
  if (angleDiff > 180) angleDiff -= 360;

  String direction;
  if (abs(angleDiff) < 10) direction = "STRAIGHT";
  else if (angleDiff > 0) direction = "RIGHT";
  else direction = "LEFT";

  // MPU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float filteredPitch = kalmanPitch.updateEstimate(pitch);
  float filteredRoll = kalmanRoll.updateEstimate(roll);
  float bmpAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.print(F("Lat:")); display.println(latitude, 4);
  // display.print(F("Lon:")); display.println(longitude, 4);
  display.print(F("TgtL:")); display.println(targetLat, 4);
  // display.print(F("TgtN:")); display.println(targetLon, 4);
  // display.print(F("Azm:")); display.println(azimuth);
  display.print(F("ToT:")); display.println((int)lastTargetBearing);
  display.print(F("Dir:")); display.println(direction);

  display.display();
}