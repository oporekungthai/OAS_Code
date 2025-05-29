String createDataString() {
  // === Compass Azimuth ===
  int azimuth = qmc.getAzimuth();

  // === GPS ===
  bool gpsFix = gps.location.isValid();
  float latitude = gpsFix ? gps.location.lat() : 0.0;
  float longitude = gpsFix ? gps.location.lng() : 0.0;
  float gpsAltitude = gpsFix ? gps.altitude.meters() : 0.0;

  // === Target Bearing and Distance ===
  float bearingToTarget = 0.0;
  float distanceToTarget = 0.0;
  if (gpsFix && targetSelected) {
    bearingToTarget = TinyGPSPlus::courseTo(latitude, longitude, targetLat, targetLon);
    distanceToTarget = TinyGPSPlus::distanceBetween(latitude, longitude, targetLat, targetLon);
  }

  // === IMU Data ===
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float rawRoll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float rawPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float filteredPitch = kalmanPitch.updateEstimate(rawPitch);
  float filteredRoll = kalmanRoll.updateEstimate(rawRoll);

  // === BMP Altitude ===
  float bmpAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  // === Compose String ===
  String dataString = "";

  if (gpsFix) {
    dataString += String(latitude, 6) + ",";
    dataString += String(longitude, 6) + ",";
    dataString += String(gpsAltitude, 2) + ",";
    dataString += String(distanceToTarget, 2) + ",";
    dataString += String(bearingToTarget, 2) + ",";
  } else {
    dataString += "N/A,N/A,N/A,N/A,N/A,";
  }

  dataString += String(azimuth) + ",";
  dataString += String(filteredPitch, 2) + ",";
  dataString += String(filteredRoll, 2) + ",";
  dataString += String(bmpAltitude, 2);

  return dataString;
}
