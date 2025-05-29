// === Globals ===
float lastAltitude = 0.0;
unsigned long lastAltitudeTime = 0;

// === Distance Calculation Function ===
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // Earth radius in meters
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

// === Main Data Creation Function ===
String createDataString() {
  // --- Sensor Readings ---
  int azimuth = qmc.getAzimuth();

  // === Cache GPS values ===
  float latitude = 0.0, longitude = 0.0, altitude = 0.0;
  float gpsSpeedMps = 0.0;
  bool gpsFix = gps.location.isValid();

  if (gpsFix) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    altitude = gps.altitude.meters();
    gpsSpeedMps = gps.speed.mps();
  }

  // === Get current time ===
  unsigned long currentTime = millis();

  // === Calculate distance to target ===
  float distanceToTarget = 0.0;
  if (gpsFix) {
    distanceToTarget = calculateDistance(latitude, longitude, targetLat, targetLon);
  }

  // === Get pressure and temperature ===
  float pressure = bmp.readPressure();
  float temperature = bmp.readTemperature();

  // === Calculate vertical velocity (altitude rate) ===
  float verticalVelocity = 0.0;
  float dt = (currentTime - lastAltitudeTime) / 1000.0;
  if (dt > 0.1) {
    verticalVelocity = (altitude - lastAltitude) / dt;
    lastAltitude = altitude;
    lastAltitudeTime = currentTime;
  }

  // === Create CSV-formatted string ===
  String dataString = "";
  dataString += String(currentTime);          dataString += ",";
  dataString += String(latitude, 6);          dataString += ",";
  dataString += String(longitude, 6);         dataString += ",";
  dataString += String(altitude, 2);          dataString += ",";
  dataString += String(azimuth);              dataString += ",";
  dataString += String(distanceToTarget, 2);  dataString += ",";
  dataString += String(gpsSpeedMps, 2);       dataString += ",";
  dataString += String(pressure, 2);          dataString += ",";
  dataString += String(temperature, 2);       dataString += ",";
  dataString += String(verticalVelocity, 2);

  return dataString;
}
