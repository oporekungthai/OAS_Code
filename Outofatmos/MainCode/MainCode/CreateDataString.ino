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

String createDataString() {
  // --- Sensor Readings ---
  int azimuth = qmc.getAzimuth();

  // === Cache GPS values ===
  float latitude = 0.0, longitude = 0.0, altitude = 0.0;
  float gpsSpeedMps = 0.0;
  bool gpsFix = gps.location.isValid();

  // Get UTC time from GPS
  int hour = 0, minute = 0, second = 0;
  if (gps.time.isValid()) {
    hour = gps.time.hour();
    minute = gps.time.minute();
    second = gps.time.second();

    // Convert UTC to GMT+7
    hour = (hour + 7) % 24;
  }

  if (gpsFix) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    altitude = gps.altitude.meters();
    gpsSpeedMps = gps.speed.mps();
  }

  // === Get current time in millis ===
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

  // === Read acceleration ===
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;
  float accelTotal = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  // === Format time string HH:MM:SS ===
  char timeString[9];
  snprintf(timeString, sizeof(timeString), "%02d:%02d:%02d", hour, minute, second);

  // === Create CSV-formatted string ===
  String dataString = "";
  dataString += String(timeString);           dataString += ",";
  dataString += String(latitude, 6);          dataString += ",";
  dataString += String(longitude, 6);         dataString += ",";
  dataString += String(altitude, 2);          dataString += ",";
  dataString += String(azimuth);               dataString += ",";
  dataString += String(distanceToTarget, 2);  dataString += ",";
  dataString += String(gpsSpeedMps, 2);       dataString += ",";
  dataString += String(pressure, 2);           dataString += ",";
  dataString += String(temperature, 2);        dataString += ",";
  dataString += String(verticalVelocity, 2);   dataString += ",";
  dataString += String(accelX, 2);             dataString += ",";
  dataString += String(accelY, 2);             dataString += ",";
  dataString += String(accelZ, 2);             dataString += ",";
  dataString += String(accelTotal, 2);

  return dataString;
}