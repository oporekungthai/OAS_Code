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
        if (gps.speed.isValid()) {
            gpsSpeedMps = gps.speed.knots() * 0.51444; // knots to m/s
        }
    }

    // === IMU readings ===
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float accelX = a.acceleration.x;
    float accelY = a.acceleration.y;
    float accelZ = a.acceleration.z;
    float accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

    float roll = atan2(accelY, accelZ) * 180.0 / PI;
    float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

    float filteredPitch = kalmanPitch.updateEstimate(pitch);
    float filteredRoll = kalmanRoll.updateEstimate(roll);

    float bmpAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    // === Calculate Bearing & Distance (if GPS is valid) ===
    float targetBearing = 0.0;
    float distanceToTarget = 0.0;
    if (gpsFix) {
        targetBearing = calculateBearing(latitude, longitude, targetLat, targetLon);
        distanceToTarget = calculateDistance(latitude, longitude, targetLat, targetLon);
    }

    // === Estimate vertical velocity from altitude change ===
    float verticalVelocity = 0.0;
    unsigned long currentTime = millis();
    float dt = (currentTime - lastAltitudeTime) / 1000.0; // ms to sec
    if (dt > 0.01) {
        verticalVelocity = (altitude - lastAltitude) / dt;
        lastAltitude = altitude;
        lastAltitudeTime = currentTime;
    }

    // === Build the data string ===
    String dataString = "";
    dataString += "Phase:" + currentPhase + ",";
    dataString += "Time:" + String(millis()) + ",";

    if (gpsFix) {
        dataString += "Lat:" + String(latitude, 6) + ",";
        dataString += "Lon:" + String(longitude, 6) + ",";
        dataString += "Alt:" + String(altitude, 2) + ",";
    } else {
        dataString += "Lat:N/A,Lon:N/A,Alt:N/A,";
    }

    dataString += "Az:" + String(azimuth) + ",";
    dataString += "Pitch:" + String(filteredPitch, 2) + ",";
    dataString += "Roll:" + String(filteredRoll, 2) + ",";
    dataString += "BmpAlt:" + String(bmpAltitude, 2) + ",";
    dataString += "Bearing:" + String(targetBearing, 2) + ",";
    dataString += "DistToTgt:" + String(distanceToTarget, 2) + ",";
    dataString += "Vel:" + String(verticalVelocity, 2) + ",";
    dataString += "GpsVel:" + String(gpsSpeedMps, 2) + ",";
    dataString += "AccelMag:" + String(accelMagnitude, 2);

    return dataString;
}
