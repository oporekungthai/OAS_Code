String createDataString() {
    // Read all sensor values first
    int azimuth = qmc.getAzimuth();
    float latitude = gps.location.isValid() ? gps.location.lat() : 0.0;
    float longitude = gps.location.isValid() ? gps.location.lng() : 0.0;
    float altitude = gps.location.isValid() ? gps.altitude.meters() : 0.0;
    bool gpsFix = gps.location.isValid();
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
    float filteredPitch = kalmanPitch.updateEstimate(pitch);
    float filteredRoll = kalmanRoll.updateEstimate(roll);
    float bmpAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    // Start building data string
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
    dataString += "BmpAlt:" + String(bmpAltitude, 2);

    return dataString;
}