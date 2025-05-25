String createDataString() {
    // Read all the sensor values *before* creating the string.
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

    // Format the data string
    String dataString = "";
     if (gpsFix) {  //Only send GPS data if we have a fix
       dataString += String(latitude, 6) + ",";
       dataString += String(longitude, 6) + ",";
       dataString += String(altitude) + ",";
    } else {
        dataString += "N/A,N/A,N/A,";  // Indicate no GPS fix
    }
    dataString += String(azimuth) + ",";
    dataString += String(filteredPitch) + ",";
    dataString += String(filteredRoll) + ",";
    dataString += String(bmpAltitude);

    return dataString;
}