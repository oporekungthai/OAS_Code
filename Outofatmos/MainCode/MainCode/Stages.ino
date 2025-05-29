
bool checkDeployment() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float accelMagnitude = sqrt(a.acceleration.x * a.acceleration.x +
                              a.acceleration.y * a.acceleration.y +
                              a.acceleration.z * a.acceleration.z);
  return accelMagnitude > RELEASE_ACCEL; // please work please work i swear to god please god please i love you
}

// bool checkTiltDown() {
//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);

//   // Calculate raw pitch angle in degrees
//   float rawPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
//   // Poop needs to be filtered by god
//   float filteredPitch = kalmanPitch.updateEstimate(rawPitch);
//   // Check if filtered pitch exceeds threshold
//   return abs(filteredPitch) > 45; // Tune threshold as needed
// }

void deployCanSat() {
  Deployer.write(60);
  delay(200);  // released?
}


