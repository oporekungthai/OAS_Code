void steerFromPitch() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float pitchRaw = atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  float pitchFiltered = pitchKalmanFilter.updateEstimate(pitchRaw);
  pitchFiltered = constrain(pitchFiltered, -90, 90);

  steer(pitchFiltered);

  Serial.print("Raw Pitch: ");
  Serial.print(pitchRaw);
  Serial.print(" Filtered Pitch: ");
  Serial.println(pitchFiltered);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Raw Pitch: ");
  display.println(pitchRaw, 1); 
  display.print("Filtered: ");
  display.println(pitchFiltered, 1); 
  display.print("Servo Angle: ");
  display.println((int)(90 + pitchFiltered)); 
  display.display();
}