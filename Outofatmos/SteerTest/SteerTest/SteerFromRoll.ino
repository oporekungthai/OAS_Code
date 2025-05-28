void steerFromRoll() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Raw
  float rollRaw = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;

  // Filtered
  float rollFiltered = rollKalmanFilter.updateEstimate(rollRaw);
  rollFiltered = constrain(rollFiltered, -90, 90);

  // steer
  steer(rollFiltered);

  // Show on OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Raw Roll: ");
  display.println(rollRaw, 1);
  display.print("Filtered: ");
  display.println(rollFiltered, 1);
  display.print("Servo Angle: ");
  display.println((int)(90 + rollFiltered));  // Assuming center = 90
  display.display();
}