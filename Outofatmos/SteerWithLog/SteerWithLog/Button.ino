void checkButtonAndSetTarget() {
  static bool lastButtonState = HIGH;
  bool currentState = digitalRead(BUTTON_PIN);

  if (lastButtonState == HIGH && currentState == LOW) {
    // Button was just pressed
    if (!targetSelected) {
      if (gps.location.isValid()) {
        targetLat = gps.location.lat();
        targetLon = gps.location.lng();

        // Optional: Save to SD
        File file = SD.open("target.txt", FILE_WRITE);
        if (file) {
          file.println(String(targetLat, 6) + "," + String(targetLon, 6));
          file.close();
        }

        // OLED Display
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("New Target Set:");
        display.setCursor(0, 10);
        display.print("Lat: "); display.println(targetLat, 6);
        display.print("Lon: "); display.println(targetLon, 6);
        display.display();

        // Serial Monitor
        Serial.println("New Target Set:");
        Serial.print("Lat: "); Serial.println(targetLat, 6);
        Serial.print("Lon: "); Serial.println(targetLon, 6);

        // NeoPixel flash white, then start blinking yellow
        pixels.fill(PXWHITE);
        pixels.show();
        delay(1000);
        pixels.fill(PXBLACK); // prepare for blinking
        pixels.show();

        // Set target flag
        targetSelected = true;
        steeringStarted = false;  // wait until second press
      } else {
        Serial.println("GPS not valid yet.");
      }
    } else if (!steeringStarted) {
      // Second press: start steering immediately
      steeringStarted = true;
      pixels.fill(PXGREEN);
      pixels.show();

      // Optional: Display start message
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Steering Started!");
      display.display();

      Serial.println("Steering Started!");
    }
  }

  lastButtonState = currentState;
}
