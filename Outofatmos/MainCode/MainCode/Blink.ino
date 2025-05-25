void blinkPixelForPhase() {
  unsigned long now = millis();

  if (now - lastBlinkTime >= BLINK_INTERVAL) {
    lastBlinkTime = now;
    yellowBlinkState = !yellowBlinkState;

    if (currentPhase == "Waiting") {
      // Blink Blue
      if (yellowBlinkState) {
        pixels.fill(PXBLUE);
      } else {
        pixels.fill(PXBLACK);
      }
      pixels.show();
    }
    else if (currentPhase == "Deployed") {
      // Blink Red
      if (yellowBlinkState) {
        pixels.fill(PXRED);
      } else {
        pixels.fill(PXBLACK);
      }
      pixels.show();
    }
    else if (currentPhase == "Tilting") {
      // Blink Yellow
      if (yellowBlinkState) {
        pixels.fill(PXYELLOW);
      } else {
        pixels.fill(PXBLACK);
      }
      pixels.show();
    }
    else if (currentPhase == "CanSatDeployed") {
      // Blink White
      if (yellowBlinkState) {
        pixels.fill(PXWHITE);
      } else {
        pixels.fill(PXBLACK);
      }
      pixels.show();
    }
    else if (currentPhase == "Steering") {
      // Solid Green (no blink)
      pixels.fill(PXGREEN);
      pixels.show();
    }
    else {
      // Default solid white if unknown phase
      pixels.fill(PXWHITE);
      pixels.show();
    }
  }
}
