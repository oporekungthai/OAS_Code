void blinkPixelForPhase() {
  unsigned long now = millis();

  if (now - lastBlinkTime >= BLINK_INTERVAL) {
    lastBlinkTime = now;
    yellowBlinkState = !yellowBlinkState;

    if (currentPhase == "Waiting") {
      if (yellowBlinkState) {
        pixels.fill(PXYELLOW);
      } else {
        pixels.fill(PXBLACK);
      }
      pixels.show();
    }
    else if (currentPhase == "Deployed") {
      if (yellowBlinkState) {
        pixels.fill(PXRED);
      } else {
        pixels.fill(PXBLACK);
      }
      pixels.show();
    }
    else if (currentPhase == "Tilting") {
      if (yellowBlinkState) {
        pixels.fill(PXRED);
      } else {
        pixels.fill(PXBLACK);
      }
      pixels.show();
    }
    else if (currentPhase == "CanSatDeployed") {
      if (yellowBlinkState) {
        pixels.fill(PXWHITE);
      } else {
        pixels.fill(PXBLACK);
      }
      pixels.show();
    }
    else if (currentPhase == "Steering") {
      pixels.fill(PXGREEN);
      pixels.show();
    }
    else {
      pixels.fill(PXWHITE);
      pixels.show();
    }
  }
}
