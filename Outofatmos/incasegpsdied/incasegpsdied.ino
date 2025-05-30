// --- GPS Fix Fail-Safe ---
bool gpsFixAcquired = false;
unsigned long gpsFixStartTime = 0;
bool skipTargetingDueToNoFix = false;

void loop() {
  static bool gpsTimerStarted = false;

  // Start GPS Fix timer on boot
  if (!gpsTimerStarted) {
    gpsFixStartTime = millis();
    gpsTimerStarted = true;
  }

  // Check if GPS fix is acquired
  if (gps.location.isValid() && gps.location.age() < 2000) {
    gpsFixAcquired = true;
  }

  // If GPS fix not acquired in 60s, skip targeting and use fallback behavior
  if (!gpsFixAcquired && millis() - gpsFixStartTime > 60000 && !skipTargetingDueToNoFix) {
    skipTargetingDueToNoFix = true;
    currentPhase = "GPS FailSafe";
    sendData("GPS Fix Timeout - Skipping Target Select");
    logData();
  }

  readSensors();

  static bool rocketDeployed = false;
  static bool tiltReady = false;
  static bool cansatDeployed = false;
  static bool steeringStarted = false;
  static unsigned long deployTime = 0;

  // Only check button if GPS fix works and not skipped
  if (!skipTargetingDueToNoFix) {
    checkButtonAndSetTarget();
  } else {
    targetSelected = true;  // Force target selected to proceed phases
  }

  if (targetSelected && !rocketDeployed && checkDeployment()) {
    rocketDeployed = true;
    deployTime = millis();
    currentPhase = "Deployed";
    sendData("Rocket Deployed");
    logData();
  }

  if (rocketDeployed && !tiltReady) {
    bool apogeeDetected = checkApogeeAndTilt();
    bool timeoutReached = CHECK_MAX_TIME_BEFORE_DEPLOY && (millis() - deployTime > MAX_TIME_BEFORE_DEPLOY);

    if (apogeeDetected || timeoutReached) {
      tiltReady = true;
      currentPhase = "Tilting";
      if (apogeeDetected && timeoutReached) {
        sendData("Apogee + Timeout");
      } else if (apogeeDetected) {
        sendData("Apogee Reached");
      } else {
        sendData("Timeout Fallback Triggered");
      }
      logData();
    }
  }

  if (tiltReady && !cansatDeployed) {
    deployCanSat();
    cansatDeployed = true;
    deployTime = millis();
    currentPhase = "CanSatDeployed";
    sendData("CanSat Deployed");
    logData();
  }

  if (cansatDeployed && millis() - deployTime >= 3500 && !steeringStarted) {
    steeringStarted = true;
    currentPhase = "Steering";
    sendData("Steering Started");
    logData();
  }

  if (steeringStarted) {
    if (gpsFixAcquired) {
      steerToTarget();  // :white_check_mark: Use real steering
    } else {
      spinInPlace();    // :white_check_mark: New fallback behavior
    }
  }

  if (millis() - lastLogTime >= LOG_INTERVAL) {
    logData();
    sendData(createDataString());
    lastLogTime = millis();
  }

  blinkPixelForPhase();
  smartDelay(200);
}
