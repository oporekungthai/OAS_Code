void logData() {
    char filename[15];

    // Check if it's time to create a new file
    if (millis() - fileStartTime >= FILE_INTERVAL * 1000) {
        fileNumber++;
        if (fileNumber > 9999) fileNumber = 0;
        fileStartTime = millis();
        logCounter = 0;
    }

    snprintf(filename, sizeof(filename), "data%04lu.txt", fileNumber);
    File dataFile = SD.open(filename, FILE_WRITE);

    if (dataFile) {
        dataFile.println(createDataString());
        dataFile.close();
        logCounter++;
    } else {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("File Error!"));
        display.display();
        delay(2000);
    }
}